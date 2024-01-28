#!/usr/bin/env python3

"""
Script to auto-generate ROS to protobuf conversion code.
Created by Robert Pafford

Introspects on the ROS message type and creates the appropriate Proto and C++ files to allow easy communication with
nanopb.

This file is split into five sections
 1. A set of classes to implement generation of Protobuf definitions from Python
 2. Classes which generate the C++ code using protobuf definition classes and other provided info
 3. Code which generates topics and parameter conversions, introspecting ROS as required to extract message data
 4. Code which processes the yaml into a usable format to be used by the code above
 5. A main function, to handle command line arguments and execute the script
"""

import enum
import fnmatch
import hashlib
import importlib
import io
import math
import os
import re
import sys
import yaml

import rosidl_parser.definition

#region Protobuf Proto Representation

PROTOBUF_INDENT_LVL = 2
NANOPB_OPT_PREFIX = "(nanopb)."

reserved_keywords = [
    "syntax", "import", "weak", "public", "package", "option", "inf", "repeated", "optional", "required", "bool",
    "string", "bytes", "float", "double", "int32", "int64", "uint32", "uint64", "sint32", "sint64", "fixed32",
    "fixed64", "sfixed32", "sfixed64", "group", "oneof", "map", "extensions", "to", "max", "reserved", "enum",
    "message", "extend", "service", "rpc", "stream", "returns"
]

def is_valid_identifier(token: str):
    return re.fullmatch(r"^[a-zA-Z_]([a-zA-Z0-9_]*)$", token) is not None and token not in reserved_keywords

def is_valid_type_name(token: str):
    if token[0] == ".":  # Type names can start with a .
        token = token[1:]
    # Make sure every segment of token is a valid identifier
    return all(map(is_valid_identifier, token.split('.')))

def is_valid_option_name(token: str):
    # Option names can consist of either extension names (type names in parenthesis) or simple names (identifiers)
    for subtoken in re.split(r"\.(?![^()]*\))", token):
        if len(subtoken) == 0:
            return False
        if subtoken[0] == "(":
            # It's an extension end
            # Must end with parenthesis if it starts with one
            if subtoken[-1] != ")":
                return False
            # Check that subtoken is valid type name
            if not is_valid_type_name(subtoken[1:-1]):
                return False
        else:
            # If it doesn't start with parenthesis, it must be a valid identifier
            if not is_valid_identifier(subtoken):
                return False
    return True

def format_nanopb_options(options: 'list[ProtobufOption]') -> str:
    return " ".join([x.as_nanopb_optdecl() for x in options if x.name.startswith(NANOPB_OPT_PREFIX)])

def filter_compilable_options(options: 'list[ProtobufOption]') -> 'list[ProtobufOption]':
    return [x for x in options if not x.name.startswith(NANOPB_OPT_PREFIX)]

class ProtobufType:
    def __init__(self, name: str, parent_type: 'ProtobufMessageType | ProtobufTopScope'):
        assert is_valid_identifier(name) or parent_type == ProtobufScalarType, name + " is invalid"
        self.name = name
        self.parent_type = parent_type
        parent_type._add_child_type(self)

    def write_decl(self, f: 'io.TextIOBase', indent=0):
        # Creates declaration of type
        # Note this must be created by each class implementing this
        raise NotImplementedError("'{}' does not implement required write_decl()".format(self.__class__.__name__))

    def write_nanopb_options(self, f: 'io.TextIOBase', scope: 'list[str]'):
        # By default do nothing, can be overidden by child methods
        pass

    def qualified_name(self, scope_type: 'ProtobufType | None' = None):
        full_name = self.name

        parent_type = self.parent_type
        while not isinstance(parent_type, ProtobufTopScope) and parent_type != ProtobufScalarType:
            if parent_type == scope_type:
                # Exit early if we find ourselves in the scope
                return full_name

            full_name = parent_type.name + "." + full_name
            parent_type = parent_type.parent_type

        return full_name

    def get_path(self) -> 'list[str]':
        scope = self.parent_type.get_path()
        scope.append(self.name)
        return scope

class ProtobufFieldCardinality(enum.Enum):
    NONE = ""
    OPTIONAL = "optional"
    REPEATED = "repeated"

class ProtobufOption:
    def __init__(self, name: str, value: int):
        assert type(value) == int or type(value) == bool or type(value) == str
        assert is_valid_option_name(name)

        self.name = name
        self.value = value

    @property
    def value_str(self):
        if type(self.value) == str:
            return self.value
        elif type(self.value) == int:
            return str(self.value)
        elif type(self.value) == bool:
            return "true" if self.value else "false"
        else:
            raise ValueError()

    def as_nanopb_optdecl(self):
        assert self.name.startswith(NANOPB_OPT_PREFIX), "Invalid nanopb option"
        return "{}:{}".format(self.name[len(NANOPB_OPT_PREFIX):], self.value_str)

    def write_decl(self, f: 'io.TextIOBase', indent=0) -> str:
        f.write(" "*indent + "option {} = {};\n".format(self.name, self.value_str))

    def as_compact_decl(self) -> str:
        return "{} = {}".format(self.name, self.value_str)

class ProtobufEnumDecl:
    def __init__(self, name: str, value: int):
        assert type(value) == int
        assert is_valid_identifier(name), "Invalid Identifier: '{}'".format(name)

        self.name = name
        self.value = value

    def write_decl(self, f: 'io.TextIOBase', indent=0) -> str:
        f.write(" "*indent + "{} = {};\n".format(self.name, self.value))

class ProtobufFieldDecl:
    def __init__(self, field_type: 'ProtobufType', name: str, num: int,
                 cardinality: 'ProtobufFieldCardinality' = ProtobufFieldCardinality.NONE,
                 options: 'list[ProtobufOption]' = []):
        assert type(cardinality) == ProtobufFieldCardinality
        assert type(num) == int
        assert is_valid_identifier(name), "Invalid Identifier: '{}'".format(name)

        self.cardinality = cardinality
        self.field_type = field_type
        self.name = name
        self.num = num
        self.options = options

    def write_nanopb_options(self, f: 'io.TextIOBase', scope: 'list[str]'):
        assert all(map(lambda x: x.name.startswith(NANOPB_OPT_PREFIX), self.options)), "Non nanopb option specified"
        nanopb_options = format_nanopb_options(self.options)
        if len(nanopb_options) > 0:
            f.write("{}.{} {}\n".format(".".join(scope), self.name, nanopb_options))

    def write_decl(self, f: 'io.TextIOBase', scope_type: 'ProtobufType', indent=0) -> str:
        if self.cardinality == ProtobufFieldCardinality.NONE:
            cardinality_str = ""
        else:
            cardinality_str = self.cardinality.value + " "

        field_options = filter_compilable_options(self.options)
        if len(field_options) > 0:
            compact_option_str = " [{}]".format(", ".join(map(lambda x: x.as_compact_decl(), field_options)))
        else:
            compact_option_str = ""

        f.write(" "*indent + "{}{} {} = {}{};\n".format(cardinality_str, self.field_type.qualified_name(scope_type),
                                                       self.name, self.num, compact_option_str))


class ProtobufMessageType(ProtobufType):
    def __init__(self, name: str, parent_type: 'ProtobufMessageType | ProtobufTopScope'):
        super().__init__(name, parent_type)
        self.child_types: 'list[ProtobufType]' = []
        self.fields: 'list[ProtobufFieldDecl]' = []
        self.options: 'list[ProtobufOption]' = []

    def _add_child_type(self, child_type: 'ProtobufType'):
        assert child_type not in map(lambda x: x.name, self.child_types), "Child already exists with same name"
        self.child_types.append(child_type)

    def add_option(self, option: 'ProtobufOption'):
        assert option.name not in map(lambda x: x.name, self.options), "Option already exists"
        self.options.append(option)

    def add_field(self, field: 'ProtobufFieldDecl'):
        assert field.name not in map(lambda x: x.name, self.fields), "Field already exists with same name"
        self.fields.append(field)

    def write_nanopb_options(self, f: 'io.TextIOBase', scope: 'list[str]'):
        current_scope = scope + [self.name]
        msg_nanopb_options = format_nanopb_options(self.options)
        if len(msg_nanopb_options) > 0:
            f.write("{} {}\n".format(".".join(current_scope), msg_nanopb_options))

        # Get all sub-message options
        for entry in self.child_types:
            entry.write_nanopb_options(f, current_scope)

        # Get all field options
        for entry in self.fields:
            entry.write_nanopb_options(f, current_scope)

    def write_decl(self, f: 'io.TextIOBase', indent=0):
        # Opening
        f.write(" " * indent + "message %s {\n" % self.name)

        # Options
        this_field_had_data = False
        for entry in filter_compilable_options(self.options):
            entry.write_decl(f, indent+PROTOBUF_INDENT_LVL)
            this_field_had_data = True

        # Sub messages
        last_field_had_data = this_field_had_data
        this_field_had_data = False
        for entry in self.child_types:
            if last_field_had_data and not this_field_had_data:
                f.write("\n")
            this_field_had_data = True
            entry.write_decl(f, indent+PROTOBUF_INDENT_LVL)

        # Fields
        last_field_had_data = this_field_had_data or last_field_had_data
        this_field_had_data = False
        for entry in self.fields:
            if last_field_had_data and not this_field_had_data:
                f.write("\n")
            this_field_had_data = True
            entry.write_decl(f, self, indent+PROTOBUF_INDENT_LVL)

        # Closing
        f.write(" " * indent + "}\n")

class ProtobufOneofType(ProtobufType):
    def __init__(self, name: str, parent_type: 'ProtobufMessageType'):
        super().__init__(name, parent_type)
        self.fields: 'list[ProtobufFieldDecl]' = []

    def add_field(self, field: 'ProtobufFieldDecl'):
        assert field.name not in map(lambda x: x.name, self.fields), "Field already exists with same name"
        self.fields.append(field)

    def write_nanopb_options(self, f: 'io.TextIOBase', scope: 'list[str]'):
        # oneof doesn't add to scope, just pass parent scope
        for entry in self.fields:
            entry.write_nanopb_options(f, scope)

    def write_decl(self, f: 'io.TextIOBase', indent=0):
        f.write(" " * indent + "oneof %s {\n" % self.name)
        for entry in self.fields:
            entry.write_decl(f, self.parent_type, indent+PROTOBUF_INDENT_LVL)
        f.write(" " * indent + "}\n")

class ProtobufEnumType(ProtobufType):
    def __init__(self, name: str, parent_type: 'ProtobufMessageType'):
        super().__init__(name, parent_type)
        self.values: 'list[ProtobufEnumDecl]' = []

    def add_value(self, value: 'ProtobufEnumDecl'):
        assert value.name not in map(lambda x: x.name, self.values), "Enum already exists with same name"
        assert value.value not in map(lambda x: x.value, self.values), "Enum already exists with same value"
        self.values.append(value)

    def write_decl(self, f: 'io.TextIOBase', indent=0):
        assert any(map(lambda x: x.value == 0, self.values))
        f.write(" " * indent + "enum %s {\n" % self.name)
        enum_zero = None
        for entry in self.values:
            if entry.value == 0:
                enum_zero = entry
                break
        assert enum_zero is not None, "Enum must have element with value 0"

        entry.write_decl(f, indent+PROTOBUF_INDENT_LVL)
        for entry in self.values:
            if entry.name == enum_zero.name:
                continue
            entry.write_decl(f, indent+PROTOBUF_INDENT_LVL)
        f.write(" " * indent + "}\n")

class ProtobufScalarType(ProtobufType):
    @classmethod
    def _add_child_type(cls, msg):
        assert type(msg) == ProtobufScalarType, "Cannot add child to scalar type"

    @classmethod
    def create_classes(cls):
        cls.type_double = ProtobufScalarType("double", cls)
        cls.type_float = ProtobufScalarType("float", cls)
        cls.type_int32 = ProtobufScalarType("int32", cls)
        cls.type_int64 = ProtobufScalarType("int64", cls)
        cls.type_uint32 = ProtobufScalarType("uint32", cls)
        cls.type_uint64 = ProtobufScalarType("uint64", cls)
        cls.type_sint32 = ProtobufScalarType("sint32", cls)
        cls.type_sint64 = ProtobufScalarType("sint64", cls)
        cls.type_fixed32 = ProtobufScalarType("fixed32", cls)
        cls.type_fixed64 = ProtobufScalarType("fixed64", cls)
        cls.type_sfixed32 = ProtobufScalarType("sfixed32", cls)
        cls.type_sfixed64 = ProtobufScalarType("sfixed64", cls)
        cls.type_bool = ProtobufScalarType("bool", cls)
        cls.type_string = ProtobufScalarType("string", cls)
        cls.type_bytes = ProtobufScalarType("bytes", cls)
ProtobufScalarType.create_classes()  # Required since python is silly

class ProtobufExtendType:
    def __init__(self, extended_name: str, parent_type: 'ProtobufTopScope'):
        assert is_valid_type_name(extended_name), "Invalid Extended Name: '{}'".format(extended_name)
        self.extended_name = extended_name
        self.parent_type = parent_type
        self.parent_type._add_extension(self)
        self.fields: 'list[ProtobufFieldDecl]' = []

    def add_field(self, field: 'ProtobufFieldDecl'):
        assert field.name not in map(lambda x: x.name, self.fields), "Field already exists with same name"
        self.fields.append(field)

    def write_nanopb_options(self, f: 'io.TextIOBase', scope: 'list[str]'):
        pass

    def write_decl(self, f: 'io.TextIOBase', indent=0):
        # Opening
        f.write(" " * indent + "extend %s {\n" % self.extended_name)

        # Fields
        for entry in self.fields:
            entry.write_decl(f, self, indent+PROTOBUF_INDENT_LVL)

        # Closing
        f.write(" " * indent + "}\n")

class ProtobufTopScope:
    def __init__(self, package: str = None):
        assert package is None or is_valid_type_name(package), "Invalid package name: '{}'".format(package)
        self.package = package
        self.child_types: 'list[ProtobufType]' = []
        self.extensions: 'list[ProtobufExtendType]' = []
        self.imports: 'list[str]' = []
        self.options: 'list[ProtobufOption]' = []

    def add_import(self, import_str: str):
        if import_str in self.imports:
            return
        self.imports.append(import_str)

    def _add_extension(self, extension: 'ProtobufExtendType'):
        assert isinstance(extension, ProtobufExtendType), "Must be extension"
        self.extensions.append(extension)

    def _add_child_type(self, child_type: 'ProtobufType'):
        assert child_type.name not in map(lambda x: x.name, self.child_types), "Child already exists with same name"
        self.child_types.append(child_type)

    def add_option(self, option: 'ProtobufOption'):
        assert option.name not in map(lambda x: x.name, self.options), "Option already exists"
        self.options.append(option)

    def write_nanopb_options(self, f: 'io.TextIOBase'):
        for entry in self.child_types:
            entry.write_nanopb_options(f, self.package.split('.'))

    def write_proto(self, f: 'io.TextIOBase') -> str:
        f.write("syntax = \"proto3\";\n")

        if self.package is not None:
            f.write("package {};\n".format(self.package))

        # Imports
        self.imports.sort()
        if len(self.imports) > 0:
            f.write("\n")
        for import_str in self.imports:
            f.write("import \"{}\";\n".format(import_str))

        # Any extend statements
        for entry in self.extensions:
            f.write("\n")
            entry.write_decl(f)

        # Any options
        if len(self.options) > 0:
            f.write("\n")
        for entry in self.options:
            entry.write_decl(f)

        # All child types
        for entry in self.child_types:
            f.write("\n")
            entry.write_decl(f)

    def get_path(self) -> 'list[str]':
        return [self.package]

#endregion

#region C++ Code Representation

CPP_INDENT_LVL = 4

def escape_c_string(string: 'str | bytes') -> str:
    def escape_c_char(c: 'str | int') -> str:
        # Using ord so it'll work with both bytes and str
        if isinstance(c, str):
            assert len(c) == 1
            c = ord(c)
        # Escape required printable characters
        if c == ord("\""):
            return "\\\""
        elif c == ord("\\"):
            return "\\\\"
        # Escape common non-printable characters so they don't get mangled by the catch all
        elif c == ord("\n"):
            return "\\n"
        elif c == ord("\0"):
            return "\\0"
        elif c == ord("\r"):
            return "\\r"
        elif c == ord("\t"):
            return "\\t"
        # Escape any non-printable characters
        elif c < 32 or (c > 126 and c < 160):
            return "\\x{:02X}".format(c)
        # If none of the checks above passed, then it's a valid string character
        else:
            return chr(c)
    # Run escape function on every character in the string, and stitch the string back together
    return "\"" + "".join(map(escape_c_char, string)) + "\""

def write_cpp_code(f: 'io.TextIOBase', src: 'list[list | str]', indent=0):
    """Writes C++ code in the format of an array. Each element can either be a string or another list of strings.

    If the element of the list is a string, it'll write the string to the file wih the specified indent. If the
    element is a list, it will add braces, then write the elements of the list indented by CPP_INDENT_LVL.

    Any dictionaries will be interpreted in case format.

    Example: ["void main()", ['printf("Hello World");']]
    Results in:
    void main()
    {
        printf("Hello World");
    }
    """
    def write_cpp_cases(case_src: 'dict[str, list[str | list]]'):
        """Writes the cases of a switch case statement, where the key are the colon, and the value is a valid
        list for write_cpp_code.
        """
        for key, value in case_src.items():
            f.write(" " * indent)
            f.write(key)
            f.write(":\n")
            write_cpp_code(f, value, indent+CPP_INDENT_LVL)

    for entry in src:
        if isinstance(entry, str):
            f.write(" " * indent)
            f.write(entry)
            f.write("\n")
        elif isinstance(entry, dict):
            write_cpp_cases(entry)
        else:
            f.write(" " * indent)
            f.write("{\n")
            write_cpp_code(f, entry, indent+CPP_INDENT_LVL)
            f.write(" " * indent)
            f.write("}\n")

def field_to_cpp_oneof_case(name: str):
    case_name = "k"
    for entry in name.split("_"):
        if len(entry) == 0:
            continue
        case_name += entry[0].upper() + entry[1:]
    return case_name

class CppFile:
    """Represents a C++ file generated for ROS to Protobuf conversion.

    This will be populated will all the information required to generate a C++ source file for conversion between
    protobuf and ROS.
    """

    def __init__(self, namespace: str):
        self.namespace = namespace
        self.include_statements: 'list[str]' = []
        self.conversions: 'list[CppMsgConversion]' = []
        self.topic_handler: 'CppTopicHandler | None' = None

    def add_include(self, path: str, system_include: bool):
        escaped_path = escape_c_string(path)
        if system_include:
            self.include_statements.append("#include <{}>".format(escaped_path[1:-1]))
        else:
            self.include_statements.append("#include {}".format(escaped_path))

    def add_conversion(self, elem: 'CppMsgConversion'):
        self.conversions.append(elem)

    def writeto(self, f: 'io.TextIOBase'):
        if len(self.include_statements) > 0:
            for elem in self.include_statements:
                f.write(elem)
                f.write("\n")
            f.write("\n")

        f.write("namespace %s {\n" % self.namespace)

        for elem in self.conversions:
            f.write("\n")
            elem.writeto(f)

        if self.topic_handler is not None:
            f.write("\n")
            self.topic_handler.writeto(f)

        f.write("\n} // end namespace %s\n" % self.namespace)

class CppFixedIntTypeDecode:
    """Utility class to decode C++ fixed int types (like uint8_t) and get useful attributes from them."""
    def __init__(self, decoded_name: str):
        if decoded_name.startswith("u"):
            decoded_name = decoded_name[1:]
            self.is_unsigned = True
        else:
            self.is_unsigned = False

        if not decoded_name.startswith("int") or not decoded_name.endswith("_t"):
            raise ValueError("Invalid fixed integer type name")
        decoded_name = decoded_name[3:-2]  # Trim prefix

        if not decoded_name in {"8", "16", "32", "64"}:
            raise ValueError("Invalid fixed integer width: '{}'".format(decoded_name))
        self.width = int(decoded_name)

        if self.is_unsigned:
            self.max_expression = "UINT{}_MAX".format(self.width)
        else:
            self.max_expression = "INT{}_MAX".format(self.width)
            self.min_expression = "INT{}_MIN".format(self.width)

class CppMsgConversion:
    """Class to handle conversion of a single ROS message type.

    This is instantiated for every ROS message which is translated. Each field of the ROS message is then added into
    this class with the required information to perform conversion.
    """
    def __init__(self, proto_msg: 'ProtobufMessageType', msg_namespaced: 'tuple[str]'):
        self.full_msg_name = '/'.join(msg_namespaced)
        # Types of ros messages
        self.cpp_protobuf_type_str = "::".join(proto_msg.get_path())
        self.cpp_ros_type_str = "::".join(msg_namespaced)

        # Compute function names
        self.ros_to_proto_func_name = "convert_from_{}".format("__".join(msg_namespaced))
        self.ros_to_proto_func_decl = "static void {}(const {}& ros_msg, {}& proto_msg)".format(
            self.ros_to_proto_func_name , self.cpp_ros_type_str, self.cpp_protobuf_type_str)
        self.proto_to_ros_func_name = "convert_to_{}".format("__".join(msg_namespaced))
        self.proto_to_ros_func_decl = "static void {}({}& ros_msg, const {}& proto_msg)".format(
            self.proto_to_ros_func_name, self.cpp_ros_type_str, self.cpp_protobuf_type_str)

        self.ros_to_proto_func_elems = []
        self.proto_to_ros_func_elems = []

        # Variables to keep track if the ros to proto or proto to ros conversions are used
        # Prevents code being generated if its never goint to be used
        self._proto_to_ros_used = False
        self._ros_to_proto_used = False
        self.child_msg_conversions: 'list[CppMsgConversion]' = []

    def gen_cpp_basic_type(self, ros_field: str, ros_field_type: str, proto_field: str, proto_field_type: str):
        """Conversion for fields with basic types (numbers/unbounded strings)"""
        ##########
        # Proto -> Ros Conversion

        # If we need to cast between int sizes, there's some extra code to add
        if ros_field_type != proto_field_type:
            protobuf_type_decoded = CppFixedIntTypeDecode(proto_field_type)
            ros_type_decoded = CppFixedIntTypeDecode(ros_field_type)

            assert ros_type_decoded.is_unsigned == protobuf_type_decoded.is_unsigned, "Cannot mix unsigned and signed types"

            # Read the protobuf message into temporary variable (of protobuf type, so we can check it)
            # <proto_type> tmp_variable = proto_msg.field();
            self.proto_to_ros_func_elems.append("{} {} = proto_msg.{}();".format(proto_field_type, ros_field, proto_field))

            # Generate bounds check depending if signed or unsigned
            if ros_type_decoded.is_unsigned:
                # tmp_variable > MAX_VAL
                bounds_check = "{} > {}".format(ros_field, ros_type_decoded.max_expression)
            else:
                # tmp_variable > MAX_VAL && tmp_variable < MIN_VAL
                bounds_check = "{0} > {1} && {0} < {2}".format(ros_field, ros_type_decoded.max_expression,
                                                                 ros_type_decoded.min_expression)

            self.proto_to_ros_func_elems += [
                "if ({})".format(bounds_check),
                [
                    "throw MsgConversionError({}, {}, \"Integer out of range\");".format(escape_c_string(self.full_msg_name),
                                                                                         escape_c_string(ros_field))
                ],
                "ros_msg.{} = {};".format(ros_field, ros_field)
            ]
        else:
            # No needs for type checking/casting, just directly assign
            self.proto_to_ros_func_elems.append("ros_msg.{} = proto_msg.{}();".format(ros_field, proto_field))

        ##########
        # Ros -> Proto Conversion

        # No checks to generate, all basic ros message types should cleanly go into protobuf messages
        self.ros_to_proto_func_elems.append("proto_msg.set_{}(ros_msg.{});".format(proto_field, ros_field))

    def gen_cpp_child_type(self, ros_field: 'str', proto_field: 'str', field_msg_conversion: 'CppMsgConversion'):
        """Conversion for fields with a sub-message type."""
        ##########
        # Proto -> Ros Conversion

        # No checks, the decode function does all the checking for us

        # convert_to_geometry_msgs__Vector3(ros_msg.linear, proto_msg.linear());
        self.proto_to_ros_func_elems.append("{}(ros_msg.{}, proto_msg.{}());".format(
            field_msg_conversion.proto_to_ros_func_name, ros_field, proto_field
        ))

        ##########
        # Ros -> Proto Conversion
        # convert_from_geometry_msgs__Vector3(ros_msg.linear, *proto_msg.mutable_linear());
        self.ros_to_proto_func_elems.append("{}(ros_msg.{}, *proto_msg.mutable_{}());".format(
            field_msg_conversion.ros_to_proto_func_name, ros_field, proto_field
        ))

        self.child_msg_conversions.append(field_msg_conversion)

    def gen_cpp_enum_type(self, ros_field: 'str', proto_field: 'str', enum_type: 'ProtobufEnumType'):
        """Conversion for fields with an enum type."""
        ##########
        # Proto -> Ros Conversion

        # No checks, since the enum has fixed values that we check during compilation, which protobuf enforces
        self.proto_to_ros_func_elems.append("ros_msg.{} = proto_msg.{}();".format(ros_field, proto_field))

        ##########
        # Ros -> Proto Conversion

        cpp_enum_type = "::".join(enum_type.get_path())

        # Check that ros message contains a valid enum value before casting field value to enum
        self.ros_to_proto_func_elems += [
            "if (!proto_msg.{}_IsValid(ros_msg.{}))".format(enum_type.name, ros_field),
            [
                "throw MsgConversionError({}, {}, \"Invalid Enum Value\");".format(escape_c_string(self.full_msg_name),
                                                                                   escape_c_string(ros_field))
            ],
            "proto_msg.set_{}(static_cast<{}>(ros_msg.{}));".format(proto_field, cpp_enum_type, ros_field)
        ]

    def gen_cpp_bounded_string_type(self, ros_field: 'str', proto_field: 'str', max_size: int):
        """Conversion for fields with a bounded string type."""
        ##########
        # Proto -> Ros Conversion

        # Check that the protobuf message string isn't out of bounds
        self.proto_to_ros_func_elems += [
            "if (proto_msg.{}().size() > {})".format(proto_field, max_size),
            [
                "throw MsgConversionError({}, {}, \"Bounded string too large\");".format(escape_c_string(self.full_msg_name),
                                                                                         escape_c_string(ros_field))
            ],
            "ros_msg.{} = proto_msg.{}();".format(ros_field, proto_field)
        ]

        ##########
        # Ros -> Proto Conversion

        # Check that the ros message string isn't out of bounds
        self.ros_to_proto_func_elems += [
            "if (ros_msg.{}.size() > {})".format(ros_field, max_size),
            [
                "throw MsgConversionError({}, {}, \"Bounded string too large\");".format(escape_c_string(self.full_msg_name),
                                                                                         escape_c_string(ros_field))
            ],
            "proto_msg.set_{}(ros_msg.{});".format(proto_field, ros_field)
        ]

    def _gen_common_cpp_loop(self, ros_field: str, proto_field: str, max_size: 'int | None', fixed_size: bool,
                             proto_to_ros_for_elems: 'list[str | list]', ros_to_proto_for_elems: 'list[str | list]'):
        """General conversion for any protobuf repeated fields (arrays, bounded, and unbounded sequences).

        Private function used by specific conversion code below.
        """
        ##########
        # Proto -> Ros Conversion

        # Add in size checks if required
        if max_size is not None:
            if fixed_size:
                self.proto_to_ros_func_elems.append("if (proto_msg.{}_size() != {})".format(proto_field, max_size))
            else:
                self.proto_to_ros_func_elems.append("if (proto_msg.{}_size() > {})".format(proto_field, max_size))
            self.proto_to_ros_func_elems.append([
                "throw MsgConversionError({}, {}, \"Invalid Array Size\");".format(escape_c_string(self.full_msg_name),
                                                                                   escape_c_string(ros_field))
            ])

        # Clear if a vector type (array types are fixed, this won't work)
        if not fixed_size:
            self.proto_to_ros_func_elems.append("ros_msg.{}.clear();".format(ros_field))

        # Copy all elements in protobuf array
        self.proto_to_ros_func_elems.append("for (int i = 0; i < proto_msg.{}_size(); i++)".format(proto_field))
        self.proto_to_ros_func_elems.append(proto_to_ros_for_elems)

        ##########
        # Ros -> Proto Conversion

        # Add in size checks if required
        if max_size is not None:
            if fixed_size:
                self.ros_to_proto_func_elems.append("if (ros_msg.{}.size() > {})".format(ros_field, max_size))
            else:
                self.ros_to_proto_func_elems.append("if (ros_msg.{}.size() != {})".format(ros_field, max_size))
            self.ros_to_proto_func_elems.append([
                "throw MsgConversionError({}, {}, \"Invalid Array Size\");".format(escape_c_string(self.full_msg_name),
                                                                                   escape_c_string(ros_field))
            ])

        # Copy in elements
        self.ros_to_proto_func_elems += [
            f"proto_msg.clear_{proto_field}();",
            f"for (auto& entry : ros_msg.{ros_field})",
            ros_to_proto_for_elems
        ]

    def gen_cpp_array_basic_type(self, ros_field: str, ros_field_type: str, proto_field: str, proto_field_type: str,
                                 max_size: 'int | None', fixed_size: bool):
        """Conversion for array fields with basic types (numbers/unbounded strings)"""
        ##########
        # Proto -> Ros Conversion
        proto_to_ros_for_elems = []

        # Perform bounds checks if ros and proto field types don't match
        if ros_field_type != proto_field_type:
            protobuf_type_decoded = CppFixedIntTypeDecode(proto_field_type)
            ros_type_decoded = CppFixedIntTypeDecode(ros_field_type)

            assert ros_type_decoded.is_unsigned == protobuf_type_decoded.is_unsigned, "Cannot mix unsigned and signed types"

            # Read the protobuf message into temporary variable (of protobuf type, so we can check it)
            # <proto_type> tmp_variable = proto_msg.field();
            proto_to_ros_for_elems.append("{} {} = proto_msg.{}(i);".format(proto_field_type, ros_field, proto_field))

            # Generate bounds check depending if signed or unsigned
            if ros_type_decoded.is_unsigned:
                # tmp_variable > MAX_VAL
                bounds_check = "{} > {}".format(ros_field, ros_type_decoded.max_expression)
            else:
                # tmp_variable > MAX_VAL && tmp_variable < MIN_VAL
                bounds_check = "{0} > {1} && {0} < {2}".format(ros_field, ros_type_decoded.max_expression,
                                                                 ros_type_decoded.min_expression)

            proto_to_ros_for_elems += [
                "if ({})".format(bounds_check),
                [
                    "throw MsgConversionError({}, {}, \"Integer out of range\");".format(escape_c_string(self.full_msg_name),
                                                                                         escape_c_string(ros_field))
                ]
            ]

            if fixed_size:
                proto_to_ros_for_elems.append("ros_msg.{}.at(i) = {};".format(ros_field, ros_field))
            else:
                proto_to_ros_for_elems.append("ros_msg.{}.push_back({});".format(ros_field, ros_field))
        else:
            # No needs for type checking/casting, just directly assign
            if fixed_size:
                # You need to call .at to assign arrays
                proto_to_ros_for_elems.append("ros_msg.{}.at(i) = proto_msg.{}(i);".format(ros_field, proto_field))
            else:
                proto_to_ros_for_elems.append("ros_msg.{}.push_back(proto_msg.{}(i));".format(ros_field, proto_field))

        ##########
        # Ros -> Proto Conversion

        # Just do direct copy, no bounds checking required
        ros_to_proto_for_elems = ["proto_msg.add_{}(entry);".format(proto_field)]

        # Call general array with for loop elements
        self._gen_common_cpp_loop(ros_field, proto_field, max_size, fixed_size, proto_to_ros_for_elems, ros_to_proto_for_elems)

    def gen_cpp_array_child_type(self, ros_field: str, proto_field: str, field_msg_conversion: 'CppMsgConversion',
                                 max_size: 'int | None', fixed_size: bool):
        """Conversion for array fields with a sub-message type."""
        ##########
        # Proto -> Ros Conversion

        # Just call child conversion function on each entry, adding new element to protobuf
        proto_to_ros_for_elems = []

        if not fixed_size:
            # If not fixed size, you need to add a new element to the array before trying to access it
            proto_to_ros_for_elems.append("ros_msg.{}.emplace_back();".format(ros_field))

        proto_to_ros_for_elems.append("{}(ros_msg.{}.at(i), proto_msg.{}(i));".format(
            field_msg_conversion.proto_to_ros_func_name, ros_field, proto_field))

        ##########
        # Ros -> Proto Conversion

        # Just call child conversion function on each entry, adding new element to protobuf
        ros_to_proto_for_elems = ["{}(entry, *proto_msg.add_{}());".format(
            field_msg_conversion.ros_to_proto_func_name, ros_field, proto_field)]

        # Call general array with for loop elements
        self._gen_common_cpp_loop(ros_field, proto_field, max_size, fixed_size, proto_to_ros_for_elems, ros_to_proto_for_elems)

        self.child_msg_conversions.append(field_msg_conversion)

    def gen_cpp_array_enum_type(self, ros_field: str, proto_field: str, enum_type: 'ProtobufEnumType',
                                 max_size: 'int | None', fixed_size: bool):
        """Conversion for array fields with an enum type."""
        ##########
        # Proto -> Ros Conversion

        # Copy each enum element without any checks
        # Syntax depends on if fixed size array
        if fixed_size:
            proto_to_ros_for_elems = ["ros_msg.{}.at(i) = proto_msg.{}(i);".format(ros_field, proto_field)]
        else:
            proto_to_ros_for_elems = ["ros_msg.{}.push_back(proto_msg.{}(i));".format(ros_field, proto_field)]

        ##########
        # Ros -> Proto Conversion

        # Get the full C++ type tsring for the enum
        cpp_enum_type = "::".join(enum_type.get_path())

        # Check that ros message contains a valid enum value before casting field value to enum
        ros_to_proto_for_elems = [
            "if (!proto_msg.{}_IsValid(entry))".format(enum_type.name),
            [
                "throw MsgConversionError({}, {}, \"Invalid Enum Value\");".format(escape_c_string(self.full_msg_name),
                                                                                   escape_c_string(ros_field))
            ],
            "proto_msg.add_{}(static_cast<{}>(entry));".format(proto_field, cpp_enum_type)
        ]

        # Call general array with for loop elements
        self._gen_common_cpp_loop(ros_field, proto_field, max_size, fixed_size, proto_to_ros_for_elems, ros_to_proto_for_elems)

    def gen_cpp_array_bounded_string_type(self, ros_field: 'str', proto_field: 'str', max_str_size: int,
                                    max_arr_size: 'int | None', fixed_arr_size: bool):
        """Conversion for array fields with a bounded string type."""
        ##########
        # Proto -> Ros Conversion

        # Check that the protobuf message string isn't out of bounds
        proto_to_ros_for_elems = [
            "if (proto_msg.{}(i).size() > {})".format(proto_field, max_str_size),
            [
                "throw MsgConversionError({}, {}, \"Bounded string too large\");".format(escape_c_string(self.full_msg_name),
                                                                                         escape_c_string(ros_field))
            ]
        ]
        if fixed_arr_size:
            proto_to_ros_for_elems.append("ros_msg.{}.at(i) = proto_msg.{}(i);".format(ros_field, proto_field))
        else:
            proto_to_ros_for_elems.append("ros_msg.{}.push_back(proto_msg.{}(i));".format(ros_field, proto_field))

        ##########
        # Ros -> Proto Conversion

        # Check that the ros message string isn't out of bounds
        ros_to_proto_for_elems = [
            "if (entry.size() > {})".format(max_str_size),
            [
                "throw MsgConversionError({}, {}, \"Bounded string too large\");".format(escape_c_string(self.full_msg_name),
                                                                                         escape_c_string(ros_field))
            ],
            "proto_msg.add_{}(entry);".format(proto_field)
        ]

        # Call general array with for loop elements
        self._gen_common_cpp_loop(ros_field, proto_field, max_arr_size, fixed_arr_size, proto_to_ros_for_elems,
                                  ros_to_proto_for_elems)

    def gen_cpp_array_bytes_type(self, ros_field: 'str', proto_field: 'str', max_size: 'int | None', fixed_size: bool):
        """Conversion for array fields which have been converted to protobuf bytes type.

        This is special as it does not use repeated fields, but instead uses a bytes object to save space on the wire.
        """
        ##########
        # Proto -> Ros Conversion

        # Get reference to ros type
        self.proto_to_ros_func_elems.append("auto& {0} = proto_msg.{0}();".format(proto_field))

        # Add in size checks if required
        if max_size is not None:
            if fixed_size:
                self.proto_to_ros_func_elems.append("if ({}.size() != {})".format(proto_field, max_size))
            else:
                self.proto_to_ros_func_elems.append("if ({}.size() > {})".format(proto_field, max_size))
            self.proto_to_ros_func_elems.append([
                "throw MsgConversionError({}, {}, \"Invalid Array Size\");".format(escape_c_string(self.full_msg_name),
                                                                                   escape_c_string(ros_field))
            ])

        # Re-assign elements of ros message
        if fixed_size:
            self.proto_to_ros_func_elems.append("std::copy({0}.begin(), {0}.end(), ros_msg.{1}.begin());".format(proto_field, ros_field))
        else:
            self.proto_to_ros_func_elems.append("ros_msg.{0}.assign({1}.begin(), {1}.end());".format(ros_field, proto_field))

        ##########
        # Ros -> Proto Conversion

        # Add in size checks if required
        if max_size is not None:
            if fixed_size:
                self.ros_to_proto_func_elems.append("if (ros_msg.{}.size() > {})".format(ros_field, max_size))
            else:
                self.ros_to_proto_func_elems.append("if (ros_msg.{}.size() != {})".format(ros_field, max_size))
            self.ros_to_proto_func_elems.append([
                "throw MsgConversionError({}, {}, \"Invalid Array Size\");".format(escape_c_string(self.full_msg_name),
                                                                                   escape_c_string(ros_field))
            ])

        # Copy in elements from vector
        self.ros_to_proto_func_elems.append("std::string {0}(ros_msg.{0}.begin(), ros_msg.{0}.end());".format(ros_field))
        self.ros_to_proto_func_elems.append("proto_msg.set_{}({});".format(proto_field, ros_field))

    def set_ros_to_proto_used(self):
        if not self._ros_to_proto_used:
            for child in self.child_msg_conversions:
                child.set_ros_to_proto_used()
            self._ros_to_proto_used = True

    def set_proto_to_ros_used(self):
        if not self._proto_to_ros_used:
            for child in self.child_msg_conversions:
                child.set_proto_to_ros_used()
            self._proto_to_ros_used = True

    def writeto(self, f: 'io.TextIOBase'):
        # In the event the message is empty (in std_msgs/msg/Empty, for example), add (void) so the compiler won't complain
        if len(self.proto_to_ros_func_elems) == 0:
            self.proto_to_ros_func_elems.append("(void) ros_msg;")
            self.proto_to_ros_func_elems.append("(void) proto_msg;")
        if len(self.ros_to_proto_func_elems) == 0:
            self.ros_to_proto_func_elems.append("(void) ros_msg;")
            self.ros_to_proto_func_elems.append("(void) proto_msg;")

        if self._ros_to_proto_used and self._proto_to_ros_used:
            write_cpp_code(f, [self.proto_to_ros_func_decl, self.proto_to_ros_func_elems, "",
                               self.ros_to_proto_func_decl, self.ros_to_proto_func_elems])
        elif self._proto_to_ros_used:
            write_cpp_code(f, [self.proto_to_ros_func_decl, self.proto_to_ros_func_elems])
        elif self._ros_to_proto_used:
            write_cpp_code(f, [self.ros_to_proto_func_decl, self.ros_to_proto_func_elems])
        else:
            raise RuntimeError(f"Generated CppMsgConversion for {self.full_msg_name}, but it was not used")

class CppParameterConversion:
    """Class to handle conversion between RCLCPP parameter types and protobuf message types.

    This will generate code to convert between the cached local variable (must acquire lock first) and a given protobuf
    message.
    """
    def __init__(self, param_decl: 'FirmwareParamDecl', cached_var_name: str, param_field_decl: 'ProtobufFieldDecl'):
        self.cached_var_name = cached_var_name
        self.param_decl = param_decl
        self.param_field_decl = param_field_decl

    def cached_to_proto_conversion(self, proto_msg_var_name: str):
        if self.param_decl.protobuf_type[0] == ProtobufScalarType.type_bytes:
            return [f"{proto_msg_var_name}->set_{self.param_field_decl.name}(std::string({self.cached_var_name}.begin(), {self.cached_var_name}.end()));"]
        elif self.param_decl.protobuf_type[1] == ProtobufFieldCardinality.REPEATED:
            return [
                f"auto array_submsg = {proto_msg_var_name}->mutable_{self.param_field_decl.name}();",
                f"for (const auto &entry : {self.cached_var_name})", [
                    "array_submsg->add_entry(entry);"
                ]
            ]
        else:
            return [f"{proto_msg_var_name}->set_{self.param_field_decl.name}({self.cached_var_name});"]

class CppTopicHandler:
    """Class to generate code for creating the ROS bridge for the protobuf messages.

    This is responsible for generating the TopicMessageHandler class.
    This creates all of the required publishers/subscribers, and route protobuf messages to the appropriate topic.
    """
    def __init__(self, comm_msg: 'ProtobufMessageType', submsg_oneof: 'ProtobufOneofType',
                 connect_field: 'ProtobufFieldDecl', ack_field: 'ProtobufFieldDecl', targets: 'list[str]'):
        self.targets = targets

        # Extract package name from comm_msg
        parent = comm_msg.parent_type
        while not isinstance(parent, ProtobufTopScope):
            parent = parent.parent_type
        self.package = parent.package

        # Save the comm_msg C++ type name
        self.comm_msg_type_str = "::".join(comm_msg.get_path())
        assert submsg_oneof.parent_type == comm_msg, "Expected oneof to be part of comm_msg"
        self.comm_msg_submsg_oneof = submsg_oneof
        assert connect_field in submsg_oneof.fields, "Expected connect field to be part of submsg oneof"
        self.comm_msg_connect_field = connect_field
        assert ack_field in comm_msg.fields, "Expected ack to be child of comm_msg"
        self.comm_msg_ack_field = ack_field

        self.constructor_elems: 'list[str | list]' = []              # C++ code to insert into constructor
        self.process_swich_case_elems: 'dict[str, list]' = {}        # C++ code to insert into switch case for processing messages
        self.private_context_entries: 'list[list[str | list]]' = []  # List of C++ code lists to be put in the private ctx

    def _create_target_enum_name(self, name: str):
        return "TARGET_" + name.upper()

    def writeto(self, f: 'io.TextIOBase'):
        f.write("class TopicMessageHandler: public MessageHandlerItf {\n")
        f.write("public:\n")

        # Create target type enums
        # Needs to be more manual since enum needs a semicolon after the ending bracket
        f.write(" " * CPP_INDENT_LVL + "enum TargetType {\n")
        target_enums = [self._create_target_enum_name(x) + "," for x in self.targets]
        write_cpp_code(f, target_enums, CPP_INDENT_LVL*2)
        f.write(" " * CPP_INDENT_LVL + "};\n")
        f.write("\n")

        # Create string target to enum type conversion map
        # Allows string lookup once, and then all other checks will be performed on the enum, which is a lot faster
        target_enum_map_decl = ["static const std::unordered_map<std::string, TargetType> targetStrMap;"]
        write_cpp_code(f, target_enum_map_decl, CPP_INDENT_LVL)
        f.write("\n")

        # Constructor
        constructor = [
            "TopicMessageHandler(rclcpp::Node& node, RosProtobufBridge &bridge, std::string targetStr): MessageHandlerItf(bridge), logger_(node.get_logger())",
            [
                # Decode the target string
                "auto targetEntry = targetStrMap.find(targetStr);",
                "if (targetEntry == targetStrMap.end())", [
                    'throw std::runtime_error("Invalid target specified: " + targetStr);'
                ],
                "TargetType target = targetEntry->second;",

                # Add all constructors after this
            ] + self.constructor_elems
        ]
        write_cpp_code(f, constructor, CPP_INDENT_LVL)
        f.write("\n")

        # Process Message
        process_msg_func = [
            f"bool processMessage(int clientId, const {self.comm_msg_type_str} &msg) override", [
                # clientId isn't used by topic handler (it's broadcast)
                "(void) clientId;",

                f"switch (msg.{self.comm_msg_submsg_oneof.name}_case())", [
                    # All previous switch case elements will appear before this
                    self.process_swich_case_elems,
                    {
                        # Return failure on any other unhandled messages
                        "default": [
                            "return false;"
                        ]
                    }
                ],
            ]
        ]
        write_cpp_code(f, process_msg_func, CPP_INDENT_LVL)
        f.write("\n")

        # Private variable initialization
        f.write("private:\n")
        private_ctx = [
            "rclcpp::Logger logger_;",
        ]
        write_cpp_code(f, private_ctx, CPP_INDENT_LVL)

        # Add all elements in private variables
        for entry in self.private_context_entries:
            f.write("\n")
            write_cpp_code(f, entry, CPP_INDENT_LVL)

        f.write("};\n")
        f.write("\n")

        # Add in the actual target string enum map definition, as this must be done outside of the function
        f.write("const std::unordered_map<std::string, TopicMessageHandler::TargetType> TopicMessageHandler::targetStrMap = {\n")
        target_enum_map_def = []
        for target in self.targets:
            target_enum_map_def.append("{\"%s\", TopicMessageHandler::%s}," % (target, self._create_target_enum_name(target)),)
        write_cpp_code(f, target_enum_map_def, CPP_INDENT_LVL)
        f.write("};\n")
        f.write("\n")

        # Add the public function to create the class
        write_cpp_code(f, [
            "std::shared_ptr<MessageHandlerItf> createTopicHandler(rclcpp::Node& node, RosProtobufBridge &bridge, std::string target)", [
                "return std::dynamic_pointer_cast<MessageHandlerItf>(std::make_shared<TopicMessageHandler>(node, bridge, target));"
            ]
        ])

    def add_publisher(self, msg_field: 'ProtobufFieldDecl', topic_info: 'FirmwareTopicDecl',
                      msg_conversion: 'CppMsgConversion'):
        # Mark that we're using the proto_to_ros function
        msg_conversion.set_proto_to_ros_used()

        # The topic enum should have a unique name as a valid c identifier, use that for topic names
        topic_identifier_str = msg_field.name.lower()
        pub_func_name = f"publish_{topic_identifier_str}"
        pub_var_name = f"pub_{topic_identifier_str}_"

        # Add the publisher variable and the publish function
        # This must be separate from the case statement, as you can't open a case with a variable declaration
        self.private_context_entries.append([
            f"rclcpp::Publisher<{msg_conversion.cpp_ros_type_str}>::SharedPtr {pub_var_name};",
            f"bool {pub_func_name}(const {msg_conversion.cpp_protobuf_type_str}& proto_msg)", [
                f"if ({pub_var_name} == nullptr)", [
                    "return false;"
                ],
                f"{msg_conversion.cpp_ros_type_str} ros_msg;",
                f"{msg_conversion.proto_to_ros_func_name}(ros_msg, proto_msg);",
                f"{pub_var_name}->publish(ros_msg);",
                "return true;"
            ]
        ])

        # Add the value to the switch case
        self.process_swich_case_elems[f"case {self.comm_msg_type_str}::{field_to_cpp_oneof_case(msg_field.name)}"] = [
            f"return {pub_func_name}(msg.{msg_field.name}());"
        ]

        # Create the publisher init ilne
        init_statement = f"{pub_var_name} = node.create_publisher<{msg_conversion.cpp_ros_type_str}>(\"{topic_info.name}\", {topic_info.qos.value});"

        # Wrap publisher init in an if statement if not all targets use this publisher
        if len(topic_info.publishers) != len(self.targets):
            target_check = []
            for publisher in topic_info.publishers:
                target_check.append("target == {}".format(self._create_target_enum_name(publisher)))
            target_check = " || ".join(target_check)

            init_block = [
                f"if ({target_check})", [
                    init_statement
                ]
            ]
        else:
            # All publishers used, just wrap directly
            init_block = [init_statement]

        # Add init block to the constructor
        self.constructor_elems += init_block

    def add_subscriber(self, msg_field: 'ProtobufFieldDecl', topic_info: 'FirmwareTopicDecl',
                      msg_conversion: 'CppMsgConversion'):
        # Mark that we're using the ros_to_proto function
        msg_conversion.set_ros_to_proto_used()

        # The topic enum should have a unique name as a valid c identifier, use that for topic names
        sub_func_name = f"callback_{msg_field.name}"
        sub_var_name = f"sub_{msg_field.name}_"

        # Define the subscriber variable, and its respective callback to the private context
        self.private_context_entries.append([
            f"rclcpp::Subscription<{msg_conversion.cpp_ros_type_str}>::SharedPtr {sub_var_name};",
            f"void {sub_func_name}(const {msg_conversion.cpp_ros_type_str}::SharedPtr ros_msg)", [
                "try", [
                    f"{self.comm_msg_type_str} proto_msg;",
                    f"proto_msg.clear_{self.comm_msg_ack_field.name}();",
                    f"{msg_conversion.ros_to_proto_func_name}(*ros_msg, *proto_msg.mutable_{msg_field.name}());",
                    "sendResponse(0, proto_msg);"
                ],
                "catch (MsgConversionError &e)", [
                    f"RCLCPP_WARN(logger_, \"Unable to encode message on topic '%s' - %s\", \"{topic_info.name}\", e.what());"
                ]
            ]
        ])

        # Create the init statement
        init_statement = f"{sub_var_name} = node.create_subscription<{msg_conversion.cpp_ros_type_str}>(\"{topic_info.name}\"," \
                         f"{topic_info.qos.value}, std::bind(&TopicMessageHandler::{sub_func_name}, this, std::placeholders::_1));"

        # Wrap init statement in an if statement if not all targets use this subscriber
        if len(topic_info.subscribers) != len(self.targets):
            target_check = []
            for subscriber in topic_info.subscribers:
                target_check.append("target == {}".format(self._create_target_enum_name(subscriber)))
            target_check = " || ".join(target_check)

            init_block = [
                f"if ({target_check})", [
                    init_statement
                ]
            ]
        else:
            # All subscribers used, just wrap directly
            init_block = [init_statement]

        # Add init block to the constructor
        self.constructor_elems += init_block

class CppParamHandler:
    """Class to generate code for creating the ROS bridge for the protobuf messages.

    This is responsible for generating the ParamMessageHandler class.
    This creates all of the required parameters on the node, and forwards these values to protobuf
    """
    def __init__(self, comm_msg: 'ProtobufMessageType', submsg_oneof: 'ProtobufOneofType',
                 ack_field: 'ProtobufFieldDecl'):

        # Extract package name from comm_msg
        parent = comm_msg.parent_type
        while not isinstance(parent, ProtobufTopScope):
            parent = parent.parent_type
        self.package = parent.package

        # Save the comm_msg C++ type name
        self.comm_msg_type_str = "::".join(comm_msg.get_path())
        assert submsg_oneof.parent_type == comm_msg, "Expected oneof to be part of comm_msg"
        self.comm_msg_submsg_oneof = submsg_oneof
        assert ack_field in comm_msg.fields, "Expected ack to be child of comm_msg"
        self.comm_msg_ack_field = ack_field

        self.constructor_elems: 'list[str | list]' = []              # C++ code to insert into constructor
        self.process_swich_case_elems: 'dict[str, list]' = {}        # C++ code to insert into switch case for processing messages
        self.private_context_entries: 'list[list[str | list]]' = []  # List of C++ code lists to be put in the private ctx
        self.parameters_initialized = False                          # Set if the single time parameter code has been added to the file

    def writeto(self, f: 'io.TextIOBase'):
        f.write("class ParamMessageHandler: public MessageHandlerItf {\n")
        f.write("public:\n")

        # Constructor
        constructor = [
                "ParamMessageHandler(rclcpp::Node& node, RosProtobufBridge &bridge): MessageHandlerItf(bridge), logger_(node.get_logger())",
        ]
        if self.parameters_initialized:
            constructor.append(self.constructor_elems)
        else:
            constructor.append(["(void) node;"])
        write_cpp_code(f, constructor, CPP_INDENT_LVL)
        f.write("\n")

        # Process Message
        process_msg_func = [
            f"bool processMessage(int clientId, const {self.comm_msg_type_str} &msg) override"
        ]
        if self.parameters_initialized:
            process_msg_func.append([
                f"if (msg.{self.comm_msg_submsg_oneof.name}_case() == {self.comm_msg_type_str}::{field_to_cpp_oneof_case(self.comm_msg_param_req_field.name)})", [
                    f"processParamReq(clientId, msg);",
                    "return true;"
                ],
                "else", [
                    "return false;"
                ]
            ])
        else:
            process_msg_func.append([
                # Empty stub for message process if no parameters are used
                "(void) msg;",
                "(void) clientId;",
                "return false;"
            ])

        write_cpp_code(f, process_msg_func, CPP_INDENT_LVL)
        f.write("\n")

        # Private variable initialization
        f.write("private:\n")
        private_ctx = [
            "rclcpp::Logger logger_;",
        ]
        write_cpp_code(f, private_ctx, CPP_INDENT_LVL)

        # Add all elements in private variables
        for entry in self.private_context_entries:
            f.write("\n")
            write_cpp_code(f, entry, CPP_INDENT_LVL)

        f.write("};\n")
        f.write("\n")

        # Add the public function to create the class
        write_cpp_code(f, [
            "std::shared_ptr<MessageHandlerItf> createParamHandler(rclcpp::Node& node, RosProtobufBridge &bridge)", [
                "return std::dynamic_pointer_cast<MessageHandlerItf>(std::make_shared<ParamMessageHandler>(node, bridge));"
            ]
        ])

    def enable_parameter_support(self, comm_msg_param_update_field: 'ProtobufFieldDecl', comm_msg_param_req_field: 'ProtobufFieldDecl',
                                 param_update_oneof: 'ProtobufOneofType'):
        """Add support for parameters by adding in required objects into C++ code
        This must be called before add_parameter
        """
        # Save the fields so add_parameter knows what to call
        self.comm_msg_param_update_field = comm_msg_param_update_field
        self.comm_msg_param_req_field = comm_msg_param_req_field
        self.param_update_oneof = param_update_oneof

        # Declare parameter private ctx to keep all parameter private context together
        self.parameter_private_ctx_entries = [
            "std::mutex param_mutex_;",
            "std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;"
        ]
        self.private_context_entries.append(self.parameter_private_ctx_entries)

        # Add required initialization code for parameters
        self.constructor_elems.append("param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(&node);")

        # Add in processParamReq function
        self.process_param_req_case = {}
        self.private_context_entries.append([
            f"void processParamReq(int clientId, const {self.comm_msg_type_str} &req_msg)", [
                # Create the response message, coping the ack and setting param update type
                f"{self.comm_msg_type_str} resp_msg;",
                f"resp_msg.set_{self.comm_msg_ack_field.name}(req_msg.{self.comm_msg_ack_field.name}());",
                f"auto param_update = resp_msg.mutable_{comm_msg_param_update_field.name}();",

                # Read and transmit the parameter under lock to prevent race conditions
                "std::scoped_lock lock{param_mutex_};",

                # Copy in the required parameter depending on requested enum
                # These will be populated in each add_parameter call
                f"switch (req_msg.{comm_msg_param_req_field.name}())", [
                    self.process_param_req_case,
                    # Default case, print warning and quit
                    {
                        "default": [
                            f'RCLCPP_WARN(logger_, "Client %d sent unexpected parameter request for param %d", clientId, req_msg.{comm_msg_param_req_field.name}());',
                            "return;"
                        ]
                    }
                ],

                # Upon successful loading of parameter, transmit response
                "sendResponse(clientId, resp_msg);"
            ]
        ])

        self.parameters_initialized = True

    def add_parameter(self, param_info: 'FirmwareParamDecl', param_field_decl: 'ProtobufFieldDecl',
                      param_enum_decl: 'ProtobufEnumDecl'):
        assert self.parameters_initialized, "Must call enable_parameter_support before add_parameter"

        cb_handle_name = f"param_{param_info.name}_cb_handle_"
        cached_var_name = f"param_{param_info.name}_val_"
        cached_requested_name = f"param_{param_info.name}_requested_"

        conversion = CppParameterConversion(param_info, cached_var_name, param_field_decl)

        # Add private variables for parameter
        self.parameter_private_ctx_entries.append(f"rclcpp::ParameterCallbackHandle::SharedPtr {cb_handle_name};")
        self.parameter_private_ctx_entries.append(f"{param_info.cpp_type[0]} {cached_var_name};")
        self.parameter_private_ctx_entries.append(f"bool {cached_requested_name} = false;")

        # Add required elements to the constructor
        param_constructor = [
            # Declare the parameter
            f"node.declare_parameter({escape_c_string(param_info.name)}, rclcpp::{param_info.param_type});",

            # Add callback for when parameter is updated, so changes can be sent over CAN
            f"{cb_handle_name} = param_subscriber_->add_parameter_callback({escape_c_string(param_info.name)}, [this](const rclcpp::Parameter & p)", [
                # Create message to hold the update
                f"{self.comm_msg_type_str} update_msg;",
                f"update_msg.clear_{self.comm_msg_ack_field.name}();",
                f"auto param_update = update_msg.mutable_{self.comm_msg_param_update_field.name}();",

                # Grab the update, and send it, all under lock
                # This prevents weird race conditions if another thread tries to get the param, and transmit it
                "std::scoped_lock lock{param_mutex_};",
                # Save the updated variable value in the cache
                f"{cached_var_name} = p.{param_info.cpp_type[1]}();",

                # Create an update message and broadcast it to all clients
                f"if ({cached_requested_name})",
                    conversion.cached_to_proto_conversion("param_update") + [
                    "sendResponse(0, update_msg);"
                ]
            ], ");",
            f"{cached_var_name} = node.get_parameter({escape_c_string(param_info.name)}).{param_info.cpp_type[1]}();"
        ]
        self.constructor_elems += param_constructor

        # Add case statement to handle read parameter requests from microcontrollers
        enum_type = "::".join(self.comm_msg_param_req_field.field_type.parent_type.get_path())
        self.process_param_req_case[f"case {enum_type}::{param_enum_decl.name}"] = [
            conversion.cached_to_proto_conversion("param_update"),
            # Mark that the parameter is requested, and we should now send out updates for it
            f"{cached_requested_name} = true;",
            "break;"
        ]

#endregion

#region Protobuf Conversion

def camel_to_snake_case(s: str):
    out_str = ""
    if "_" in s:
        print("Camel case string '{}' contains underscore, this may not convert properly".format(s), file=sys.stderr)
    for i, c in enumerate(s):
        if c.isupper():
            if i != 0:
                out_str += "_"
            out_str += c.lower()
        else:
            out_str += c
    return out_str

field_type_lookup = {
    # Scalar Type, Field bit size (or None if not integer), ROS field CPP type, protobuf field CPP type
    "boolean": (ProtobufScalarType.type_bool, None, "bool", "bool"),
    "octet": (ProtobufScalarType.type_int32, 8, "int8_t", "int32_t"),       # octet translates to ros byte, which is a depreciated alias for int8 in ros1
    "float": (ProtobufScalarType.type_float, None, "float", "float"),
    "double": (ProtobufScalarType.type_double, None, "double", "double"),
    "int8": (ProtobufScalarType.type_sint32, 8, "int8_t", "int32_t"),
    "uint8": (ProtobufScalarType.type_uint32, 8, "uint8_t", "uint32_t"),
    "int16": (ProtobufScalarType.type_sint32, 16, "int16_t", "int32_t"),
    "uint16": (ProtobufScalarType.type_uint32, 16, "uint16_t", "uint32_t"),
    "int32": (ProtobufScalarType.type_sint32, 32, "int32_t", "int32_t"),
    "uint32": (ProtobufScalarType.type_uint32, 32, "uint32_t", "uint32_t"),
    "int64": (ProtobufScalarType.type_sint64, 64, "int64_t", "int64_t"),
    "uint64": (ProtobufScalarType.type_uint64, 64, "uint64_t", "uint64_t"),
    "short": (ProtobufScalarType.type_sint32, 16, "int16_t", "int32_t"),
    "unsigned short": (ProtobufScalarType.type_uint32, 16, "uint16_t", "uint32_t"),
    "long": (ProtobufScalarType.type_sint32, 32, "int32_t", "int32_t"),
    "unsigned long": (ProtobufScalarType.type_uint32, 32, "uint32_t", "uint32_t"),
    "long long": (ProtobufScalarType.type_sint64, 64, "int64_t", "int64_t"),
    "unsigned long long": (ProtobufScalarType.type_uint64, 64, "uint64_t", "uint64_t"),
}

class ProtobufRosConversion:
    """Class to organize the various files required to dynamically create the ROS to Protobuf bridge.

    This generates the Protobuf proto file, the nanopb options file, and the C++ file to create the ROS required
    ROS publishers/subscribers.

    After initializing with the constant mapping in the YAML file, each topic entry in the yaml file can then
    loaded into this class.

    The conversion is performed by importing and introspecting the ROS message during runtime. This requires the script
    be running with the ROS messages inside your python path. This simplifies discovery, but limits the script to
    running on machines with a functional ROS install.
    """
    DEFAULT_FILEHASH = 0  # Filehash to set when trying to compute the current file hash
    def __init__(self, const_map: 'FirmwareConstantMapping', targets: 'list[str]', cpp_header_path: str):
        self.const_map = const_map
        self.targets = targets
        self.top_proto = ProtobufTopScope("titan_pb")

        # Add shared comm message
        self.comm_msg = ProtobufMessageType("comm_msg", self.top_proto)
        self.comm_msg_oneof = ProtobufOneofType("msg", self.comm_msg)
        # Add the special connect message. This MUST not change to preserve compataibility across connect messages
        connect_field = ProtobufFieldDecl(ProtobufScalarType.type_fixed32, "connect_ver", 1)
        self.comm_msg_oneof.add_field(connect_field)
        # Add types to comm_msg. This ID and types MUST not change to allow compatability across connect messages
        ack_field = ProtobufFieldDecl(ProtobufScalarType.type_uint32, "ack", 2)
        self.comm_msg.add_field(ack_field)

        # Create the C++ Topic Handler File
        self.cpp_topic_file = CppFile("RosProtobufBridge")
        self.cpp_topic_handler = CppTopicHandler(self.comm_msg, self.comm_msg_oneof, connect_field, ack_field, targets)
        self.cpp_topic_file.add_include(cpp_header_path, False)
        self.cpp_topic_file.add_include("protobridge.pb.h", False)
        self.cpp_topic_file.add_include("rclcpp/rclcpp.hpp", True)
        self.cpp_topic_file.topic_handler = self.cpp_topic_handler

        # Create the C++ Parameter Handler File
        self.cpp_param_file = CppFile("RosProtobufBridge")
        self.cpp_param_handler = CppParamHandler(self.comm_msg, self.comm_msg_oneof, ack_field)
        self.cpp_param_file.add_include(cpp_header_path, False)
        self.cpp_param_file.add_include("protobridge.pb.h", False)
        self.cpp_param_file.add_include("rclcpp/rclcpp.hpp", True)
        self.cpp_param_file.topic_handler = self.cpp_param_handler

        # Add filehash constant for connect
        # This will first be stored as an extension in the file
        # However, this can only be decoded as a descriptor by the host system
        # Nanopb doesn't copy arbitrary option values into the output file
        self.top_proto.add_import("google/protobuf/descriptor.proto")
        self.file_extensions = ProtobufExtendType("google.protobuf.MessageOptions", self.top_proto)
        protocol_version_field = ProtobufFieldDecl(ProtobufScalarType.type_fixed32, "protocol_version", 1010)
        self.file_extensions.add_field(protocol_version_field)
        self.protocol_version_option = ProtobufOption(f"({protocol_version_field.name})", self.DEFAULT_FILEHASH)
        self.comm_msg.add_option(self.protocol_version_option)
        # This sets the comm_msg msg id to the connect message
        # It "technically" isn't designed for this, but we aren't using message ids since we only have one message
        # And in a sense, it is a unique ID based on how the file was generated to prevent collisions
        self.nanopb_msgid_option = ProtobufOption("(nanopb).msgid", self.DEFAULT_FILEHASH)
        self.comm_msg.add_option(self.nanopb_msgid_option)

        # Define fields to be filled in by ros messages
        # If you need to add extra fields or enums, increment these values here
        self.next_msg_type_idx = 3  # The next message ID available to be placed in comm_msg_oneof
        self.ros_msg_lookup = {}    # ROS message (msg_namespaced) to existing ProtobufMessageType for that message
        self.msg_packages = []      # List of all ROS message packages required by the C++ source file
        self.parameters_added = False

    def _compute_filehash(self):
        # Reset the filehash to fixed value
        self.nanopb_msgid_option.value = self.DEFAULT_FILEHASH
        self.protocol_version_option.value = self.DEFAULT_FILEHASH

        # Generate the protobuf and options file into buffer
        buf = io.StringIO()
        # Use null character to separate, as this shouldn't be present in the text file
        self.top_proto.write_proto(buf)
        buf.write('\0')
        self.top_proto.write_nanopb_options(buf)
        buf.write('\0')

        # Compute sha1 hash of buffer, using the fixed filehash value
        # This ensures that any changes to the protobuf file that either the agent or client is running on will change
        # the connect hash, preventing weird undefined behavior from occurring
        sha1hash = hashlib.sha1()
        buf.seek(0)
        sha1hash.update(buf.read().encode())

        # Set the option values to the newly computed filehash
        hash_value = int.from_bytes(sha1hash.digest()[:4], 'big')
        self.nanopb_msgid_option.value = hash_value
        self.protocol_version_option.value = hash_value

    def write_proto(self, f: 'io.TextIOBase'):
        self._compute_filehash()
        self.top_proto.write_proto(f)

    def write_nanopb_options(self, f: 'io.TextIOBase'):
        self._compute_filehash()
        self.top_proto.write_nanopb_options(f)

    def write_cpp_artifacts(self, cmake_filename, target_name):
        cpp_topic_filename = "TopicMessageHandler.cpp"
        cpp_param_filename = "ParamMessageHandler.cpp"

        artifact_dir = os.path.dirname(os.path.realpath(cmake_filename))

        with open(cmake_filename, 'w') as f:
            for package in self.msg_packages:
                f.write("find_package({} REQUIRED)\n".format(package))
            f.write("ament_target_dependencies({} {})\n".format(target_name, " ".join(self.msg_packages)))
            f.write("target_sources(%s PUBLIC\n" % target_name)
            f.write("    ${CMAKE_CURRENT_LIST_DIR}/%s\n" % cpp_topic_filename)
            f.write("    ${CMAKE_CURRENT_LIST_DIR}/%s\n" % cpp_param_filename)
            f.write(")\n")

        with open(os.path.join(artifact_dir, cpp_topic_filename), 'w') as f:
            self.cpp_topic_file.writeto(f)

        with open(os.path.join(artifact_dir, cpp_param_filename), 'w') as f:
            self.cpp_param_file.writeto(f)

    def add_topic(self, topic: 'FirmwareTopicDecl'):
        """Adds a new topic to the conversion."""

        # Convert the ros message to protobuf, and get the object type (so we can make a field with it)
        protobuf_msg_obj, cpp_conversion = self._convert_message(tuple(topic.type.split("/")))

        topic_field_name = topic.name.lower().replace('/', '_')

        # Add a new oneof for this topic in the protobuf comm_msg
        msg_field_decl = ProtobufFieldDecl(protobuf_msg_obj, topic_field_name, self.next_msg_type_idx)
        self.comm_msg_oneof.add_field(msg_field_decl)
        self.next_msg_type_idx += 1

        if len(topic.publishers) > 0:
            self.cpp_topic_handler.add_publisher(msg_field_decl, topic, cpp_conversion)
        if len(topic.subscribers) > 0:
            self.cpp_topic_handler.add_subscriber(msg_field_decl, topic, cpp_conversion)

    def add_parameter(self, param: 'FirmwareParamDecl'):
        if not self.parameters_added:
            # Add required objects if a parameter hasn't been added yet
            self.parameters_added = True

            # Create the update message
            self.param_update_msg = ProtobufMessageType("param_update_msg", self.top_proto)
            self.param_update_msg_oneof = ProtobufOneofType("param", self.param_update_msg)
            self.param_update_next_idx = 1
            self.param_update_array_msg_lookup = {}

            # Add update message to the comm oneof
            comm_msg_param_update_field = ProtobufFieldDecl(self.param_update_msg, "param_update", self.next_msg_type_idx)
            self.comm_msg_oneof.add_field(comm_msg_param_update_field)
            self.next_msg_type_idx += 1

            # Create the request to the comm oneof
            self.param_req_enum = ProtobufEnumType("param_name_enum", self.comm_msg)
            update_enum_entry = ProtobufFieldDecl(self.param_req_enum, "param_request", self.next_msg_type_idx)
            self.comm_msg_oneof.add_field(update_enum_entry)
            self.next_msg_type_idx += 1

            # Enable parameter support in C++, passing the objects we just made
            self.cpp_param_handler.enable_parameter_support(comm_msg_param_update_field, update_enum_entry, self.param_update_msg_oneof)

        # Add parameter to protobuf objects
        # Get the type for the field, depending on if its repeated or not
        param_field_opts = []
        if param.protobuf_type[1] == ProtobufFieldCardinality.REPEATED:
            param_field_type = self._create_param_array_submsg(param.protobuf_type[0])
        else:
            param_field_type = param.protobuf_type[0]
            # If string or bytes field, mark as pointer
            if param_field_type == ProtobufScalarType.type_string or param_field_type == ProtobufScalarType.type_bytes:
                param_field_opts = [ProtobufOption("(nanopb).type", "FT_POINTER")]

        # Add parameter into the parameter message oneof
        param_field_decl = ProtobufFieldDecl(param_field_type, param.name, self.param_update_next_idx, options = param_field_opts)
        self.param_update_msg_oneof.add_field(param_field_decl)

        # Add the parameter name to the enum so it can be requested
        # Enum value can be equal to the field # - 1 (since enums must have a 0 value)
        param_enum_decl = ProtobufEnumDecl("PARAM_" + param.name.upper(), self.param_update_next_idx - 1)
        self.param_req_enum.add_value(param_enum_decl)
        self.param_update_next_idx += 1

        self.cpp_param_handler.add_parameter(param, param_field_decl, param_enum_decl)

    def _create_param_array_submsg(self, base_type: 'ProtobufScalarType'):
        # Creates submessage wrapper for repeated parameter types, since oneof doesn't support repeated directly
        if base_type in self.param_update_array_msg_lookup:
            # Return existing message if it already exists
            return self.param_update_array_msg_lookup[base_type]

        # If not, create it
        array_submsg = ProtobufMessageType("param_{}_array".format(base_type.name), self.param_update_msg)
        opts = [ProtobufOption("(nanopb).type", "FT_POINTER")]
        array_submsg.add_field(ProtobufFieldDecl(base_type, "entry", 1, ProtobufFieldCardinality.REPEATED, opts))
        self.param_update_array_msg_lookup[base_type] = array_submsg

        return array_submsg

    def _convert_message(self, msg_namespaced: 'tuple[str]') -> 'tuple[ProtobufMessageType, CppMsgConversion]':
        """Converts a given ROS message namespaced to a ProtobufMessageType, and adds it to the top scope under
        its package namespace.

        Additionally generates the required C++ code to perform the message conversion to ROS.
        """
        # Make sure namespace is proper
        assert type(msg_namespaced) == tuple and all(map(lambda x: type(x) == str, msg_namespaced))

        ########################################
        # Try to import message from ROS
        ########################################

        # Ensure that our message namespace is how we expect
        if len(msg_namespaced) != 3 and msg_namespaced[1] != "msg":
            raise RuntimeError("Invalid Message Name: '{}'".format('/'.join(msg_namespaced)))

        if msg_namespaced in self.ros_msg_lookup:
            assert self.ros_msg_lookup[msg_namespaced] is not None, "Recursion loop detected, this code does not support cyclic message references"
            return self.ros_msg_lookup[msg_namespaced]
        self.ros_msg_lookup[msg_namespaced] = None  # Later set once message is fully decoded

        # Now try to load in our message
        assert not any(map(lambda x: '.' in x, msg_namespaced)), "Message type cannot contain periods"
        ros_msg = None
        try:
            msg_module = importlib.import_module('.'.join(msg_namespaced[:-1]))
            ros_msg = getattr(msg_module, msg_namespaced[-1])
        except:
            pass
        if ros_msg is None:
            raise RuntimeError("Could not locate message: '{}'".format('/'.join(msg_namespaced)))

        msg_fields = {}
        if len(ros_msg.SLOT_TYPES) != len(ros_msg.__slots__):
            raise RuntimeError("Number of slots and slot types don't line up")

        # Sanity check to make sure we actually grab the official names supported
        # This is since ROS makes it a nightmare to convert the underlying SLOT_TYPE to the actual message name
        msg_remaining_fields = list(ros_msg.get_fields_and_field_types().keys())

        for i, field_name in enumerate(ros_msg.__slots__):
            field_type = ros_msg.SLOT_TYPES[i]

            # Remove leading underscore from slot name
            if field_name[0] != '_':
                raise RuntimeError("Invalid slot name: '{}'".format(field_name))
            field_name = field_name[1:]

            # Sanity check to make sure its in the "official" field list
            if field_name not in msg_remaining_fields:
                raise RuntimeError("Could not find field name: '{}' in message but was in __slots__".format(field_name))
            msg_remaining_fields.remove(field_name)

            # Now append it to our message fields
            assert field_name not in msg_fields
            msg_fields[field_name] = field_type

        if len(msg_remaining_fields) > 0:
            raise RuntimeError("Extra fields declared but not listed in __slots__: {}".format(', '.join(msg_remaining_fields)))

        # Read the constants
        msg_metaclass = ros_msg.__class__
        # They use name mangling to protect the __constants variable, but its the easiest way to get the constants
        # So we're going to undo the name mangling to retrieve them
        extracted_consts = getattr(msg_metaclass, '_{}__constants'.format(msg_metaclass.__name__))
        # Only copy over int32 constants, since that's the only values enums support in protobuf
        msg_constants = {}
        for const_name, const_val in extracted_consts.items():
            if type(const_name) is str :
                if type(const_val) is int and const_val >= -(2**32) and const_val < 2**32:
                    msg_constants[const_name] = const_val
                if type(const_val) is bytes and len(const_val) == 1:
                    msg_constants[const_name] = const_val[0]

        # Map constants to its corresponding field as configured in YAML
        msg_const_map = self.const_map.map_constants('/'.join(msg_namespaced), msg_fields.keys(), msg_constants)
        if msg_const_map is None:
            # If None was returned, that means the YAML didn't have an entry for this message
            # If we have constants, warn
            # This is different from an empty dict, which means that there was an entry, but it decided to ignore all of them
            msg_const_map = {}
            if len(msg_constants) > 0:
                print("Message '{}' has constants, but there is no constant map specified in the configuration yaml."
                      .format('/'.join(msg_namespaced)), file=sys.stderr)

        ########################################
        # Create C++/Protobuf msg containers
        ########################################

        # We're now good to begin creating the Protobuf Message
        package_name = msg_namespaced[0]
        msg_name = msg_namespaced[2]

        # Workaround in the event a package or message uses a reserved keyword as the name
        if package_name in reserved_keywords:
            package_name += "_"
        if msg_name in reserved_keywords:
            msg_name += "_"

        # Nest all messages under their respective package name
        package_ns_msg = next((child for child in self.top_proto.child_types if child.name == package_name), None)
        if package_ns_msg is None:
            package_ns_msg = ProtobufMessageType(package_name, self.top_proto)

        # Create the message type, and begin populating the fields
        protobuf_msg = ProtobufMessageType(msg_name, package_ns_msg)

        # Create the conversion for this specific message
        cpp_ros_include = "{}/msg/{}.hpp".format(package_name, camel_to_snake_case(msg_name))
        cpp_msg_conversion = CppMsgConversion(protobuf_msg, msg_namespaced)

        ########################################
        # Enumerate through fields
        ########################################

        field_idx = 1
        for field_name, field_type in msg_fields.items():
            field_cardinality = ProtobufFieldCardinality.NONE
            protobuf_type = None
            field_options = []

            # Workaround in the event a field uses a reserved keyword as the name
            if field_name in reserved_keywords:
                proto_field_name = "_" + field_name
            else:
                proto_field_name = field_name

            ####################
            # Enum specific handling
            #
            is_enum = field_name in msg_const_map
            if is_enum:
                enum_type_decl = ProtobufEnumType(field_name + "_enum", protobuf_msg)
                zero_value_added = False
                max_enum_width = 1
                for enum_name in msg_const_map[field_name]:
                    enum_value = msg_constants[enum_name]
                    if type(enum_value) != int or enum_value < -(2 << 31) or enum_value >= (2 << 31):
                        raise RuntimeError("Cannot convert constant '{}' in message '{}' to protobuf enum."
                                           .format(enum_name, '/'.join(msg_namespaced)))
                    if enum_value == 0:
                        zero_value_added = True
                        enum_width = 1
                    else:
                        enum_width = int(math.log2(enum_value))+1
                    if enum_width > max_enum_width:
                        max_enum_width = enum_width
                    enum_type_decl.add_value(ProtobufEnumDecl(enum_name, enum_value))
                if not zero_value_added:
                    enum_type_decl.add_value(ProtobufEnumDecl("_{}_RESERVED_ZERO_VALUE".format(field_name.upper()), 0))


            ####################
            # Rosidl Type Decoding
            #
            if type(field_type) == rosidl_parser.definition.BasicType:
                lookup_data = field_type_lookup[field_type.typename]
                if is_enum and lookup_data[1] is not None:
                    # Only convert to enum if it's an integer (bit width in type lookup is not None)
                    # Make sure that all enum values can fit into the ROS integer
                    if max_enum_width > lookup_data[1]:
                        raise RuntimeError("Cannot fit all enum values into field '{}' (width {}) of '{}'"
                                           .format(field_name, lookup_data[1], '/'.join(msg_namespaced)))
                    protobuf_type = enum_type_decl
                    cpp_msg_conversion.gen_cpp_enum_type(field_name, proto_field_name, enum_type_decl)
                else:
                    protobuf_type = lookup_data[0]
                    if lookup_data[1] is not None:
                        field_options.append(ProtobufOption("(nanopb).int_size", lookup_data[1]))
                    cpp_msg_conversion.gen_cpp_basic_type(field_name, lookup_data[2], proto_field_name, lookup_data[3])
            elif type(field_type) == rosidl_parser.definition.NamespacedType:
                protobuf_type, field_msg_conversion = self._convert_message(field_type.namespaced_name())
                cpp_msg_conversion.gen_cpp_child_type(field_name, proto_field_name, field_msg_conversion)
            elif type(field_type) == rosidl_parser.definition.BoundedString:
                protobuf_type = ProtobufScalarType.type_string
                field_options.append(ProtobufOption("(nanopb).max_size", field_type.maximum_size))
                cpp_msg_conversion.gen_cpp_bounded_string_type(field_name, proto_field_name, field_type.maximum_size)
            elif type(field_type) == rosidl_parser.definition.UnboundedString:
                protobuf_type = ProtobufScalarType.type_string
                field_options.append(ProtobufOption("(nanopb).type", "FT_POINTER"))
                cpp_msg_conversion.gen_cpp_basic_type(field_name, "std::string", proto_field_name, "std::string")

            ####################
            # Rosidl Array Type Decoding
            #
            elif type(field_type) == rosidl_parser.definition.Array or \
                    type(field_type) == rosidl_parser.definition.BoundedSequence or \
                    type(field_type) == rosidl_parser.definition.UnboundedSequence:

                nested_type = field_type.value_type

                # If its an array of 8-bit values, convert to a byte array
                should_change_to_bytes = False
                if type(nested_type) == rosidl_parser.definition.BasicType and not is_enum:
                    if nested_type.typename == "int8":
                        should_change_to_bytes = True
                    elif nested_type.typename == "uint8":
                        should_change_to_bytes = True
                    elif nested_type.typename == "octet":
                        should_change_to_bytes = True

                if should_change_to_bytes:
                    protobuf_type = ProtobufScalarType.type_bytes
                    if type(field_type) == rosidl_parser.definition.Array:
                        field_options.append(ProtobufOption("(nanopb).max_size", field_type.size))
                        field_options.append(ProtobufOption("(nanopb).fixed_length", True))
                        field_max_size = field_type.size
                        field_size_fixed = True
                    if type(field_type) == rosidl_parser.definition.BoundedSequence:
                        field_options.append(ProtobufOption("(nanopb).max_size", field_type.maximum_size))
                        field_max_size = field_type.maximum_size
                        field_size_fixed = False
                    if type(field_type) == rosidl_parser.definition.UnboundedSequence:
                        field_options.append(ProtobufOption("(nanopb).type", "FT_POINTER"))
                        field_max_size = None
                        field_size_fixed = False
                    cpp_msg_conversion.gen_cpp_array_bytes_type(field_name, proto_field_name, field_max_size, field_size_fixed)
                else:
                    has_set_pointer_type = False

                    field_cardinality = ProtobufFieldCardinality.REPEATED
                    if type(field_type) == rosidl_parser.definition.Array:
                        field_options.append(ProtobufOption("(nanopb).max_count", field_type.size))
                        field_options.append(ProtobufOption("(nanopb).fixed_count", True))
                        field_max_size = field_type.size
                        field_size_fixed = True
                    if type(field_type) == rosidl_parser.definition.BoundedSequence:
                        field_options.append(ProtobufOption("(nanopb).max_count", field_type.maximum_size))
                        field_max_size = field_type.maximum_size
                        field_size_fixed = False
                    if type(field_type) == rosidl_parser.definition.UnboundedSequence and not has_set_pointer_type:
                        field_options.append(ProtobufOption("(nanopb).type", "FT_POINTER"))
                        field_max_size = None
                        field_size_fixed = False
                        has_set_pointer_type = True

                    if type(nested_type) == rosidl_parser.definition.BasicType:
                        lookup_data = field_type_lookup[nested_type.typename]
                        if is_enum and lookup_data[1] is not None:
                            # Only convert field to enum if field is an integer (bit width in type lookup is not None)
                            # Make sure that all enum values can fit into the ROS integer
                            if max_enum_width > lookup_data[1]:
                                raise RuntimeError("Cannot fit all enum values into field '{}' (width {}) of '{}'"
                                                .format(field_name, lookup_data[1], '/'.join(msg_namespaced)))
                            protobuf_type = enum_type_decl

                            cpp_msg_conversion.gen_cpp_array_enum_type(field_name, proto_field_name, enum_type_decl,
                                                                       field_max_size, field_size_fixed)
                        else:
                            protobuf_type = lookup_data[0]
                            if lookup_data[1] is not None:
                                field_options.append(ProtobufOption("(nanopb).int_size", lookup_data[1]))
                            cpp_msg_conversion.gen_cpp_array_basic_type(field_name, lookup_data[2], proto_field_name,
                                                                        lookup_data[3], field_max_size, field_size_fixed)
                    elif type(nested_type) == rosidl_parser.definition.NamespacedType:
                        protobuf_type, field_msg_conversion = self._convert_message(nested_type.namespaced_name())
                        cpp_msg_conversion.gen_cpp_array_child_type(field_name, proto_field_name, field_msg_conversion,
                                                                    field_max_size, field_size_fixed)
                    elif type(nested_type) == rosidl_parser.definition.BoundedString:
                        protobuf_type = ProtobufScalarType.type_string
                        field_options.append(ProtobufOption("(nanopb).max_size", nested_type.maximum_size))
                        cpp_msg_conversion.gen_cpp_array_bounded_string_type(field_name, proto_field_name, nested_type.maximum_size,
                                                                             field_max_size, field_size_fixed)
                    elif type(nested_type) == rosidl_parser.definition.UnboundedString:
                        protobuf_type = ProtobufScalarType.type_string
                        if not has_set_pointer_type:
                            field_options.append(ProtobufOption("(nanopb).type", "FT_POINTER"))
                            has_set_pointer_type = True
                        cpp_msg_conversion.gen_cpp_array_basic_type(field_name, "std::string", proto_field_name,
                                                                    "std::string", field_max_size, field_size_fixed)
                    else:
                        raise RuntimeError("Unsupported nested rosidl type: '{}' (found in {})"
                                           .format(type(nested_type).__qualname__, '/'.join(msg_namespaced)))

            ####################
            # Unsupported Types
            #
            elif type(field_type) == rosidl_parser.definition.BoundedWString or \
                    type(field_type) == rosidl_parser.definition.UnboundedWString:
                raise RuntimeError("Wide strings are not supported (found in {})".format('/'.join(msg_namespaced)))
            else:
                raise RuntimeError("Unexpected rosidl type: '{}' (found in {})"
                                   .format(type(field_type).__qualname__, '/'.join(msg_namespaced)))

            ####################
            # Enum Checks and Saving
            #
            if is_enum and protobuf_type != enum_type_decl:
                # Logic in case the yaml tries to turn a string into an enum, it'll error out
                raise RuntimeError("Could convert field '{}' of message '{}' to protobuf enum".format(field_name, '/'.join(msg_namespaced)))

            protobuf_msg.add_field(ProtobufFieldDecl(protobuf_type, proto_field_name, field_idx, field_cardinality, field_options))
            field_idx += 1

        self.cpp_topic_file.add_include(cpp_ros_include, False)
        self.cpp_topic_file.add_conversion(cpp_msg_conversion)
        if package_name not in self.msg_packages:
            self.msg_packages.append(package_name)

        self.ros_msg_lookup[msg_namespaced] = (protobuf_msg, cpp_msg_conversion)
        return protobuf_msg, cpp_msg_conversion

#endregion

#region YAML Configuration Decoding

class FirmwareTopicConfigError(RuntimeError):
    def __init__(self, msg: str):
        super().__init__(msg)
        self.msg = msg

class TopicQOS(enum.Enum):
    """Enum for all QOS types supported, and the corresponding string value to generate with C++ code"""
    system_default = "rclcpp::SystemDefaultsQoS()"
    sensor_data = "rclcpp::SensorDataQoS()"

class FirmwareConstantMapping:
    """Decodes the firmware constant map in the YAML.

    The ProtobufRosConversion class can then use this function to easily find which field the constants in a message
    correspond to.
    """
    class MessageConstantMap:
        def __init__(self, name, const_map: 'dict[str, str]'):
            # Input sanity checking
            if not type(const_map) is dict or not type(name) is str or \
                  not all((type(x) is str and type(y) is str for x, y in const_map.items())):
                raise FirmwareTopicConfigError("Invalid constant map for message '{}'".format(name))

            self.name = name
            self.const_map = const_map.copy()

        def map_constants(self, fields: 'list[str]', constants: 'list[str]') -> 'dict[str, list[str]]':
            unmapped_constants = False
            field_map: 'dict[str, list[str]]' = {}

            match_used = {key: False for key in self.const_map}

            for const in constants:
                dest_field = None
                for match_pattern, field_name in self.const_map.items():
                    if fnmatch.fnmatchcase(const, match_pattern):
                        match_used[match_pattern] = True
                        dest_field = field_name
                        break
                if dest_field is None:
                    # If we couldn't find the field this constant belongs to in the YAML, set flag to warn when done
                    unmapped_constants = True
                    continue
                if dest_field == "":
                    # Empty field means ignore this constant
                    continue

                # Make sure dest_field is a valid field in the message
                elif dest_field not in fields:
                    raise FirmwareTopicConfigError("Unable to find referenced field '{}' in message '{}'"
                                                   .format(dest_field, self.name))

                if dest_field not in field_map:
                    field_map[dest_field] = [const]
                else:
                    field_map[dest_field].append(const)

            unused_matches = [name for name, used in match_used.items() if not used]
            if len(unused_matches) > 0:
                print("Constant map for '{}' did not use the following matches: {}"
                      .format(self.name, ', '.join(unused_matches)), file=sys.stderr)

            if unmapped_constants:
                print("Constant map did not resolve all constants for message '{}'".format(self.name), file=sys.stderr)

            return field_map


    def __init__(self, mapping_cfg: dict):
        self.const_map: 'dict[str, FirmwareConstantMapping.MessageConstantMap]' = {}
        if not type(mapping_cfg) is dict:
            raise FirmwareTopicConfigError("Invalid type of constant_mapping")

        for msg_name, msg_cfg in mapping_cfg.items():
            self.const_map[msg_name] = self.MessageConstantMap(msg_name, msg_cfg)

    def map_constants(self, msg_name: str, fields: 'list[str]', constants: 'list[str]') -> 'dict[str, list[str]]':
        if msg_name in self.const_map:
            return self.const_map[msg_name].map_constants(fields, constants)
        else:
            return None


class FirmwareParamDecl:
    protobuf_type_lookup = {
        "PARAMETER_BOOL": (ProtobufScalarType.type_bool, ProtobufFieldCardinality.NONE),
        "PARAMETER_INTEGER": (ProtobufScalarType.type_sint64, ProtobufFieldCardinality.NONE),
        "PARAMETER_DOUBLE": (ProtobufScalarType.type_double, ProtobufFieldCardinality.NONE),
        "PARAMETER_STRING": (ProtobufScalarType.type_string, ProtobufFieldCardinality.NONE),
        "PARAMETER_BYTE_ARRAY": (ProtobufScalarType.type_bytes, ProtobufFieldCardinality.NONE),
        "PARAMETER_BOOL_ARRAY": (ProtobufScalarType.type_bool, ProtobufFieldCardinality.REPEATED),
        "PARAMETER_INTEGER_ARRAY": (ProtobufScalarType.type_sint64, ProtobufFieldCardinality.REPEATED),
        "PARAMETER_DOUBLE_ARRAY": (ProtobufScalarType.type_double, ProtobufFieldCardinality.REPEATED),
        "PARAMETER_STRING_ARRAY": (ProtobufScalarType.type_string, ProtobufFieldCardinality.REPEATED),
    }

    cpp_type_lookup = {
        "PARAMETER_BOOL": ("bool", "as_bool"),
        "PARAMETER_INTEGER": ("int64_t", "as_int"),
        "PARAMETER_DOUBLE": ("double", "as_double"),
        "PARAMETER_STRING": ("std::string", "as_string"),
        "PARAMETER_BYTE_ARRAY": ("std::vector<uint8_t>", "as_byte_array"),
        "PARAMETER_BOOL_ARRAY": ("std::vector<bool>", "as_bool_array"),
        "PARAMETER_INTEGER_ARRAY": ("std::vector<int64_t>", "as_integer_array"),
        "PARAMETER_DOUBLE_ARRAY": ("std::vector<double>", "as_double_array"),
        "PARAMETER_STRING_ARRAY": ("std::vector<std::string>", "as_string_array"),
    }

    def __init__(self, name: str, param_type: str):
        if not isinstance(name, str):
            raise FirmwareTopicConfigError("Invalid type for parameter name: '{}'".format(type(name).__qualname__))
        if not param_type in self.protobuf_type_lookup:
            raise FirmwareTopicConfigError("Invalid type for parameter {}: '{}'".format(name, param_type))
        if not is_valid_identifier(name):
            raise FirmwareTopicConfigError("Invalid parameter name: '{}'".format(name))
        self.name = name
        self.param_type = param_type
        self.protobuf_type = self.protobuf_type_lookup[param_type]
        self.cpp_type = self.cpp_type_lookup[param_type]



class FirmwareTopicDecl:
    """Decodes a topic entry in the YAML.

    Performs sanity checks on the data and converts it to an easy to use form format for the ProtobufRosConversion class.
    """
    def __init__(self, name: str, data: dict, targets: 'list[str]'):
        self.targets = targets
        data = data.copy()  # Create shallow copy so we can pop without disturbing parent data

        # Decode type;
        if not "type" in data:
            raise FirmwareTopicConfigError("Topic '{}' missing required key 'type'".format(name))
        topic_type = data.pop("type")
        if not isinstance(topic_type, str):
            raise FirmwareTopicConfigError("Topic '{}' has invalid object type for key 'type': '{}'"
                                           .format(name, type(topic_type).__qualname__))

        # Decode qos
        if not "qos" in data:
            raise FirmwareTopicConfigError("Topic '{}' missing required key 'qos'".format(name))
        qos_str = data.pop("qos")
        if not isinstance(qos_str, str):
            raise FirmwareTopicConfigError("Topic '{}' has invalid object type for key 'qos': '{}'"
                                           .format(name, type(qos_str).__qualname__))
        try:
            qos = TopicQOS[qos_str]
        except KeyError as exc:
            raise FirmwareTopicConfigError("Topic '{}' has invalid qos: '{}'. Must be one of: {}"
                                           .format(name, qos_str, ', '.join([e.name for e in TopicQOS]))) from exc

        # Decode publisher and subscriber topics
        if "publishers" in data:
            publishers = data.pop("publishers")
            if not isinstance(publishers, list):
                raise FirmwareTopicConfigError("Topic '{}' has invalid object type for key 'publishers': '{}'"
                                               .format(name, type(publishers).__qualname__))
            if not all(map(lambda x: x in targets, publishers)):
                raise FirmwareTopicConfigError("Topic '{}' has invalid publisher target in: {}"
                                               .format(name, ', '.join(subscribers)))
        else:
            # If not declared, default to None
            publishers = []

        if "subscribers" in data:
            subscribers = data.pop("subscribers")
            if not isinstance(subscribers, list):
                raise FirmwareTopicConfigError("Topic '{}' has invalid object type for key 'subscribers': '{}'"
                                               .format(name, type(subscribers).__qualname__))
            if not all(map(lambda x: x in targets, subscribers)):
                raise FirmwareTopicConfigError("Topic '{}' has invalid subscriber target in: {}"
                                               .format(name, ', '.join(subscribers)))
        else:
            # If not declared, default to False
            subscribers = []

        # Error if we have unexpected keys
        if len(data) > 0:
            raise FirmwareTopicConfigError("Topic '{}' has unexpected parameters: {}".format(name, ', '.join(data.keys())))

        # This must have at least either publisher or subscriber set
        if len(subscribers) == 0 and len(publishers) == 0:
            raise FirmwareTopicConfigError("Topic '{}' does not have either 'fw_subscriber' or 'fw_subscriber' set."
                                           .format(name))

        self.name = name
        self.type = topic_type
        self.qos = qos
        self.publishers = publishers
        self.subscribers = subscribers

#endregion

def main():
    if len(sys.argv) != 5 and len(sys.argv) != 6:
        print("Usage: {} [config] [proto_out] [cmake_out] [cmake_target] [Optional: cpp_include]".format(os.path.basename(__file__)), file=sys.stderr)
        print("\tconfig: The input YAML configuration file", file=sys.stderr)
        print("\tproto_out: The output protobuf file path. Must end in .proto - Will also write a .options file", file=sys.stderr)
        print("\tcmake_out: The output for the generated cmake file to include the required packages", file=sys.stderr)
        print("\tcmake_target: The name for the target to set the dependencies in cmake_out", file=sys.stderr)
        print("\tcpp_include: The include path for the bridge header. Defaults to riptide_fw_bridge/RosProtobufBridge.hpp", file=sys.stderr)
        exit(1)

    ########################################
    # Read arguments
    ########################################

    # Parse arguments
    yaml_file = sys.argv[1]
    out_proto_file = sys.argv[2]
    out_cmake_file = sys.argv[3]
    out_cmake_target = sys.argv[4]
    cpp_header_path = sys.argv[5] if len(sys.argv) > 5 else "riptide_fw_bridge/RosProtobufBridge.hpp"

    out_name, out_ext = os.path.splitext(out_proto_file)
    assert out_ext == ".proto", "Output file must end in .proto (required for nanopb to play nice with .options)"
    out_options_file = out_name + ".options"

    ##################################################
    # Extract all top-level keys from the config
    ##################################################

    with open(yaml_file, "r") as f:
        yaml_config: dict = yaml.safe_load(f)
    if "topics" not in yaml_config:
        raise FirmwareTopicConfigError("Missing top-level key 'topics'")
    topic_cfg = yaml_config.pop("topics")
    if not isinstance(topic_cfg, dict):
        raise FirmwareTopicConfigError("Key 'topics' must be of type dictionary")

    if "targets" not in yaml_config:
        raise FirmwareTopicConfigError("Missing top-level key 'targets'")
    targets = yaml_config.pop("targets")
    if not isinstance(targets, list):
        raise FirmwareTopicConfigError("Key 'targets' must be of type array")
    for target in targets:
        if not isinstance(target, str):
            raise FirmwareTopicConfigError("Targets contains entry of invalid type: '{}'".format(type(target).__name__))
        if not re.fullmatch(r"^[a-z][a-z0-9_]*$", target):
            raise FirmwareTopicConfigError("Invalid target '{}'. Must be lowercase C identifier".format(target))

    if "constant_mapping" not in yaml_config:
        raise FirmwareTopicConfigError("Missing top-level key 'constant_mapping'")
    const_map = FirmwareConstantMapping(yaml_config.pop("constant_mapping"))

    if "parameters" in yaml_config:
        parameter_cfg = yaml_config.pop("parameters")
    else:
        parameter_cfg = {}
    if not isinstance(parameter_cfg, dict):
        raise FirmwareTopicConfigError("Key 'parameters' must be of type dictionary")

    if len(yaml_config) > 0:
        # Error if we have unexpected keys
        raise FirmwareTopicConfigError("Configuration has unexpected keys: {}".format(', '.join(yaml_config.keys())))

    ########################################
    # Perform the conversion
    ########################################

    ros_convert = ProtobufRosConversion(const_map, targets, cpp_header_path)
    for topic_name, topic_data in topic_cfg.items():
        if not isinstance(topic_data, dict):
            raise FirmwareTopicConfigError("Topic '{}' must be of type dict, but found type {}"
                                           .format(topic_name, type(topic_data).__qualname__))
        ros_convert.add_topic(FirmwareTopicDecl(topic_name, topic_data, targets))

    for param_name, param_type in parameter_cfg.items():
        if not isinstance(param_type, str):
            raise FirmwareTopicConfigError("Parameter '{}' must be of type str, but found type {}"
                                           .format(param_name, type(param_type).__qualname__))
        ros_convert.add_parameter(FirmwareParamDecl(param_name, param_type))

    ########################################
    # Write Outputs
    ########################################

    with open(out_proto_file, "w") as f:
        ros_convert.write_proto(f)

    with open(out_options_file, "w") as f:
        ros_convert.write_nanopb_options(f)

    ros_convert.write_cpp_artifacts(out_cmake_file, out_cmake_target)


if __name__ == "__main__":
    try:
        main()
    except FirmwareTopicConfigError as e:
        print("\nError decoding yaml config: {}\n".format(e.args[0]), file=sys.stderr)
        exit(1)
