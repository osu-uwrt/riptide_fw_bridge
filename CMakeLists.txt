cmake_minimum_required(VERSION 3.8)
project(riptide_fw_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Protobuf REQUIRED)

# Generate C++ Bridge and Protobuf Files
# Input Files:
set(PROTOBRIDGE_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/scripts/protobridge_gen.py")
set(PROTOBRIDGE_CFG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/firmware_msgs.yaml")
# Output Files:
set(PROTOBRIDGE_DIR "${CMAKE_BINARY_DIR}/protobridge")
set(PROTOBRIDGE_PROTO_FILE "${PROTOBRIDGE_DIR}/protobridge.proto")
string(REGEX REPLACE "[.]proto$" ".options" PROTOBRIDGE_OPTIONS_FILE ${PROTOBRIDGE_PROTO_FILE})
set(PROTOBRIDGE_CMAKE_FILE "${PROTOBRIDGE_DIR}/protobridge.cmake")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${PROTOBRIDGE_SCRIPT}" "${PROTOBRIDGE_CFG_FILE}")
file(MAKE_DIRECTORY ${PROTOBRIDGE_DIR})
execute_process(COMMAND "${PROTOBRIDGE_SCRIPT}" "${PROTOBRIDGE_CFG_FILE}" "${PROTOBRIDGE_PROTO_FILE}"
  "${PROTOBRIDGE_CMAKE_FILE}" fw_bridge
  COMMAND_ERROR_IS_FATAL ANY)

# Generate C++ Libraries from Protobuf Proto Files
protobuf_generate_cpp(PROTO_SRC PROTO_HEADER "${PROTOBRIDGE_PROTO_FILE}")
get_filename_component(PROTO_HEADER_PATH "${PROTO_HEADER}" DIRECTORY)

# Compile executable
add_executable(fw_bridge
  src/main.cpp
  src/RosProtobufBridge.cpp
  "${PROTO_SRC}"
)
target_include_directories(fw_bridge PRIVATE "${PROTO_HEADER_PATH}" include/)
ament_target_dependencies(fw_bridge rclcpp Protobuf)
include("${PROTOBRIDGE_CMAKE_FILE}")

# Compile get_version command
add_executable(get_version src/get_version.cpp "${PROTO_SRC}")
target_include_directories(get_version PRIVATE "${PROTO_HEADER_PATH}")
ament_target_dependencies(get_version Protobuf)

#Install executables
install(TARGETS fw_bridge get_version
  DESTINATION lib/${PROJECT_NAME}
)

# Install protobuf configuration files
install(FILES "${PROTOBRIDGE_PROTO_FILE}" "${PROTOBRIDGE_OPTIONS_FILE}"
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
