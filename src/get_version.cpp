/*
 * Simple program to print the current version of the generated protobuf
 * Useful as a command in the package to easily get the protobuf version for whatever reason
 *
 * Pass -q to only print the protocol version
 */

#include <iostream>
#include "protobridge.pb.h"

int main(int argc, char** argv) {
    titan_pb::comm_msg msg;
    auto& options = msg.descriptor()->options();
    uint32_t protocol_version = options.GetExtension(titan_pb::protocol_version);

    if (argc > 1 && std::string(argv[1]) == "-q") {
        std::cout << protocol_version << std::endl;
    }
    else {
        std::cout << "Protobridge Protocol Version: " << protocol_version << std::endl;
    }

    return 0;
}
