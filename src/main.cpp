#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "riptide_fw_bridge/RosProtobufBridge.hpp"

#include "protobridge.pb.h"

void txCb(int clientId, const std::string& data) {
    std::cout << "Transmitting to client " << clientId << ":" << std::endl;
    titan_pb::comm_msg msg;
    if (!msg.ParseFromString(data)) {
        std::cout << "Unable to deserialize!" << std::endl;
    } else {
        std::cout << msg.DebugString() << std::endl;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() < 2) {
        std::cerr << "Invalid args!" << std::endl;
        std::cerr << "Usage: " << args.at(0) << " [target]" << std::endl;
        return 1;
    }
    auto bridgeItf = RosProtobufBridge::RosProtobufBridge(args.at(1), txCb);
    bridgeItf.spin();
    rclcpp::shutdown();
    return 0;
}
