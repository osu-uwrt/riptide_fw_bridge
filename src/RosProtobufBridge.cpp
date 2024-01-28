#include "riptide_fw_bridge/RosProtobufBridge.hpp"

namespace RosProtobufBridge {

static std::string lookupCommFieldName(const titan_pb::comm_msg& msg, int fieldNum) {
    auto msgDescriptor = msg.GetDescriptor();
    const google::protobuf::FieldDescriptor* fieldDescriptor = nullptr;
    if (msgDescriptor != nullptr && (fieldDescriptor = msgDescriptor->FindFieldByNumber(fieldNum)) != nullptr) {
        return fieldDescriptor->name();
    }
    else {
        return "Unknown Topic Num " + std::to_string(fieldNum);
    }
}

RosProtobufBridge::RosProtobufBridge(std::string target, ProtobufTxCB txCallback): txCallback_(txCallback) {
    // Retreive protocol version from ROS
    titan_pb::comm_msg msg;
    auto& options = msg.descriptor()->options();
    protocol_version_ = options.GetExtension(titan_pb::protocol_version);

    // Create node to handle messages
    node_ = std::make_shared<rclcpp::Node>("fw_bridge_" + target);

    // Create the various message handlers
    topicHandler = createTopicHandler(*node_, *this, target);
    paramHandler = createParamHandler(*node_, *this);
}

void RosProtobufBridge::processPacket(int clientId, const void *data, size_t size) {
    // Decode tbhe incoming packet
    titan_pb::comm_msg msg;
    if (size > INT_MAX || !msg.ParseFromArray(data, (int) size)) {
        RCLCPP_WARN(node_->get_logger(), "Corrupted protobuf packet received from client %d", clientId);
        return;
    }

    // Try processing packet
    try {
        bool send_ack = false;

        // Handle connect
        if (msg.msg_case() == titan_pb::comm_msg::kConnectVer) {
            if (msg.connect_ver() == protocol_version_) {
                RCLCPP_INFO(node_->get_logger(), "Client %d Connected", clientId);
                send_ack = true;
            }
            else {
                RCLCPP_WARN(node_->get_logger(), "Client %d attepting to connect with invalid protocol version 0x%u (expected %u)", clientId, msg.connect_ver(), protocol_version_);
            }
        }

        // Try handling with message handlers
        else if (topicHandler->processMessage(clientId, msg)) {
            send_ack = true;
        }
        else if (paramHandler->processMessage(clientId, msg)) {
            // No need to send ack, since it'll transmit it manually
        }

        // We couldn't process message, give a warning
        else if (msg.msg_case() == titan_pb::comm_msg::MSG_NOT_SET) {
            RCLCPP_WARN(node_->get_logger(), "Client %d sent packet without populating a message", clientId);
        }
        else {
            RCLCPP_WARN(node_->get_logger(), "Client %d published on '%s', which does not have an associated handler (check that publisher is enabled for target)", clientId, lookupCommFieldName(msg, msg.msg_case()).c_str());
        }

        // Finally send ack if needed
        if (send_ack && msg.ack() != 0) {
            titan_pb::comm_msg ack_resp;
            ack_resp.set_ack(msg.ack());
            ack_resp.clear_msg();
            sendResponse(clientId, ack_resp);
        }
    }
    // If any decodes fail, report it
    catch (MsgConversionError &e) {
        if (msg.msg_case() == titan_pb::comm_msg::MSG_NOT_SET) {
            RCLCPP_WARN(node_->get_logger(), "Client %d published invalid message with topic not set? - %s", clientId, e.what());
        }
        else {
            RCLCPP_WARN(node_->get_logger(), "Client %d published invalid message on '%s' - %s", clientId, lookupCommFieldName(msg,msg.msg_case()).c_str(), e.what());
        }
    }
}

void RosProtobufBridge::sendResponse(int clientId, const titan_pb::comm_msg &msg) {
    std::string data;
    if (!msg.SerializeToString(&data)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to serialize message to client %d", clientId);
        return;
    }

    std::scoped_lock lock{tx_mutex_};
    txCallback_(clientId, data);
}

void RosProtobufBridge::spin() {
    rclcpp::spin(node_);
}

}
