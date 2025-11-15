#include <rclcpp/rclcpp.hpp>
#include "robomas_package/msg/send_current_cmd.hpp" // 仮のメッセージ型
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>
#include <unistd.h>

class SenderNode : public rclcpp::Node {
public:
    SenderNode(const std::string& ifname = "can0") : Node("Sender_node"), ifname_(ifname) {
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            throw std::runtime_error("Socket creation failed");
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ-1);
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "ioctl SIOCGIFINDEX failed");
            close(sock_);
            throw std::runtime_error("ioctl failed");
        }
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket to interface %s", ifname_.c_str());
            close(sock_);
            throw std::runtime_error("Bind failed");
        }

        subscription_ = this->create_subscription<robomas_package::msg::SendCurrentCmd>(
            "motor_tx", 10,
            std::bind(&SenderNode::topic_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to motor_tx topic");
    }

    ~SenderNode() {
        if (sock_ >= 0) {
            close(sock_);
        }
    }

private:
    void topic_callback(const robomas_package::msg::SendCurrentCmd::SharedPtr msg) {
        struct can_frame frame;
        frame.can_id = msg->id;
        frame.can_dlc = 8;
        frame.data[0] = (msg->current1 >> 8) & 0xFF;
        frame.data[1] = msg->current1 & 0xFF;
        frame.data[2] = (msg->current2 >> 8) & 0xFF;
        frame.data[3] = msg->current2 & 0xFF;
        frame.data[4] = (msg->current3 >> 8) & 0xFF;
        frame.data[5] = msg->current3 & 0xFF;
        frame.data[6] = (msg->current4 >> 8) & 0xFF;
        frame.data[7] = msg->current4 & 0xFF;
        int nbytes = write(sock_, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame");
        }
    }
    std::string ifname_;
    int sock_;
    rclcpp::Subscription<robomas_package::msg::SendCurrentCmd>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::string ifname = (argc > 1) ? argv[1] : "can0";
    auto node = std::make_shared<SenderNode>(ifname);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}