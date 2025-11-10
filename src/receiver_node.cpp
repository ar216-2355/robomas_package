#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <unistd.h>
#include <cstring>
#include <string>
#include <sstream>
#include <iomanip>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("can_receiver");
  auto pub = node->create_publisher<std_msgs::msg::String>("can_rx", 10);

  const char * ifname = (argc > 1) ? argv[1] : "can0";

  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ-1);
  ioctl(sock, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(sock, (struct sockaddr*)&addr, sizeof(addr));

  struct can_frame frame;

  while (rclcpp::ok()) {
    if (read(sock, &frame, sizeof(frame)) < 0) continue;

    std::ostringstream ss;
    ss << "0x" << std::hex << frame.can_id << " [" << std::dec << (int)frame.can_dlc << "] ";
    for(int i = 0; i < frame.can_dlc; i++){
      ss << std::setw(2) << std::setfill('0') << std::hex << (int)frame.data[i] << " ";
    }

    std::string text = ss.str();
    RCLCPP_INFO(node->get_logger(), "%s", text.c_str());

    std_msgs::msg::String msg;
    msg.data = text;
    pub->publish(msg);
  }

  close(sock);
  rclcpp::shutdown();
  return 0;
}
