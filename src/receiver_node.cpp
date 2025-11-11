#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "robomas_package/msg/motor_feedback.hpp"

#include <unistd.h>
#include <cstring>
#include <string>
#include <vector>
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
  auto pub = node->create_publisher<robomas_package::msg::MotorFeedback>("motor_rx", 10);

  const char * ifname = (argc > 1) ? argv[1] : "can0";

  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ-1);
  ioctl(sock, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(sock, (struct sockaddr*)&addr, sizeof(addr));

  // フィルター設定　（filters配列にpush_backで入れる）
  std::vector<struct can_filter> filters;
  for (uint16_t id = 0x201; id <= 0x208; ++id) {
    struct can_filter f;
    f.can_id = id;
    f.can_mask = CAN_SFF_MASK; // 標準ID(11bit)での完全一致
    filters.push_back(f);
  }
  int ret = setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(struct can_filter));
  if(ret < 0) {
    RCLCPP_ERROR(node->get_logger(), "setsockopt failed");
    return -1;
  }

  struct can_frame frame;

  while (rclcpp::ok()) {
    if (read(sock, &frame, sizeof(frame)) < 0) continue;

    robomas_package::msg::MotorFeedback msg;

    if(frame.can_id >= 0x201 && frame.can_id <= 0x208){
      msg.id = (uint8_t)(frame.can_id - (uint8_t)0x200);
      msg.angle = (static_cast<uint16_t>((frame.data[0]) << 8) | frame.data[1]);
      msg.rotational_speed = (static_cast<uint16_t>((frame.data[2]) << 8) | frame.data[3]);
      msg.current = (static_cast<uint16_t>((frame.data[4]) << 8) | frame.data[5]);
      msg.motor_tempelature = frame.data[6];

      pub->publish(msg);
    }
    
  }

  close(sock);
  rclcpp::shutdown();
  return 0;
}
