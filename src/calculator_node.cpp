#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "robomas_package/msg/motor_feedback.hpp"

#include <atomic>
#include <string>

struct MotorState{
    uint8_t motor_type = 1; // 1: M2006, 2: M3508
    uint8_t control_mode = 0; // 0: Current, 1: Speed, 2: Position
    std::atomic<float> position{0.0f};
    std::atomic<float> velocity{0.0f};
    std::atomic<float> current{0.0f};

    
}

class CalculatorNode : public rclcpp::Node
{
public:
    CalculatorNode() : Node("calculator_node")
    {
        subscription_ = this->create_subscription<robomas_package::msg::MotorFeedback>("mtoro_rx", 10, 
            std::bind(&CalculatorNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const robomas_package::msg::MotorFeedback::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Motor ID: %d, Position: %f, Velocity: %f, Current: %f",
                    msg->id, msg->position, msg->velocity, msg->current);
    }
    rclcpp::Subscription<robomas_package::msg::MotorFeedback>::SharedPtr subscription_;

}