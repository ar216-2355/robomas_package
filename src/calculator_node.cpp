#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "robomas_package/msg/motor_feedback.hpp"
#include "robomas_package/msg/motor_cmd.hpp"
#include "robomas_package/msg/motor_cmd_array.hpp"
#include "robomas_package/msg/send_current_cmd.hpp"

#include <atomic>
#include <string>
#include <array>
#include <cmath>
#include <chrono>

static constexpr uint8_t motor_number = 8; // 使用するモーターの台数（最大8台）

static constexpr float CURRENT_MAX = 15000.0f;
static constexpr float VELOCITY_MAX = 5000.0f;

static constexpr float ROBOMAS_FEEDBACK_HZ = 500.0f;   // ロボマスのフィードバックデータ送信周期を1000Hzから500Hzに落として欲しい
static constexpr float DT = 1.0f / ROBOMAS_FEEDBACK_HZ;

enum class MotorType { M2006, M3508 };
enum class CtrlMode  : uint8_t { CURRENT=0, SPEED=1, POSITION=2 };

struct Pid {
    std::atomic<float> kp{0.3f};
    std::atomic<float> ki{0.01f};
    std::atomic<float> kd{0.01f};
    float i_sum{0}; // 積分項
    float prev_e{0}; // 前回の値
    std::atomic<float> out_min{-1e9f};
    std::atomic<float> out_max{1e9f};

    float step(float e, float dt) {
        i_sum += e * dt;
        float d = (e - prev_e) / dt; // 微分項
        prev_e = e;
        float u = kp.load()*e + ki.load()*i_sum + kd.load()*d;
        if (u > out_max.load()) u = out_max.load();
        if (u < out_min.load()) u = out_min.load();
        return u;
    }

    void reset() {
        i_sum = 0.0f;
        prev_e = 0.0f;
    }
};

struct MotorState {
    uint8_t id; // 1 ~ 8   （この変数は使わない可能性が高い）
    std::atomic<MotorType> type{MotorType::M3508};
    std::atomic<CtrlMode>  mode{CtrlMode::CURRENT};

    int16_t real_current{0};
    int16_t real_speed{0};
    float real_position{0};

    std::atomic<float> current{0.0f};
    float speed{0.0f};
    float position{0.0f};

    int ancle{0}; // 0 ~ 8191
    std::atomic<float> target_current = 0.0f;
    std::atomic<float> target_speed = 0.0f;
    std::atomic<float> target_position = 0.0f;
    std::atomic<int16_t> motor_temperature{0};

    Pid pid_spd;
    Pid pid_pos;
};

static std::array<MotorState, motor_number> M; // 最大8台のモーターを想定

// ロボマスが30000rpm以上にならない上での設計
float rotation_cnt_from_angle(uint16_t angle, float prev_position, int16_t rotational_speed) {
    // angle: 0 ~ 8191
    int pp_integer = std::floor(prev_position);  // pp は previous position の意
    float pp_decimal = prev_position - pp_integer;
    if (rotational_speed > 0) {
        if (pp_decimal < angle / 8192.0f) {
            return pp_integer + angle / 8192.0f;
        } else {
            return (pp_integer + 1) + angle / 8192.0f;
        }
    } else if (rotational_speed < 0) {
        if (angle / 8192.0f < pp_decimal) {
            return pp_integer + angle / 8192.0f;
        } else {
            return (pp_integer - 1) + angle / 8192.0f;
        }
    } else {
        return pp_integer + angle / 8192.0f;
    }
}

// ---- ROS2 ノード ----
class CalculatorNode : public rclcpp::Node {
public:
    CalculatorNode() : Node("calculator_node")
    {
        subscription_ = this->create_subscription<robomas_package::msg::MotorFeedback>("motor_rx", 10, 
            std::bind(&CalculatorNode::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<robomas_package::msg::SendCurrentCmd>("motor_tx", 10);
        timer_ =this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&CalculatorNode::timer_callback, this));
        target_sub_ = this->create_subscription<robomas_package::msg::MotorCmdArray>("motor_cmd_array", 10,
            std::bind(&CalculatorNode::target_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const robomas_package::msg::MotorFeedback::SharedPtr msg) {
        uint8_t i = msg->id - 1;
        
        if (msg->motor_temperature == 0) {
            M[i].type.store(MotorType::M2006);
        }

        M[i].real_current = msg->current;
        M[i].real_speed = msg->rotational_speed;
        M[i].real_position = rotation_cnt_from_angle(msg->angle, M[i].real_position, msg->rotational_speed);
        auto mode = M[i].mode.load();
        switch (mode) {
            case CtrlMode::CURRENT: {
                float target = M[i].target_current.load();
                M[i].current.store(target);
                break;
            }
            case CtrlMode::SPEED: {
                float target = M[i].target_speed.load();
                float e_spd = target - M[i].real_speed;
                M[i].current.store(M[i].pid_spd.step(e_spd, DT));
                break;
            }
            case CtrlMode::POSITION: {
                float target = M[i].target_position.load();
                float e_pos = target - M[i].real_position;
                float spd_cmd = M[i].pid_pos.step(e_pos, DT);
                float e_spd = spd_cmd - M[i].real_speed;
                M[i].current.store(M[i].pid_spd.step(e_spd, DT));
                break;
            }
        }
        if (M[i].current.load() > CURRENT_MAX) {
            M[i].current.store(CURRENT_MAX);
        }
        if (M[i].current.load() < -CURRENT_MAX) {
            M[i].current.store(-CURRENT_MAX);
        }
    }

    void timer_callback() {
        auto message = robomas_package::msg::SendCurrentCmd();
        message.id = 0x200;
        message.current1 = static_cast<int16_t>(M[0].current.load());
        message.current2 = static_cast<int16_t>(M[1].current.load());
        message.current3 = static_cast<int16_t>(M[2].current.load());
        message.current4 = static_cast<int16_t>(M[3].current.load());
        publisher_->publish(message);

        if (motor_number <= 4) return;
        
        message.id = 0x1FF;
        message.current1 = static_cast<int16_t>(M[4].current.load());
        message.current2 = static_cast<int16_t>(M[5].current.load());
        message.current3 = static_cast<int16_t>(M[6].current.load());
        message.current4 = static_cast<int16_t>(M[7].current.load());
        publisher_->publish(message);
        
    }

    void target_callback(const robomas_package::msg::MotorCmdArray::SharedPtr msg) {
        for (const auto& cmd : msg->cmds) {
            uint8_t i = cmd.id - 1;
            M[i].mode.store(static_cast<CtrlMode>(cmd.mode));
            switch (M[i].mode.load()) {
                case CtrlMode::CURRENT:
                    M[i].target_current.store(cmd.value);
                    break;
                case CtrlMode::SPEED:
                    M[i].target_speed.store(cmd.value);
                    break;
                case CtrlMode::POSITION:
                    M[i].target_position.store(cmd.value);
                    break;
            }
        }
    }

    rclcpp::Subscription<robomas_package::msg::MotorFeedback>::SharedPtr subscription_;
    rclcpp::Subscription<robomas_package::msg::MotorCmdArray>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<robomas_package::msg::SendCurrentCmd>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalculatorNode>());
    rclcpp::shutdown();
    return 0;
}