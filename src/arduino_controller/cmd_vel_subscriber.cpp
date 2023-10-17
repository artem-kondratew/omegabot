#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "connect.hpp"


namespace {
    int last_command = -1;
}


class VelocitySubscriber : public rclcpp::Node {
public:
    VelocitySubscriber()
    : Node("cmd_vel_subscriber")
    {
        this->declare_parameter("velocity", 255);
        this->declare_parameter("rotate_velocity", 255);

      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&VelocitySubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist & msg) const {

        int velocity = this->get_parameter("velocity").as_int();
        int rotate_velocity = this->get_parameter("rotate_velocity").as_int();

        Connect::resetCommand();

        if (msg.linear.x == 0 && msg.angular.z == 0) {
            Connect::stop();
            last_command = STOP_TASK;
        }

        if (msg.linear.x > 0) {
            if (last_command == MOVE_BACKWARD_TASK) {
                return;
            }
            Connect::moveForward(velocity);
            last_command = MOVE_FORWARD_TASK;
        }

        if (msg.linear.x < 0) {
            if (last_command == MOVE_FORWARD_TASK) {
                return;
            }
            Connect::moveBackward(velocity);
            last_command = MOVE_BACKWARD_TASK;
        }

        if (msg.linear.x == 0 && msg.angular.z < 0) {
            Connect::turnRight(rotate_velocity);
            last_command = TURN_RIGHT_TASK;
        }

        if (msg.linear.x == 0 && msg.angular.z > 0) {
            Connect::turnLeft(rotate_velocity);
            last_command = TURN_LEFT_TASK;
        }
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};


int main(int argc, char * argv[]) {
    if (!Connect::setConnection()) {
        return 0;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocitySubscriber>());
    rclcpp::shutdown();
    return 0;
}
