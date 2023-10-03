#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "connect.hpp"


namespace {
    int last_command = -1;
}


class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist & msg) const {

        Connect::resetCommand();

        if (msg.linear.x == 0 && msg.angular.z == 0) {
            Connect::stop();
            last_command = STOP_TASK;
        }

        if (msg.linear.x > 0) {
            if (last_command == MOVE_BACKWARD_TASK) {
                return;
            }
            Connect::moveForward();
            last_command = MOVE_FORWARD_TASK;
        }

        if (msg.linear.x < 0) {
            if (last_command == MOVE_FORWARD_TASK) {
                return;
            }
            Connect::moveBackward();
            last_command = MOVE_BACKWARD_TASK;
        }

        if (msg.linear.x == 0 && msg.angular.z < 0) {
            Connect::turnRight();
            last_command = TURN_RIGHT_TASK;
        }

        if (msg.linear.x == 0 && msg.angular.z > 0) {
            Connect::turnLeft();
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
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
