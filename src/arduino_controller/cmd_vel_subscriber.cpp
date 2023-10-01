#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "connect.hpp"


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
        RCLCPP_INFO(this->get_logger(), "I heard: '%f %f'", msg.linear.x, msg.angular.z);

        Connect::resetCommand();

        if (msg.linear.x > 0) {
            Connect::moveForward();
            Connect::ledOn();
            RCLCPP_INFO(this->get_logger(), "I heard: move %d", Connect::getMessageAnswer());
        }

        if (msg.linear.x == 0 && msg.angular.z == 0) {
            Connect::stop();
            Connect::ledOff();
            RCLCPP_INFO(this->get_logger(), "I heard: stop %d", Connect::getMessageAnswer());
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
