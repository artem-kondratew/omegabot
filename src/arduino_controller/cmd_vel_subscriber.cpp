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
    //int last_command = PING_TASK;
    void topic_callback(const geometry_msgs::msg::Twist & msg) const {
<<<<<<< HEAD
        //RCLCPP_INFO(this->get_logger(), "I heard: '%f %f'", msg.linear.x, msg.angular.z);
=======
>>>>>>> 5e28341e19345d05dac9bb810ca98255e3b08389

        Connect::resetCommand();

        if (msg.linear.x == 0 && msg.angular.z == 0) {
            Connect::stop();
            last_command = STOP_TASK;
<<<<<<< HEAD
            // RCLCPP_INFO(this->get_logger(), "I heard: stop %d", Connect::getMessageAnswer());
=======
>>>>>>> 5e28341e19345d05dac9bb810ca98255e3b08389
        }

        if (msg.linear.x > 0) {
            if (last_command == MOVE_BACKWARD_TASK) {
                return;
            }
            Connect::moveForward();
            last_command = MOVE_FORWARD_TASK;
<<<<<<< HEAD
            // RCLCPP_INFO(this->get_logger(), "I heard: move forward %d", Connect::getMessageAnswer());
=======
>>>>>>> 5e28341e19345d05dac9bb810ca98255e3b08389
        }

        if (msg.linear.x < 0) {
            if (last_command == MOVE_FORWARD_TASK) {
                return;
            }
            Connect::moveBackward();
            last_command = MOVE_BACKWARD_TASK;
<<<<<<< HEAD
            // RCLCPP_INFO(this->get_logger(), "I heard: move backward %d", Connect::getMessageAnswer());
=======
>>>>>>> 5e28341e19345d05dac9bb810ca98255e3b08389
        }

        if (msg.linear.x == 0 && msg.angular.z < 0) {
            Connect::turnRight();
            last_command = TURN_RIGHT_TASK;
<<<<<<< HEAD
            // RCLCPP_INFO(this->get_logger(), "I heard: turn right %d", Connect::getMessageAnswer());
=======
>>>>>>> 5e28341e19345d05dac9bb810ca98255e3b08389
        }

        if (msg.linear.x == 0 && msg.angular.z > 0) {
            Connect::turnLeft();
            last_command = TURN_LEFT_TASK;
<<<<<<< HEAD
            // RCLCPP_INFO(this->get_logger(), "I heard: turn left %d", Connect::getMessageAnswer());
=======
>>>>>>> 5e28341e19345d05dac9bb810ca98255e3b08389
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
