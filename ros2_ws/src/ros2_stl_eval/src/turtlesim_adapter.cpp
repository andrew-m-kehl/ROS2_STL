/*
Turtlesim Adapter: 
Converts turtlesim pose messages published from nav_msg::msg::Odometry to std_msg::msg::Float64MultiArray.

Inputs:
        Topic: /turtle1/pose    - Subscriber: turtlesim::msg::Pose - Receives turtlesim position and velocity.
Outputs:
        Topic: /stl_input       - Publisher: std_msgs/msg/Float64MultiArray (Time, X, Y, V)

Launch:
ros2 run driver turtlesim_adapter --ros-args -p odom_topic:=/robot1/odom
*/

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>
#include <vector>

class TurtlesimAdapter : public rclcpp::Node {
public:
    TurtlesimAdapter() : Node("turtlesim_adapter"), start_time_(-1.0) {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/stl_input", 10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtlesimAdapter::pose_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Turtlesim Adapter bridging /turtle1/pose -> /stl_input");
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        double current_time = this->now().nanoseconds() / 1e9;
        
        // Record start time on the first message
        if (start_time_ < 0.0) {
            start_time_ = current_time;
        }
        
        // Normalize time to start at 0.0
        double normalized_time = current_time - start_time_;

        auto array_msg = std_msgs::msg::Float64MultiArray();
        array_msg.data = {normalized_time, msg->x, msg->y, msg->linear_velocity};
        publisher_->publish(array_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    double start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlesimAdapter>());
    rclcpp::shutdown();
    return 0;
}