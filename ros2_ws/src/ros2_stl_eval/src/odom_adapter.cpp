/*
Odomoetry Adapter for iRobot Create 3: 
Converts ROS2 position and velocity messages published from nav_msg::msg::Odometry to std_msg::msg::Float64MultiArray.

Inputs:
        Topic: /odom - Subscriber: nav_msg::msg::Odometry - Receives robot odometry.
        Parameters: odom_topic - Specifies which odometry topic to subscribe to.
Outputs:
        Topic: /stl_input - Publisher : std_msgs/msg/Float64MultiArray (Time, X, Y, V)

Launch:
ros2 run driver odom_adapter --ros-args -p odom_topic:=/robot1/odom
*/

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>
#include <vector>

class OdomAdapter : public rclcpp::Node {
public:
    OdomAdapter() : Node("odom_adapter"), start_time_(-1.0) {
        // Create 3 standard odometry topic
        this->declare_parameter("odom_topic", "/odom");
        std::string odom_topic = this->get_parameter("odom_topic").as_string();

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/stl_input", 10);
        
        // Subscribe to nav_msgs/Odometry instead of turtlesim/Pose
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&OdomAdapter::odom_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Odometry Adapter bridging %s -> /stl_input", odom_topic.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double current_time = this->now().nanoseconds() / 1e9;
        
        if (start_time_ < 0.0) {
            start_time_ = current_time;
        }
        double normalized_time = current_time - start_time_;

        auto array_msg = std_msgs::msg::Float64MultiArray();
        
        // Extract X, Y, and Linear X Velocity
        array_msg.data = {
            normalized_time, 
            msg->pose.pose.position.x, 
            msg->pose.pose.position.y,
            msg->twist.twist.linear.x
        };
        
        publisher_->publish(array_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    double start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomAdapter>());
    rclcpp::shutdown();
    return 0;
}