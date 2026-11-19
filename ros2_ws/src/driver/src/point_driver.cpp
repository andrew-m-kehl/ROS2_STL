/*
Point Driver: 
A geometric waypoint driver that moves a turtlesim robot sequentially through a list of provided X/Y coordinates, calculating the angles and speed to reach each point. 

Inputs:
    Topic: /turtle1/pose -  Subscriber: turtlesim::msg::Pose - Receives the turtlesims current position, orientation and velocity.
      	   /manual_override - Subscriber: std_msgs::msg::Bool - Boolean to determine automation or manual control. 
      		    
      
    Parameters: waypoints_x, waypoints_y

Outputs:
    Topic: /turtle1/cmd_vel - Publisher: geometry_msgs::msg::Twist - Velocity commands.
     
To launch:
ros2 run driver point_driver --ros-args -p waypoints_x:="[2.0, 8.0, 8.0, 2.0]" 
                                        -p waypoints_y:="[2.0, 2.0, 8.0, 8.0]"
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

struct Point {
    double x, y;
};

class PointDriver : public rclcpp::Node {
public:
    PointDriver() : Node("point_driver"), current_waypoint_(0), is_manual_(false) {
        
        this->declare_parameter("waypoints_x", std::vector<double>{});
        this->declare_parameter("waypoints_y", std::vector<double>{});
        this->declare_parameter("max_speed", 4.0);
        
        std::vector<double> wx = this->get_parameter("waypoints_x").as_double_array();
        std::vector<double> wy = this->get_parameter("waypoints_y").as_double_array();
        max_speed_ = this->get_parameter("max_speed").as_double();

        for (size_t i = 0; i < wx.size() && i < wy.size(); ++i) {
            waypoints_.push_back({wx[i], wy[i]});
        }
 
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&PointDriver::pose_callback, this, std::placeholders::_1));
            
        // Subscribe to manual override state
        override_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/manual_override", 10, std::bind(&PointDriver::override_callback, this, std::placeholders::_1));

        // Control loop timer (50 Hz)
        timer_ = this->create_wall_timer(20ms, std::bind(&PointDriver::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Waypoint Driver Started! Waiting for TurtleSim to spawn...");
    }

private:
    std::vector<Point> waypoints_;
    size_t current_waypoint_;
    double max_speed_;
    
    turtlesim::msg::Pose current_pose_;
    bool pose_received_ = false;
    bool is_manual_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr override_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void override_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        is_manual_ = msg->data;
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        current_pose_ = *msg;
        if (!pose_received_) {
            pose_received_ = true;
            RCLCPP_INFO(this->get_logger(), "TurtleSim connected! Starting path.");
        }
    }

    void control_loop() {
        if (!pose_received_) return;

        // Yield control if manual override is active
        if (is_manual_) return;

        if (waypoints_.empty()){
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for waypoints.");
            return;
        }

        Point target = waypoints_[current_waypoint_];
        double dx = target.x - current_pose_.x;
        double dy = target.y - current_pose_.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        auto twist = geometry_msgs::msg::Twist();

        // If waypoint is reached, set next waypoint.
        if (distance < 0.1) {
            current_waypoint_ = (current_waypoint_ + 1) % waypoints_.size();
            return;
        }

        // Set Angle
        double target_theta = std::atan2(dy, dx);
        double angle_diff = target_theta - current_pose_.theta;

        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        twist.angular.z = 4 * angle_diff;
        
        if (std::fabs(angle_diff) < 0.2) {
            twist.linear.x = 1.5 * distance;
            if (twist.linear.x > max_speed_) twist.linear.x = max_speed_; 
        } 
        else {
            twist.linear.x = 0.0;
        }

        publisher_->publish(twist);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}