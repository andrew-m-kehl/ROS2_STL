#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

#define MAXSPEED 2

struct Point {
    double x, y;
};

class TurtleSquareDriver : public rclcpp::Node {
public:
    TurtleSquareDriver() : Node("square_driver"), current_waypoint_(0), is_manual_(false) {
        
        this->declare_parameter("waypoints_x", std::vector<double>{});
        this->declare_parameter("waypoints_y", std::vector<double>{});

        std::vector<double> wx = this->get_parameter("waypoints_x").as_double_array();
        std::vector<double> wy = this->get_parameter("waypoints_y").as_double_array();

        for (size_t i = 0; i < wx.size() && i < wy.size(); ++i) {
            waypoints_.push_back({wx[i], wy[i]});
        }
 
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleSquareDriver::pose_callback, this, std::placeholders::_1));
            
        // Subscribe to manual override state
        override_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/manual_override", 10, std::bind(&TurtleSquareDriver::override_callback, this, std::placeholders::_1));

        // Control loop timer (50 Hz)
        timer_ = this->create_wall_timer(20ms, std::bind(&TurtleSquareDriver::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Waypoint Driver Started! Waiting for TurtleSim to spawn...");
    }

private:
    std::vector<Point> waypoints_;
    size_t current_waypoint_;
    
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
            if (twist.linear.x > MAXSPEED) twist.linear.x = MAXSPEED; 
        } 
        else {
            twist.linear.x = 0.0;
        }

        publisher_->publish(twist);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSquareDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}