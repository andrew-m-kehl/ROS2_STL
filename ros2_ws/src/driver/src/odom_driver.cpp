/*
Odometry Driver: 
A time-based trajectory driver that calculates angular velocities, linear velocities and duration to move a turtlesim robot sequentially through a list of provided X/Y coordinates, calculating the angles and speed to reach each point. 

Inputs:
    Topic: /turtle1/pose -  Subscriber: turtlesim::msg::Pose - Receives the turtlesims current position, orientation and velocity.
      	   /manual_override - Subscriber: std_msgs::msg::Bool - Boolean to determine automation or manual control. 
      		    
      
    Parameters: waypoints_x, waypoints_y

Outputs:
    Topic: /turtle1/cmd_vel - Publisher: geometry_msgs::msg::Twist  - Velocity commands.
*/

/*Odomoetry adapter for iRobot Create 3. 

Inputs:
        Topic: /odom (nav_msg::msg::Odometry)
        Parameters: odom_topic
Outputs:
        Topic: /stl_input (Time, X, Y, V)

To launch:
ros2 run driver odom_driver --ros-args -p waypoints_x:="[2.0, 8.0, 8.0, 2.0]" 
                                       -p waypoints_y:="[2.0, 2.0, 8.0, 8.0]"

*/

#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

struct Point {
    double x, y;
};

struct PathStep {
    enum Type { TURN, DRIVE, WAIT } type;
    double linear_x;
    double angular_z;
    double duration;
    
    double start_x, start_y, start_theta;
    double end_x, end_y, end_theta;
};

enum class DriverMode {
    AUTO,
    MANUAL,
    BRAKING,
    RECOVERING
};

class OdomDriver : public rclcpp::Node {
public:
    OdomDriver() 
    : Node("odom_driver"), mode_(DriverMode::AUTO), step_index_(0), pose_received_(false), path_built_(false) {
        
        this->declare_parameter("waypoints_x", std::vector<double>{});
        this->declare_parameter("waypoints_y", std::vector<double>{});

        std::vector<double> wx = this->get_parameter("waypoints_x").as_double_array();
        std::vector<double> wy = this->get_parameter("waypoints_y").as_double_array();

        for (size_t i = 0; i < wx.size() && i < wy.size(); ++i) {
            waypoints_.push_back({wx[i], wy[i]});
        }

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&OdomDriver::pose_callback, this, std::placeholders::_1));
            
        override_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/manual_override", 10, std::bind(&OdomDriver::override_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Dynamic Trajectory Driver Started! Waiting for TurtleSim...");
    }

private:
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void build_dynamic_path() {
        if (waypoints_.empty()) return;
        path_.clear();

        double cur_x = current_pose_.x;
        double cur_y = current_pose_.y;
        double cur_theta = current_pose_.theta;
        
        double linear_vel = 2.0; 
        double angular_vel = 1.5; 
        double tick = 0.016;

        auto add_turn = [&](double target_theta) {
            double angle_diff = normalize_angle(target_theta - cur_theta);
            if (std::abs(angle_diff) > 1e-5) {
                int ticks = std::max(1, static_cast<int>(std::round(std::abs(angle_diff) / angular_vel / tick)));
                double duration = ticks * tick;
                double actual_w = angle_diff / duration;
                
                path_.push_back({PathStep::TURN, 0.0, actual_w, duration, cur_x, cur_y, cur_theta, cur_x, cur_y, cur_theta + angle_diff});
                cur_theta = normalize_angle(cur_theta + angle_diff);
            }
        };

        auto add_drive = [&](double target_x, double target_y) {
            double dx = target_x - cur_x;
            double dy = target_y - cur_y;
            double dist = std::hypot(dx, dy);
            if (dist > 1e-5) {
                int ticks = std::max(1, static_cast<int>(std::round(dist / linear_vel / tick)));
                double duration = ticks * tick;
                double actual_v = dist / duration;
                
                path_.push_back({PathStep::DRIVE, actual_v, 0.0, duration, cur_x, cur_y, cur_theta, target_x, target_y, cur_theta});
                cur_x = target_x;
                cur_y = target_y;
            }
        };
        
        // 1. Initial path: current spawn pose -> Waypoint 0
        double target_heading = std::atan2(waypoints_[0].y - cur_y, waypoints_[0].x - cur_x);
        add_turn(target_heading);
        add_drive(waypoints_[0].x, waypoints_[0].y);
        
        // Pre-align the robot to match the exact heading it will have when it finishes a lap 
        // (the heading from the last waypoint to the first waypoint) so the loop transition is seamless.
        size_t last_wp = waypoints_.size() - 1;
        double final_lap_heading = std::atan2(waypoints_[0].y - waypoints_[last_wp].y, waypoints_[0].x - waypoints_[last_wp].x);
        add_turn(final_lap_heading);

        circuit_start_index_ = path_.size();

        for (size_t i = 0; i < waypoints_.size(); ++i) {
            size_t next_i = (i + 1) % waypoints_.size();
            double th = std::atan2(waypoints_[next_i].y - waypoints_[i].y, waypoints_[next_i].x - waypoints_[i].x);
            add_turn(th);
            add_drive(waypoints_[next_i].x, waypoints_[next_i].y);
        }
        
        path_built_ = true;
        RCLCPP_INFO(this->get_logger(), "Dynamic path generated successfully!");
    }

    void get_ideal_pose(size_t idx, double elapsed, double &x, double &y, double &theta) {
        if (idx >= path_.size()) return;
        double frac = std::clamp(elapsed / path_[idx].duration, 0.0, 1.0);
        const auto& step = path_[idx];
        
        x = step.start_x + (step.end_x - step.start_x) * frac;
        y = step.start_y + (step.end_y - step.start_y) * frac;
        theta = step.start_theta + (step.end_theta - step.start_theta) * frac;
    }

    void build_recovery_path() {
        recovery_path_.clear();
        recovery_index_ = 0;
        recovery_elapsed_ = 0.0;

        double target_x, target_y, target_theta;
        get_ideal_pose(saved_step_index_, saved_step_elapsed_, target_x, target_y, target_theta);

        double dx = target_x - current_pose_.x;
        double dy = target_y - current_pose_.y;
        double dist = std::hypot(dx, dy);
        double current_heading = current_pose_.theta;

        if (dist > 0.001) {
            double angle_to_target = std::atan2(dy, dx);
            double turn1 = normalize_angle(angle_to_target - current_heading);

            if (std::abs(turn1) > 1e-5) {
                int ticks_t1 = std::max(1, static_cast<int>(std::round(std::abs(turn1) / 1.5 / 0.016)));
                double dur_t1 = ticks_t1 * 0.016;
                recovery_path_.push_back({PathStep::TURN, 0.0, turn1 / dur_t1, dur_t1, 0,0,0,0,0,0});
                recovery_path_.push_back({PathStep::WAIT, 0.0, 0.0, 0.016, 0,0,0,0,0,0}); 
            }

            int ticks_d = std::max(1, static_cast<int>(std::round(dist / 2.0 / 0.016)));
            double dur_d = ticks_d * 0.016;
            recovery_path_.push_back({PathStep::DRIVE, dist / dur_d, 0.0, dur_d, 0,0,0,0,0,0});
            recovery_path_.push_back({PathStep::WAIT, 0.0, 0.0, 0.016, 0,0,0,0,0,0}); 

            current_heading = angle_to_target;
        }

        double turn2 = normalize_angle(target_theta - current_heading);
        if (std::abs(turn2) > 1e-5) {
            int ticks_t2 = std::max(1, static_cast<int>(std::round(std::abs(turn2) / 1.5 / 0.016)));
            double dur_t2 = ticks_t2 * 0.016;
            recovery_path_.push_back({PathStep::TURN, 0.0, turn2 / dur_t2, dur_t2, 0,0,0,0,0,0});
            recovery_path_.push_back({PathStep::WAIT, 0.0, 0.0, 0.016, 0,0,0,0,0,0}); 
        }
    }

    double compute_synced_elapsed(size_t idx, const turtlesim::msg::Pose &pose) {
        const auto& step = path_[idx];
        if (step.type == PathStep::DRIVE) {
            double dist_remaining = std::hypot(step.end_x - pose.x, step.end_y - pose.y);
            int ticks_rem = std::clamp(static_cast<int>(std::round(dist_remaining / step.linear_x / 0.016)), 0, static_cast<int>(step.duration / 0.016));
            return step.duration - (ticks_rem * 0.016);
        }
        return saved_step_elapsed_;
    }

    void override_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && mode_ == DriverMode::AUTO) {
            saved_pose_ = current_pose_;
            saved_step_index_ = step_index_;
            saved_step_elapsed_ = step_elapsed_;
            brake_ticks_ = 0;
            mode_ = DriverMode::BRAKING;
            RCLCPP_INFO(this->get_logger(), "Manual Override Active | Braking before yielding...");
        } 
        else if (!msg->data && mode_ == DriverMode::MANUAL) {
            build_recovery_path();
            mode_ = DriverMode::RECOVERING;
            RCLCPP_INFO(this->get_logger(), "Manual Override Released | Returning to saved point...");
        }
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        current_pose_ = *msg;

        if (!pose_received_) {
            pose_received_ = true;
            step_elapsed_ = 0.0;
            build_dynamic_path();
            RCLCPP_INFO(this->get_logger(), "TurtleSim connected! Listening for /manual_override state.");
        }

        if (!path_built_ || path_.empty()) return;

        if (mode_ == DriverMode::BRAKING) {
            publisher_->publish(geometry_msgs::msg::Twist());
            if (++brake_ticks_ >= 3) {
                mode_ = DriverMode::MANUAL;
                RCLCPP_INFO(this->get_logger(), "Yielding cmd_vel control to manual_driver.");
            }
            return;
        }

        // If manual, completely yield the control loop
        if (mode_ == DriverMode::MANUAL) return;

        if (mode_ == DriverMode::RECOVERING) {
            if (recovery_index_ < recovery_path_.size() && 
                recovery_elapsed_ >= recovery_path_[recovery_index_].duration - 1e-6) {
                recovery_elapsed_ = 0.0;
                recovery_index_++;
            }

            if (recovery_index_ >= recovery_path_.size()) {
                mode_ = DriverMode::AUTO;
                step_index_ = saved_step_index_;
                step_elapsed_ = compute_synced_elapsed(step_index_, current_pose_);
                RCLCPP_INFO(this->get_logger(), "Back on track! Resuming circuit.");
            } else {
                auto twist = geometry_msgs::msg::Twist();
                twist.linear.x = recovery_path_[recovery_index_].linear_x;
                twist.angular.z = recovery_path_[recovery_index_].angular_z;
                publisher_->publish(twist);
                recovery_elapsed_ += 0.016;
                return;
            }
        }

        if (step_elapsed_ >= path_[step_index_].duration - 1e-6) {
            step_elapsed_ = 0.0;
            step_index_++;
            if (step_index_ >= path_.size()) step_index_ = circuit_start_index_;
        }

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = path_[step_index_].linear_x;
        twist.angular.z = path_[step_index_].angular_z;
        publisher_->publish(twist);

        step_elapsed_ += 0.016;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr override_sub_;
    
    std::vector<Point> waypoints_;
    std::vector<PathStep> path_;
    std::vector<PathStep> recovery_path_;
    
    DriverMode mode_;
    size_t circuit_start_index_ = 0;
    size_t step_index_;
    double step_elapsed_ = 0.0;

    turtlesim::msg::Pose current_pose_;
    turtlesim::msg::Pose saved_pose_;
    size_t saved_step_index_ = 0;
    double saved_step_elapsed_ = 0.0;

    size_t recovery_index_ = 0;
    double recovery_elapsed_ = 0.0;

    int brake_ticks_ = 0;
    bool pose_received_;
    bool path_built_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomDriver>());
    rclcpp::shutdown();
    return 0;
}