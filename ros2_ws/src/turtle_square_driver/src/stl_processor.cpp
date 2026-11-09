#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <vector>
#include "stl_driver.h"

class STLProcessor : public rclcpp::Node {
public:
    STLProcessor() : Node("stl_processor"), was_in_zone_(false), current_sim_time_(0.0) {
        
        // 1. Load Waypoints
        this->declare_parameter("waypoints_x", std::vector<double>{});
        this->declare_parameter("waypoints_y", std::vector<double>{});
        waypoints_x_ = this->get_parameter("waypoints_x").as_double_array();
        waypoints_y_ = this->get_parameter("waypoints_y").as_double_array();

        // 2. Load Dynamic Formulas
        this->declare_parameter("phi_names", std::vector<std::string>{});
        this->declare_parameter("phi_formulas", std::vector<std::string>{});
        this->declare_parameter("phi_lookaheads", std::vector<double>{});

        phi_names_ = this->get_parameter("phi_names").as_string_array();
        std::vector<std::string> formulas = this->get_parameter("phi_formulas").as_string_array();
        phi_lookaheads_ = this->get_parameter("phi_lookaheads").as_double_array();

        // 3. Dynamically Construct the STLRom Base Logic
        std::string base_logic = "signal x, y\n";
        std::string in_bounds_def = "in_bounds := ";
        
        for (size_t i = 0; i < waypoints_x_.size(); ++i) {
            double min_x = waypoints_x_[i] - 0.5;
            double max_x = waypoints_x_[i] + 0.5;
            double min_y = waypoints_y_[i] - 0.5;
            double max_y = waypoints_y_[i] + 0.5;
            
            base_logic += "z" + std::to_string(i) + " := (x[t] > " + std::to_string(min_x) + 
                          ") and (x[t] < " + std::to_string(max_x) + 
                          ") and (y[t] > " + std::to_string(min_y) + 
                          ") and (y[t] < " + std::to_string(max_y) + ")\n";
            
            if (i > 0) in_bounds_def += " or ";
            in_bounds_def += "z" + std::to_string(i);
        }
        base_logic += in_bounds_def + "\n";

        // Append all dynamic formulas
        for (const auto& f : formulas) {
            base_logic += f;
        }

        RCLCPP_INFO(this->get_logger(), "--- Constructed STL Logic ---\n%s", base_logic.c_str());
        
        // 4. Initialize STLRom engine
        driver_.parse_string(base_logic);

        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&STLProcessor::pose_callback, this, std::placeholders::_1));
            
        metrics_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/stl_metrics", 10);
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        current_sim_time_ += 0.016;

        // --- 1. Dynamic Geometric Boundary Checking ---
        bool in_zone = false;
        for (size_t i = 0; i < waypoints_x_.size(); ++i) {
            if (msg->x > waypoints_x_[i] - 0.5 && msg->x < waypoints_x_[i] + 0.5 &&
                msg->y > waypoints_y_[i] - 0.5 && msg->y < waypoints_y_[i] + 0.5) {
                in_zone = true;
                break;
            }
        }
        
        if (in_zone && !was_in_zone_) {
            RCLCPP_INFO(this->get_logger(), "IN at time: %.3fs", current_sim_time_);
            was_in_zone_ = true;
        } else if (!in_zone && was_in_zone_) {
            RCLCPP_INFO(this->get_logger(), "OUT at time: %.3fs", current_sim_time_);
            was_in_zone_ = false;
        }

        // --- 2. Temporal Logic Evaluation ---
        std::vector<double> sample = {current_sim_time_, msg->x, msg->y};
        driver_.add_sample(sample);

        // Build the dynamic data array (Index 0 = time, 1...N = satisfaction scores)
        std::vector<float> metrics_data;
        metrics_data.push_back(static_cast<float>(current_sim_time_));

        for (size_t i = 0; i < phi_names_.size(); ++i) {
            float rob = 0.0f;
            try { 
                double eval_time = std::max(0.0, current_sim_time_ - phi_lookaheads_[i]);
                rob = driver_.get_online_rob(phi_names_[i], eval_time)[0]; 
            } catch (...) {}
            metrics_data.push_back(rob > 0 ? 1.0f : 0.0f);
        }

        // --- 3. Package and Publish ---
        auto metrics_msg = std_msgs::msg::Float32MultiArray();
        metrics_msg.data = metrics_data;
        metrics_pub_->publish(metrics_msg);
    }

    std::vector<double> waypoints_x_;
    std::vector<double> waypoints_y_;
    
    std::vector<std::string> phi_names_;
    std::vector<double> phi_lookaheads_;
    
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr metrics_pub_;
    
    STLRom::STLDriver driver_;
    bool was_in_zone_;
    double current_sim_time_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<STLProcessor>());
    rclcpp::shutdown();
    return 0;
}