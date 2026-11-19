#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <vector>
#include "stl_driver.h"

class STLProcessor : public rclcpp::Node {
public:
    STLProcessor() : Node("stl_processor"), was_in_zone_(false), current_sim_time_(0.0) {
        
        // 1. Load basic topics and config
        this->declare_parameter("input_topic", "/stl_input");
        this->declare_parameter("metrics_topic", "/stl_metrics");
        this->declare_parameter("signal_names", std::vector<std::string>{"x", "y", "v"});
        this->declare_parameter("formula_file", "");

        input_topic_ = this->get_parameter("input_topic").as_string();
        metrics_topic_ = this->get_parameter("metrics_topic").as_string();
        signal_names_ = this->get_parameter("signal_names").as_string_array();
        std::string formula_file = this->get_parameter("formula_file").as_string();

        // 2. Load the formulas we want to publish to the Python Plotter
        this->declare_parameter("phi_names", std::vector<std::string>{});
        this->declare_parameter("phi_lookaheads", std::vector<double>{});

        phi_names_ = this->get_parameter("phi_names").as_string_array();
        phi_lookaheads_ = this->get_parameter("phi_lookaheads").as_double_array();

        if (formula_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No formula_file provided!");
            return;
        }

        // 3. Initialize STLRom engine by directly parsing the file
        RCLCPP_INFO(this->get_logger(), "Loading STL formulas from: %s", formula_file.c_str());
        driver_.parse_file(formula_file);

        array_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            input_topic_, 10, std::bind(&STLProcessor::array_callback, this, std::placeholders::_1));
            
        metrics_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(metrics_topic_, 10);
    }

private:
    void array_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.empty()) return;

        current_sim_time_ = msg->data[0];

        // --- 1. Feed STLRom engine ---
        std::vector<double> sample;
        sample.push_back(current_sim_time_);
        for (size_t i = 0; i < signal_names_.size(); ++i) {
            if (i + 1 < msg->data.size()) {
                sample.push_back(msg->data[i + 1]);
            } else {
                sample.push_back(0.0);
            }
        }
        
        driver_.add_sample(sample);

        // --- 2. Dynamic Geometric Boundary Logging (using STLRom directly!) ---
        try {
            // Ask STLRom if the in_bounds logic from the file is true right now
            float in_bounds_rob = driver_.get_online_rob("in_bounds", current_sim_time_)[0];
            bool in_zone = (in_bounds_rob > 0.0);
            
            if (in_zone && !was_in_zone_) {
                RCLCPP_INFO(this->get_logger(), "IN at time: %.3fs", current_sim_time_);
                was_in_zone_ = true;
            } else if (!in_zone && was_in_zone_) {
                RCLCPP_INFO(this->get_logger(), "OUT at time: %.3fs", current_sim_time_);
                was_in_zone_ = false;
            }
        } catch (...) {
            // Fails silently if 'in_bounds' isn't defined in the .stl file.
        }

        // --- 3. Evaluate Configured Formulas and Publish ---
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

        auto metrics_msg = std_msgs::msg::Float32MultiArray();
        metrics_msg.data = metrics_data;
        metrics_pub_->publish(metrics_msg);
    }

    std::vector<std::string> phi_names_;
    std::vector<double> phi_lookaheads_;
    std::vector<std::string> signal_names_;
    
    std::string input_topic_;
    std::string metrics_topic_;
    
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr array_sub_;
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