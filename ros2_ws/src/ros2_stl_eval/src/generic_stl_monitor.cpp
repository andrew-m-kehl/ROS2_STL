#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "stl_driver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

class GenericSTLMonitor : public rclcpp::Node
{
public:
    GenericSTLMonitor()
        : Node("stl_monitor_node"){

        // 1. Declare ROS parameters
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/stl_input");
        metrics_topic_ = this->declare_parameter<std::string>("metrics_topic", "/stl_metrics");
        formula_file_ = this->declare_parameter<std::string>("formula_file", "");
        signal_names_ = this->declare_parameter<std::vector<std::string>>("signal_names", std::vector<std::string>{"x", "y", "v"});
        phi_names_ = this->declare_parameter<std::vector<std::string>>("phi_names", std::vector<std::string>{});
        phi_lookaheads_ = this->declare_parameter<std::vector<double>>("phi_lookaheads", std::vector<double>{});
        
        publish_rate_ = this->declare_parameter<double>("publish_rate", 0.0);
        bench_log_interval_ = this->declare_parameter<int>("bench_log_interval", 500);

        // 2. Validate parameters
        if (formula_file_.empty()) {
            throw std::runtime_error("Required parameter 'formula_file' was not provided.");
            return;
        }

        if (signal_names_.empty()) {
            throw std::runtime_error("Parameter 'signal_names' cannot be empty.");
            return;
        }

        if (phi_names_.empty()) {
            throw std::runtime_error("Parameter 'phi_names' cannot be empty.");
            return;
        }

        if (phi_names_.size() != phi_lookaheads_.size()) {
            throw std::runtime_error("phi_names and phi_lookaheads must have the same length.");
            return;
        }

        if (publish_rate_ < 0.0) {
            throw std::runtime_error("publish_rate cannot be negative.");
            return;
        }

        if (bench_log_interval_ < 0) {
            throw std::runtime_error("bench_log_interval cannot be negative.");
            return;
        }

        for (double lookahead : phi_lookaheads_) {
            if (!std::isfinite(lookahead) || lookahead < 0.0) {
                throw std::runtime_error("All phi_lookaheads must be finite and non-negative.");
                return;
            }
        }

        // The largest lookahead.
        max_lookahead_ = *std::max_element(
            phi_lookaheads_.begin(), phi_lookaheads_.end());

        // 3. Initialize STLRom
        RCLCPP_INFO(this->get_logger(), "Loading STL formulas from: %s", formula_file_.c_str());
        if (!stl_driver_.parse_file(formula_file_)) {
            throw std::runtime_error("STLRom failed to parse formula file: " + formula_file_);
        }

        RCLCPP_INFO(this->get_logger(), "Successfully loaded STL formula file.");

        // 4. Create publisher and subscriber
        metrics_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(metrics_topic_, 10);
        input_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                input_topic_, 10, std::bind(&GenericSTLMonitor::input_callback, this, std::placeholders::_1));

        // 5. Display configuration
        RCLCPP_INFO(this->get_logger(), "*----------------------------------------*");
        RCLCPP_INFO(this->get_logger(), "Generic STL Monitor initialized");
        RCLCPP_INFO(this->get_logger(), "Input topic:  %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", metrics_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Formula file: %s", formula_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Number of signals: %zu", signal_names_.size());
        RCLCPP_INFO(this->get_logger(), "Number of formulas: %zu", phi_names_.size());

        for (size_t i = 0; i < phi_names_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  %s (lookahead: %.2f seconds)", phi_names_[i].c_str(), phi_lookaheads_[i]);
        }

        RCLCPP_INFO(this->get_logger(), "Maximum lookahead: %.2f seconds", max_lookahead_);
        RCLCPP_INFO(this->get_logger(), "*----------------------------------------*");
    }

    ~GenericSTLMonitor() override
    {
        print_final_statistics();
    }

private:
    // Configuration
    std::string input_topic_;
    std::string metrics_topic_;
    std::string formula_file_;
    std::vector<std::string> signal_names_;
    std::vector<std::string> phi_names_;
    std::vector<double> phi_lookaheads_;
    double publish_rate_{0.0};
    double max_lookahead_{0.0};
    int bench_log_interval_{500};

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
        input_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        metrics_pub_;

    // STLRom
    STLRom::STLDriver stl_driver_;

    // Runtime state
    bool received_first_sample_{false};
    double first_timestamp_{0.0};
    double last_timestamp_{0.0};
    size_t sample_count_{0};
    size_t published_count_{0};
    size_t rejected_count_{0};
    std::chrono::steady_clock::time_point last_publish_time_;

    // Benchmark statistics
    double latency_sum_ms_{0.0};
    double latency_min_ms_{std::numeric_limits<double>::max()};
    double latency_max_ms_{0.0};

    // Keep a bounded history to calculate percentiles and standard deviation.
    static constexpr size_t MAX_LATENCY_HISTORY = 100000;

    std::deque<double> latency_history_;

    // Input callback
    void input_callback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        const auto callback_start = std::chrono::steady_clock::now();

        // Expected input:
        //
        // [timestamp, signal_0, signal_1, signal_2, ...]
        // For the current configuration:
        // [timestamp, x, y, v]

        const size_t expected_size = signal_names_.size() + 1;

        if (msg->data.size() != expected_size) {
            rejected_count_++;
            RCLCPP_WARN(this->get_logger(), "Rejected input: expected %zu values, received %zu.", expected_size, msg->data.size());
            return;
        }

        // Validate timestamp and signal values
        const double timestamp = msg->data[0];
        for (double value : msg->data) {
            if (!std::isfinite(value)) {
                rejected_count_++;
                RCLCPP_WARN(this->get_logger(), "Rejected input containing a non-finite value.");
                return;
            }
        }

        if (received_first_sample_ && timestamp <= last_timestamp_) {
            rejected_count_++;
            RCLCPP_WARN(this->get_logger(), "Rejected non-increasing timestamp: %.6f (previous %.6f)", timestamp, last_timestamp_);
            return;
        }

        // Initialize time reference
        if (!received_first_sample_) {
            first_timestamp_ = timestamp;
            received_first_sample_ = true;
            RCLCPP_INFO(this->get_logger(), "Received first sample at t = %.6f", timestamp);
        }

        last_timestamp_ = timestamp;
        sample_count_++;

        // Add sample to STLRom
        // STLRom expects:
        // [timestamp, signal_0, signal_1, ...]

        std::vector<double> sample(msg->data.begin(), msg->data.end());
        try {
            stl_driver_.add_sample(sample);
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "STLRom failed to add sample: %s", e.what());
            return;
        }

        // Warmup
        const double elapsed_time = timestamp - first_timestamp_;
        if (elapsed_time < max_lookahead_) {
            if (sample_count_ == 1 || sample_count_ % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), "Warming up: %.2f / %.2f seconds", elapsed_time, max_lookahead_);
            }
            return;
        }

        // Optional publication-rate limiting
        const auto now = std::chrono::steady_clock::now();

        if (publish_rate_ > 0.0 && published_count_ > 0) {
            const double elapsed_since_publish =
                std::chrono::duration<double>(now - last_publish_time_).count();

            const double minimum_interval = 1.0 / publish_rate_;
            if (elapsed_since_publish < minimum_interval) {
                return;
            }
        }

        // Evaluate all formulas
        std_msgs::msg::Float64MultiArray output;
        output.data.reserve(2 + phi_names_.size() * 3);

        // First output value: current timestamp.
        output.data.push_back(timestamp);
        std::vector<std::string> verdicts;

        for (size_t i = 0; i < phi_names_.size(); ++i) {
            const std::string &formula_name = phi_names_[i];
            const double lookahead = phi_lookaheads_[i];

            // Evaluate the formula at the requested lookback time.
            // Example:
            // timestamp = 20 seconds
            // lookahead = 12 seconds
            // evaluation time = 8 seconds

            const double evaluation_time = timestamp - lookahead;
            try {
                const std::vector<double> robustness =
                    stl_driver_.get_online_rob(
                        formula_name,
                        evaluation_time);

                if (robustness.size() < 3) {
                    RCLCPP_ERROR(this->get_logger(), "Formula '%s' returned fewer than 3 robustness values.", formula_name.c_str());
                    return;
                }

                const double estimate = robustness[0];
                const double lower_bound = robustness[1];
                const double upper_bound = robustness[2];

                // Check the result before publishing.
                if (!std::isfinite(estimate) || !std::isfinite(lower_bound) || !std::isfinite(upper_bound)) {
                    RCLCPP_ERROR(this->get_logger(), "Formula '%s' returned non-finite robustness values.", formula_name.c_str());
                    return;
                }

                // Append: estimate, lower bound, upper bound
                output.data.push_back(estimate);
                output.data.push_back(lower_bound);
                output.data.push_back(upper_bound);

                // Determine verdict.
                std::string verdict;

                if (lower_bound > 0.0) {
                    verdict = "SATISFIED";
                }
                else if (upper_bound < 0.0) {
                    verdict = "VIOLATED";
                }
                else {
                    verdict = "UNCERTAIN";
                }
                verdicts.push_back(verdict);
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "STLRom evaluation failed for '%s': %s", formula_name.c_str(), e.what());
                return;
            }
        }

        // Calculate callback latency
        const auto callback_end = std::chrono::steady_clock::now();
        const double latency_ms = std::chrono::duration<double, std::milli>(callback_end - callback_start).count();
        output.data.push_back(latency_ms);

        // Publish metrics
        metrics_pub_->publish(output);
        published_count_++;
        last_publish_time_ = std::chrono::steady_clock::now();

        // Update benchmark statistics
        update_latency_statistics(latency_ms);

        // Periodic benchmark logging
        if (bench_log_interval_ > 0 && published_count_ % static_cast<size_t>(bench_log_interval_) == 0) {
            print_benchmark(timestamp, sample, verdicts, output);
        }
    }

    // Update latency statistics
    void update_latency_statistics(double latency_ms)
    {
        latency_sum_ms_ += latency_ms;
        latency_min_ms_ = std::min(latency_min_ms_, latency_ms);
        latency_max_ms_ = std::max(latency_max_ms_, latency_ms);
        latency_history_.push_back(latency_ms);
        if (latency_history_.size() > MAX_LATENCY_HISTORY) {
            latency_history_.pop_front();
        }
    }

    // Calculate percentile
    double calculate_percentile(
        std::vector<double> values,
        double percentile) const
    {
        if (values.empty()) {
            return 0.0;
        }

        std::sort(values.begin(), values.end());
        const size_t index = static_cast<size_t>(
            std::ceil(percentile * values.size())) - 1;
        return values[std::min(index, values.size() - 1)];
    }

    // Print periodic benchmark
    void print_benchmark(
        double timestamp,
        const std::vector<double> &sample,
        const std::vector<std::string> &verdicts,
        const std_msgs::msg::Float64MultiArray &output)
    {
        std::ostringstream log;
        log << std::fixed << std::setprecision(4);
        log << "[t=" << timestamp << "s] ";
        log << "samples=" << sample_count_;
        // Print input signal values.
        log << " | ";
        for (size_t i = 0; i < signal_names_.size(); ++i) {
            log << signal_names_[i] << "=" << sample[i + 1];
            if (i + 1 < signal_names_.size()) {
                log << " ";
            }
        }

        // Print robustness values and verdicts.
        size_t output_index = 1;
        for (size_t i = 0; i < phi_names_.size(); ++i) {
            const double estimate = output.data[output_index];
            const double lower = output.data[output_index + 1];
            const double upper = output.data[output_index + 2];
            output_index += 3;
            log << " | " << phi_names_[i]
                << "=[est=" << estimate
                << ", low=" << lower
                << ", up=" << upper << "]"
                << " " << verdicts[i];
        }
        log << " | latency=" << output.data.back() << "ms";
        RCLCPP_INFO(this->get_logger(), "%s", log.str().c_str());
    }

    // Print final benchmark statistics
    void print_final_statistics()
    {
        if (published_count_ == 0) {
            RCLCPP_INFO(this->get_logger(), "No STL metrics were published.");
            return;
        }

        const double mean_latency = latency_sum_ms_ / static_cast<double>(published_count_);

        std::vector<double> values(
            latency_history_.begin(),
            latency_history_.end());

        const double p95 = calculate_percentile(values, 0.95);
        const double p99 = calculate_percentile(values, 0.99);
        double variance = 0.0;

        for (double value : values) {
            variance += (value - mean_latency) * (value - mean_latency);
        }

        // The history may be bounded, so use its size for the variance calculation.
        if (!values.empty()) {
            variance /= static_cast<double>(values.size());
        }
        const double stddev = std::sqrt(variance);
        RCLCPP_INFO(this->get_logger(), "*========================================*");
        RCLCPP_INFO(this->get_logger(), "STL Monitor Final Statistics");
        RCLCPP_INFO(this->get_logger(), "Received samples: %zu", sample_count_);
        RCLCPP_INFO(this->get_logger(), "Published metrics: %zu", published_count_);
        RCLCPP_INFO(this->get_logger(), "Rejected samples: %zu", rejected_count_);
        RCLCPP_INFO(this->get_logger(), "Mean latency: %.4f ms", mean_latency);
        RCLCPP_INFO(this->get_logger(), "Min latency: %.4f ms", latency_min_ms_);
        RCLCPP_INFO(this->get_logger(), "Max latency: %.4f ms", latency_max_ms_);
        RCLCPP_INFO(this->get_logger(), "Std deviation: %.4f ms", stddev);
        RCLCPP_INFO(this->get_logger(), "P95 latency: %.4f ms", p95);
        RCLCPP_INFO(this->get_logger(), "P99 latency: %.4f ms", p99);
        RCLCPP_INFO(this->get_logger(), "*========================================*");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<GenericSTLMonitor>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("stl_monitor_node"), "Failed to start STL monitor: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}