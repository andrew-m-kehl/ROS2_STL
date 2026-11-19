/*
Generic STL Monitor (C++): robot-agnostic online STLRom monitor for ROS 2.

C++ port of generic_stl_monitor.py. It subscribes to a Float64MultiArray
published by one of the adapter nodes (odom_adapter, turtlesim_adapter, ...)
with layout

    [t, sig_0, sig_1, ..., sig_N]            (t = seconds, strictly increasing)

and evaluates an STL formula online over a sliding window using stlrom and returns a robustness score.

Inputs:
        Topic: <input_topic>      - Subscriber: std_msgs/msg/Float64MultiArray
Outputs:
        Topic: /stl_robustness    - Publisher : std_msgs/msg/Float64MultiArray
                                    [t, rob_estimate, rob_low, rob_up, latency_ms]

Parameters:
  Required:
    formula_file        (str)    Absolute path to the .stl formula file
    formula_name        (str)    Name of the formula to evaluate
    signal_names        (str)    Comma-separated names, must match .stl signal declarations
  Optional:
    input_topic         (str,    default /stl_input)  Topic to subscribe to
    window              (double, default 10.0)        Sliding window in seconds (>= formula horizon)
    params              (str,    default "")          Inline STL param overrides: "W2=5,W3=3"
    publish_rate        (double, default 0.0)         Hz, 0 = evaluate on every message
    bench_log_interval  (int,    default 500)         Print stats every N evaluations

Launch example (iRobot Create 3, with odom_adapter running):
    ros2 run ros2_stl_eval generic_stl_monitor --ros-args \
        -p formula_file:=/path/to/formulas.stl \
        -p formula_name:=phi \
        -p signal_names:=x,y,velocity \
        -p window:=2.0

ros2 run ros2_stl_eval stl_monitor --ros-args 
        -p formula_file:="/home/Andy/Desktop/CodingFiles/ROS2_STL/ros2_ws/src/ros2_stl_eval/config/parameters.stl" 
        -p formula_name:="phi1" 
        -p signal_names:="x,y,v" 
        -p window:=2.0


Note: unlike the Python node, there is no msg_type / signal_fields mode. C++
cannot resolve dotted field paths at runtime, so field extraction lives in the
adapters. This node only consumes the Float64MultiArray layout above.
*/

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <std_msgs/msg/float64_multi_array.hpp>

// stlrom's include/ dir is added with -idirafter (see CMake) because it ships a
// signal.h that must not shadow the system <signal.h>.
#include "stl_driver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <fstream>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::string trim(const std::string & s)
{
    const char * ws = " \t\r\n";
    const auto b = s.find_first_not_of(ws);
    if (b == std::string::npos) {
        return "";
    }
    const auto e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
}

std::vector<std::string> split_csv(const std::string & s)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item = trim(item);
        if (!item.empty()) {
            out.push_back(item);
        }
    }
    return out;
}

std::string join(const std::vector<std::string> & v, const std::string & sep)
{
    std::string out;
    for (size_t i = 0; i < v.size(); ++i) {
        out += (i ? sep : "") + v[i];
    }
    return out;
}

std::string fmt(const char * f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char * f, ...)
{
    char buf[512];
    va_list args;
    va_start(args, f);
    std::vsnprintf(buf, sizeof(buf), f, args);
    va_end(args);
    return std::string(buf);
}

}  // namespace

class STLMonitorNode : public rclcpp::Node {
public:
    STLMonitorNode() : Node("stl_monitor_node")
    {
        // ── Parameters ───────────────────────────────────────────────────────
        const auto formula_file = this->declare_parameter<std::string>("formula_file", "");
        formula_name_           = this->declare_parameter<std::string>("formula_name", "");
        const auto names_str    = this->declare_parameter<std::string>("signal_names", "");
        const auto input_topic  = this->declare_parameter<std::string>("input_topic", "/stl_input");
        window_                 = this->declare_parameter<double>("window", 10.0);
        const auto params_str   = this->declare_parameter<std::string>("params", "");
        publish_rate_           = this->declare_parameter<double>("publish_rate", 0.0);
        log_interval_           = this->declare_parameter<int64_t>("bench_log_interval", 500);

        // ── Validation ───────────────────────────────────────────────────────
        std::vector<std::string> errors;
        if (formula_file.empty()) {
            errors.push_back("formula_file is required.");
        } else if (!std::ifstream(formula_file).good()) {
            errors.push_back("Formula file not found: " + formula_file);
        }
        if (formula_name_.empty()) {
            errors.push_back("formula_name is required.");
        }
        if (names_str.empty()) {
            errors.push_back(
                "signal_names is required. Provide a comma-separated list matching the "
                "signals in your .stl file, e.g. signal_names:=\"x,y,velocity\"");
        }
        if (window_ <= 0.0) {
            errors.push_back(fmt("window must be > 0, got %f", window_));
        }
        if (log_interval_ <= 0) {
            errors.push_back(fmt("bench_log_interval must be > 0, got %ld", static_cast<long>(log_interval_)));
        }
        if (!errors.empty()) {
            for (const auto & e : errors) {
                RCLCPP_ERROR(get_logger(), "%s", e.c_str());
            }
            throw std::runtime_error("invalid parameters");
        }

        signal_names_ = split_csv(names_str);
        n_signals_    = signal_names_.size();
        if (n_signals_ == 0) {
            RCLCPP_ERROR(get_logger(), "signal_names contained no valid names.");
            throw std::runtime_error("invalid parameters");
        }

        // ── STLRom setup ─────────────────────────────────────────────────────
        if (!driver_.parse_file(formula_file)) {
            RCLCPP_ERROR(get_logger(), "Failed to parse formula file: %s", formula_file.c_str());
            throw std::runtime_error("stl parse failure");
        }
        RCLCPP_INFO(get_logger(), "Loaded formula file: %s", formula_file.c_str());

        // stlrom silently returns an empty result for unknown formula names, so
        // check up front and list what is available.
        if (driver_.formula_map.find(formula_name_) == driver_.formula_map.end()) {
            std::vector<std::string> available;
            for (const auto & kv : driver_.formula_map) {
                available.push_back(kv.first);
            }
            RCLCPP_ERROR(
                get_logger(), "Formula \"%s\" not found in %s. Available: %s",
                formula_name_.c_str(), formula_file.c_str(), join(available, ", ").c_str());
            throw std::runtime_error("unknown formula");
        }

        // Inject the ROS window size as STL parameter W so the formula horizon
        // always matches the buffer. W2/W3/W4 stay as defined in the .stl file.
        try {
            if (!driver_.parse_string(fmt("param W=%.17g", window_))) {
                RCLCPP_WARN(get_logger(), "Failed to inject window parameter W");
            }
        } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "Failed to inject window parameter W: %s", e.what());
        }

        // Inline overrides, syntax: -p params:="W2=5.0,W3=3.0"
        for (const auto & pair : split_csv(params_str)) {
            const auto eq = pair.find('=');
            if (eq == std::string::npos) {
                continue;
            }
            const std::string key = trim(pair.substr(0, eq));
            const std::string val = trim(pair.substr(eq + 1));
            try {
                if (driver_.parse_string("param " + key + "=" + val)) {
                    RCLCPP_INFO(get_logger(), "Param override: %s = %s", key.c_str(), val.c_str());
                } else {
                    RCLCPP_WARN(get_logger(), "Failed to apply param %s=%s", key.c_str(), val.c_str());
                }
            } catch (const std::exception & e) {
                RCLCPP_WARN(get_logger(), "Failed to apply param %s=%s: %s", key.c_str(), val.c_str(), e.what());
            }
        }

        // ── Internal state ───────────────────────────────────────────────────
        sample_buf_.assign(1 + n_signals_, 0.0);  // [t, sig_0, ..., sig_N], reused every sample

        out_msg_.data.assign(5, 0.0);
        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/stl_robustness", 10);

        sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            input_topic, 10,
            std::bind(&STLMonitorNode::array_callback, this, std::placeholders::_1));
        RCLCPP_INFO(
            get_logger(), "Subscribed to %s [Float64MultiArray] | expected layout: [t, %s]",
            input_topic.c_str(), join(signal_names_, ", ").c_str());

        if (publish_rate_ > 0.0) {
            timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / publish_rate_)),
                std::bind(&STLMonitorNode::timer_eval, this));
            RCLCPP_INFO(get_logger(), "Evaluation Mode: Timer-driven (%.2f Hz)", publish_rate_);
        } else {
            RCLCPP_INFO(get_logger(), "Evaluation Mode: Message-driven (every incoming sample)");
        }

        RCLCPP_INFO(
            get_logger(), "STL Monitor ready | formula=\"%s\" | signals=[%s] | window=%.1fs",
            formula_name_.c_str(), join(signal_names_, ", ").c_str(), window_);
    }

    /// Rolling summary of evaluation latency. Also called once on shutdown.
    void print_bench_stats()
    {
        if (latencies_.size() < 2) {
            return;
        }
        std::vector<double> sorted(latencies_.begin(), latencies_.end());
        std::sort(sorted.begin(), sorted.end());
        const size_t n = sorted.size();

        const double mean = std::accumulate(sorted.begin(), sorted.end(), 0.0) / n;
        double sq = 0.0;
        for (double v : sorted) {
            sq += (v - mean) * (v - mean);
        }
        const double stdev = std::sqrt(sq / (n - 1));  // sample stdev, as statistics.stdev

        std::string rate = (publish_rate_ <= 0.0) ? "every msg" : fmt("%g Hz", publish_rate_);
        std::string bar;
        for (int i = 0; i < 60; ++i) {
            bar += "\u2500";
        }

        RCLCPP_INFO(get_logger(), "%s", bar.c_str());
        RCLCPP_INFO(get_logger(), "  BENCHMARK STATS \u2014 %lu evaluations", static_cast<unsigned long>(eval_count_));
        RCLCPP_INFO(get_logger(), "  Formula       : %s", formula_name_.c_str());
        RCLCPP_INFO(get_logger(), "  Window        : %gs", window_);
        RCLCPP_INFO(get_logger(), "  Signals       : %s", join(signal_names_, ", ").c_str());
        RCLCPP_INFO(get_logger(), "  Sample count  : %lu", static_cast<unsigned long>(sample_count_));
        RCLCPP_INFO(get_logger(), "  Eval rate     : %s", rate.c_str());
        RCLCPP_INFO(get_logger(), "  Latency mean  : %.4f ms", mean);
        RCLCPP_INFO(get_logger(), "  Latency min   : %.4f ms", sorted.front());
        RCLCPP_INFO(get_logger(), "  Latency max   : %.4f ms", sorted.back());
        RCLCPP_INFO(get_logger(), "  Latency stdev : %.4f ms", stdev);
        RCLCPP_INFO(get_logger(), "  Latency p95   : %.4f ms", sorted[static_cast<size_t>(n * 0.95)]);
        RCLCPP_INFO(get_logger(), "  Latency p99   : %.4f ms", sorted[static_cast<size_t>(n * 0.99)]);
        RCLCPP_INFO(get_logger(), "%s", bar.c_str());
    }

private:
    // ── Callbacks ────────────────────────────────────────────────────────────

    void array_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        const auto & data = msg->data;
        const size_t expected = 1 + n_signals_;
        if (data.size() < expected) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Received array of length %zu, expected >= %zu ([t, %s]). Skipping.",
                data.size(), expected, join(signal_names_, ", ").c_str());
            return;
        }

        // Timestamp comes from the message (the adapter already normalised it).
        // STLRom requires strictly increasing timestamps.
        const double t = data[0];
        if (t <= last_t_) {
            return;
        }
        last_t_ = t;

        std::copy(data.begin(), data.begin() + expected, sample_buf_.begin());

        driver_.add_sample(sample_buf_);  // takes a copy, so reusing the buffer is safe
        ++sample_count_;
        has_pending_ = true;

        if (publish_rate_ <= 0.0) {
            has_pending_ = false;
            evaluate(t);
        }
    }

    void timer_eval()
    {
        // Only evaluate if a new sample arrived since the last evaluation.
        if (has_pending_) {
            has_pending_ = false;
            evaluate(last_t_);
        }
    }

    // ── STL evaluation ───────────────────────────────────────────────────────

    /// Evaluates at eval_t = t - window (left edge of the sliding window).
    /// Signal values for the log snapshot are read from sample_buf_ (latest sample).
    void evaluate(double t)
    {
        // Warmup: cannot evaluate until a full window of data is available.
        if (t < window_) {
            const int sec = static_cast<int>(t);
            if (sec != last_warmup_log_sec_) {  // at most one log line per second
                last_warmup_log_sec_ = sec;
                RCLCPP_INFO(
                    get_logger(), "[WARMUP] %.1fs / %.0fs (%lu samples so far)",
                    t, window_, static_cast<unsigned long>(sample_count_));
            }
            return;
        }

        if (!warmed_up_) {
            warmed_up_ = true;
            RCLCPP_INFO(
                get_logger(), "[OK] Window full at t=%.2fs | %lu samples collected | monitoring \"%s\"",
                t, static_cast<unsigned long>(sample_count_), formula_name_.c_str());
        }

        const double eval_t = t - window_;

        // get_online_rob() returns {estimate, low_bound, up_bound} at eval_t
        std::vector<double> rob;
        const auto t0 = std::chrono::steady_clock::now();
        try {
            rob = driver_.get_online_rob(formula_name_, eval_t);
        } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "STL evaluation failed at t=%.2fs: %s", t, e.what());
            return;
        }
        const double latency_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        ++eval_count_;
        latencies_.push_back(latency_ms);
        if (latencies_.size() > kMaxLatencySamples) {
            latencies_.pop_front();
        }

        if (rob.size() < 3) {
            return;  // no robustness available yet
        }
        const double rob_estimate = rob[0];
        const double rob_low      = rob[1];
        const double rob_up       = rob[2];

        // Logging. Format is parsed by benchmark_runner.py, do not change keywords.
        if (rcutils_logging_logger_is_enabled_for(get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO)) {
            std::string verdict;
            if (rob_low > 0) {
                verdict = "SATISFIED";
            } else if (rob_up < 0) {
                verdict = "VIOLATED";
            } else {
                verdict = fmt("UNCERTAIN (est=%.4f)", rob_estimate);
            }

            std::string snapshot;
            const size_t n_snap = std::min<size_t>(n_signals_, 4);
            for (size_t i = 0; i < n_snap; ++i) {
                snapshot += (i ? " " : "") + fmt("%s=%.2f", signal_names_[i].c_str(), sample_buf_[i + 1]);
            }

            RCLCPP_INFO(
                get_logger(),
                "[t=%.2fs] samples=%lu | %s | rob=[est=%.4f, low=%.4f, up=%.4f] | %s | latency=%.3fms",
                t, static_cast<unsigned long>(sample_count_), snapshot.c_str(),
                rob_estimate, rob_low, rob_up, verdict.c_str(), latency_ms);
        }

        // Publish [t, estimate, low, up, latency_ms] (message is pre-allocated)
        out_msg_.data[0] = t;
        out_msg_.data[1] = rob_estimate;
        out_msg_.data[2] = rob_low;
        out_msg_.data[3] = rob_up;
        out_msg_.data[4] = latency_ms;
        pub_->publish(out_msg_);

        if (eval_count_ % static_cast<uint64_t>(log_interval_) == 0) {
            print_bench_stats();
        }
    }

    // ── Members ──────────────────────────────────────────────────────────────

    // Config
    std::string formula_name_;
    std::vector<std::string> signal_names_;
    size_t n_signals_ = 0;
    double window_ = 10.0;
    double publish_rate_ = 0.0;
    int64_t log_interval_ = 500;

    // STLRom
    STLRom::STLDriver driver_;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Float64MultiArray out_msg_;

    // Sample state
    std::vector<double> sample_buf_;   // [t, sig_0, ..., sig_N]
    double last_t_ = -1.0;             // -1 so the first sample always passes
    uint64_t sample_count_ = 0;
    bool has_pending_ = false;
    bool warmed_up_ = false;
    int last_warmup_log_sec_ = -1;

    // Benchmarking (bounded so a long run does not grow without limit)
    static constexpr size_t kMaxLatencySamples = 100000;
    std::deque<double> latencies_;
    uint64_t eval_count_ = 0;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        auto node = std::make_shared<STLMonitorNode>();
        rclcpp::spin(node);
        node->print_bench_stats();
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("stl_monitor_node"), "Startup failed: %s", e.what());
        rc = 1;
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return rc;
}
