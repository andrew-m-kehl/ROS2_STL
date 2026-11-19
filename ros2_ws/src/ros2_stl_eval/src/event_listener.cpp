#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <vector>

class EventListener : public rclcpp::Node
{
public:
    EventListener() : Node("event_listener")
    {
        metrics_topic_ = this->declare_parameter<std::string>("metrics_topic", "/stl_metrics");
        verdicts_topic_ = this->declare_parameter<std::string>("verdicts_topic", "/stl_verdicts");
        phi_names_ = this->declare_parameter<std::vector<std::string>>(
            "phi_names", std::vector<std::string>{});

        // -2 = uninitialized so the first valid state always logs
        last_states_.assign(phi_names_.size(), -2);
        verdicts_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(verdicts_topic_, 10);
        metrics_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(metrics_topic_, 10,
            std::bind(&EventListener::metrics_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Event Listener bridging %s -> %s (%zu formulas)",
            metrics_topic_.c_str(), verdicts_topic_.c_str(), phi_names_.size());
    }

private:
    static const char * state_to_str(int state)
    {
        if (state == 1)  return "SATISFIED";
        if (state == 0) return "VIOLATED";
        return "UNCERTAIN";
    }

    void metrics_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        const size_t n = phi_names_.size();
        const size_t expected = 2 + 3 * n;  // [t, (est, low, up) * n, latency_ms]
        if (msg->data.size() < expected) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Expected >= %zu values, got %zu.", expected, msg->data.size());
            return;
        }

        const double t = msg->data[0];

        std_msgs::msg::Float64MultiArray out;
        out.data.reserve(1 + n);
        out.data.push_back(t);

        for (size_t i = 0; i < n; ++i) {
            const double est = msg->data[1 + 3 * i];
            const double low = msg->data[1 + 3 * i + 1];
            const double up  = msg->data[1 + 3 * i + 2];

            int state = 0;  // UNCERTAIN
            if (low > 0.0) {
                state = 1;  // SATISFIED
            } else if (up < 0.0) {
                state = 0; // VIOLATED
            }

            if (state != last_states_[i]) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[%s] %s at t=%.3fs (est=%.4f)",
                    phi_names_[i].c_str(), state_to_str(state), t, est);
                last_states_[i] = state;
            }

            out.data.push_back(static_cast<double>(state));
        }

        verdicts_pub_->publish(out);
    }

    std::string metrics_topic_;
    std::string verdicts_topic_;
    std::vector<std::string> phi_names_;
    std::vector<int> last_states_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr verdicts_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EventListener>());
    rclcpp::shutdown();
    return 0;
}