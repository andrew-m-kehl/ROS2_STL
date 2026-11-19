/*Manual Driver: A standalone teleoperation node that allows manual control of the robot using W/S/A/D. 

Inputs: 
        Keyboard

Outputs: 
        Topic: /turtle1/cmd_vel
*/


#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class ManualDriver : public rclcpp::Node {
public:
    ManualDriver() : Node("manual_driver"), is_manual_(false), running_(true) {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        override_pub_ = this->create_publisher<std_msgs::msg::Bool>("/manual_override", 10);
        
        keyboard_thread_ = std::thread(&ManualDriver::keyboard_listener, this);
        RCLCPP_INFO(this->get_logger(), "Manual Driver Ready. Press SPACE to toggle override. Use W/A/S/D to drive.");
    }

    ~ManualDriver() {
        running_ = false;
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
    }

private:
    void keyboard_listener() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

        while (running_ && rclcpp::ok()) {
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                if (c == ' ') {
                    is_manual_ = !is_manual_;
                    auto msg = std_msgs::msg::Bool();
                    msg.data = is_manual_;
                    override_pub_->publish(msg);
                } 
                else if (is_manual_) {
                    auto twist = geometry_msgs::msg::Twist();
                    if (c == 'w') twist.linear.x = 2.0;
                    else if (c == 's') twist.linear.x = -2.0;
                    else if (c == 'a') twist.angular.z = 2.0;
                    else if (c == 'd') twist.angular.z = -2.0;
                    
                    if (twist.linear.x != 0.0 || twist.angular.z != 0.0) {
                        cmd_pub_->publish(twist);
                    }
                }
            }
            std::this_thread::sleep_for(10ms);
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr override_pub_;
    std::thread keyboard_thread_;
    bool is_manual_;
    bool running_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualDriver>());
    rclcpp::shutdown();
    return 0;
}