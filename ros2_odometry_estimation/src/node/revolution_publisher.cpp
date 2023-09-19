#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include "std_msgs/msg/float64.hpp"
#include "vesc_msgs/msg/vesc_state.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include <chrono>
#include <functional>


using std::placeholders::_1;
using namespace std::chrono_literals;
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

class Revolution : public rclcpp::Node // MODIFY NAME
{
public:
    Revolution() : Node("revolution_publisher") // MODIFY NAME
    {
        right_wheel_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
    "sensors/core_front_right", rclcpp::QoS{10}, std::bind(
      &Revolution::revolution_right_callback, this,
      _1));

       left_wheel_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
    "sensors/core_front_left", rclcpp::QoS{10}, std::bind(
      &Revolution::revolution_left_callback, this,
      _1));

    right_wheel_revolution_pub_ = create_publisher<std_msgs::msg::Float64>("right_wheel_revolution", rclcpp::QoS{10});
    left_wheel_revolution_pub_ = create_publisher<std_msgs::msg::Float64>("left_wheel_revolution", rclcpp::QoS{10});
    initial_displacement_right_ = NAN;
    initial_displacement_left_ = NAN;



    }

private:
     void revolution_right_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
    {

        TimePoint current_time = Clock::now();
        std::chrono::duration<double> dt = (current_time - previous_time_right_)/60; //converting to minutes
        double displacement_right = msg->state.displacement;
        // RCLCPP_INFO(this->get_logger(), "displacement_right: %.2f", displacement_right);
        if (isnan(initial_displacement_right_))
        {
            initial_displacement_right_ = displacement_right;
        }
        auto current_revolution = (displacement_right-initial_displacement_right_)/60;
        auto rpm_right = (current_revolution-previous_revolution_right_)/dt.count();
        previous_revolution_right_ = current_revolution;
        previous_time_right_ = current_time;
        // RCLCPP_INFO(this->get_logger(), "initial_displacement_right: %.2f", initial_displacement);
        // RCLCPP_INFO(this->get_logger(), "revolution right: %.2f", revolution);
        // RCLCPP_INFO(this->get_logger(), "current_rev: %.2f", current_revolution);
        // RCLCPP_INFO(this->get_logger(), "prev_rev: %.2f", previous_revolution_right_);
        // RCLCPP_INFO(this->get_logger(), "rpm: %.2f", rpm);
        auto new_msg = std_msgs::msg::Float64();
        new_msg.data = rpm_right;
        right_wheel_revolution_pub_ ->publish(new_msg);


    }

    void revolution_left_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
    {   
        TimePoint current_time = Clock::now();
        std::chrono::duration<double> dt = (current_time - previous_time_left_)/60;
        double displacement_left = msg->state.displacement;
        // RCLCPP_INFO(this->get_logger(), "displacement_left: %.2f", displacement_left);
        if (isnan(initial_displacement_left_))
        {
            initial_displacement_left_ = displacement_left;
        }
        auto current_revolution = (displacement_left-initial_displacement_left_)/60;
        auto rpm_left = (current_revolution-previous_revolution_left_)/dt.count();
        previous_revolution_left_ = current_revolution;
        previous_time_left_ = current_time;
        // RCLCPP_INFO(this->get_logger(), "initial_displacement_right: %.2f", initial_displacement);
        // RCLCPP_INFO(this->get_logger(), "revolution left: %.2f", revolution);
        auto new_msg = std_msgs::msg::Float64();
        new_msg.data = rpm_left;
        left_wheel_revolution_pub_ ->publish(new_msg);



    }



    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr right_wheel_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr left_wheel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_revolution_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_revolution_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> rpms_left_;
    std::vector<double> rpms_right_;
    double initial_displacement_right_; 
    double initial_displacement_left_; 
    double previous_revolution_right_ = 0; 
    double previous_revolution_left_ = 0;
    TimePoint previous_time_right_{};
    TimePoint previous_time_left_{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Revolution>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
