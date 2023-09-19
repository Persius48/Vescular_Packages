#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include "vesc_msgs/msg/vesc_state.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include <chrono>
#include <functional>


using std::placeholders::_1;
using namespace std::chrono_literals;
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

class AvgErpm : public rclcpp::Node // MODIFY NAME
{
public:
    AvgErpm() : Node("average_erpm") // MODIFY NAME
    {
        right_wheel_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
    "sensors/core_front_right", rclcpp::QoS{10}, std::bind(
      &AvgErpm::avg_erpm_right_callback, this,
      _1));

       left_wheel_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
    "sensors/core_front_left", rclcpp::QoS{10}, std::bind(
      &AvgErpm::avg_erpm_left_callback, this,
      _1));

      initial_displacement = NAN;

        // publisher_ = this->create_publisher<my_robot_interfaces::msg::WheelVelocities>("avg_erpm", 10);
        // timer_ = this->create_wall_timer(100ms, std::bind(&AvgErpm::publish, this));
    }

private:
     void avg_erpm_right_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "%ld", msg->data);
        auto speed_erpm = msg->state.speed ;
        double speed = (speed_erpm)/10;
        if (std::fabs(speed) < 0.05)
        {
            speed = 0.0;
        }
        RCLCPP_INFO(this->get_logger(), "Right ERPM : %.2f", speed);
        rpms_right_.push_back(speed);




    }

    void avg_erpm_left_callback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "%ld", msg->data);
        // auto speed_erpm = msg->state.speed ;
        // double speed = (speed_erpm)/10;
        // if (std::fabs(speed) < 0.05)
        // {
        //     speed = 0.0;
        // }
        // rpms_left_.push_back(speed);
        // RCLCPP_INFO(this->get_logger(), "pid_pos_right: %.2f", pid_pos_left);


    }

    // void publish()
    // {
    //      // calculate passed time since last publish
    //     TimePoint current_time = Clock::now();
    //     std::chrono::duration<double> dt = current_time - previous_time_;
    //     // calculate average of received rpm signals
    //     double rpm_left_avg = std::accumulate(rpms_left_.begin(), rpms_left_.end(), 0.0) / rpms_left_.size();
    //     double rpm_right_avg =
    //         std::accumulate(rpms_right_.begin(), rpms_right_.end(), 0.0) / rpms_right_.size();

    //     // auto message = my_robot_interfaces::msg::WheelVelocities();
    //     RCLCPP_INFO(this->get_logger(), "Right RPM average: %.2f", rpm_right_avg);
    //     RCLCPP_INFO(this->get_logger(), "Left RPM average: %.2f", rpm_left_avg);


    //     rpms_left_.clear();
    //     rpms_right_.clear();
    // }


    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr right_wheel_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr left_wheel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> rpms_left_;
    std::vector<double> rpms_right_;
    double initial_displacement; 
    TimePoint previous_time_{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AvgErpm>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
