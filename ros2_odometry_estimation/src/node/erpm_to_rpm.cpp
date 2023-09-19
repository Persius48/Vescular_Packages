#include "rclcpp/rclcpp.hpp"
#include "vesc_msgs/msg/vesc_state.hpp"
#include "std_msgs/msg/int64.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"


using std::placeholders::_1;

class ErpmToRpm : public rclcpp::Node // MODIFY NAME
{
public:
    ErpmToRpm() : Node("erpm_to_rpm") // MODIFY NAME
    {
        right_wheel_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
    "sensors/core_front_right", rclcpp::QoS{10}, std::bind(
      &ErpmToRpm::erpm_to_rpm_callback_right, this,
      _1));

       left_wheel_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
    "sensors/core_front_left", rclcpp::QoS{10}, std::bind(
      &ErpmToRpm::erpm_to_rpm_callback_left, this,
      _1));

        right_wheel_rpm_pub_ = create_publisher<std_msgs::msg::Int64>("right_wheel_rpm", rclcpp::QoS{10});
        left_wheel_rpm_pub_ = create_publisher<std_msgs::msg::Int64>("left_wheel_rpm", rclcpp::QoS{10});
    }

private:
     void erpm_to_rpm_callback_right(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "%ld", msg->data);
        auto speed_erpm = msg->state.speed ;
        auto new_msg = std_msgs::msg::Int64();
        new_msg.data = speed_erpm/3.5 ;  //divideed by pole pairs
        right_wheel_rpm_pub_ ->publish(new_msg);


    }

    void erpm_to_rpm_callback_left(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "%ld", msg->data);
        auto speed_erpm = msg->state.speed ;
        auto new_msg = std_msgs::msg::Int64();
        new_msg.data = speed_erpm/3.52 ;
        left_wheel_rpm_pub_ ->publish(new_msg);


    }



    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr right_wheel_sub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr right_wheel_rpm_pub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr left_wheel_sub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr left_wheel_rpm_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ErpmToRpm>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
