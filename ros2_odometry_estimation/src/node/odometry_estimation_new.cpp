
#include "rclcpp/rclcpp.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "vesc_msgs/msg/vesc_state.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include <memory>
#include <chrono>
#include <functional>
#include <numeric>
#include <cmath>
#include <limits>
#include<string>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

struct VehicleState {
  double x;
  double y;
  double yaw;
  double linear_vel;
  double angular_vel; 
};


class OdometryEstimator : public rclcpp::Node 
{
// Constructor
public:

    OdometryEstimator() : Node("odometry_publisher")
{

  // create subscribers
  right_wheel_subscriber_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
      "sensors/core_front_right", 10,
      std::bind(&OdometryEstimator::handleRightWheelInput, this, std::placeholders::_1));
  left_wheel_subscriber_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
       "sensors/core_front_left", 10,
      std::bind(&OdometryEstimator::handleLeftWheelInput, this, std::placeholders::_1));

  // create publisher and timer
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel/odometry", 10);
  publish_tf_ = declare_parameter("publish_tf", false);
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }
  timer_ = this->create_wall_timer(70ms, std::bind(&OdometryEstimator::publish, this));
}

private:

void handleRightWheelInput(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{
  // rpms_right_.push_back(rpm_right->data);
  auto speed_erpm = (msg->state.speed) ;
  double speed = (speed_erpm)/10;
  // if (std::fabs(speed) < 0.05)
  // {
  //   speed = 0.0;
  // }
  rpms_right_.push_back(speed);
}

void handleLeftWheelInput(const vesc_msgs::msg::VescStateStamped::SharedPtr msg)
{

  auto speed_erpm = (msg->state.speed );
  double speed = (speed_erpm)/10;
  // if (std::fabs(speed) <.05 )
  // {
  //   speed = 0.0;
  // }
  rpms_left_.push_back(speed);
}

VehicleState calculateNextState(double rpm_left_wheel, double rpm_right_wheel,
                                                        VehicleState& prev_state, double dt)
{
  VehicleState new_state{};
  // calculate wheel velocities
  double v_l = static_cast<double>(rpm_left_wheel) / 60 * 2 * WHEEL_RADIUS * M_PI;
  double v_r = static_cast<double>(rpm_right_wheel) / 60 * 2 * WHEEL_RADIUS * M_PI;
  double robot_linear_velocity = (v_r+v_l)/2;
  double robot_angular_velocity = (v_r-v_l)/VEHICLE_TRACK;
  double d_l = v_l* dt;
  double d_r = v_r*dt;
  double d = (d_l+d_r)/2;
  double dyaw = (d_r-d_l)/VEHICLE_TRACK;


  new_state.x = (prev_state.x+d*std::cos(prev_state.yaw+dyaw/2));
  new_state.y = (prev_state.y+d*std::sin(prev_state.yaw+dyaw/2));
  new_state.yaw = wrapAngle(prev_state.yaw+dyaw);
  new_state.linear_vel = robot_linear_velocity;
  new_state.angular_vel = robot_angular_velocity;


  return new_state;
}

double wrapAngle(double angle)
{
  while (angle >= 2 * M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle;
}

void publish()
{
  // calculate passed time since last publish
  TimePoint current_time = Clock::now();

  std::chrono::duration<double> dt = current_time - previous_time_;
  // RCLCPP_INFO(this->get_logger(), "time difference(dt): %.2f", dt.count());
  // calculate average of received rpm signals
  double rpm_left_avg = std::accumulate(rpms_left_.begin(), rpms_left_.end(), 0.0) / rpms_left_.size();
  double rpm_right_avg = std::accumulate(rpms_right_.begin(), rpms_right_.end(), 0.0) / rpms_right_.size();
  rpms_left_.clear();
  rpms_right_.clear();
  // calculate new state based on input
  VehicleState new_state = calculateNextState(rpm_left_avg, rpm_right_avg, state_, dt.count());
  RCLCPP_INFO(this->get_logger(), "current x: %.2f", new_state.x);
  RCLCPP_INFO(this->get_logger(), "current y: %.2f", new_state.y);
  RCLCPP_INFO(this->get_logger(), "current yaw: %.2f", new_state.yaw);
  RCLCPP_INFO(this->get_logger(), "current linear vel %.2f", new_state.linear_vel);
  RCLCPP_INFO(this->get_logger(), "current angular vel %.2f", new_state.angular_vel);
  // create quaternion from yaw angle
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, new_state.yaw);
  // fill message and publish
  auto message = nav_msgs::msg::Odometry();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "odom";
  message.child_frame_id = "base_footprint";
  message.pose.pose.position.x = new_state.x;
  message.pose.pose.position.y = new_state.y;
  message.pose.pose.orientation.x = quat.x();
  message.pose.pose.orientation.y = quat.y();
  message.pose.pose.orientation.z = quat.z();
  message.pose.pose.orientation.w = quat.w();
  message.twist.twist.linear.x = new_state.linear_vel;
  message.twist.twist.linear.y = 0;
  message.twist.twist.angular.z = new_state.angular_vel;
  publisher_->publish(message);
  // update internal state
  state_ = new_state;
  previous_time_ = current_time;

  //publish tf
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.header.stamp = this->get_clock()->now();

    tf.transform.translation.x = new_state.x;
    tf.transform.translation.y = new_state.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = message.pose.pose.orientation;

    // tf.transform.translation.x = 0.0;
    // tf.transform.translation.y = 0.0;
    // tf.transform.translation.z = 0.0;
    // tf.transform.rotation.x = 0;
    // tf.transform.rotation.y = 0;
    // tf.transform.rotation.z = 0;
    // tf.transform.rotation.w = 1;



    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }
}


VehicleState state_{0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> rpms_left_;
std::vector<double> rpms_right_;
const double WHEEL_RADIUS = 0.045;
const double VEHICLE_TRACK = 0.221;
bool publish_tf_;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
// rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr right_wheel_subscriber_;
// rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr left_wheel_subscriber_;
rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr right_wheel_subscriber_;
rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr left_wheel_subscriber_;
rclcpp::TimerBase::SharedPtr timer_;
TimePoint previous_time_{};

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryEstimator>());
  rclcpp::shutdown();
  return 0;
}