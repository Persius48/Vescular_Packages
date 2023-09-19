#ifndef ODOMETRY_ESTIMATION_NODE_H
#define ODOMETRY_ESTIMATION_NODE_H

#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int64.hpp>
#include <vector>

#include "vesc_msgs/msg/vesc_state.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

#include "vehicle_models.h"
#include "rclcpp/rclcpp.hpp"

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

class OdometryEstimator : public rclcpp::Node {
 public:
  OdometryEstimator();

 private:
  void handleRightWheelInput(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
  void handleLeftWheelInput(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
  // void handleLeftWheelInput(const std_msgs::msg::Int64::SharedPtr rpm_left);
  void publish();
  VehicleModelPtr vehicle_model_{nullptr};
  VehicleState state_{0.0, 0.0, 0.0};
  std::vector<double> rpms_left_;
  std::vector<double> rpms_right_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  // rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr right_wheel_subscriber_;
  // rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr left_wheel_subscriber_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr right_wheel_subscriber_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr left_wheel_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  TimePoint previous_time_{};
};

#endif  // ODOMETRY_ESTIMATION_NODE_H