#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
from geometry_msgs.msg import Twist

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__("rpm_control") # MODIFY NAME
        self.wheel_radius = .045
        self.wheel_distance = .221

        self.right_wheel_vel_publisher = self.create_publisher(Float64, "commands/motor/speed_right", 10)
        self.left_wheel_vel_publisher = self.create_publisher(Float64, "commands/motor/speed_left", 10)
        self.cmd_vel_subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        v_right_rpm = self.mps_to_rpm(linear_velocity - angular_velocity * self.wheel_distance / 2)
        v_left_rpm = self.mps_to_rpm(linear_velocity + angular_velocity * self.wheel_distance / 2)

        right_wheel_velocity_msg = Float64()
        right_wheel_velocity_msg.data  = v_right_rpm*10 #multiplying with pole pair

        left_wheel_velocity_msg = Float64()
        left_wheel_velocity_msg.data  = v_left_rpm*10 #multiplying with pole pair

        self.right_wheel_vel_publisher.publish(right_wheel_velocity_msg)
        self.left_wheel_vel_publisher.publish(left_wheel_velocity_msg)

    def mps_to_rpm(self, mps):
        return (mps * 60) / (2 * math.pi * self.wheel_radius)



def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveController() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()