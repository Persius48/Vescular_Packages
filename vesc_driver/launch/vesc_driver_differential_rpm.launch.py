import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    vesc_config_right = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config.yaml'
        )
    vesc_config_left = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config_2.yaml'
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            name="config_front_right",
            default_value=vesc_config_right,
            description="VESC yaml configuration file for front right wheel.(ttyACM1)",
            ),

        DeclareLaunchArgument(
            name="config_front_left",
            default_value=vesc_config_left,
            description="VESC yaml configuration file for front left wheel.(ttyACM0)",
            ),

        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node_front_right',
            parameters=[LaunchConfiguration("config_front_right")],
            remappings=[
                ("commands/motor/speed","commands/motor/speed_right"),
                ("sensors/core","sensors/core_front_right"),
                ("sensors/imu","sensors/imu/front_right"),
                ("sensors/imu/raw","sensors/imu/raw_front_right")

            ]

        ),

        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node_front_left_',
            parameters=[LaunchConfiguration("config_front_left")],
            remappings=[
                ("commands/motor/speed","commands/motor/speed_left"),
                ("commands/motor/duty_cycle_can","commands/motor/duty_cycle_left_rear_can"),
                ("sensors/core","sensors/core_front_left"),
                ("sensors/imu","sensors/imu/front_left"),
                ("sensors/imu/raw","sensors/imu/raw_front_left")

            ]

        ),



    ])