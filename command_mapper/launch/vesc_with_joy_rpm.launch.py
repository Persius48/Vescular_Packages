from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     package="ros2_vesc_drv",
            #     executable="vesc_diff_drv",
            #     name="vesc_diff_drv",
            # ),
            Node(package="joy", executable="joy_node", name="joy_node"),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                parameters=[
                    {"axis_angular": 2},
                    {"axis_linear": {"x": 3}},
                    {"require_enable_button": True},
                    {"scale_linear": {"x":1.0 }},
                    {"scale_angular": {"yaw": 1.0}},
                ],
            ),
            Node(
                package="command_mapper",
                executable="rpm_control",
                name="vesc_diff_rpm_control",
            ),
        ]
    )