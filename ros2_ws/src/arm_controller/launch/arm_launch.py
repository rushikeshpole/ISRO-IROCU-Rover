from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arm controller — IK + serial bridge to ESP32
        Node(
            package="arm_controller",
            executable="arm_controller_node",
            name="arm_controller",
            output="screen",
            parameters=[{
                "serial_port": "/dev/ttyUSB0",
                "serial_baud": 115200,
            }],
        ),

        # MediaPipe detection + 3D position estimation
        Node(
            package="arm_controller",
            executable="perception_node",
            name="perception",
            output="screen",
            parameters=[{
                "camera_index": 0,
                "score_threshold": 0.5,
                "use_depth": False,
            }],
        ),
    ])
