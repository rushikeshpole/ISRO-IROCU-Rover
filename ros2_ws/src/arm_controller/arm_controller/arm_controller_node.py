"""
ROS2 arm controller node.

Subscribes to detected object positions, runs IK, and sends
joint angle commands to the ESP32 over serial.

Topics:
  /detected_object  (geometry_msgs/PointStamped) — target in base frame
  /arm/joint_states (sensor_msgs/JointState)      — current joint angles
  /arm/status       (std_msgs/String)              — state machine status

Serial: sends comma-separated joint angles to ESP32 at 115200 baud.
"""

import rclpy
from rclpy.node import Node
import serial
import numpy as np

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../../..'))
from ik_solver.ik_solver import IKSolver, ArmConfig


class ArmState:
    IDLE = "IDLE"
    MOVING_TO_TARGET = "MOVING_TO_TARGET"
    GRASPING = "GRASPING"
    RETURNING = "RETURNING"


SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
HOME_JOINTS = np.array([0.0, 0.3, -0.5, 0.2])  # safe home position


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__("arm_controller")

        self.ik = IKSolver(ArmConfig())
        self.state = ArmState.IDLE
        self.current_joints = HOME_JOINTS.copy()

        # Serial to ESP32
        try:
            self.serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1.0)
            self.get_logger().info(f"Serial connected: {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial failed: {e}")
            self.serial = None

        # Subscribers
        self.create_subscription(
            PointStamped, "/detected_object", self.target_callback, 10
        )

        # Publishers
        self.joint_pub = self.create_publisher(JointState, "/arm/joint_states", 10)
        self.status_pub = self.create_publisher(String, "/arm/status", 10)

        # State machine timer at 10Hz
        self.create_timer(0.1, self.state_machine_tick)

        self.target_position = None
        self.get_logger().info("Arm controller ready")

    def target_callback(self, msg: PointStamped):
        """New object detection received — update target."""
        self.target_position = np.array([
            msg.point.x,
            msg.point.y,
            msg.point.z,
        ])
        self.get_logger().info(
            f"New target: ({self.target_position[0]:.3f}, "
            f"{self.target_position[1]:.3f}, "
            f"{self.target_position[2]:.3f})"
        )
        if self.state == ArmState.IDLE:
            self.state = ArmState.MOVING_TO_TARGET

    def state_machine_tick(self):
        """10Hz state machine."""
        if self.state == ArmState.MOVING_TO_TARGET:
            self._move_to_target()
        elif self.state == ArmState.GRASPING:
            self._grasp()
        elif self.state == ArmState.RETURNING:
            self._return_home()

        self._publish_status()

    def _move_to_target(self):
        if self.target_position is None:
            self.state = ArmState.IDLE
            return

        joints = self.ik.solve(
            self.target_position[0],
            self.target_position[1],
            self.target_position[2],
            end_effector_pitch=-0.3,   # approach from above
        )

        if joints is None:
            self.get_logger().warn("IK: target unreachable")
            self.state = ArmState.IDLE
            return

        self._send_joints(joints)
        self.current_joints = joints
        self.state = ArmState.GRASPING

    def _grasp(self):
        """Close gripper — send gripper close command to ESP32."""
        if self.serial:
            self.serial.write(b"GRIP_CLOSE\n")
        self.state = ArmState.RETURNING

    def _return_home(self):
        self._send_joints(HOME_JOINTS)
        self.current_joints = HOME_JOINTS.copy()
        self.target_position = None
        self.state = ArmState.IDLE

    def _send_joints(self, joints: np.ndarray):
        """Send joint angles (degrees) to ESP32 over serial."""
        if self.serial is None:
            return
        degrees = np.degrees(joints)
        cmd = ",".join(f"{d:.1f}" for d in degrees) + "\n"
        self.serial.write(cmd.encode())
        self.get_logger().debug(f"Sent: {cmd.strip()}")

        # Publish joint state
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["j1_base", "j2_shoulder", "j3_elbow", "j4_wrist"]
        msg.position = joints.tolist()
        self.joint_pub.publish(msg)

    def _publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
