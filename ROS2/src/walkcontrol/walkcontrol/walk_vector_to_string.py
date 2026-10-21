#!/usr/bin/env python3
"""ROS2: WalkVector_v1 [vx, vy, 0, 0, 0, absolute_yaw_rad] -> String.

X is forward, Y is left. Output angles are relative, integer degrees.
One incoming vector produces one string; no inference or step timer here.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String


def quaternion_yaw(q):
    """Yaw in radians, in the same world frame as the model's yaw_target."""
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("Invalid pose quaternion")
    x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def vector_to_command(data, yaw_now, stop_command=""):
    """Decode the model's vector. Return None for hold without a stop string."""
    values = [float(v) for v in data]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError("Expected six finite values: [vx, vy, 0, 0, 0, yaw_target]")
    if not math.isfinite(yaw_now):
        raise ValueError("Invalid current yaw")
    if any(abs(v) > 1e-6 for v in values[2:5]):
        raise ValueError("Walking requires vz, roll and pitch commands to be zero")

    vx, vy, yaw_target = values[0], values[1], values[5]
    delta = yaw_target - yaw_now
    degrees = math.degrees(math.atan2(math.sin(delta), math.cos(delta)))
    # Match the model's 1-degree deadband, with tolerance for Float32 rounding.
    angle = 0 if abs(degrees) < 1.0 - 1e-4 else int(math.floor(abs(degrees) + 0.5))
    if degrees < 0:
        angle = -angle

    moving = max(abs(vx), abs(vy)) > 1e-4
    if abs(angle) > (15 if moving else 60):
        raise ValueError("Turn exceeds the model's limit: 15 moving / 60 in place")
    if not moving:
        return f"turnAuto_{-angle}" if angle else (stop_command or None)

    # The current model uses one translation axis. X wins a tie for other inputs.
    if abs(vx) >= abs(vy):
        direction = "forwardAuto" if vx > 0 else "backwardAuto"
    else:
        direction = "leftAuto" if vy > 0 else "rightAuto"
    return f"{direction}_{angle}"


class WalkVectorToString(Node):
    def __init__(self):
        super().__init__("walk_vector_to_string")
        defaults = {
            "input_topic": "/walk/control", "output_topic": "/morphogear_sudo_cmd",
            "pose_topic": "/mavros/vision_pose/pose", "odom_topic": "",
            "pose_timeout_s": 0.5, "stop_command": "",
        }
        self.p = {key: self.declare_parameter(key, value).value
                  for key, value in defaults.items()}
        if self.p["pose_timeout_s"] <= 0:
            raise ValueError("pose_timeout_s must be positive")
        self.yaw, self.pose_time = None, -math.inf
        self.pub = self.create_publisher(String, self.p["output_topic"], 10)
        self.create_subscription(Float32MultiArray, self.p["input_topic"], self.on_vector, 10)
        if self.p["odom_topic"]:
            self.create_subscription(Odometry, self.p["odom_topic"], self.on_odom,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(PoseStamped, self.p["pose_topic"], self.on_pose,
                                     qos_profile_sensor_data)
        self.get_logger().info(f"{self.p['input_topic']} -> {self.p['output_topic']}")

    def update_yaw(self, q):
        try:
            self.yaw = quaternion_yaw(q)
            self.pose_time = time.monotonic()
        except ValueError as error:
            self.get_logger().warning(str(error))

    def on_pose(self, msg):
        self.update_yaw(msg.pose.orientation)

    def on_odom(self, msg):
        self.update_yaw(msg.pose.pose.orientation)

    def on_vector(self, msg):
        if self.yaw is None or time.monotonic() - self.pose_time > self.p["pose_timeout_s"]:
            self.get_logger().warning("Vector skipped: waiting for a fresh pose")
            return
        try:
            command = vector_to_command(msg.data, self.yaw, self.p["stop_command"])
        except (ValueError, TypeError, OverflowError) as error:
            self.get_logger().warning(f"Vector skipped: {error}")
            return
        if command is not None:
            self.pub.publish(String(data=command))


def main(args=None):
    rclpy.init(args=args)
    node = WalkVectorToString()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
