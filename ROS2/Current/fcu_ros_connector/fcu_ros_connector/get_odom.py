#!/usr/bin/env python3
"""
odom_to_vision_pose
-------------------
Bridges a nav_msgs/Odometry stream into MAVROS' vision-pose input.

It subscribes to an Odometry topic (pose reported in an FLU / ENU-aligned
world frame, which is what ROS produces by default) and republishes just the
pose as a geometry_msgs/PoseStamped on the MAVROS vision_pose topic.

MAVROS expects the vision pose in ENU. ROS odometry is already ENU-aligned
(x=East/Forward, y=North/Left, z=Up), so the pose is forwarded as-is by
default. If your source is actually NED, set the `input_is_ned` parameter to
true and the node will rotate it into ENU before publishing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


def ned_to_enu_position(x, y, z):
    """NED (North, East, Down) -> ENU (East, North, Up)."""
    return y, x, -z


def ned_to_enu_quaternion(qx, qy, qz, qw):
    """
    Rotate an orientation quaternion from an NED convention into ENU.
    This applies the standard fixed ENU<->NED frame transform
    (a 180 deg rotation about the North-East / (1,1,0) axis).
    """
    return qy, qx, -qz, qw


class OdomToVisionPose(Node):
    def __init__(self):
        super().__init__('odom_to_vision_pose')

        # ---- Parameters ----------------------------------------------------
        self.declare_parameter('odom_topic', '/odometry')
        self.declare_parameter('vision_pose_topic', '/mavros/vision_pose/pose')
        # If empty, the frame_id from the incoming Odometry header is kept.
        self.declare_parameter('output_frame_id', 'map')
        # Set true only if the incoming Odometry pose is expressed in NED.
        self.declare_parameter('input_is_ned', False)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.vision_pose_topic = self.get_parameter('vision_pose_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.input_is_ned = bool(self.get_parameter('input_is_ned').value)

        # ---- QoS ------------------------------------------------------------
        # Odometry is a sensor-like stream: subscribe best-effort so we don't
        # block on a lossy source. MAVROS' vision_pose subscriber is reliable,
        # so publish reliable to match it.
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- Pub / Sub ------------------------------------------------------
        self.pub = self.create_publisher(
            PoseStamped, self.vision_pose_topic, pub_qos)
        self.sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, sub_qos)

        self.get_logger().info(
            f"Bridging '{self.odom_topic}' (Odometry) -> "
            f"'{self.vision_pose_topic}' (PoseStamped). "
            f"input_is_ned={self.input_is_ned}"
        )

    def odom_callback(self, msg: Odometry):
        out = PoseStamped()

        # Preserve source timestamp so MAVROS/EKF fusion sees correct timing.
        out.header.stamp = msg.header.stamp
        out.header.frame_id = (
            self.output_frame_id if self.output_frame_id else msg.header.frame_id
        )

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation

        if self.input_is_ned:
            x, y, z = ned_to_enu_position(p.x, p.y, p.z)
            qx, qy, qz, qw = ned_to_enu_quaternion(o.x, o.y, o.z, o.w)
            out.pose.position.x = x
            out.pose.position.y = y
            out.pose.position.z = z
            out.pose.orientation.x = qx
            out.pose.orientation.y = qy
            out.pose.orientation.z = qz
            out.pose.orientation.w = qw
        else:
            # FLU / ENU-aligned: forward the pose unchanged.
            out.pose = msg.pose.pose

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToVisionPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
