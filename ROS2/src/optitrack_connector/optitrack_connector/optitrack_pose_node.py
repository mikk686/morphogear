#!/usr/bin/env python3
"""OptiTrack/Motive (NatNet) -> ROS 2 PoseStamped publisher.

Drop-in replacement for the previous pyvicon node:

    before: VICON  -> pyvicon_datastream.ObjectTracker -> PoseStamped
    now:    Motive -> NatNetClient rigid_body_listener -> PoseStamped

Same topic, same message type, same 40 Hz timer, same frame_id "/map".
Only the tracker backend and the coordinate math differ.
"""

from __future__ import annotations

import math
import socket
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from .NatNetClient import NatNetClient


# ---------------------------------------------------------------------------
# Coordinate transform: raw Motive frame -> ROS ENU/FLU
# ---------------------------------------------------------------------------
# Raw Motive frame of this rigid body:
#   +X_opt = left
#   +Y_opt = up
#   +Z_opt = forward
#
# ROS world frame (map / ENU):
#   +X_ros = forward
#   +Y_ros = left
#   +Z_ros = up
#
# So:
#   X_ros = Z_opt
#   Y_ros = X_opt
#   Z_ros = Y_opt
R_ROS_FROM_OPT = np.array(
    [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)


def detect_local_ip(remote_ip: str) -> str:
    """Local IPv4 address the kernel would use to reach Motive."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((remote_ip, 1510))
        return sock.getsockname()[0]
    finally:
        sock.close()


def quaternion_to_rotation_matrix(qx, qy, qz, qw) -> np.ndarray:
    """Unit quaternion -> 3x3 rotation matrix."""
    q = np.array([qx, qy, qz, qw], dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.eye(3, dtype=float)

    q /= norm
    x, y, z, w = q

    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotation_matrix_to_quaternion(R) -> np.ndarray:
    """3x3 rotation matrix -> quaternion [x, y, z, w].

    Uses the largest-diagonal (Shepperd) branch, which stays numerically
    stable near 180 degrees where the naive trace formula blows up.
    """
    R = np.asarray(R, dtype=float)
    trace = float(np.trace(R))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def optitrack_pose_to_ros(position, rotation):
    """Raw NatNet pose -> ROS ENU/FLU position (m) and quaternion [x,y,z,w].

    Position is rotated with R_ROS_FROM_OPT. Orientation uses the matching
    similarity transform so the body axes get relabelled exactly like the
    world axes:

        R_ros = R_ROS_FROM_OPT @ R_opt @ R_ROS_FROM_OPT.T

    NatNet already reports metres, so there is no /1000 here, and orientation
    arrives as a quaternion already - no Euler conversion needed.
    """
    p_opt = np.asarray(position, dtype=float)
    qx, qy, qz, qw = (float(v) for v in rotation)

    p_ros = R_ROS_FROM_OPT @ p_opt

    R_opt = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    R_ros = R_ROS_FROM_OPT @ R_opt @ R_ROS_FROM_OPT.T
    q_ros = rotation_matrix_to_quaternion(R_ros)

    return p_ros, q_ros


class OptiTrackPose(Node):
    """Receive NatNet rigid body frames, publish PoseStamped at 40 Hz."""

    def __init__(self):
        super().__init__("optitrack_connector")

        # ---- Parameters ----------------------------------------------------
        self.declare_parameter("motive_ip", "192.168.50.209")
        self.declare_parameter("local_ip", "")          # empty -> auto-detect
        self.declare_parameter("rigid_body_id", 1)
        self.declare_parameter("topic_name", "/mavros/vision_pose/pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("tracking_timeout", 0.20)

        self.motive_ip = self.get_parameter("motive_ip").value
        self.local_ip = (
            self.get_parameter("local_ip").value or detect_local_ip(self.motive_ip)
        )
        self.rigid_body_id = int(self.get_parameter("rigid_body_id").value)
        topic_name = self.get_parameter("topic_name").value
        self.frame_id = self.get_parameter("frame_id").value
        rate = float(self.get_parameter("rate").value)
        self.tracking_timeout_s = float(self.get_parameter("tracking_timeout").value)

        if rate < 4.0:
            raise ValueError("Output rate must be >= 4 Hz for ArduPilot ExternalNav")

        # ---- Publisher (same as the pyvicon node) ---------------------------
        self.publisher_ = self.create_publisher(PoseStamped, topic_name, 10)

        # ---- Shared state, written by NatNet thread, read by ROS timer ------
        self._lock = threading.Lock()
        self._position = None          # np.array([x, y, z]) metres, ROS frame
        self._orientation = None       # np.array([x, y, z, w]), ROS frame
        self._last_update = 0.0
        self._pose_received = False
        self._stale_reported = False
        self._rx_count = 0
        self._tx_count = 0
        self._invalid_count = 0

        # ---- NatNet client --------------------------------------------------
        self.natnet_client = NatNetClient()
        if hasattr(self.natnet_client, "set_print_level"):
            self.natnet_client.set_print_level(0)

        if hasattr(self.natnet_client, "set_server_address"):
            self.natnet_client.set_server_address(self.motive_ip)
            self.natnet_client.set_client_address(self.local_ip)
            self.natnet_client.set_use_multicast(True)
            self.natnet_client.rigid_body_listener = self._rigid_body_callback
        else:
            # Compatibility with older OptiTrack NatNet sample clients.
            self.natnet_client.serverIPAddress = self.motive_ip
            self.natnet_client.localIPAddress = self.local_ip
            self.natnet_client.use_multicast = True
            self.natnet_client.rigidBodyListener = self._rigid_body_callback

        self.get_logger().info(f"Connecting to Motive at {self.motive_ip}")
        self.get_logger().info(f"Local IP       : {self.local_ip}")
        self.get_logger().info(f"Rigid body ID  : {self.rigid_body_id}")
        self.get_logger().info(f"Publishing on  : {topic_name}")
        self.get_logger().info(f"Output rate    : {rate:.1f} Hz")

        # NatNet's run() blocks, so give it its own thread.
        self._natnet_thread = threading.Thread(
            target=self._natnet_worker, daemon=True
        )
        self._natnet_thread.start()

        # ---- Publish timer, mirrors the pyvicon 0.025 s timer ---------------
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

    # ------------------------------------------------------------------
    # NatNet thread
    # ------------------------------------------------------------------
    def _natnet_worker(self):
        try:
            success = self.natnet_client.run()
            if success is False:
                self.get_logger().error("NatNet client failed to start")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"NatNet client stopped: {exc}")

    @staticmethod
    def _finite(values) -> bool:
        return all(math.isfinite(float(v)) for v in values)

    def _rigid_body_callback(self, rigid_body_id, position, rotation):
        """Called by NatNet for every rigid body frame it receives."""
        if int(rigid_body_id) != self.rigid_body_id:
            return

        self._rx_count += 1

        if (
            len(position) != 3
            or len(rotation) != 4
            or not self._finite((*position, *rotation))
        ):
            self._invalid_count += 1
            return

        try:
            p_ros, q_ros = optitrack_pose_to_ros(position, rotation)
        except Exception:  # noqa: BLE001
            self._invalid_count += 1
            return

        if not self._finite((*p_ros, *q_ros)):
            self._invalid_count += 1
            return

        with self._lock:
            self._position = p_ros
            self._orientation = q_ros
            self._last_update = time.monotonic()
            self._pose_received = True
            self._stale_reported = False

    # ------------------------------------------------------------------
    # ROS publish timer
    # ------------------------------------------------------------------
    def timer_callback(self):
        now = time.monotonic()
        with self._lock:
            have_pose = self._pose_received
            age = now - self._last_update if have_pose else math.inf
            position = None if self._position is None else self._position.copy()
            orientation = (
                None if self._orientation is None else self._orientation.copy()
            )

        if position is None or orientation is None:
            return

        print(position, orientation)
        if age > self.tracking_timeout_s:
            if not self._stale_reported:
                self.get_logger().warn(
                    f"OptiTrack pose stale ({age:.3f} s) - not publishing"
                )
                self._stale_reported = True
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])

        msg.pose.orientation.x = float(orientation[0])
        msg.pose.orientation.y = float(orientation[1])
        msg.pose.orientation.z = float(orientation[2])
        msg.pose.orientation.w = float(orientation[3])

        self.publisher_.publish(msg)
        self._tx_count += 1

        if self._tx_count % 200 == 0:
            self.get_logger().info(
                f"POS=({position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f}) m  "
                f"rx={self._rx_count} tx={self._tx_count} "
                f"invalid={self._invalid_count}"
            )

    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            if hasattr(self.natnet_client, "shutdown"):
                self.natnet_client.shutdown()
            elif hasattr(self.natnet_client, "stop"):
                self.natnet_client.stop()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"NatNet shutdown warning: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = OptiTrackPose()

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
