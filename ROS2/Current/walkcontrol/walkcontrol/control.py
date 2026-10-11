#!/usr/bin/env python3
"""ROS2: depth + pose + waypoint -> the model's vector, once per walking step."""
import math
import time

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray

if __package__:
    from . import model
    from . import commands
else:
    import model
    import commands


class WalkController(Node):
    def __init__(self):
        super().__init__("walk_vector_controller")
        defaults = {
            "model_path": "/home/orangepi/ros2_ws/src/walkcontrol/walkcontrol/policy_036200.pt", "device": "cpu",
            "depth_topic": "/depth",
            "camera_info_topic": "/camera/camera/depth/camera_info",
            "pose_topic": "/mavros/vision_pose/pose", "odom_topic": "/odometry",
            "waypoint_topic": "/b3/waypoint", "control_topic": "/walk/control",
            "enable_topic": "/nn/enable", "enabled": False, "dry_run": False,
            "inference_hz": 5.0, "step_interval_s": 0.0, "sensor_timeout_s": .5,
            "depth_unit_m": .001, "min_valid_fraction": .15,
        }
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        p = self.p
        if min(p["inference_hz"], p["sensor_timeout_s"], p["depth_unit_m"]) <= 0:
            raise ValueError("Frequency, timeout and depth unit must be positive")
        torch.set_num_threads(2)
        self.network, self.cfg, _ = model.load_policy(p["model_path"], p["device"])
        self.interval = max(self.cfg.step_duration_s, p["step_interval_s"])
        self.enabled, self.hidden, self.goal = p["enabled"], None, None
        self.depth, self.xy, self.yaw = None, None, 0.
        self.roll, self.pitch = 0., 0.
        self.previous = np.zeros(3, dtype=np.float32)
        self.depth_time, self.pose_time, self.next_step = -math.inf, -math.inf, 0.
        self.pose_frame, self.goal_frame, self.held = "", "", False
        self.camera_ok = not bool(p["camera_info_topic"])
        self.pub = self.create_publisher(Float32MultiArray, p["control_topic"], 10)
        self.create_subscription(Image, p["depth_topic"], self.on_depth, qos_profile_sensor_data)
        if p["camera_info_topic"]:
            self.create_subscription(CameraInfo, p["camera_info_topic"], self.on_camera_info, qos_profile_sensor_data)
        if p["odom_topic"]:
            self.create_subscription(Odometry, p["odom_topic"], self.on_odom, qos_profile_sensor_data)
        else:
            self.create_subscription(PoseStamped, p["pose_topic"], self.on_pose, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, p["waypoint_topic"], self.on_goal, 10)
        self.create_subscription(Bool, p["enable_topic"], self.on_enable, 10)
        self.create_timer(1/p["inference_hz"], self.control_loop)
        self.get_logger().info(f"Vector output; step interval {self.interval:.2f}s; dry_run={p['dry_run']}")

    def on_camera_info(self, msg):
        c = self.cfg
        actual = [msg.p[0], msg.p[5], msg.p[2], msg.p[6]] if msg.p[0] > 0 else [msg.k[0], msg.k[4], msg.k[2], msg.k[5]]
        expected = [c.native_fx, c.native_fy, c.native_cx, c.native_cy]
        self.camera_ok = ((msg.width, msg.height) == (c.native_width, c.native_height)
                          and np.allclose(actual, expected, rtol=.03, atol=1.))
        if not self.camera_ok:
            self.get_logger().error("CameraInfo differs from training config; recalibrate and retrain")

    def on_depth(self, msg):
        try:
            if (msg.width, msg.height) != (self.cfg.native_width, self.cfg.native_height):
                raise ValueError("Expected a 424x240 depth stream (or the checkpoint's calibrated size)")
            self.depth = model.decode_depth(msg, self.p["depth_unit_m"])
            self.depth_time = time.monotonic()
        except (ValueError, TypeError) as error:
            self.depth = None
            self.get_logger().error(str(error))

    def update_pose(self, pose, frame):
        p, q = pose.position, pose.orientation
        if not np.isfinite([p.x, p.y, q.x, q.y, q.z, q.w]).all():
            return
        try:
            self.roll, self.pitch, self.yaw = model.quaternion_to_rpy(q.x, q.y, q.z, q.w)
        except ValueError:
            return
        self.xy, self.pose_frame = np.array([p.x, p.y]), frame
        self.pose_time = time.monotonic()

    def on_pose(self, msg):
        self.update_pose(msg.pose, msg.header.frame_id)

    def on_odom(self, msg):
        self.update_pose(msg.pose.pose, msg.header.frame_id)

    def on_goal(self, msg):
        goal = np.array([msg.pose.position.x, msg.pose.position.y])
        if not np.isfinite(goal).all():
            return
        if msg.header.frame_id and self.pose_frame and msg.header.frame_id != self.pose_frame:
            self.get_logger().error("Pose and waypoint must use the same world frame")
            return
        if self.goal is None or not np.allclose(goal, self.goal, atol=.001) or msg.header.frame_id != self.goal_frame:
            self.goal, self.goal_frame, self.hidden = goal, msg.header.frame_id, None
        # Preserve next_step: waypoint updates never interrupt the interval of a sent step.

    def on_enable(self, msg):
        self.enabled = bool(msg.data)
        self.hidden = None
        if not self.enabled:
            self.hold()

    def hold(self):
        if self.xy is not None and not self.held and not self.p["dry_run"]:
            self.pub.publish(Float32MultiArray(data=[0., 0., 0., 0., 0., float(self.yaw)]))
        self.held, self.hidden = self.xy is not None, None
        self.previous[:] = 0.

    @torch.inference_mode()
    def control_loop(self):
        now = time.monotonic()
        if (not self.enabled or self.depth is None or self.xy is None or self.goal is None
                or now-self.depth_time > self.p["sensor_timeout_s"] or now-self.pose_time > self.p["sensor_timeout_s"]
                or max(abs(self.roll), abs(self.pitch)) > math.radians(20)
                or (self.goal_frame and self.goal_frame != self.pose_frame)):
            self.hold()
            return
        if np.linalg.norm(self.goal-self.xy) <= self.cfg.goal_tolerance_m:
            self.hold(); self.goal = None
            return
        if now < self.next_step:
            return
        t = lambda x: torch.as_tensor(np.asarray(x), dtype=torch.float32, device=self.p["device"])
        try:
            image = model.depth_input(t(self.depth)[None], self.cfg)
            if float(image[:, 1].mean()) < self.p["min_valid_fraction"]:
                self.hold()
                return
            state = model.state_input(t(self.xy[None]), t([self.yaw]), t(self.goal[None]),
                                      t([self.roll]), t([self.pitch]), t(self.previous[None]))
            vector, hidden = self.network(image, state, self.hidden)
            if not torch.isfinite(vector).all():
                raise ValueError("Nonfinite command")
            if self.p["dry_run"]:
                self.get_logger().info(f"Preview: {vector[0].cpu().tolist()}")
            else:
                self.pub.publish(Float32MultiArray(data=vector[0].cpu().tolist()))
                self.previous = commands.relative(vector, t([self.yaw]), self.cfg)[0].cpu().numpy()
            self.hidden, self.held = hidden, False
            self.next_step = time.monotonic()+self.interval
        except (RuntimeError, ValueError) as error:
            self.get_logger().error(f"Inference failed: {error}")
            self.hold()


def main(args=None):
    rclpy.init(args=args)
    node = WalkController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hold()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
