#!/usr/bin/env python3
import json
import math
import queue
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
import torch
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .model import Model
from .rotation import quaternion_to_matrix


torch.set_grad_enabled(False)


class LearnedFlightController(Node):
    """
    DiffPhysDrone inference/controller bridge.

    Inputs
    ------
    depth_topic : sensor_msgs/msg/Image
        Supported encodings: 16UC1/mono16 in millimetres, or 32FC1 in metres.

    pose_topic : geometry_msgs/msg/PoseStamped
        pose: world/map frame (ENU), as in fcu_adapter.
        orientation: body -> world quaternion.
        World-frame velocity is estimated from consecutive pose samples.
        Odometry is used only as an internal container for pose and velocity.

    waypoint_topic : geometry_msgs/msg/PoseStamped
        waypoint position in the same world/map frame as odometry.pose.

    Output
    ------
    control_topic : std_msgs/msg/String
        JSON for fcu_adapter.on_motion():
        {"frame_id": "map", "values": [vx_enu, vy_enu, vz_enu, yaw_enu],
         "is_body_frame": false, "use_yaw_rate": false}.
        Linear velocity is in ENU [m/s]; yaw is absolute in ENU [rad].

    Linear-command semantics use a stabilized velocity adapter:
    v_des_world = v_current_world + (a_pred - v_pred) * lookahead,
    followed by the original speed clamp and slew-rate limit.
    No additional real-flight safety limits are configured here.
    """

    def __init__(self):
        super().__init__('nn_controller_inference_node')

        # ---------------------------- Parameters ----------------------------
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('pose_topic', '/mavros/vision_pose/pose')
        self.declare_parameter('waypoint_topic', '/b3/waypoint')
        self.declare_parameter('control_topic', '/motion_unity')
        self.declare_parameter('enable_topic', '/nn/enable')
        self.declare_parameter('inference_enabled', True)
        self.declare_parameter('frame_id', 'map')

        self.declare_parameter(
            'net_weight',
            '/home/orangepi/ros2_ws/src/flightcontrol/flightcontrol/policy_flight.pth',
        )
        self.declare_parameter('no_odom', False)
        self.declare_parameter('strict_checkpoint', True)

        self.declare_parameter('target_speed', 1.0)
        self.declare_parameter('margin', 0.5)
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('max_inference_hz', 15.0)
        self.declare_parameter('max_integration_dt', 0.20)

        # Velocity adapter parameters.
        # The NN output (a_pred - v_pred) is acceleration-like. We convert it
        # into a velocity target using a fixed lookahead horizon, then limit
        # both command speed and command slew rate.
        self.declare_parameter('velocity_lookahead_s', 0.35)
        self.declare_parameter('max_command_accel', 5.0)

        # Active sensing: affects YAW only. Linear NN commands are unchanged.
        # This matches the B3_corner_v2 logic:
        # OPEN -> waypoint yaw
        # DENSE -> bounded blend toward the learned motion direction.
        self.declare_parameter('active_sensing_blend', True)
        self.declare_parameter('active_sensing_lookahead_s', 0.35)
        self.declare_parameter('active_sensing_min_xy_speed', 0.20)
        self.declare_parameter('active_sensing_alpha', 0.35)
        self.declare_parameter('active_sensing_max_offset_deg', 35.0)
        self.declare_parameter('active_sensing_deadband_deg', 2.0)

        # Depth-density Schmitt hysteresis used only to gate active sensing.
        self.declare_parameter('depth_density_gate', True)
        self.declare_parameter('depth_density_alert_m', 6.0)
        self.declare_parameter('depth_density_enter', 0.40)
        self.declare_parameter('depth_density_exit', 0.25)
        self.declare_parameter('depth_density_ignore_bottom_frac', 0.20)

        # First-order angular low-pass filter for the published absolute yaw.
        # tau=0 disables filtering; default keeps the response smooth at 15 Hz.
        self.declare_parameter('yaw_smoothing_tau_s', 0.20)

        # Preserve the preprocessing used by the supplied code.py.
        # The final NN tensor is [1, 1, 12, 16].
        self.declare_parameter('crop_ratio', 0.82)
        self.declare_parameter('resize_width', 48)
        self.declare_parameter('resize_height', 36)
        self.declare_parameter('pool_size', 3)
        self.declare_parameter('depth_min_m', 0.3)
        self.declare_parameter('depth_max_m', 24.0)

        self.declare_parameter('show_hud', False)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.waypoint_topic = self.get_parameter('waypoint_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.enable_topic = self.get_parameter('enable_topic').value
        self.inference_enabled = bool(self.get_parameter('inference_enabled').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.no_odom = bool(self.get_parameter('no_odom').value)
        self.strict_checkpoint = bool(self.get_parameter('strict_checkpoint').value)
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.margin = float(self.get_parameter('margin').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.max_integration_dt = float(self.get_parameter('max_integration_dt').value)
        self.velocity_lookahead_s = float(
            self.get_parameter('velocity_lookahead_s').value
        )
        self.max_command_accel = float(
            self.get_parameter('max_command_accel').value
        )

        self.active_sensing_blend = bool(
            self.get_parameter('active_sensing_blend').value
        )
        self.active_sensing_lookahead_s = float(
            self.get_parameter('active_sensing_lookahead_s').value
        )
        self.active_sensing_min_xy_speed = float(
            self.get_parameter('active_sensing_min_xy_speed').value
        )
        self.active_sensing_alpha = float(
            self.get_parameter('active_sensing_alpha').value
        )
        self.active_sensing_max_offset_deg = float(
            self.get_parameter('active_sensing_max_offset_deg').value
        )
        self.active_sensing_deadband_deg = float(
            self.get_parameter('active_sensing_deadband_deg').value
        )

        self.depth_density_gate = bool(
            self.get_parameter('depth_density_gate').value
        )
        self.depth_density_alert_m = float(
            self.get_parameter('depth_density_alert_m').value
        )
        self.depth_density_enter = float(
            self.get_parameter('depth_density_enter').value
        )
        self.depth_density_exit = float(
            self.get_parameter('depth_density_exit').value
        )
        self.depth_density_ignore_bottom_frac = float(
            self.get_parameter('depth_density_ignore_bottom_frac').value
        )
        self.yaw_smoothing_tau_s = float(
            self.get_parameter('yaw_smoothing_tau_s').value
        )

        self.crop_ratio = float(self.get_parameter('crop_ratio').value)
        self.resize_width = int(self.get_parameter('resize_width').value)
        self.resize_height = int(self.get_parameter('resize_height').value)
        self.pool_size = int(self.get_parameter('pool_size').value)
        self.depth_min_m = float(self.get_parameter('depth_min_m').value)
        self.depth_max_m = float(self.get_parameter('depth_max_m').value)
        self.show_hud = bool(self.get_parameter('show_hud').value)
        max_hz = float(self.get_parameter('max_inference_hz').value)
        self._min_dt = 1.0 / max_hz if max_hz > 0.0 else 0.0
        self._default_dt = self._min_dt if self._min_dt > 0.0 else 1.0 / 15.0
        self._last_inference_t: Optional[float] = None

        if self.velocity_lookahead_s <= 0.0:
            raise ValueError('velocity_lookahead_s must be > 0')
        if self.max_command_accel <= 0.0:
            raise ValueError('max_command_accel must be > 0')

        if self.active_sensing_lookahead_s <= 0.0:
            raise ValueError('active_sensing_lookahead_s must be > 0')
        if self.active_sensing_min_xy_speed < 0.0:
            raise ValueError('active_sensing_min_xy_speed must be >= 0')
        if not (0.0 <= self.active_sensing_alpha <= 1.0):
            raise ValueError('active_sensing_alpha must be in [0, 1]')
        if not (0.0 < self.active_sensing_max_offset_deg <= 180.0):
            raise ValueError('active_sensing_max_offset_deg must be in (0, 180]')
        if self.active_sensing_deadband_deg < 0.0:
            raise ValueError('active_sensing_deadband_deg must be >= 0')
        if self.depth_density_alert_m <= 0.0:
            raise ValueError('depth_density_alert_m must be > 0')
        if not (0.0 <= self.depth_density_exit < self.depth_density_enter <= 1.0):
            raise ValueError(
                'Require 0 <= depth_density_exit < depth_density_enter <= 1'
            )
        if not (0.0 <= self.depth_density_ignore_bottom_frac < 0.8):
            raise ValueError(
                'depth_density_ignore_bottom_frac must be in [0, 0.8)'
            )
        if self.yaw_smoothing_tau_s < 0.0:
            raise ValueError('yaw_smoothing_tau_s must be >= 0')

        if not (0.0 < self.crop_ratio <= 1.0):
            raise ValueError('crop_ratio must be in (0, 1]')

        if self.resize_height % self.pool_size != 0 or self.resize_width % self.pool_size != 0:
            raise ValueError('resize dimensions must be divisible by pool_size')

        pooled_h = self.resize_height // self.pool_size
        pooled_w = self.resize_width // self.pool_size
        if (pooled_h, pooled_w) != (12, 16):
            raise ValueError(
                f'NN expects 12x16 depth input after pooling, got {pooled_h}x{pooled_w}'
            )

        # ------------------------------ Device ------------------------------
        self._device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Inference device: {self._device}')

        # ------------------------------- Model ------------------------------
        self._dim_obs = 7 if self.no_odom else 10
        self.model = Model(self._dim_obs, 6).eval().to(self._device)

        net_weight = str(self.get_parameter('net_weight').value)
        self._load_checkpoint(net_weight)

        # Warm-up. Do not use model.dim_obs: the original Model does not define it.
        with torch.inference_mode():
            depth_dummy = torch.zeros(1, 1, 12, 16, device=self._device)
            state_dummy = torch.zeros(1, self._dim_obs, device=self._device)
            _ = self._forward_model(depth_dummy, state_dummy, hidden=None)

        self.get_logger().info('Neural network initialized and warmed up.')

        # ------------------------------- State ------------------------------
        self.odometry: Optional[Odometry] = None
        self.wp_global: Optional[torch.Tensor] = None
        self.waypoint_active = False
        self.h = None  # GRU hidden state
        self._last_odom_rx_t: Optional[float] = None
        self._last_vel_cmd_world: Optional[torch.Tensor] = None
        self._last_target_yaw: Optional[float] = None

        # Active-sensing depth-density hysteresis state.
        # False = OPEN, True = DENSE.
        self._dense_mode = False

        self._pos_buf = torch.zeros(3, dtype=torch.float32)
        self._quat_buf = torch.zeros(4, dtype=torch.float32)  # [w, x, y, z]
        self._vel_buf = torch.zeros(3, dtype=torch.float32)

        self._margin = torch.tensor(
            [[self.margin]], dtype=torch.float32, device=self._device
        )

        self._crop_shape = None
        self._crop_h = 0
        self._crop_w = 0

        # ------------------------------- HUD --------------------------------
        self._hud_queue: queue.Queue = queue.Queue(maxsize=1)
        if self.show_hud:
            threading.Thread(target=self._hud_loop, daemon=True).start()

        # --------------------------- ROS interfaces -------------------------
        self.control_pub = self.create_publisher(String, self.control_topic, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.callback_pose, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.callback_depth, qos_profile_sensor_data
        )
        self.waypoint_sub = self.create_subscription(
            PoseStamped, self.waypoint_topic, self.callback_waypoint, 10
        )
        self.enable_sub = self.create_subscription(
            Bool, self.enable_topic, self.callback_enable, 10
        )

    # ======================================================================
    # Model/checkpoint helpers
    # ======================================================================

    def _load_checkpoint(self, path: str) -> None:
        obj = torch.load(path, map_location=self._device)

        if isinstance(obj, dict) and 'state_dict' in obj and isinstance(obj['state_dict'], dict):
            state_dict = obj['state_dict']
        elif isinstance(obj, dict) and 'model' in obj and isinstance(obj['model'], dict):
            state_dict = obj['model']
        else:
            state_dict = obj

        if not isinstance(state_dict, dict):
            raise TypeError(f'Unsupported checkpoint format in {path}')

        # Common DataParallel prefix.
        if state_dict and all(isinstance(k, str) for k in state_dict.keys()):
            if any(k.startswith('module.') for k in state_dict.keys()):
                state_dict = {
                    (k[7:] if k.startswith('module.') else k): v
                    for k, v in state_dict.items()
                }

        missing, unexpected = self.model.load_state_dict(state_dict, strict=False)

        if missing:
            self.get_logger().error(f'Checkpoint missing keys: {missing}')
        if unexpected:
            self.get_logger().error(f'Checkpoint unexpected keys: {unexpected}')

        if self.strict_checkpoint and (missing or unexpected):
            raise RuntimeError(
                'Checkpoint is not strictly compatible with Model. '
                'Refusing to run inference with partially initialized weights.'
            )

    def _forward_model(self, depth: torch.Tensor, state: torch.Tensor, hidden=None):
        """Support the original 3-output Model and the common 2-output wrapper."""
        out = self.model(depth, state, hidden)

        if not isinstance(out, (tuple, list)):
            raise RuntimeError('Model.forward() must return a tuple/list')

        if len(out) == 3:
            act, _, new_hidden = out
        elif len(out) == 2:
            act, new_hidden = out
        else:
            raise RuntimeError(f'Unexpected Model.forward() output length: {len(out)}')

        return act, new_hidden

    # ======================================================================
    # Depth helpers
    # ======================================================================

    def _decode_depth(self, msg: Image) -> np.ndarray:
        enc = msg.encoding.lower()

        if enc in ('16uc1', 'mono16'):
            base_dtype = np.dtype(np.uint16)
            scale = 1.0 / 1000.0
        elif enc == '32fc1':
            base_dtype = np.dtype(np.float32)
            scale = 1.0
        else:
            raise ValueError(
                f'Unsupported depth encoding: {msg.encoding}. '
                'Expected 16UC1/mono16 (mm) or 32FC1 (m).'
            )

        dtype = base_dtype.newbyteorder('>' if msg.is_bigendian else '<')
        itemsize = dtype.itemsize
        row_elems = (msg.step // itemsize) if msg.step else msg.width

        arr = np.frombuffer(msg.data, dtype=dtype)
        expected = msg.height * row_elems
        if arr.size < expected:
            raise ValueError(
                f'Depth buffer too small: {arr.size} elements, expected at least {expected}'
            )

        arr = arr[:expected].reshape(msg.height, row_elems)[:, :msg.width]
        depth_m = arr.astype(np.float32, copy=True)
        depth_m *= scale
        return depth_m

    def _preprocess_depth(self, depth_m: np.ndarray) -> torch.Tensor:
        h, w = depth_m.shape

        if self._crop_shape != (h, w):
            self._crop_h = round((1.0 - self.crop_ratio) * h / 2.0)
            self._crop_w = round((1.0 - self.crop_ratio) * w / 2.0)
            self._crop_shape = (h, w)

        # Invalid/zero depth = no return -> treat as far range.
        invalid = (~np.isfinite(depth_m)) | (depth_m <= 0.0)
        depth_m[invalid] = self.depth_max_m
        np.clip(depth_m, self.depth_min_m, self.depth_max_m, out=depth_m)

        depth_nn = 3.0 / depth_m - 0.6

        y0 = self._crop_h
        y1 = h - self._crop_h if self._crop_h > 0 else h
        x0 = self._crop_w
        x1 = w - self._crop_w if self._crop_w > 0 else w
        cropped = depth_nn[y0:y1, x0:x1]

        if cropped.size == 0:
            raise ValueError('Depth crop produced an empty image')

        resized = cv2.resize(
            cropped,
            (self.resize_width, self.resize_height),
            interpolation=cv2.INTER_NEAREST,
        )

        p = self.pool_size
        pooled = resized.reshape(
            self.resize_height // p,
            p,
            self.resize_width // p,
            p,
        ).max(axis=(1, 3))

        tensor = torch.from_numpy(np.ascontiguousarray(pooled)).to(
            self._device, dtype=torch.float32, non_blocking=True
        )
        return tensor[None, None]

    # ======================================================================
    # Timestamp helper
    # ======================================================================

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _compute_depth_density(self, depth_m: np.ndarray) -> float:
        """Obstacle density used only for active-sensing OPEN/DENSE gating."""
        # Training uses a 64x48 rendered depth frame for the density gate.
        # Keep this separate from the unchanged baseline 12x16 NN preprocessing.
        metric = cv2.resize(
            depth_m.astype(np.float32, copy=False),
            (64, 48),
            interpolation=cv2.INTER_NEAREST,
        )

        keep_h = max(
            1,
            int(round(
                48 * (1.0 - self.depth_density_ignore_bottom_frac)
            )),
        )
        roi = metric[:keep_h, :]

        valid = np.isfinite(roi) & (roi > 0.0)
        valid_count = int(valid.sum())
        if valid_count <= 0:
            return 0.0

        close = valid & (roi < self.depth_density_alert_m)
        return float(close.sum()) / float(valid_count)

    def _update_dense_mode(self, density: float) -> None:
        if self.depth_density_gate:
            if (not self._dense_mode) and density >= self.depth_density_enter:
                self._dense_mode = True
            elif self._dense_mode and density <= self.depth_density_exit:
                self._dense_mode = False
        else:
            self._dense_mode = True

    def _smooth_yaw(self, raw_yaw: float, dt: float, current_yaw: float) -> float:
        """Frame-rate-independent low-pass filter on a wrapped yaw angle."""
        raw_yaw = self._wrap_pi(raw_yaw)

        if self.yaw_smoothing_tau_s <= 0.0:
            self._last_target_yaw = raw_yaw
            return raw_yaw

        if self._last_target_yaw is None:
            # Start from the real vehicle heading to avoid a first-command jump.
            self._last_target_yaw = self._wrap_pi(current_yaw)

        alpha = 1.0 - math.exp(
            -max(float(dt), 1e-4) / self.yaw_smoothing_tau_s
        )
        delta = self._wrap_pi(raw_yaw - self._last_target_yaw)
        smoothed = self._wrap_pi(
            self._last_target_yaw + alpha * delta
        )
        self._last_target_yaw = smoothed
        return smoothed

    # ======================================================================
    # ROS callbacks
    # ======================================================================

    def callback_enable(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled == self.inference_enabled:
            return

        self.inference_enabled = enabled
        self.h = None
        self._last_vel_cmd_world = None
        self._last_inference_t = None
        self._last_target_yaw = None
        self._dense_mode = False
        self.get_logger().info(
            'Inference enabled.' if enabled else 'Inference disabled.'
        )

    def callback_waypoint(self, msg: PoseStamped) -> None:
        self.wp_global = torch.tensor(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=torch.float32,
        )
        self.waypoint_active = True
        self.h = None  # new mission segment -> reset recurrent memory
        self._last_vel_cmd_world = None
        self._last_target_yaw = None
        self._dense_mode = False

        self.get_logger().info(
            f'New waypoint: x={self.wp_global[0]:.2f}, '
            f'y={self.wp_global[1]:.2f}, z={self.wp_global[2]:.2f}'
        )

    def callback_pose(self, msg: PoseStamped) -> None:
        # PoseStamped has no velocity. Estimate it in ENU and reuse the
        # controller's existing internal Odometry representation.
        odom = Odometry()
        odom.header = msg.header
        odom.pose.pose = msg.pose

        if self.odometry is not None:
            stamp = self._stamp_to_sec(msg.header.stamp)
            previous_stamp = self._stamp_to_sec(self.odometry.header.stamp)
            if stamp > previous_stamp > 0.0:
                dt = stamp - previous_stamp
            else:
                # Also support publishers with zero or repeated timestamps.
                dt = time.monotonic() - self._last_odom_rx_t

            if dt > 0.0:
                p = msg.pose.position
                previous = self.odometry.pose.pose.position
                v = odom.twist.twist.linear
                v.x = (p.x - previous.x) / dt
                v.y = (p.y - previous.y) / dt
                v.z = (p.z - previous.z) / dt

        self.callback_odometry(odom)

    def callback_odometry(self, msg: Odometry) -> None:
        self._last_odom_rx_t = time.monotonic()
        self.odometry = msg

    def callback_depth(self, msg: Image) -> None:
        if not self.inference_enabled:
            return

        if self.odometry is None:
            self.get_logger().warn(
                'No pose received. Skipping inference.',
                throttle_duration_sec=2.0,
            )
            return

        now = time.monotonic()
        if self._last_inference_t is not None:
            elapsed = now - self._last_inference_t
            if elapsed < self._min_dt:
                return
            dt = min(elapsed, self.max_integration_dt)
        else:
            dt = self._default_dt
        self._last_inference_t = now

        # ----------------------------- Odometry -----------------------------
        p = self.odometry.pose.pose.position
        q = self.odometry.pose.pose.orientation
        v = self.odometry.twist.twist.linear

        self._pos_buf[:] = torch.tensor([p.x, p.y, p.z], dtype=torch.float32)
        self._quat_buf[:] = torch.tensor([q.w, q.x, q.y, q.z], dtype=torch.float32)
        self._vel_buf[:] = torch.tensor([v.x, v.y, v.z], dtype=torch.float32)

        pos_world = self._pos_buf.clone()
        R_body_world = quaternion_to_matrix(self._quat_buf)
        vel_raw = self._vel_buf.clone()

        # The pose difference already gives velocity in the world/ENU frame.
        vel_world = vel_raw

        current_yaw = math.atan2(
            float(R_body_world[1, 0]), float(R_body_world[0, 0])
        )

        # Match code.py yaw semantics.
        target_yaw = 0.0

        # Preserve the original behavior while waiting for a waypoint.
        if not self.waypoint_active or self.wp_global is None:
            self._publish_command(torch.zeros(3), target_yaw)
            return

        # ------------------------- Waypoint command -------------------------
        vec_to_wp_world = self.wp_global - pos_world
        dist = float(torch.linalg.norm(vec_to_wp_world))

        # Same yaw calculation as code.py:
        # real_dir = waypoint_world - current_position_world
        # target_yaw = atan2(real_dir_y, real_dir_x)
        target_yaw = torch.atan2(
            vec_to_wp_world[1], vec_to_wp_world[0]
        ).item()

        if dist <= self.waypoint_tolerance:
            self.waypoint_active = False
            self.h = None
            self.get_logger().info('Waypoint reached. Sending zero linear velocity.')
            self._publish_command(torch.zeros(3), target_yaw)
            return

        # This matches the training logic:
        # target_v points directly to the target and its norm is capped.
        target_speed_now = min(dist, self.target_speed)
        target_v_world = vec_to_wp_world / dist * target_speed_now

        # ------------------------ Yaw-aligned frame -------------------------
        # Same construction as main_cuda.py: x follows body forward projected
        # to the horizontal plane, z is world up.
        fwd = R_body_world[:, 0].clone()
        fwd[2] = 0.0
        fwd_norm = float(torch.linalg.norm(fwd))

        if fwd_norm < 1e-6:
            fwd = torch.tensor(
                [math.cos(current_yaw), math.sin(current_yaw), 0.0],
                dtype=torch.float32,
            )
        else:
            fwd /= fwd_norm

        up = torch.tensor([0.0, 0.0, 1.0], dtype=torch.float32)
        left = torch.cross(up, fwd, dim=-1)
        R_aligned = torch.stack([fwd, left, up], dim=-1)

        # ---------------------------- NN state ------------------------------
        R_aligned_d = R_aligned.to(self._device)
        R_body_world_d = R_body_world.to(self._device)
        vel_world_d = vel_world.to(self._device)
        target_v_world_d = target_v_world.to(self._device)

        state_parts = [
            target_v_world_d[None] @ R_aligned_d,
            R_body_world_d[None, 2],
            self._margin,
        ]
        if not self.no_odom:
            state_parts.insert(0, vel_world_d[None] @ R_aligned_d)
        state = torch.cat(state_parts, dim=-1)

        # -------------------------- Depth + model ---------------------------
        try:
            depth_m = self._decode_depth(msg)
            depth_for_hud = depth_m.copy() if self.show_hud else None

            # Active-sensing gate uses metric depth only. NN preprocessing below
            # is unchanged.
            depth_density = self._compute_depth_density(depth_m)
            self._update_dense_mode(depth_density)

            depth_tensor = self._preprocess_depth(depth_m)
        except Exception as exc:
            self.get_logger().error(
                f'Depth preprocessing failed: {exc}',
                throttle_duration_sec=2.0,
            )
            self._publish_command(torch.zeros(3), target_yaw)
            return

        with torch.inference_mode():
            act_d, self.h = self._forward_model(depth_tensor, state, self.h)

        # Original DiffPhysDrone output is an acceleration-like command in the
        # yaw-aligned frame (plus a learned velocity estimate). Reconstruct the
        # world-frame translational acceleration exactly as in training.
        act = act_d.detach().cpu()
        a_pred_world, v_pred_world, *_ = (
            R_aligned @ act.reshape(3, -1)
        ).unbind(-1)
        a_cmd_world = a_pred_world - v_pred_world

        # ------------------------------------------------------------------
        # Stable velocity adapter
        # ------------------------------------------------------------------
        # The NN quantity (a_pred - v_pred) is acceleration-like. Using it as
        # a pure direction and normalizing every nonzero vector to 5 m/s caused
        # command flips close to obstacles. Using only a_cmd * dt, on the other
        # hand, produced very slow flight.
        #
        # Compromise:
        #   1) project the acceleration-like output over a fixed lookahead;
        #   2) clamp desired speed;
        #   3) rate-limit change of the WORLD-frame velocity command;
        #   4) send the final WORLD/ENU command to fcu_adapter.
        control_world = a_cmd_world

        # ------------------------------------------------------------------
        # Active sensing: YAW only
        # ------------------------------------------------------------------
        # Keep the original waypoint bearing as the nominal yaw.
        waypoint_yaw = target_yaw
        raw_target_yaw = waypoint_yaw

        motion_dir_world = (
            vel_world
            + control_world * self.active_sensing_lookahead_s
        )
        motion_xy = motion_dir_world[:2]
        motion_speed_xy = float(torch.linalg.norm(motion_xy))

        active_sensing_gate = (
            self.active_sensing_blend
            and self._dense_mode
            and motion_speed_xy >= self.active_sensing_min_xy_speed
        )

        if active_sensing_gate:
            motion_yaw = math.atan2(
                float(motion_xy[1]),
                float(motion_xy[0]),
            )

            yaw_delta = self._wrap_pi(motion_yaw - waypoint_yaw)
            max_offset = math.radians(
                self.active_sensing_max_offset_deg
            )
            yaw_delta = max(
                -max_offset,
                min(max_offset, yaw_delta),
            )

            yaw_offset = self.active_sensing_alpha * yaw_delta

            # Small yaw corrections are suppressed exactly as in B3_corner_v2.
            if abs(yaw_offset) >= math.radians(
                self.active_sensing_deadband_deg
            ):
                raw_target_yaw = self._wrap_pi(
                    waypoint_yaw + yaw_offset
                )

        # Smooth only the final yaw command. Translational control is untouched.
        target_yaw = self._smooth_yaw(
            raw_target_yaw,
            dt,
            current_yaw,
        )

        vel_des_world = (
            vel_world + control_world * self.velocity_lookahead_s
        )

        # Slow down naturally when very close to the waypoint. Away from the
        # waypoint this is simply the configured target_speed (default 5 m/s).
        speed_limit = target_speed_now
        vel_des_speed = float(torch.linalg.norm(vel_des_world))
        if vel_des_speed > speed_limit and vel_des_speed > 1e-6:
            vel_des_world = vel_des_world * (speed_limit / vel_des_speed)

        # Slew-rate limit. Limit in WORLD frame so a yaw rotation by itself
        # does not look like an enormous translational command jump.
        if self._last_vel_cmd_world is None:
            previous_cmd_world = vel_world.clone()
        else:
            previous_cmd_world = self._last_vel_cmd_world

        cmd_delta_world_raw = vel_des_world - previous_cmd_world
        raw_delta_norm = float(torch.linalg.norm(cmd_delta_world_raw))
        max_delta_v = self.max_command_accel * dt

        if raw_delta_norm > max_delta_v and raw_delta_norm > 1e-6:
            cmd_delta_world_limited = (
                cmd_delta_world_raw * (max_delta_v / raw_delta_norm)
            )
        else:
            cmd_delta_world_limited = cmd_delta_world_raw

        vel_cmd_world = previous_cmd_world + cmd_delta_world_limited

        # Numerical safety: the slew-limited command should normally already
        # satisfy the speed bound, but clamp once more after the update.
        vel_cmd_speed = float(torch.linalg.norm(vel_cmd_world))
        if vel_cmd_speed > speed_limit and vel_cmd_speed > 1e-6:
            vel_cmd_world = vel_cmd_world * (speed_limit / vel_cmd_speed)

        self._last_vel_cmd_world = vel_cmd_world.clone()

        self._publish_command(vel_cmd_world, target_yaw)

        if self.show_hud and depth_for_hud is not None:
            vel_cmd_body = R_body_world.T @ vel_cmd_world
            try:
                self._hud_queue.put_nowait(
                    (
                        depth_for_hud,
                        depth_tensor.detach().cpu(),
                        pos_world,
                        vel_world,
                        vel_cmd_body,
                        target_v_world,
                        a_cmd_world,
                    )
                )
            except queue.Full:
                pass

    # ======================================================================
    # Output
    # ======================================================================

    def _publish_command(self, vel_world: torch.Tensor, target_yaw: float) -> None:
        # fcu_adapter.on_motion(): ENU velocity + absolute ENU yaw.
        command = {
            'frame_id': self.frame_id,
            'values': [
                float(vel_world[0]),
                float(vel_world[1]),
                float(vel_world[2]),
                float(target_yaw),
            ],
            'is_body_frame': False,
            'use_yaw_rate': False,
        }
        self.control_pub.publish(String(data=json.dumps(command, allow_nan=False)))

    # ======================================================================
    # Optional HUD
    # ======================================================================

    def _hud_loop(self) -> None:
        while rclpy.ok():
            try:
                data = self._hud_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                self.update_hud(*data)
            except Exception as exc:
                self.get_logger().warn(
                    f'HUD error: {exc}', throttle_duration_sec=2.0
                )

    def update_hud(
        self,
        raw_depth,
        nn_depth_tensor,
        pos_world,
        vel_world,
        cmd_vel_body,
        target_v_world,
        a_cmd_world,
    ) -> None:
        finite = np.where(np.isfinite(raw_depth), raw_depth, self.depth_max_m)
        vis_raw = cv2.normalize(
            finite, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        vis_raw_color = cv2.applyColorMap(vis_raw, cv2.COLORMAP_JET)
        h_orig, w_orig = vis_raw_color.shape[:2]

        nn_np = nn_depth_tensor.squeeze().numpy()
        vis_nn = cv2.normalize(
            nn_np, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        vis_nn_color = cv2.applyColorMap(vis_nn, cv2.COLORMAP_JET)
        vis_nn_resized = cv2.resize(
            vis_nn_color, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST
        )

        cv2.putText(
            vis_raw_color, '1) Original Depth', (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
        )
        cv2.putText(
            vis_nn_resized, '2) NN Input', (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
        )

        top_row = cv2.hconcat([vis_raw_color, vis_nn_resized])
        info_panel = np.zeros((190, top_row.shape[1], 3), dtype=np.uint8)
        font, fs, th = cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1

        lines = [
            f'World pos: {pos_world[0]:.2f}, {pos_world[1]:.2f}, {pos_world[2]:.2f}',
            f'World vel: {vel_world[0]:.2f}, {vel_world[1]:.2f}, {vel_world[2]:.2f}',
            f'Target v : {target_v_world[0]:.2f}, {target_v_world[1]:.2f}, {target_v_world[2]:.2f}',
            f'NN accel : {a_cmd_world[0]:.2f}, {a_cmd_world[1]:.2f}, {a_cmd_world[2]:.2f}',
            f'Body cmd : {cmd_vel_body[0]:.2f}, {cmd_vel_body[1]:.2f}, {cmd_vel_body[2]:.2f}',
        ]
        for i, line in enumerate(lines):
            cv2.putText(info_panel, line, (20, 30 + 32 * i), font, fs, (255, 255, 255), th)

        cv2.imshow('DiffPhysDrone Control HUD', cv2.vconcat([top_row, info_panel]))
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LearnedFlightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
