#!/usr/bin/env python3
"""
Self-contained walking robot controller: depth + odometry → walk command.

Outputs:
  - Float32MultiArray: [vx, vy, 0, 0, 0, yaw]
  - String: "directionAuto_angle" (e.g., "forwardAuto_0", "turnAuto_15")

All dependencies inlined - no external imports needed.
Just inference loop: receive sensors, preprocess, run model, publish.
"""

import math
import time
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from torch.distributions import Categorical, Normal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, String

torch.set_grad_enabled(False)


# ============================================================================
# Model and preprocessing (inlined from model_walk.py)
# ============================================================================

# Action constants
STOP, FORWARD, BACKWARD, LEFT, RIGHT, TURN = 0, 1, 2, 3, 4, 5
ACTION_NAMES = ['stop', 'forward', 'backward', 'left', 'right', 'turn']
STATE_DIM = 15


def depth_input(depth_m, cfg, valid_mask=None):
    """
    Metric optical-Z depth [B,H,W] -> [B,2,48,64].
    
    Channel 0: closest VALID inverse depth in each output bin, scaled to [0,1].
    Channel 1: valid coverage in [0,1]; zero depth is UNKNOWN, never free space.
    """
    if depth_m.ndim != 3:
        raise ValueError('Expected [batch,height,width] depth')
    
    valid = torch.isfinite(depth_m) & (depth_m >= cfg.depth_min_m) & (depth_m <= cfg.depth_max_m)
    if valid_mask is not None:
        valid = valid & valid_mask.bool()
    
    clean = torch.where(valid, depth_m, torch.ones_like(depth_m) * cfg.depth_max_m)
    inv = (1/clean.clamp_min(cfg.depth_min_m) - 1/cfg.depth_max_m) / (1/cfg.depth_min_m - 1/cfg.depth_max_m)
    inv = torch.where(valid, inv, -torch.ones_like(inv))[:, None]
    
    size = (cfg.height, cfg.width)
    inv = F.adaptive_max_pool2d(inv, size)
    coverage = F.adaptive_avg_pool2d(valid[:, None].float(), size)
    
    return torch.cat([torch.where(coverage > 0, inv, torch.zeros_like(inv)), coverage], 1)


def state_input(xy, yaw, goal_xy, body_velocity, yaw_rate, roll, pitch,
                previous_mode, previous_turn, cfg):
    """Build state tensor from odometry and goal."""
    delta = goal_xy - xy
    c, s = yaw.cos(), yaw.sin()
    rel = torch.stack([delta[:, 0]*c + delta[:, 1]*s, -delta[:, 0]*s + delta[:, 1]*c], -1)
    dist = rel.norm(dim=-1, keepdim=True)
    turn_scale = math.radians(cfg.max_turn_deg)
    
    return torch.cat([
        rel / dist.clamp_min(1e-6),
        (dist / 6).clamp(0, 2),
        (body_velocity / (cfg.straight_step_m / cfg.step_duration_s)).clamp(-3, 3),
        (yaw_rate[:, None] / (turn_scale / cfg.step_duration_s)).clamp(-3, 3),
        roll.sin()[:, None],
        pitch.sin()[:, None],
        F.one_hot(previous_mode.long(), 6).float(),
        (previous_turn[:, None] / turn_scale).clamp(-1, 1)
    ], -1)


class WalkPolicy(nn.Module):
    """B3 spatial encoder + GRU + hybrid categorical/continuous actor."""
    
    def __init__(self):
        super().__init__()
        self.stem = nn.Sequential(
            nn.Conv2d(2, 24, 3, 2, 1, bias=False), nn.LeakyReLU(0.05),
            nn.Conv2d(24, 48, 3, 2, 1, bias=False), nn.LeakyReLU(0.05),
            nn.Conv2d(48, 96, 3, 2, 1, bias=False), nn.LeakyReLU(0.05),
            nn.Conv2d(96, 128, 3, 1, 1, bias=False), nn.LeakyReLU(0.05),
            nn.Flatten(), nn.Linear(128*6*8, 192, bias=False)
        )
        self.v_proj = nn.Linear(STATE_DIM, 192)
        self.gru = nn.GRUCell(192, 192)
        self.mode_head = nn.Linear(192, 6)
        self.turn_head = nn.Linear(192, 1)
        self.value_head = nn.Linear(192, 1)
        self.log_std = nn.Parameter(torch.tensor(-0.25))
        
        nn.init.normal_(self.mode_head.weight, std=0.01)
        nn.init.zeros_(self.mode_head.bias)
        nn.init.normal_(self.turn_head.weight, std=0.01)
        nn.init.zeros_(self.turn_head.bias)
    
    def forward(self, image, state, hidden=None):
        if image.shape[1:] != (2, 48, 64) or state.shape != (image.shape[0], STATE_DIM):
            raise ValueError(f'Invalid inputs {image.shape}, {state.shape}')
        
        fused = F.leaky_relu(self.stem(image) + self.v_proj(state), 0.05)
        hidden = self.gru(fused, hidden)
        z = F.leaky_relu(hidden, 0.05)
        
        return (
            self.mode_head(z),
            self.turn_head(z).squeeze(-1),
            self.value_head(z).squeeze(-1),
            hidden
        )
    
    def distributions(self, logits, mean, allowed=None):
        if allowed is not None:
            if not bool(allowed[:, 0].all()):
                raise ValueError('STOP must always be permitted')
            logits = logits.masked_fill(~allowed.bool(), -1e9)
        
        return (
            Categorical(logits=logits),
            Normal(mean, self.log_std.clamp(-2, 1).exp().expand_as(mean))
        )
    
    def act(self, image, state, hidden=None, deterministic=False, allowed=None):
        logits, mean, value, h = self(image, state, hidden)
        cat, normal = self.distributions(logits, mean, allowed)
        
        mode = cat.logits.argmax(-1) if deterministic else cat.sample()
        raw_turn = mean if deterministic else normal.sample()
        logp = cat.log_prob(mode) + (mode == TURN) * normal.log_prob(raw_turn)
        
        return mode, raw_turn, logp, value, h


class WalkConfig:
    """Minimal config with only essential runtime fields."""
    
    def __init__(self, **kwargs):
        # Geometry
        self.diameter_m = kwargs.get('diameter_m', 1.0)
        self.height_m = kwargs.get('height_m', 0.55)
        self.body_origin_height_m = kwargs.get('body_origin_height_m', 0.275)
        
        # Camera
        self.camera_x_m = kwargs.get('camera_x_m', 0.390)
        self.camera_y_m = kwargs.get('camera_y_m', 0.0)
        self.camera_height_m = kwargs.get('camera_height_m', 0.250)
        
        # Image size
        self.width = kwargs.get('width', 64)
        self.height = kwargs.get('height', 48)
        self.horizontal_fov_deg = kwargs.get('horizontal_fov_deg', 87.0)
        self.vertical_fov_deg = kwargs.get('vertical_fov_deg', 58.0)
        
        # Depth range
        self.depth_min_m = kwargs.get('depth_min_m', 0.30)
        self.depth_max_m = kwargs.get('depth_max_m', 12.0)
        
        # Motion primitives
        self.step_duration_s = kwargs.get('step_duration_s', 2.0)
        self.straight_step_m = kwargs.get('straight_step_m', 0.26)
        self.turn_forward_projection_m = kwargs.get('turn_forward_projection_m', 0.11)
        self.max_turn_deg = kwargs.get('max_turn_deg', 15.0)
        
        # Navigation
        self.goal_tolerance_m = kwargs.get('goal_tolerance_m', 0.18)
    
    def intrinsics(self):
        """Return camera intrinsics [fx, fy, cx, cy]."""
        fx = self.width / (2 * math.tan(math.radians(self.horizontal_fov_deg) / 2))
        fy = self.height / (2 * math.tan(math.radians(self.vertical_fov_deg) / 2))
        cx = (self.width - 1) / 2
        cy = (self.height - 1) / 2
        return (fx, fy, cx, cy)


def load_policy(path, device='cpu'):
    """Load checkpoint and extract model, config, metadata."""
    obj = torch.load(path, map_location=device, weights_only=True)
    
    cfg = WalkConfig(**obj['config'])
    model = WalkPolicy().to(device)
    model.load_state_dict(obj['model'], strict=True)
    
    return model.eval(), cfg, obj


# ============================================================================
# Utility functions (inlined from walk_runtime.py)
# ============================================================================

def quaternion_to_rpy(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw."""
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(np.clip(2*(w*y - z*x), -1, 1))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


def decode_depth(height, width, encoding, step, is_bigendian, data, scale_16u=0.001):
    """Decode ROS Image to numpy depth array."""
    if encoding in ('16UC1', 'mono16'):
        dtype = np.dtype('>u2' if is_bigendian else '<u2')
        scale = scale_16u
    elif encoding == '32FC1':
        dtype = np.dtype('>f4' if is_bigendian else '<f4')
        scale = 1.0
    else:
        raise ValueError(f'Unsupported encoding: {encoding}')
    
    array = np.ndarray(
        (height, width),
        dtype=dtype,
        buffer=data,
        strides=(step, dtype.itemsize)
    )
    return array.astype(np.float32) * scale


def wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2*math.pi) - math.pi


# ============================================================================
# Minimal walking controller
# ============================================================================

class MinimalWalkController(Node):
    """
    Self-contained walking controller with all dependencies inlined.
    
    Input:
      - Depth image (sensor_msgs/Image)
      - Odometry (nav_msgs/Odometry)
      - Goal waypoint (geometry_msgs/PoseStamped)
    
    Output:
      - Walk command vector (std_msgs/Float32MultiArray): [vx, vy, 0, 0, 0, yaw]
      - Walk command string (std_msgs/String): "directionAuto_angle"
    
    No external dependencies. Just: receive → preprocess → infer → publish.
    """
    
    def __init__(self):
        super().__init__('minimal_walk_controller')
        
        # Parameters
        self.declare_parameter('model_path', '/home/orangepi/ros2_ws/src/walkcontrol/walkcontrol/policy_last.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('inference_hz', 5.0)
        self.declare_parameter('depth_topic', '/depth')
        self.declare_parameter('odom_topic', '/odometry')
        self.declare_parameter('waypoint_topic', '/waypoint')
        self.declare_parameter('control_topic', '/walk/control')
        self.declare_parameter('control_string_topic', '/morphogear_sudo_cmd')
        
        # Read parameters
        model_path = str(self.get_parameter('model_path').value)
        self.device = str(self.get_parameter('device').value)
        hz = float(self.get_parameter('inference_hz').value)
        
        # Load model
        torch.set_num_threads(2)
        self.model, self.cfg, self.meta = load_policy(model_path, self.device)
        self.h = None  # GRU hidden state
        
        self.get_logger().info(
            f'Model loaded: update {self.meta.get("update", "unknown")}, '
            f'step duration {self.cfg.step_duration_s}s'
        )
        
        # Simple state buffers
        self.depth = None
        self.depth_time = None
        
        self.position = None  # [x, y] in world frame
        self.yaw = None
        self.roll = None
        self.pitch = None
        self.velocity = None  # [vx, vy] in body frame
        self.yaw_rate = None
        self.odom_time = None
        
        self.goal = None  # [x, y] in world frame
        
        # Step timing (simple wall-clock latch)
        self.step_duration = self.cfg.step_duration_s
        self.last_step_time = None
        
        # Previous action (for model state input)
        self.prev_mode = STOP
        self.prev_turn = 0.0
        
        # Publishers
        self.control_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter('control_topic').value),
            10
        )
        
        self.control_string_pub = self.create_publisher(
            String,
            str(self.get_parameter('control_string_topic').value),
            10
        )
        
        # Subscribers
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self.callback_depth,
            10
        )
        
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.callback_odom,
            10
        )
        
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('waypoint_topic').value),
            self.callback_waypoint,
            10
        )
        
        # Control loop timer
        self.create_timer(1.0 / hz, self.control_loop)
        
        self.get_logger().info(f'Controller running at {hz} Hz')
    
    # ------------------------------------------------------------------------
    # Callbacks: just unpack and store
    # ------------------------------------------------------------------------
    
    def callback_depth(self, msg: Image) -> None:
        """Receive depth image."""
        try:
            self.depth = decode_depth(
                msg.height, msg.width, msg.encoding,
                msg.step, msg.is_bigendian, msg.data
            )
            self.depth_time = time.monotonic()
        except Exception as e:
            self.get_logger().warn(f'Depth decode failed: {e}')
            self.depth = None
    
    def callback_odom(self, msg: Odometry) -> None:
        """Receive odometry - unpack to simple buffers."""
        try:
            # Position
            p = msg.pose.pose.position
            self.position = np.array([p.x, p.y])
            
            # Orientation
            q = msg.pose.pose.orientation
            self.roll, self.pitch, self.yaw = quaternion_to_rpy(q.x, q.y, q.z, q.w)
            
            # Velocity (assuming body frame from odometry)
            v = msg.twist.twist.linear
            self.velocity = np.array([v.x, v.y])
            
            # Yaw rate
            self.yaw_rate = float(msg.twist.twist.angular.z)
            
            self.odom_time = time.monotonic()
            
        except Exception as e:
            self.get_logger().warn(f'Odometry failed: {e}')
    
    def callback_waypoint(self, msg: PoseStamped) -> None:
        """Receive goal waypoint."""
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        
        # Reset hidden state on new goal
        self.h = None
        self.last_step_time = None
        
        self.get_logger().info(f'New goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')
    
    # ------------------------------------------------------------------------
    # Control loop: the core
    # ------------------------------------------------------------------------
    
    def control_loop(self) -> None:
        """
        Main loop: check sensors → preprocess → infer → publish.
        
        Runs at inference_hz (e.g., 5 Hz).
        Commits new action every step_duration_s (e.g., 2 s).
        """
        now = time.monotonic()
        
        # Check if sensors are ready
        if not self._sensors_ready():
            return
        
        # Check if goal is set
        if self.goal is None:
            self._publish_stop()
            return
        
        # Check if reached goal (simple distance check)
        distance = np.linalg.norm(self.goal - self.position)
        if distance < self.cfg.goal_tolerance_m:
            self.get_logger().info('Goal reached')
            self._publish_stop()
            return
        
        # Check if ready for new step
        if not self._step_ready(now):
            # Between steps - just republish current command
            return
        
        # Ready for inference - run model
        try:
            # Preprocess depth
            depth_tensor = self._preprocess_depth()
            
            # Build state
            state_tensor = self._build_state()
            
            # Run inference
            with torch.inference_mode():
                mode, raw, _, _, h = self.model.act(
                    depth_tensor,
                    state_tensor,
                    self.h,
                    deterministic=True,
                    allowed=torch.ones(1, 6, dtype=torch.bool, device=self.device)
                )
            
            # Extract action
            mode = int(mode[0])
            turn = 0.0
            if mode == TURN:
                turn = float(raw[0].tanh()) * math.radians(self.cfg.max_turn_deg)
            
            # Commit new step
            self.last_step_time = now
            self.prev_mode = mode
            self.prev_turn = turn
            self.h = h.detach()
            
            # Publish command (both formats)
            self._publish_command(mode, turn)
            
            self.get_logger().info(
                f'Action: {ACTION_NAMES[mode]}, turn={math.degrees(turn):+.1f}°'
            )
            
        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
            self._publish_stop()
    
    # ------------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------------
    
    def _sensors_ready(self) -> bool:
        """Check if depth and odometry are recent."""
        if self.depth is None or self.depth_time is None:
            return False
        if self.position is None or self.odom_time is None:
            return False
        
        now = time.monotonic()
        timeout = 0.5  # 500ms timeout
        
        if now - self.depth_time > timeout:
            return False
        if now - self.odom_time > timeout:
            return False
        
        return True
    
    def _step_ready(self, now: float) -> bool:
        """Check if enough time has passed for new step."""
        if self.last_step_time is None:
            return True
        return (now - self.last_step_time) >= self.step_duration
    
    def _preprocess_depth(self) -> torch.Tensor:
        """Preprocess depth to model input."""
        d = torch.as_tensor(self.depth.copy(), device=self.device)[None]
        return depth_input(d, self.cfg)
    
    def _build_state(self) -> torch.Tensor:
        """Build state tensor for model."""
        t = lambda a: torch.tensor(a, dtype=torch.float32, device=self.device)
        
        return state_input(
            t([self.position.tolist()]),
            t([self.yaw]),
            t([self.goal.tolist()]),
            t([self.velocity.tolist()]),
            t([self.yaw_rate]),
            t([self.roll]),
            t([self.pitch]),
            torch.tensor([self.prev_mode], device=self.device),
            t([self.prev_turn]),
            self.cfg
        )
    
    def _action_to_string(self, mode: int, turn_rad: float) -> str:
        """
        Convert action to string command format.
        
        Format: "directionAuto_angle"
        Examples:
          - mode=1 (forward) → "forwardAuto_0"
          - mode=5 (turn), turn=+10° → "turnAuto_10"
          - mode=3 (left) → "leftAuto_0"
        """
        direction = ACTION_NAMES[mode]
        angle_deg = int(round(math.degrees(turn_rad)))
        return f"{direction}Auto_{angle_deg}"
    
    def _publish_command(self, mode: int, turn_rad: float) -> None:
        """
        Publish walk command in both formats:
          1. Float32MultiArray: [vx, vy, 0, 0, 0, yaw]
          2. String: "directionAuto_angle"
        """
        # Calculate endpoint displacement in body frame
        step_length = self.cfg.straight_step_m
        forward_proj = self.cfg.turn_forward_projection_m
        
        # Map mode to velocity
        if mode == STOP:
            vx, vy = 0.0, 0.0
            yaw = self.yaw
        elif mode == FORWARD:
            vx = step_length / self.step_duration
            vy = 0.0
            yaw = self.yaw
        elif mode == BACKWARD:
            vx = -step_length / self.step_duration
            vy = 0.0
            yaw = self.yaw
        elif mode == LEFT:
            vx = 0.0
            vy = step_length / self.step_duration
            yaw = self.yaw
        elif mode == RIGHT:
            vx = 0.0
            vy = -step_length / self.step_duration
            yaw = self.yaw
        elif mode == TURN:
            vx = forward_proj / self.step_duration
            vy = 0.0
            yaw = wrap_angle(self.yaw + turn_rad)
        else:
            vx, vy = 0.0, 0.0
            yaw = self.yaw
        
        # Publish Float32MultiArray
        vector_msg = Float32MultiArray()
        vector_msg.data = [float(vx), float(vy), 0.0, 0.0, 0.0, float(yaw)]
        self.control_pub.publish(vector_msg)
        
        # Publish String
        string_msg = String()
        string_msg.data = self._action_to_string(mode, turn_rad)
        self.control_string_pub.publish(string_msg)
    
    def _publish_stop(self) -> None:
        """Publish stop command in both formats."""
        # Float32MultiArray
        vector_msg = Float32MultiArray()
        yaw = self.yaw if self.yaw is not None else 0.0
        vector_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, float(yaw)]
        self.control_pub.publish(vector_msg)
        
        # # String
        # string_msg = String()
        # string_msg.data = "stopAuto_0"
        # self.control_string_pub.publish(string_msg)


# ============================================================================
# Main
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MinimalWalkController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
