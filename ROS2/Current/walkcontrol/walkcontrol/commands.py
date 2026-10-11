"""One shared command convention for the network, simulator and ROS2.

Vector: [vx, vy, 0, 0, 0, absolute_yaw_rad]. Body axes are FLU.
Velocities mean displacement per step divided by the step duration.
"""
import math

import torch

STOP, FORWARD, BACKWARD, LEFT, RIGHT, ROTATE, FORWARD_TURN = range(7)
ACTION_NAMES = ["stop", "forward", "backward", "left", "right", "rotate", "forward_turn"]


def wrap(angle):
    return torch.atan2(angle.sin(), angle.cos())


def project(raw, yaw, cfg):
    """Project three continuous network outputs onto supported walking steps.

    The network has no mode/classification head. Nearest-vector projection
    belongs to its output layer and is also applied to sampled PPO actions.
    """
    desired = raw.tanh()
    candidates = raw.new_zeros(*raw.shape[:-1], 7, 3)
    candidates[..., FORWARD, 0] = 1.
    candidates[..., BACKWARD, 0] = -1.
    candidates[..., LEFT, 1] = 1.
    candidates[..., RIGHT, 1] = -1.
    ratio = cfg.turn_forward_projection_m / cfg.straight_step_m
    limit = cfg.max_forward_turn_deg / cfg.max_rotate_deg
    candidates[..., ROTATE, 2] = desired[..., 2]
    candidates[..., FORWARD_TURN, 0] = ratio
    candidates[..., FORWARD_TURN, 2] = desired[..., 2].clamp(-limit, limit)
    costs = (candidates - desired[..., None, :]).square().sum(-1)
    # At zero yaw a turning gait must not silently become an 11 cm straight gait.
    tiny_turn = desired[..., 2].abs() * cfg.max_rotate_deg < cfg.turn_deadband_deg
    costs[..., ROTATE] = costs[..., ROTATE].masked_fill(tiny_turn, float("inf"))
    costs[..., FORWARD_TURN] = costs[..., FORWARD_TURN].masked_fill(tiny_turn, float("inf"))
    index = costs.argmin(-1)
    chosen = candidates.gather(-2, index[..., None, None].expand(*index.shape, 1, 3)).squeeze(-2)
    vector = raw.new_zeros(*raw.shape[:-1], 6)
    vector[..., :2] = chosen[..., :2] * (cfg.straight_step_m / cfg.step_duration_s)
    vector[..., 5] = wrap(yaw + chosen[..., 2] * math.radians(cfg.max_rotate_deg))
    return vector


def relative(vector, yaw, cfg):
    """Executed command normalized to [-1,1], for the next policy observation."""
    speed = cfg.straight_step_m / cfg.step_duration_s
    return torch.cat([vector[..., :2] / speed,
                      wrap(vector[..., 5] - yaw)[..., None] / math.radians(cfg.max_rotate_deg)], -1)


def identify(vector, yaw, cfg):
    """Names for metrics only. No mode is sent to the robot."""
    vx, vy = vector[..., 0], vector[..., 1]
    angle = wrap(vector[..., 5] - yaw)
    mode = torch.zeros_like(vx, dtype=torch.long)
    turning = angle.abs() >= math.radians(cfg.turn_deadband_deg) - 1e-6
    moving = vx.abs() + vy.abs() > 1e-6
    mode = torch.where(vx > 1e-6, FORWARD, mode)
    mode = torch.where(vx < -1e-6, BACKWARD, mode)
    mode = torch.where(vy > 1e-6, LEFT, mode)
    mode = torch.where(vy < -1e-6, RIGHT, mode)
    mode = torch.where(turning & ~moving, ROTATE, mode)
    return torch.where(turning & moving, FORWARD_TURN, mode)


def latent_target(vector, yaw, cfg):
    """Training-only target: center of the desired command's projection region."""
    return torch.atanh(relative(vector, yaw, cfg).clamp(-.96, .96))


def make_vector(vx, vy, dyaw_deg, yaw):
    """Create a feasible teacher/test command; yaw can be a batch tensor."""
    vector = yaw.new_zeros(*yaw.shape, 6)
    vector[..., 0], vector[..., 1] = vx, vy
    vector[..., 5] = wrap(yaw + torch.as_tensor(dyaw_deg, device=yaw.device) * math.pi / 180)
    return vector
