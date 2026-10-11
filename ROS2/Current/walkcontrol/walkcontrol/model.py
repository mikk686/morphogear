"""Small CNN + GRU with ONE public output: the six-element control vector.

The second return value is recurrent memory, not another prediction head.
A value function and an exploration distribution exist only in train.py.
"""
import math
from types import SimpleNamespace

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

if __package__:
    from .commands import project
else:
    from commands import project

FORMAT = "diffphyswalk-vector-v1"
HIDDEN_SIZE = 96
STATE_SIZE = 10


def depth_input(depth_m, cfg):
    """Metres [B,H,W] -> inverse depth + coverage [B,2,36,64], without a crop.

    Invalid measurements do not become nearby obstacles. Area averaging of
    valid inverse depth reduces isolated speckles; coverage keeps unknown
    regions distinguishable from measured free space.
    """
    valid = torch.isfinite(depth_m) & (depth_m >= cfg.depth_min_m) & (depth_m <= cfg.depth_max_m)
    clean = torch.where(valid, depth_m, cfg.depth_max_m)
    inverse = (1. / clean - 1. / cfg.depth_max_m) / (1. / cfg.depth_min_m - 1. / cfg.depth_max_m)
    size = (cfg.height, cfg.width)
    coverage = F.adaptive_avg_pool2d(valid[:, None].float(), size)
    average = F.adaptive_avg_pool2d(torch.where(valid, inverse, 0.)[:, None], size)
    return torch.cat([average / coverage.clamp_min(1e-6), coverage], 1)


def state_input(xy, yaw, goal, roll, pitch, previous):
    delta = goal - xy
    c, s = yaw.cos(), yaw.sin()
    body = torch.stack([delta[:, 0] * c + delta[:, 1] * s,
                        -delta[:, 0] * s + delta[:, 1] * c], -1)
    distance = body.norm(dim=-1, keepdim=True)
    return torch.cat([body / distance.clamp_min(1e-6), (distance / 6).clamp(0, 2),
                      roll.sin()[:, None], pitch.sin()[:, None], previous,
                      yaw.sin()[:, None], yaw.cos()[:, None]], -1)


class WalkPolicy(nn.Module):
    def __init__(self, cfg):
        super().__init__()
        self.cfg = cfg
        self.cnn = nn.Sequential(
            nn.Conv2d(2, 16, 3, 2, 1), nn.LeakyReLU(.05),
            nn.Conv2d(16, 32, 3, 2, 1), nn.LeakyReLU(.05),
            nn.Conv2d(32, 64, 3, 2, 1), nn.LeakyReLU(.05),
            nn.AdaptiveAvgPool2d((3, 5)), nn.Flatten(), nn.Linear(64 * 3 * 5, HIDDEN_SIZE),
        )
        self.state = nn.Linear(STATE_SIZE - 2, HIDDEN_SIZE)
        self.gru = nn.GRUCell(HIDDEN_SIZE, HIDDEN_SIZE)
        self.vector_head = nn.Linear(HIDDEN_SIZE, 3)
        nn.init.normal_(self.vector_head.weight, std=.01)
        nn.init.zeros_(self.vector_head.bias)

    def features(self, image, state, hidden=None):
        # Absolute yaw is needed only to express the final command in world axes.
        # The learned decision uses body-relative inputs and stays rotation invariant.
        fused = F.leaky_relu(self.cnn(image) + self.state(state[..., :-2]), .05)
        return self.gru(fused, hidden)

    def latent(self, image, state, hidden=None):
        hidden = self.features(image, state, hidden)
        return self.vector_head(F.leaky_relu(hidden, .05)), hidden

    def forward(self, image, state, hidden=None):
        raw, hidden = self.latent(image, state, hidden)
        yaw = torch.atan2(state[..., -2], state[..., -1])
        return project(raw, yaw, self.cfg), hidden


def load_policy(path, device="cpu"):
    checkpoint = torch.load(path, map_location=device, weights_only=True)
    if checkpoint.get("format") != FORMAT:
        raise ValueError("Use a WalkVector_v1 checkpoint; walk_v2/v3 heads are incompatible")
    cfg = SimpleNamespace(**checkpoint["config"])
    network = WalkPolicy(cfg).to(device)
    network.load_state_dict(checkpoint["model"], strict=True)
    return network.eval(), cfg, checkpoint


def quaternion_to_rpy(x, y, z, w):
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("Invalid pose quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(float(np.clip(2*(w*y-z*x), -1, 1))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def decode_depth(msg, unit_m=.001):
    """ROS 16UC1 (millimetres by default) or 32FC1 (metres); honours row padding."""
    if msg.encoding in ("16UC1", "mono16"):
        dtype, scale = np.dtype(">u2" if msg.is_bigendian else "<u2"), unit_m
    elif msg.encoding == "32FC1":
        dtype, scale = np.dtype(">f4" if msg.is_bigendian else "<f4"), 1.
    else:
        raise ValueError(f"Unsupported depth encoding: {msg.encoding}")
    if msg.step < msg.width*dtype.itemsize or len(msg.data) < msg.step*msg.height:
        raise ValueError("Depth buffer or row stride is too small")
    data = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.step, dtype.itemsize))
    return data.astype(np.float32) * scale
