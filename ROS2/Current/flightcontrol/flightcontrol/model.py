from collections.abc import Mapping
from pathlib import Path

import torch
from torch import nn
from torch.nn import functional as F


class Model(nn.Module):
    def __init__(self, dim_obs=10, dim_action=6, input_hw=(12, 16)):
        super().__init__()
        if tuple(input_hw) != (12, 16):
            raise ValueError('Baseline input must be 12x16')
        self.input_hw = tuple(input_hw)
        self.dim_obs = dim_obs
        height, width = (v // 2 - 4 for v in self.input_hw)
        self.stem = nn.Sequential(
            nn.Conv2d(1, 32, 2, 2, bias=False), nn.LeakyReLU(.05),
            nn.Conv2d(32, 64, 3, bias=False), nn.LeakyReLU(.05),
            nn.Conv2d(64, 128, 3, bias=False), nn.LeakyReLU(.05),
            nn.Flatten(), nn.Linear(128 * height * width, 192, bias=False))
        self.v_proj = nn.Linear(dim_obs, 192)
        self.gru = nn.GRUCell(192, 192)
        self.fc = nn.Linear(192, dim_action, bias=False)
        self.act = nn.LeakyReLU(.05)
        with torch.no_grad():
            self.v_proj.weight.mul_(.5)
            self.fc.weight.mul_(.01)

    def reset(self):
        """Hidden state belongs to the controller and is passed explicitly."""

    def forward(self, depth, state, hidden=None):
        if depth.ndim != 4 or tuple(depth.shape[1:]) != (1, *self.input_hw):
            raise ValueError(f'Expected [B,1,{self.input_hw[0]},{self.input_hw[1]}]')
        if tuple(state.shape) != (depth.shape[0], self.dim_obs):
            raise ValueError(f'Expected observation [B,{self.dim_obs}]')
        hidden = self.gru(self.act(self.stem(depth) + self.v_proj(state)), hidden)
        return self.fc(self.act(hidden)), None, hidden


def depth_input(depth_m, input_hw=(12, 16), crop_ratio=1.):
    """Metric [48,64] -> center crop -> normalize -> max-pool -> [1,1,H,W].

Pooling normalized inverse depth keeps the nearest obstacle in each 4x4 cell,
matching the baseline training pipeline. Inference adds no training noise.
"""
    if tuple(depth_m.shape) != (48, 64) or not 0 < crop_ratio <= 1:
        raise ValueError('Expected metric depth [48,64] and crop_ratio in (0,1]')
    if tuple(input_hw) != (12, 16):
        raise ValueError('Unsupported baseline input size')
    h = max(1, round(48 * crop_ratio))
    w = max(1, round(64 * crop_ratio))
    y, x = (48 - h) // 2, (64 - w) // 2
    depth = depth_m[y:y+h, x:x+w][None, None]
    depth = torch.where(torch.isfinite(depth) & (depth > 0), depth, .3)
    size = tuple(v * 4 for v in input_hw)
    if tuple(depth.shape[-2:]) != size:
        depth = F.interpolate(depth, size=size, mode='nearest')
    return F.max_pool2d(3. / depth.clamp(.3, 24.) - .6, 4, 4)


def clean_weights(obj):
    if not isinstance(obj, Mapping):
        raise ValueError('Checkpoint must contain a state_dict')
    for key in ('state_dict', 'model_state_dict', 'model'):
        if key in obj:
            obj = obj[key]
            break
    if not isinstance(obj, Mapping):
        raise ValueError('Checkpoint model must be a state_dict, not a pickled model')
    rename = {'observation_fc.weight': 'v_proj.weight',
              'observation_fc.bias': 'v_proj.bias', 'action_fc.weight': 'fc.weight'}
    out = {}
    for key, value in obj.items():
        if not isinstance(key, str) or not isinstance(value, torch.Tensor):
            raise ValueError('State dictionary must contain only named tensors')
        while key.startswith(('module.', '_orig_mod.')):
            key = key.split('.', 1)[1]
        key = rename.get(key, key)
        if key in out:
            raise ValueError(f'Duplicate checkpoint key: {key}')
        out[key] = value
    return out


def inference_model(path, dim_obs=10, device='cpu'):
    path = Path(path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f'Baseline weights not found: {path}. Set net_weight.')
    state = clean_weights(torch.load(path, map_location='cpu', weights_only=True))
    shape = tuple(state.get('stem.7.weight', torch.empty(0)).shape)
    inputs = {(192, 1024): (12, 16)}
    if shape not in inputs:
        raise ValueError(f'Unsupported baseline CNN projection {shape}; expected '
                         'stem.7.weight [192,1024]. B3 weights do not fit.')
    model = Model(dim_obs, 6, inputs[shape])
    try:
        model.load_state_dict(state, strict=True)
    except RuntimeError as exc:
        raise ValueError('Incompatible baseline weights: need observation=10, '
                         'action=6, GRU=192 and matching CNN.\n' + str(exc)) from exc
    if any(not bool(torch.isfinite(t).all()) for t in model.state_dict().values()):
        raise ValueError('Checkpoint contains non-finite weights')
    return model.to(device).eval()
