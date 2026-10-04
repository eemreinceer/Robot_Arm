"""PyTorch IK model definitions for arm_ml."""

from __future__ import annotations

try:
    import torch
    import torch.nn as nn
except ImportError:  # pragma: no cover
    torch = None
    nn = None


class IKNet(nn.Module if nn is not None else object):
    """MLP mapping 7D TCP pose [xyz + quat] to 6 normalized joint angles."""

    def __init__(self) -> None:
        if nn is None:
            raise ImportError("PyTorch is required to instantiate IKNet")
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(7, 256), nn.BatchNorm1d(256), nn.ReLU(),
            nn.Linear(256, 512), nn.BatchNorm1d(512), nn.ReLU(), nn.Dropout(0.1),
            nn.Linear(512, 512), nn.BatchNorm1d(512), nn.ReLU(), nn.Dropout(0.1),
            nn.Linear(512, 256), nn.BatchNorm1d(256), nn.ReLU(),
            nn.Linear(256, 6), nn.Tanh(),
        )

    def forward(self, inputs):
        return self.net(inputs)


def load_model(checkpoint_path: str, device: str = "cpu"):
    if torch is None:
        raise ImportError("PyTorch is required to load IKNet")
    model = IKNet().to(device)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    state = checkpoint.get("model_state_dict", checkpoint) if isinstance(checkpoint, dict) else checkpoint
    model.load_state_dict(state)
    model.eval()
    return model
