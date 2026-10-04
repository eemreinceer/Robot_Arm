#!/usr/bin/env python3
"""Train IKNet from an arm_ml generated NPZ dataset."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

from arm_ml.kinematics import JOINT_LIMITS, URDF_JOINTS
from arm_ml.model import IKNet

try:
    from torch.utils.tensorboard import SummaryWriter
except ImportError:  # pragma: no cover
    SummaryWriter = None


def _subset(data: np.ndarray, mask: np.ndarray) -> torch.Tensor:
    return torch.from_numpy(data[mask].astype(np.float32))


def _origin_transform(xyz, rpy, device: torch.device, dtype: torch.dtype) -> torch.Tensor:
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float32)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float32)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float32)
    transform = np.eye(4, dtype=np.float32)
    transform[:3, :3] = rz @ ry @ rx
    transform[:3, 3] = np.asarray(xyz, dtype=np.float32)
    return torch.tensor(transform, device=device, dtype=dtype)


def _axis_rotation(axis_values, angle: torch.Tensor) -> torch.Tensor:
    axis = torch.tensor(axis_values, device=angle.device, dtype=angle.dtype)
    axis = axis / torch.clamp(torch.linalg.norm(axis), min=1e-12)
    x, y, z = axis[0], axis[1], axis[2]
    c = torch.cos(angle)
    s = torch.sin(angle)
    cc = 1.0 - c
    batch = angle.shape[0]
    rot = torch.zeros((batch, 4, 4), device=angle.device, dtype=angle.dtype)
    rot[:, 0, 0] = c + x * x * cc
    rot[:, 0, 1] = x * y * cc - z * s
    rot[:, 0, 2] = x * z * cc + y * s
    rot[:, 1, 0] = y * x * cc + z * s
    rot[:, 1, 1] = c + y * y * cc
    rot[:, 1, 2] = y * z * cc - x * s
    rot[:, 2, 0] = z * x * cc - y * s
    rot[:, 2, 1] = z * y * cc + x * s
    rot[:, 2, 2] = c + z * z * cc
    rot[:, 3, 3] = 1.0
    return rot


def _denormalize_joints_torch(normalized_joints: torch.Tensor) -> torch.Tensor:
    limits = torch.tensor(JOINT_LIMITS, device=normalized_joints.device, dtype=normalized_joints.dtype)
    center = (limits[:, 0] + limits[:, 1]) / 2.0
    half_range = (limits[:, 1] - limits[:, 0]) / 2.0
    return normalized_joints * half_range + center


def torch_fk_positions(normalized_joints: torch.Tensor) -> torch.Tensor:
    joint_angles = _denormalize_joints_torch(normalized_joints)
    batch = joint_angles.shape[0]
    transform = torch.eye(4, device=joint_angles.device, dtype=joint_angles.dtype).repeat(batch, 1, 1)
    for index, (xyz, rpy, axis) in enumerate(URDF_JOINTS):
        origin = _origin_transform(xyz, rpy, joint_angles.device, joint_angles.dtype).unsqueeze(0).repeat(batch, 1, 1)
        transform = transform @ origin @ _axis_rotation(axis, joint_angles[:, index])
    return transform[:, :3, 3]


def train(
    dataset: Path,
    output: Path,
    epochs: int,
    batch_size: int,
    lr: float,
    patience: int,
    fk_lambda: float,
    log_dir: Optional[Path],
    resume: Optional[Path],
) -> None:
    loaded = np.load(dataset)
    poses = loaded["normalized_poses"]
    joints = loaded["normalized_joints"]
    split = loaded["split"]
    train_ds = TensorDataset(_subset(poses, split == 0), _subset(joints, split == 0))
    val_ds = TensorDataset(_subset(poses, split == 1), _subset(joints, split == 1))
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True, drop_last=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = IKNet().to(device)
    if resume is not None:
        checkpoint = torch.load(resume, map_location=device)
        model.load_state_dict(checkpoint.get("model_state_dict", checkpoint))
        print(f"resumed model weights from {resume}")
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=max(1, epochs))
    criterion = nn.MSELoss()
    writer = SummaryWriter(str(log_dir)) if SummaryWriter is not None and log_dir is not None else None
    best_val = float("inf")
    stale_epochs = 0
    output.parent.mkdir(parents=True, exist_ok=True)

    for epoch in range(1, epochs + 1):
        model.train()
        train_loss = 0.0
        train_joint_loss = 0.0
        train_fk_loss = 0.0
        seen = 0
        for x, y in train_loader:
            x, y = x.to(device), y.to(device)
            optimizer.zero_grad(set_to_none=True)
            pred = model(x)
            joint_loss = criterion(pred, y)
            fk_loss = criterion(torch_fk_positions(pred), x[:, :3])
            loss = joint_loss + fk_lambda * fk_loss
            loss.backward()
            optimizer.step()
            count = x.shape[0]
            train_loss += loss.item() * count
            train_joint_loss += joint_loss.item() * count
            train_fk_loss += fk_loss.item() * count
            seen += count
        scheduler.step()
        train_loss /= max(1, seen)
        train_joint_loss /= max(1, seen)
        train_fk_loss /= max(1, seen)

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for x, y in val_loader:
                x, y = x.to(device), y.to(device)
                pred = model(x)
                joint_loss = criterion(pred, y)
                fk_loss = criterion(torch_fk_positions(pred), x[:, :3])
                val_loss += (joint_loss + fk_lambda * fk_loss).item() * x.shape[0]
        val_loss /= max(1, len(val_ds))
        print(
            f"epoch={epoch} train_loss={train_loss:.6f} "
            f"joint_loss={train_joint_loss:.6f} fk_loss={train_fk_loss:.6f} val_loss={val_loss:.6f}"
        )
        if writer is not None:
            writer.add_scalar("loss/train", train_loss, epoch)
            writer.add_scalar("loss/train_joint", train_joint_loss, epoch)
            writer.add_scalar("loss/train_fk_position", train_fk_loss, epoch)
            writer.add_scalar("loss/val", val_loss, epoch)
            writer.add_scalar("lr", scheduler.get_last_lr()[0], epoch)

        if val_loss < best_val:
            best_val = val_loss
            stale_epochs = 0
            torch.save(
                {
                    "model_state_dict": model.state_dict(),
                    "val_loss": best_val,
                    "fk_lambda": fk_lambda,
                },
                output,
            )
        else:
            stale_epochs += 1
            if stale_epochs >= patience:
                print(f"early stopping after {epoch} epochs")
                break

    if writer is not None:
        writer.close()
    checkpoint = torch.load(output, map_location=device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    scripted_path = output.with_name(output.stem + "_scripted.pt")
    scripted = torch.jit.script(model)
    scripted.save(str(scripted_path))
    print(f"best_val={best_val:.6f} checkpoint={output} torchscript={scripted_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=Path("src/arm_ml/data/ik_dataset_v1.npz"))
    parser.add_argument("--output", type=Path, default=Path("src/arm_ml/models/ik_net.pt"))
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--patience", type=int, default=10)
    parser.add_argument("--fk-lambda", type=float, default=10.0)
    parser.add_argument("--log-dir", type=Path, default=Path("src/arm_ml/runs/ik_net"))
    parser.add_argument("--resume", type=Path)
    args = parser.parse_args()
    train(args.dataset, args.output, args.epochs, args.batch_size, args.lr, args.patience, args.fk_lambda, args.log_dir, args.resume)


if __name__ == "__main__":
    main()
