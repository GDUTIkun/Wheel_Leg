#!/usr/bin/env python3
"""Capture STM32 state frames, save CSV, and generate analysis plots."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import struct
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable


HEAD = b"\xA5\x5A"
FRAME_TYPE_STATE = 0x81
JOINT_COUNT = 6
MAX_PAYLOAD_LEN = 160
STATE_PAYLOAD_SIZE = 4 + 9 * 4 + JOINT_COUNT * 3 * 4 + 4 + 3 * 4

PI = math.pi
L1 = 0.18
L2 = 0.225

PLOT_GROUP_ALIASES = {
    "all": [
        "imu_euler",
        "gyro",
        "accel",
        "joint_pos",
        "joint_vel",
        "joint_effort",
        "leg_pose",
        "leg_rate",
        "status",
    ],
    "all_robot_states": [
        "imu_euler",
        "gyro",
        "accel",
        "joint_pos",
        "joint_vel",
        "joint_effort",
        "leg_pose",
        "leg_rate",
        "status",
    ],
}

PLOT_GROUP_CHOICES = sorted(
    {
        "all",
        "all_robot_states",
        "imu_euler",
        "gyro",
        "accel",
        "joint_pos",
        "joint_vel",
        "joint_effort",
        "leg_pose",
        "leg_rate",
        "status",
    }
)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def read_u32_le(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def read_f32_le(data: bytes, offset: int) -> float:
    return struct.unpack_from("<f", data, offset)[0]


def normalize_angle_delta(angle_delta: float) -> float:
    while angle_delta > PI:
        angle_delta -= 2.0 * PI
    while angle_delta < -PI:
        angle_delta += 2.0 * PI
    return angle_delta


def compute_leg_kinematics(hip_absolute: float, calf_absolute: float) -> tuple[float, float]:
    x = L1 * math.cos(hip_absolute) + L2 * math.cos(calf_absolute)
    y_clockwise = L1 * math.sin(hip_absolute) + L2 * math.sin(calf_absolute)
    leg_length = math.hypot(x, y_clockwise)
    phi = math.atan2(y_clockwise, x)
    return leg_length, phi


def finite_mean(values: Iterable[float]) -> float:
    clean = [value for value in values if math.isfinite(value)]
    if not clean:
        return math.nan
    return statistics.fmean(clean)


def finite_std(values: Iterable[float]) -> float:
    clean = [value for value in values if math.isfinite(value)]
    if len(clean) < 2:
        return math.nan
    return statistics.pstdev(clean)


def expand_plot_groups(spec: str) -> list[str]:
    selected: list[str] = []
    for raw_name in spec.split(","):
        name = raw_name.strip()
        if not name:
            continue
        expanded = PLOT_GROUP_ALIASES.get(name, [name])
        for item in expanded:
            if item not in PLOT_GROUP_CHOICES:
                raise SystemExit(
                    f"Unknown plot group '{item}'. Choices: {', '.join(PLOT_GROUP_CHOICES)}"
                )
            if item not in selected and item not in PLOT_GROUP_ALIASES:
                selected.append(item)
    if not selected:
        raise SystemExit("At least one plot group must be selected.")
    return selected


@dataclass
class ProtocolStateFrame:
    stm_tick_ms: int
    roll: float
    pitch: float
    yaw: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    acc_x: float
    acc_y: float
    acc_z: float
    joint_position: list[float]
    joint_velocity: list[float]
    joint_effort: list[float]
    online_mask: int
    safety_state: int
    last_command_timeout: int
    knee_limit_flag: int
    comm_rx_error_count: int
    comm_crc_error_count: int
    can_error_count: int


@dataclass
class CaptureRow:
    host_time_sec: float
    elapsed_sec: float
    stm_tick_ms: int
    stm_time_sec: float
    roll: float
    pitch: float
    yaw: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    acc_x: float
    acc_y: float
    acc_z: float
    left_hip_pos: float
    left_knee_pos: float
    left_wheel_pos: float
    right_hip_pos: float
    right_knee_pos: float
    right_wheel_pos: float
    left_hip_vel: float
    left_knee_vel: float
    left_wheel_vel: float
    right_hip_vel: float
    right_knee_vel: float
    right_wheel_vel: float
    left_hip_effort: float
    left_knee_effort: float
    left_wheel_effort: float
    right_hip_effort: float
    right_knee_effort: float
    right_wheel_effort: float
    left_leg_length: float
    right_leg_length: float
    left_phi: float
    right_phi: float
    left_phi_rate_raw: float
    right_phi_rate_raw: float
    left_phi_rate_lpf: float
    right_phi_rate_lpf: float
    left_length_rate_raw: float
    right_length_rate_raw: float
    left_length_rate_lpf: float
    right_length_rate_lpf: float
    online_mask: int
    safety_state: int
    last_command_timeout: int
    knee_limit_flag: int
    comm_rx_error_count: int
    comm_crc_error_count: int
    can_error_count: int


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.frames_ok = 0
        self.crc_errors = 0
        self.length_errors = 0
        self.sync_losses = 0
        self.unknown_types = 0

    def push(self, chunk: bytes) -> list[ProtocolStateFrame]:
        self.buffer.extend(chunk)
        frames: list[ProtocolStateFrame] = []

        while True:
            head_index = self.buffer.find(HEAD)
            if head_index < 0:
                if self.buffer:
                    self.sync_losses += len(self.buffer)
                    keep = self.buffer[-1:] if self.buffer[-1:] == HEAD[:1] else b""
                    self.buffer.clear()
                    self.buffer.extend(keep)
                break

            if head_index > 0:
                self.sync_losses += head_index
                del self.buffer[:head_index]

            if len(self.buffer) < 6:
                break

            payload_len = self.buffer[3]
            if payload_len > MAX_PAYLOAD_LEN:
                self.length_errors += 1
                del self.buffer[0]
                continue

            frame_len = 2 + 1 + 1 + 2 + payload_len + 2
            if len(self.buffer) < frame_len:
                break

            body = bytes(self.buffer[2 : 2 + 4 + payload_len])
            expected_crc = crc16_ccitt(body)
            actual_crc = struct.unpack_from("<H", self.buffer, 2 + 4 + payload_len)[0]
            if actual_crc != expected_crc:
                self.crc_errors += 1
                del self.buffer[0]
                continue

            frame_type = self.buffer[2]
            payload = bytes(self.buffer[6 : 6 + payload_len])
            del self.buffer[:frame_len]

            if frame_type != FRAME_TYPE_STATE:
                self.unknown_types += 1
                continue
            if payload_len != STATE_PAYLOAD_SIZE:
                self.length_errors += 1
                continue

            decoded = decode_state_payload(payload)
            if decoded is None:
                self.length_errors += 1
                continue
            self.frames_ok += 1
            frames.append(decoded)

        return frames


def decode_state_payload(payload: bytes) -> ProtocolStateFrame | None:
    if len(payload) != STATE_PAYLOAD_SIZE:
        return None

    offset = 0
    stm_tick_ms = read_u32_le(payload, offset)
    offset += 4
    roll = read_f32_le(payload, offset)
    offset += 4
    pitch = read_f32_le(payload, offset)
    offset += 4
    yaw = read_f32_le(payload, offset)
    offset += 4
    gyro_x = read_f32_le(payload, offset)
    offset += 4
    gyro_y = read_f32_le(payload, offset)
    offset += 4
    gyro_z = read_f32_le(payload, offset)
    offset += 4
    acc_x = read_f32_le(payload, offset)
    offset += 4
    acc_y = read_f32_le(payload, offset)
    offset += 4
    acc_z = read_f32_le(payload, offset)
    offset += 4

    joint_position = []
    joint_velocity = []
    joint_effort = []
    for _ in range(JOINT_COUNT):
        joint_position.append(read_f32_le(payload, offset))
        offset += 4
        joint_velocity.append(read_f32_le(payload, offset))
        offset += 4
        joint_effort.append(read_f32_le(payload, offset))
        offset += 4

    online_mask = payload[offset]
    offset += 1
    safety_state = payload[offset]
    offset += 1
    last_command_timeout = payload[offset]
    offset += 1
    knee_limit_flag = payload[offset]
    offset += 1
    comm_rx_error_count = read_u32_le(payload, offset)
    offset += 4
    comm_crc_error_count = read_u32_le(payload, offset)
    offset += 4
    can_error_count = read_u32_le(payload, offset)

    return ProtocolStateFrame(
        stm_tick_ms=stm_tick_ms,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        gyro_x=gyro_x,
        gyro_y=gyro_y,
        gyro_z=gyro_z,
        acc_x=acc_x,
        acc_y=acc_y,
        acc_z=acc_z,
        joint_position=joint_position,
        joint_velocity=joint_velocity,
        joint_effort=joint_effort,
        online_mask=online_mask,
        safety_state=safety_state,
        last_command_timeout=last_command_timeout,
        knee_limit_flag=knee_limit_flag,
        comm_rx_error_count=comm_rx_error_count,
        comm_crc_error_count=comm_crc_error_count,
        can_error_count=can_error_count,
    )


def build_row(
    frame: ProtocolStateFrame,
    host_time_sec: float,
    first_host_time_sec: float,
    prev_row: CaptureRow | None,
    phi_rate_lpf_alpha: float,
    length_rate_lpf_alpha: float,
) -> CaptureRow:
    left_leg_length, left_phi = compute_leg_kinematics(
        frame.joint_position[0], frame.joint_position[1]
    )
    right_leg_length, right_phi = compute_leg_kinematics(
        frame.joint_position[3], frame.joint_position[4]
    )

    left_phi_rate_raw = math.nan
    right_phi_rate_raw = math.nan
    left_length_rate_raw = math.nan
    right_length_rate_raw = math.nan
    left_phi_rate_lpf = math.nan
    right_phi_rate_lpf = math.nan
    left_length_rate_lpf = math.nan
    right_length_rate_lpf = math.nan

    if prev_row is None:
        left_phi_rate_lpf = 0.0
        right_phi_rate_lpf = 0.0
        left_length_rate_lpf = 0.0
        right_length_rate_lpf = 0.0
    else:
        dt = (frame.stm_tick_ms - prev_row.stm_tick_ms) * 1e-3
        if dt > 0.0:
            left_phi_rate_raw = normalize_angle_delta(left_phi - prev_row.left_phi) / dt
            right_phi_rate_raw = normalize_angle_delta(right_phi - prev_row.right_phi) / dt
            left_length_rate_raw = (left_leg_length - prev_row.left_leg_length) / dt
            right_length_rate_raw = (right_leg_length - prev_row.right_leg_length) / dt

            left_phi_rate_lpf = (
                phi_rate_lpf_alpha * prev_row.left_phi_rate_lpf
                + (1.0 - phi_rate_lpf_alpha) * left_phi_rate_raw
            )
            right_phi_rate_lpf = (
                phi_rate_lpf_alpha * prev_row.right_phi_rate_lpf
                + (1.0 - phi_rate_lpf_alpha) * right_phi_rate_raw
            )
            left_length_rate_lpf = (
                length_rate_lpf_alpha * prev_row.left_length_rate_lpf
                + (1.0 - length_rate_lpf_alpha) * left_length_rate_raw
            )
            right_length_rate_lpf = (
                length_rate_lpf_alpha * prev_row.right_length_rate_lpf
                + (1.0 - length_rate_lpf_alpha) * right_length_rate_raw
            )
        else:
            left_phi_rate_lpf = prev_row.left_phi_rate_lpf
            right_phi_rate_lpf = prev_row.right_phi_rate_lpf
            left_length_rate_lpf = prev_row.left_length_rate_lpf
            right_length_rate_lpf = prev_row.right_length_rate_lpf

    return CaptureRow(
        host_time_sec=host_time_sec,
        elapsed_sec=host_time_sec - first_host_time_sec,
        stm_tick_ms=frame.stm_tick_ms,
        stm_time_sec=frame.stm_tick_ms * 1e-3,
        roll=frame.roll,
        pitch=frame.pitch,
        yaw=frame.yaw,
        gyro_x=frame.gyro_x,
        gyro_y=frame.gyro_y,
        gyro_z=frame.gyro_z,
        acc_x=frame.acc_x,
        acc_y=frame.acc_y,
        acc_z=frame.acc_z,
        left_hip_pos=frame.joint_position[0],
        left_knee_pos=frame.joint_position[1],
        left_wheel_pos=frame.joint_position[2],
        right_hip_pos=frame.joint_position[3],
        right_knee_pos=frame.joint_position[4],
        right_wheel_pos=frame.joint_position[5],
        left_hip_vel=frame.joint_velocity[0],
        left_knee_vel=frame.joint_velocity[1],
        left_wheel_vel=frame.joint_velocity[2],
        right_hip_vel=frame.joint_velocity[3],
        right_knee_vel=frame.joint_velocity[4],
        right_wheel_vel=frame.joint_velocity[5],
        left_hip_effort=frame.joint_effort[0],
        left_knee_effort=frame.joint_effort[1],
        left_wheel_effort=frame.joint_effort[2],
        right_hip_effort=frame.joint_effort[3],
        right_knee_effort=frame.joint_effort[4],
        right_wheel_effort=frame.joint_effort[5],
        left_leg_length=left_leg_length,
        right_leg_length=right_leg_length,
        left_phi=left_phi,
        right_phi=right_phi,
        left_phi_rate_raw=left_phi_rate_raw,
        right_phi_rate_raw=right_phi_rate_raw,
        left_phi_rate_lpf=left_phi_rate_lpf,
        right_phi_rate_lpf=right_phi_rate_lpf,
        left_length_rate_raw=left_length_rate_raw,
        right_length_rate_raw=right_length_rate_raw,
        left_length_rate_lpf=left_length_rate_lpf,
        right_length_rate_lpf=right_length_rate_lpf,
        online_mask=frame.online_mask,
        safety_state=frame.safety_state,
        last_command_timeout=frame.last_command_timeout,
        knee_limit_flag=frame.knee_limit_flag,
        comm_rx_error_count=frame.comm_rx_error_count,
        comm_crc_error_count=frame.comm_crc_error_count,
        can_error_count=frame.can_error_count,
    )


def write_csv(rows: list[CaptureRow], path: Path) -> None:
    if not rows:
        return
    with path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(asdict(rows[0]).keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def write_summary(rows: list[CaptureRow], parser: FrameParser, path: Path) -> None:
    duration = rows[-1].elapsed_sec - rows[0].elapsed_sec if len(rows) > 1 else 0.0
    sample_rate = (len(rows) - 1) / duration if duration > 0.0 else math.nan
    left_phi_std_deg = math.degrees(finite_std(row.left_phi for row in rows))
    right_phi_std_deg = math.degrees(finite_std(row.right_phi for row in rows))
    left_length_std_mm = finite_std(row.left_leg_length for row in rows) * 1000.0
    right_length_std_mm = finite_std(row.right_leg_length for row in rows) * 1000.0
    left_hip_vel_std = finite_std(row.left_hip_vel for row in rows)
    left_knee_vel_std = finite_std(row.left_knee_vel for row in rows)
    left_wheel_vel_std = finite_std(row.left_wheel_vel for row in rows)
    right_hip_vel_std = finite_std(row.right_hip_vel for row in rows)
    right_knee_vel_std = finite_std(row.right_knee_vel for row in rows)
    right_wheel_vel_std = finite_std(row.right_wheel_vel for row in rows)

    lines = [
        f"samples={len(rows)}",
        f"duration_sec={duration:.6f}",
        f"effective_sample_rate_hz={sample_rate:.3f}",
        f"frames_ok={parser.frames_ok}",
        f"crc_errors={parser.crc_errors}",
        f"length_errors={parser.length_errors}",
        f"sync_losses={parser.sync_losses}",
        f"unknown_types={parser.unknown_types}",
        "",
        "left_leg:",
        f"  phi_mean_deg={math.degrees(finite_mean(row.left_phi for row in rows)):.6f}",
        f"  phi_std_deg={left_phi_std_deg:.6f}",
        f"  phi_rate_raw_std_deg_s={math.degrees(finite_std(row.left_phi_rate_raw for row in rows)):.6f}",
        f"  phi_rate_lpf_std_deg_s={math.degrees(finite_std(row.left_phi_rate_lpf for row in rows)):.6f}",
        f"  length_mean_m={finite_mean(row.left_leg_length for row in rows):.6f}",
        f"  length_std_mm={left_length_std_mm:.6f}",
        f"  length_rate_raw_std_mm_s={finite_std(row.left_length_rate_raw for row in rows) * 1000.0:.6f}",
        f"  length_rate_lpf_std_mm_s={finite_std(row.left_length_rate_lpf for row in rows) * 1000.0:.6f}",
        "",
        "right_leg:",
        f"  phi_mean_deg={math.degrees(finite_mean(row.right_phi for row in rows)):.6f}",
        f"  phi_std_deg={right_phi_std_deg:.6f}",
        f"  phi_rate_raw_std_deg_s={math.degrees(finite_std(row.right_phi_rate_raw for row in rows)):.6f}",
        f"  phi_rate_lpf_std_deg_s={math.degrees(finite_std(row.right_phi_rate_lpf for row in rows)):.6f}",
        f"  length_mean_m={finite_mean(row.right_leg_length for row in rows):.6f}",
        f"  length_std_mm={right_length_std_mm:.6f}",
        f"  length_rate_raw_std_mm_s={finite_std(row.right_length_rate_raw for row in rows) * 1000.0:.6f}",
        f"  length_rate_lpf_std_mm_s={finite_std(row.right_length_rate_lpf for row in rows) * 1000.0:.6f}",
        "",
        "joint_velocity_std_rad_s:",
        f"  left_hip={left_hip_vel_std:.6f}",
        f"  left_knee={left_knee_vel_std:.6f}",
        f"  left_wheel={left_wheel_vel_std:.6f}",
        f"  right_hip={right_hip_vel_std:.6f}",
        f"  right_knee={right_knee_vel_std:.6f}",
        f"  right_wheel={right_wheel_vel_std:.6f}",
    ]
    path.write_text("\n".join(lines) + "\n")


def plot_rows(rows: list[CaptureRow], path: Path, plot_groups: list[str]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required for plotting: sudo apt install python3-matplotlib"
        ) from exc

    t = [row.elapsed_sec for row in rows]
    fig, axes = plt.subplots(len(plot_groups), 1, figsize=(16, 3.6 * len(plot_groups)), sharex=True)
    if len(plot_groups) == 1:
        axes = [axes]

    for axis, group in zip(axes, plot_groups):
        if group == "imu_euler":
            axis.plot(t, [math.degrees(row.roll) for row in rows], label="roll")
            axis.plot(t, [math.degrees(row.pitch) for row in rows], label="pitch")
            axis.plot(t, [math.degrees(row.yaw) for row in rows], label="yaw")
            axis.set_ylabel("deg")
            axis.set_title("IMU Euler")
        elif group == "gyro":
            axis.plot(t, [row.gyro_x for row in rows], label="gyro_x")
            axis.plot(t, [row.gyro_y for row in rows], label="gyro_y")
            axis.plot(t, [row.gyro_z for row in rows], label="gyro_z")
            axis.set_ylabel("rad/s")
            axis.set_title("Gyro")
        elif group == "accel":
            axis.plot(t, [row.acc_x for row in rows], label="acc_x")
            axis.plot(t, [row.acc_y for row in rows], label="acc_y")
            axis.plot(t, [row.acc_z for row in rows], label="acc_z")
            axis.set_ylabel("m/s^2")
            axis.set_title("Acceleration")
        elif group == "joint_pos":
            axis.plot(t, [row.left_hip_pos for row in rows], label="left_hip")
            axis.plot(t, [row.left_knee_pos for row in rows], label="left_knee")
            axis.plot(t, [row.left_wheel_pos for row in rows], label="left_wheel")
            axis.plot(t, [row.right_hip_pos for row in rows], label="right_hip")
            axis.plot(t, [row.right_knee_pos for row in rows], label="right_knee")
            axis.plot(t, [row.right_wheel_pos for row in rows], label="right_wheel")
            axis.set_ylabel("rad")
            axis.set_title("Joint Position")
        elif group == "joint_vel":
            axis.plot(t, [row.left_hip_vel for row in rows], label="left_hip_vel")
            axis.plot(t, [row.left_knee_vel for row in rows], label="left_knee_vel")
            axis.plot(t, [row.left_wheel_vel for row in rows], label="left_wheel_vel")
            axis.plot(t, [row.right_hip_vel for row in rows], label="right_hip_vel")
            axis.plot(t, [row.right_knee_vel for row in rows], label="right_knee_vel")
            axis.plot(t, [row.right_wheel_vel for row in rows], label="right_wheel_vel")
            axis.set_ylabel("rad/s")
            axis.set_title("Joint Velocity")
        elif group == "joint_effort":
            axis.plot(t, [row.left_hip_effort for row in rows], label="left_hip_effort")
            axis.plot(t, [row.left_knee_effort for row in rows], label="left_knee_effort")
            axis.plot(t, [row.left_wheel_effort for row in rows], label="left_wheel_effort")
            axis.plot(t, [row.right_hip_effort for row in rows], label="right_hip_effort")
            axis.plot(t, [row.right_knee_effort for row in rows], label="right_knee_effort")
            axis.plot(t, [row.right_wheel_effort for row in rows], label="right_wheel_effort")
            axis.set_ylabel("Nm")
            axis.set_title("Joint Effort")
        elif group == "leg_pose":
            axis.plot(t, [math.degrees(row.left_phi) for row in rows], label="left_phi")
            axis.plot(t, [math.degrees(row.right_phi) for row in rows], label="right_phi")
            axis.plot(t, [row.left_leg_length for row in rows], label="left_length")
            axis.plot(t, [row.right_leg_length for row in rows], label="right_length")
            axis.set_ylabel("deg / m")
            axis.set_title("Derived Phi And Leg Length")
        elif group == "leg_rate":
            axis.plot(
                t,
                [math.degrees(row.left_phi_rate_raw) for row in rows],
                label="left_phi_rate_raw",
                alpha=0.5,
            )
            axis.plot(
                t,
                [math.degrees(row.left_phi_rate_lpf) for row in rows],
                label="left_phi_rate_lpf",
            )
            axis.plot(
                t,
                [math.degrees(row.right_phi_rate_raw) for row in rows],
                label="right_phi_rate_raw",
                alpha=0.5,
            )
            axis.plot(
                t,
                [math.degrees(row.right_phi_rate_lpf) for row in rows],
                label="right_phi_rate_lpf",
            )
            axis.plot(
                t,
                [row.left_length_rate_lpf * 1000.0 for row in rows],
                label="left_length_rate_lpf_mm_s",
                linestyle="--",
            )
            axis.plot(
                t,
                [row.right_length_rate_lpf * 1000.0 for row in rows],
                label="right_length_rate_lpf_mm_s",
                linestyle="--",
            )
            axis.set_ylabel("deg/s, mm/s")
            axis.set_title("Leg Rate")
        elif group == "status":
            axis.plot(t, [row.online_mask for row in rows], label="online_mask")
            axis.plot(t, [row.safety_state for row in rows], label="safety_state")
            axis.plot(t, [row.last_command_timeout for row in rows], label="last_cmd_timeout")
            axis.plot(t, [row.knee_limit_flag for row in rows], label="knee_limit_flag")
            axis.plot(t, [row.comm_rx_error_count for row in rows], label="comm_rx_error_count")
            axis.plot(t, [row.comm_crc_error_count for row in rows], label="comm_crc_error_count")
            axis.plot(t, [row.can_error_count for row in rows], label="can_error_count")
            axis.set_ylabel("count/state")
            axis.set_title("Protocol And Safety Status")

        axis.grid(True)
        axis.legend(ncol=3, fontsize=9)

    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def open_serial(port: str, baud: int):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: sudo apt install python3-serial") from exc

    return serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.05,
        write_timeout=1.0,
        exclusive=True,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture STM32 state frames, save a CSV, and generate plots for "
            "robot state analysis."
        )
    )
    parser.add_argument("--port", default="/dev/ttyAMA4", help="Serial port.")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate.")
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Capture duration in seconds.",
    )
    parser.add_argument(
        "--output-dir",
        default="logs/stm32_state_capture",
        help="Directory where CSV, plot, and summary will be written.",
    )
    parser.add_argument(
        "--prefix",
        default=time.strftime("%Y%m%d-%H%M%S"),
        help="Output file prefix.",
    )
    parser.add_argument(
        "--phi-rate-lpf-alpha",
        type=float,
        default=0.95,
        help="LPF alpha for phi_rate replay, matching the ROS estimator default.",
    )
    parser.add_argument(
        "--length-rate-lpf-alpha",
        type=float,
        default=0.90,
        help="LPF alpha for length_rate replay.",
    )
    parser.add_argument(
        "--plot-groups",
        default="all_robot_states",
        help=(
            "Comma-separated plot groups. Choices: "
            + ", ".join(PLOT_GROUP_CHOICES)
            + ". Aliases: all, all_robot_states."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.duration <= 0.0:
        raise SystemExit("--duration must be positive")
    if not 0.0 <= args.phi_rate_lpf_alpha < 1.0:
        raise SystemExit("--phi-rate-lpf-alpha must be in [0, 1)")
    if not 0.0 <= args.length_rate_lpf_alpha < 1.0:
        raise SystemExit("--length-rate-lpf-alpha must be in [0, 1)")
    plot_groups = expand_plot_groups(args.plot_groups)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"{args.prefix}.csv"
    plot_path = output_dir / f"{args.prefix}.png"
    summary_path = output_dir / f"{args.prefix}.summary.txt"

    parser = FrameParser()
    rows: list[CaptureRow] = []

    print(
        f"Capturing STM32 state from {args.port} at {args.baud} baud for "
        f"{args.duration:.2f}s"
    )
    print(f"Output prefix: {output_dir / args.prefix}")

    start_monotonic = time.monotonic()
    first_host_time_sec = 0.0

    with open_serial(args.port, args.baud) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        while (time.monotonic() - start_monotonic) < args.duration:
            chunk = ser.read(4096)
            host_time_sec = time.time()
            if not chunk:
                continue

            decoded_frames = parser.push(chunk)
            for frame in decoded_frames:
                if not rows:
                    first_host_time_sec = host_time_sec
                prev_row = rows[-1] if rows else None
                rows.append(
                    build_row(
                        frame,
                        host_time_sec,
                        first_host_time_sec,
                        prev_row,
                        args.phi_rate_lpf_alpha,
                        args.length_rate_lpf_alpha,
                    )
                )

    if not rows:
        raise SystemExit("No valid STM32 state frames were captured.")

    write_csv(rows, csv_path)
    write_summary(rows, parser, summary_path)
    plot_rows(rows, plot_path, plot_groups)

    duration = rows[-1].elapsed_sec - rows[0].elapsed_sec if len(rows) > 1 else 0.0
    sample_rate = (len(rows) - 1) / duration if duration > 0.0 else math.nan
    print(f"Captured {len(rows)} frames, effective_rate={sample_rate:.2f} Hz")
    print(f"Plot groups: {', '.join(plot_groups)}")
    print(f"CSV: {csv_path}")
    print(f"Plot: {plot_path}")
    print(f"Summary: {summary_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
