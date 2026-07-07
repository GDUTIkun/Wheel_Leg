#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from wheel_leg_msgs.msg import ControlLoopDebug, JointCommand, StandControlState


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def joint_map(msg: JointState) -> Dict[str, tuple[float, float, float]]:
    mapped: Dict[str, tuple[float, float, float]] = {}
    for index, name in enumerate(msg.name):
        position = msg.position[index] if index < len(msg.position) else math.nan
        velocity = msg.velocity[index] if index < len(msg.velocity) else math.nan
        effort = msg.effort[index] if index < len(msg.effort) else math.nan
        mapped[name] = (position, velocity, effort)
    return mapped


def effort_map(msg: JointCommand) -> Dict[str, float]:
    mapped: Dict[str, float] = {}
    for index, name in enumerate(msg.joint_names):
        mapped[name] = msg.efforts[index] if index < len(msg.efforts) else math.nan
    return mapped


@dataclass
class DebugSample:
    stamp_sec: float = math.nan
    ref_primary: float = math.nan
    now_primary: float = math.nan
    ref_secondary: float = math.nan
    now_secondary: float = math.nan


class LeftLegOscillationCapture(Node):
    def __init__(self, output_path: Path, duration_sec: float) -> None:
        super().__init__("left_leg_oscillation_capture")
        self._output_path = output_path
        self._duration_sec = duration_sec
        self._start_wall_time = time.time()
        self._rows: List[Dict[str, float]] = []
        self._lock = threading.Lock()

        self._latest_robot_state_raw: StandControlState | None = None
        self._latest_joint_states_raw: JointState | None = None
        self._latest_joint_command: JointCommand | None = None
        self._latest_debug: Dict[str, DebugSample] = {}

        self.create_subscription(
            StandControlState, "/robot_state_raw", self._on_robot_state_raw, 20
        )
        self.create_subscription(
            JointState, "/joint_states_raw", self._on_joint_states_raw, 20
        )
        self.create_subscription(
            JointCommand, "/joint_command", self._on_joint_command, 20
        )
        self.create_subscription(
            ControlLoopDebug,
            "/debug/control/lqr_hip_torque",
            lambda msg: self._on_debug_msg("lqr_hip_torque", msg),
            20,
        )
        self.create_subscription(
            ControlLoopDebug,
            "/debug/control/vmc_projection",
            lambda msg: self._on_debug_msg("vmc_projection", msg),
            20,
        )
        self.create_subscription(
            ControlLoopDebug,
            "/debug/control/vmc_torque_column",
            lambda msg: self._on_debug_msg("vmc_torque_column", msg),
            20,
        )
        self.create_subscription(
            ControlLoopDebug,
            "/debug/control/balance",
            lambda msg: self._on_debug_msg("balance", msg),
            20,
        )

        self.create_subscription(
            StandControlState, "/robot_state", self._on_robot_state, 20
        )

        self.get_logger().info(
            "Capturing left-leg oscillation data from /robot_state, "
            "/robot_state_raw, /joint_states_raw, /joint_command, "
            "/debug/control/lqr_hip_torque, /debug/control/vmc_projection, "
            "/debug/control/vmc_torque_column, /debug/control/balance"
        )

    def done(self) -> bool:
        return (time.time() - self._start_wall_time) >= self._duration_sec

    def write_csv(self) -> None:
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        if not self._rows:
            raise RuntimeError("No samples captured.")
        fieldnames = list(self._rows[0].keys())
        with self._output_path.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._rows)

    def print_summary(self) -> None:
        print(f"captured_rows: {len(self._rows)}")
        if not self._rows:
            return
        left_phi = [row["left_phi"] for row in self._rows if math.isfinite(row["left_phi"])]
        right_phi = [row["right_phi"] for row in self._rows if math.isfinite(row["right_phi"])]
        left_hip_cmd = [
            row["cmd_left_hip"] for row in self._rows if math.isfinite(row["cmd_left_hip"])
        ]
        right_hip_cmd = [
            row["cmd_right_hip"] for row in self._rows if math.isfinite(row["cmd_right_hip"])
        ]
        if left_phi:
            print(
                "left_phi_range_deg: "
                f"{math.degrees(min(left_phi)):.3f} -> {math.degrees(max(left_phi)):.3f}"
            )
        if right_phi:
            print(
                "right_phi_range_deg: "
                f"{math.degrees(min(right_phi)):.3f} -> {math.degrees(max(right_phi)):.3f}"
            )
        if left_hip_cmd:
            print(
                "cmd_left_hip_range_nm: "
                f"{min(left_hip_cmd):+.3f} -> {max(left_hip_cmd):+.3f}"
            )
        if right_hip_cmd:
            print(
                "cmd_right_hip_range_nm: "
                f"{min(right_hip_cmd):+.3f} -> {max(right_hip_cmd):+.3f}"
            )
        phi_diff = [
            row["left_right_phi_diff_deg"]
            for row in self._rows
            if math.isfinite(row["left_right_phi_diff_deg"])
        ]
        hip_cmd_diff = [
            row["left_right_cmd_hip_diff_nm"]
            for row in self._rows
            if math.isfinite(row["left_right_cmd_hip_diff_nm"])
        ]
        if phi_diff:
            print(
                "left_minus_right_phi_diff_deg_range: "
                f"{min(phi_diff):+.3f} -> {max(phi_diff):+.3f}"
            )
        if hip_cmd_diff:
            print(
                "left_minus_right_cmd_hip_diff_nm_range: "
                f"{min(hip_cmd_diff):+.3f} -> {max(hip_cmd_diff):+.3f}"
            )
        for direction in ("phi_increasing", "phi_decreasing"):
            direction_rows = [
                row
                for row in self._rows
                if row["left_motion_phase"] == direction
            ]
            if not direction_rows:
                continue
            left_rate = [
                abs(row["left_phi_rate"])
                for row in direction_rows
                if math.isfinite(row["left_phi_rate"])
            ]
            left_rate_raw = [
                abs(row["left_phi_rate_raw"])
                for row in direction_rows
                if math.isfinite(row["left_phi_rate_raw"])
            ]
            left_hip_vel = [
                abs(row["raw_left_hip_vel"])
                for row in direction_rows
                if math.isfinite(row["raw_left_hip_vel"])
            ]
            right_rate = [
                abs(row["right_phi_rate"])
                for row in direction_rows
                if math.isfinite(row["right_phi_rate"])
            ]
            print(f"{direction}_samples: {len(direction_rows)}")
            if left_rate:
                print(
                    f"{direction}_left_phi_rate_abs_peak: {max(left_rate):.4f}"
                )
            if left_rate_raw:
                print(
                    f"{direction}_left_phi_rate_raw_abs_peak: {max(left_rate_raw):.4f}"
                )
            if left_hip_vel:
                print(
                    f"{direction}_raw_left_hip_vel_abs_peak: {max(left_hip_vel):.4f}"
                )
            if right_rate:
                print(
                    f"{direction}_right_phi_rate_abs_peak: {max(right_rate):.4f}"
                )
        near_target_rows = [
            row for row in self._rows if row["left_near_target_5deg"] == 1.0
        ]
        if near_target_rows:
            left_rate = [
                abs(row["left_phi_rate"])
                for row in near_target_rows
                if math.isfinite(row["left_phi_rate"])
            ]
            right_rate = [
                abs(row["right_phi_rate"])
                for row in near_target_rows
                if math.isfinite(row["right_phi_rate"])
            ]
            print(f"left_near_target_5deg_samples: {len(near_target_rows)}")
            if left_rate:
                print(f"left_near_target_5deg_left_phi_rate_abs_peak: {max(left_rate):.4f}")
            if right_rate:
                print(f"left_near_target_5deg_right_phi_rate_abs_peak: {max(right_rate):.4f}")

    def _on_robot_state_raw(self, msg: StandControlState) -> None:
        with self._lock:
            self._latest_robot_state_raw = msg

    def _on_joint_states_raw(self, msg: JointState) -> None:
        with self._lock:
            self._latest_joint_states_raw = msg

    def _on_joint_command(self, msg: JointCommand) -> None:
        with self._lock:
            self._latest_joint_command = msg

    def _on_debug_msg(self, name: str, msg: ControlLoopDebug) -> None:
        with self._lock:
            self._latest_debug[name] = DebugSample(
                stamp_sec=stamp_to_sec(msg.header.stamp),
                ref_primary=msg.ref_primary,
                now_primary=msg.now_primary,
                ref_secondary=msg.ref_secondary,
                now_secondary=msg.now_secondary,
            )

    def _on_robot_state(self, msg: StandControlState) -> None:
        with self._lock:
            robot_state_raw = self._latest_robot_state_raw
            joint_states_raw = self._latest_joint_states_raw
            joint_command = self._latest_joint_command
            lqr = self._latest_debug.get("lqr_hip_torque", DebugSample())
            vmc = self._latest_debug.get("vmc_projection", DebugSample())
            vmc_column = self._latest_debug.get("vmc_torque_column", DebugSample())
            balance = self._latest_debug.get("balance", DebugSample())

        if robot_state_raw is None or joint_states_raw is None or joint_command is None:
            return

        joints = joint_map(joint_states_raw)
        commands = effort_map(joint_command)
        stamp_sec = stamp_to_sec(msg.header.stamp)
        target_phi_deg = balance.ref_primary
        target_phi_rad = math.radians(target_phi_deg) if math.isfinite(target_phi_deg) else math.nan
        left_phi_error_deg = phi_error_deg(msg.left_phi, target_phi_rad)
        right_phi_error_deg = phi_error_deg(msg.right_phi, target_phi_rad)
        row = {
            "host_time_sec": time.time(),
            "elapsed_sec": time.time() - self._start_wall_time,
            "state_stamp_sec": stamp_sec,
            "raw_state_stamp_sec": stamp_to_sec(robot_state_raw.header.stamp),
            "target_phi_deg": target_phi_deg,
            "target_phi_rad": target_phi_rad,
            "avg_phi_deg": balance.now_primary,
            "target_pitch_deg": balance.ref_secondary,
            "body_pitch_deg": balance.now_secondary,
            "body_roll": msg.body_roll,
            "body_pitch": msg.body_pitch,
            "body_roll_rate": msg.body_roll_rate,
            "body_pitch_rate": msg.body_pitch_rate,
            "left_hip_absolute": msg.left_hip_absolute,
            "left_calf_absolute": msg.left_calf_absolute,
            "left_leg_length": msg.left_leg_length,
            "left_phi": msg.left_phi,
            "left_phi_rate": msg.left_phi_rate,
            "left_phi_error_to_target_deg": left_phi_error_deg,
            "left_abs_phi_error_to_target_deg": abs_or_nan(left_phi_error_deg),
            "left_near_target_2deg": near_target_flag(left_phi_error_deg, 2.0),
            "left_near_target_5deg": near_target_flag(left_phi_error_deg, 5.0),
            "right_hip_absolute": msg.right_hip_absolute,
            "right_calf_absolute": msg.right_calf_absolute,
            "right_leg_length": msg.right_leg_length,
            "right_phi": msg.right_phi,
            "right_phi_rate": msg.right_phi_rate,
            "right_phi_error_to_target_deg": right_phi_error_deg,
            "right_abs_phi_error_to_target_deg": abs_or_nan(right_phi_error_deg),
            "right_near_target_2deg": near_target_flag(right_phi_error_deg, 2.0),
            "right_near_target_5deg": near_target_flag(right_phi_error_deg, 5.0),
            "left_phi_raw": robot_state_raw.left_phi,
            "left_phi_rate_raw": robot_state_raw.left_phi_rate,
            "right_phi_raw": robot_state_raw.right_phi,
            "right_phi_rate_raw": robot_state_raw.right_phi_rate,
            "left_motion_phase": classify_motion_phase(msg.left_phi_rate),
            "right_motion_phase": classify_motion_phase(msg.right_phi_rate),
            "left_right_hip_absolute_diff_deg": math.degrees(
                msg.left_hip_absolute - msg.right_hip_absolute
            ),
            "left_right_calf_absolute_diff_deg": math.degrees(
                msg.left_calf_absolute - msg.right_calf_absolute
            ),
            "left_right_phi_diff_deg": math.degrees(msg.left_phi - msg.right_phi),
            "left_right_phi_rate_diff_deg_s": math.degrees(
                msg.left_phi_rate - msg.right_phi_rate
            ),
            "left_right_phi_rate_abs_ratio": abs_ratio(
                msg.left_phi_rate, msg.right_phi_rate
            ),
            "left_right_leg_length_diff_mm": 1000.0
            * (msg.left_leg_length - msg.right_leg_length),
            "raw_left_hip_pos": joints.get("left_hip", (math.nan, math.nan, math.nan))[0],
            "raw_left_hip_vel": joints.get("left_hip", (math.nan, math.nan, math.nan))[1],
            "raw_left_hip_effort": joints.get("left_hip", (math.nan, math.nan, math.nan))[2],
            "raw_left_knee_pos": joints.get("left_knee", (math.nan, math.nan, math.nan))[0],
            "raw_left_knee_vel": joints.get("left_knee", (math.nan, math.nan, math.nan))[1],
            "raw_left_knee_effort": joints.get("left_knee", (math.nan, math.nan, math.nan))[2],
            "raw_right_hip_pos": joints.get("right_hip", (math.nan, math.nan, math.nan))[0],
            "raw_right_hip_vel": joints.get("right_hip", (math.nan, math.nan, math.nan))[1],
            "raw_right_hip_effort": joints.get("right_hip", (math.nan, math.nan, math.nan))[2],
            "raw_right_knee_pos": joints.get("right_knee", (math.nan, math.nan, math.nan))[0],
            "raw_right_knee_vel": joints.get("right_knee", (math.nan, math.nan, math.nan))[1],
            "raw_right_knee_effort": joints.get("right_knee", (math.nan, math.nan, math.nan))[2],
            "left_right_hip_vel_abs_ratio": abs_ratio(
                joints.get("left_hip", (math.nan, math.nan, math.nan))[1],
                joints.get("right_hip", (math.nan, math.nan, math.nan))[1],
            ),
            "left_right_knee_vel_abs_ratio": abs_ratio(
                joints.get("left_knee", (math.nan, math.nan, math.nan))[1],
                joints.get("right_knee", (math.nan, math.nan, math.nan))[1],
            ),
            "left_hip_vel_sign_match": sign_match(
                msg.left_phi_rate,
                joints.get("left_hip", (math.nan, math.nan, math.nan))[1],
            ),
            "left_knee_vel_sign_match": sign_match(
                msg.left_phi_rate,
                joints.get("left_knee", (math.nan, math.nan, math.nan))[1],
            ),
            "right_hip_vel_sign_match": sign_match(
                msg.right_phi_rate,
                joints.get("right_hip", (math.nan, math.nan, math.nan))[1],
            ),
            "right_knee_vel_sign_match": sign_match(
                msg.right_phi_rate,
                joints.get("right_knee", (math.nan, math.nan, math.nan))[1],
            ),
            "cmd_left_hip": commands.get("left_hip", math.nan),
            "cmd_left_knee": commands.get("left_knee", math.nan),
            "cmd_right_hip": commands.get("right_hip", math.nan),
            "cmd_right_knee": commands.get("right_knee", math.nan),
            "left_right_cmd_hip_diff_nm": commands.get("left_hip", math.nan)
            - commands.get("right_hip", math.nan),
            "left_right_cmd_knee_diff_nm": commands.get("left_knee", math.nan)
            - commands.get("right_knee", math.nan),
            "lqr_left_hip_torque": lqr.ref_primary,
            "lqr_right_hip_torque": lqr.now_primary,
            "left_leg_length_force": lqr.ref_secondary,
            "right_leg_length_force": lqr.now_secondary,
            "left_right_lqr_hip_diff_nm": lqr.ref_primary - lqr.now_primary,
            "vmc_left_hip_torque": vmc.ref_primary,
            "vmc_left_knee_torque": vmc.now_primary,
            "vmc_right_hip_torque": vmc.ref_secondary,
            "vmc_right_knee_torque": vmc.now_secondary,
            "left_right_vmc_hip_diff_nm": vmc.ref_primary - vmc.ref_secondary,
            "left_right_vmc_knee_diff_nm": vmc.now_primary - vmc.now_secondary,
            "vmc_column_left_hip": vmc_column.ref_primary,
            "vmc_column_left_knee": vmc_column.now_primary,
            "vmc_column_right_hip": vmc_column.ref_secondary,
            "vmc_column_right_knee": vmc_column.now_secondary,
        }
        self._rows.append(row)


def classify_motion_phase(phi_rate: float, threshold: float = 0.2) -> str:
    if not math.isfinite(phi_rate):
        return "unknown"
    if phi_rate > threshold:
        return "phi_increasing"
    if phi_rate < -threshold:
        return "phi_decreasing"
    return "near_static"


def sign_match(reference: float, observed: float, deadband: float = 1e-3) -> float:
    if not math.isfinite(reference) or not math.isfinite(observed):
        return math.nan
    if abs(reference) <= deadband or abs(observed) <= deadband:
        return 0.0
    return 1.0 if reference * observed > 0.0 else -1.0


def phi_error_deg(phi_rad: float, target_phi_rad: float) -> float:
    if not math.isfinite(phi_rad) or not math.isfinite(target_phi_rad):
        return math.nan
    return math.degrees(phi_rad - target_phi_rad)


def abs_or_nan(value: float) -> float:
    return abs(value) if math.isfinite(value) else math.nan


def near_target_flag(error_deg: float, threshold_deg: float) -> float:
    if not math.isfinite(error_deg):
        return math.nan
    return 1.0 if abs(error_deg) <= threshold_deg else 0.0


def abs_ratio(numerator: float, denominator: float, deadband: float = 1e-6) -> float:
    if not math.isfinite(numerator) or not math.isfinite(denominator):
        return math.nan
    if abs(denominator) <= deadband:
        return math.inf if abs(numerator) > deadband else 0.0
    return abs(numerator) / abs(denominator)


def spin_until_done(node: LeftLegOscillationCapture) -> None:
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.05)
    except ExternalShutdownException:
        return


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Capture left/right leg state, command, and VMC/LQR debug data to diagnose left hip oscillation."
    )
    parser.add_argument(
        "--output",
        default="./logs/left_leg_oscillation_capture.csv",
        help="CSV output path.",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=10.0,
        help="Capture duration in seconds.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = LeftLegOscillationCapture(Path(args.output), args.duration_sec)
    print(
        f"Capturing for {args.duration_sec:.1f}s. "
        "Keep the robot in the problematic condition, then send me the CSV."
    )

    try:
        spin_until_done(node)
    except KeyboardInterrupt:
        print("\nInterrupted. Writing captured samples...")
    finally:
        node.write_csv()
        node.print_summary()
        print(f"Wrote CSV: {args.output}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
