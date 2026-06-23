#!/usr/bin/env python3

import argparse
import csv
import math
import statistics
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from wheel_leg_msgs.msg import ControlLoopDebug


TOPICS = {
    "leg_length": "/debug/plot/leg_length",
    "leg_length_output": "/debug/control/leg_length_output",
    "anti_crash": "/debug/plot/anti_crash",
    "turn_internal": "/debug/plot/turn_internal",
}


@dataclass
class DebugSample:
    stamp_sec: float = math.nan
    ref_primary: float = math.nan
    now_primary: float = math.nan
    ref_secondary: float = math.nan
    now_secondary: float = math.nan


class TurnLegBalanceCapture(Node):
    def __init__(self, output_path: Path) -> None:
        super().__init__("turn_leg_balance_capture")
        self._output_path = output_path
        self._lock = threading.Lock()
        self._latest: Dict[str, DebugSample] = {}
        self._phase = "init"
        self._rows: List[Dict[str, float]] = []

        for name, topic in TOPICS.items():
            self.create_subscription(
                ControlLoopDebug,
                topic,
                lambda msg, name=name: self._on_debug_msg(name, msg),
                10,
            )

        self.create_timer(0.02, self._sample)
        self.get_logger().info(
            "Capturing leg/anti-crash turn data. Subscribed to: "
            + ", ".join(TOPICS.values())
        )

    def set_phase(self, phase: str) -> None:
        with self._lock:
            self._phase = phase

    def write_csv(self) -> None:
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        with self._output_path.open("w", newline="") as csv_file:
            fieldnames = [
                "wall_time_sec",
                "phase",
                "leg_target_left",
                "leg_now_left",
                "leg_target_right",
                "leg_now_right",
                "leg_left_error",
                "leg_right_error",
                "leg_output_left",
                "leg_output_right",
                "leg_output_diff",
                "leg_output_avg",
                "anti_crash_ref",
                "anti_crash_now",
                "swerving_speed_ff",
                "anti_crash_output",
                "left_minus_right_hip_torque",
            ]
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._rows)

    def print_phase_summary(self) -> None:
        phases = ["turn_right_small", "turn_right_medium", "turn_left_small", "turn_left_medium"]
        metrics = [
            "leg_now_left",
            "leg_now_right",
            "leg_left_error",
            "leg_right_error",
            "leg_output_left",
            "leg_output_right",
            "swerving_speed_ff",
            "anti_crash_output",
            "left_minus_right_hip_torque",
            "anti_crash_now",
        ]
        print("\nPhase summary (mean values):")
        for phase in phases:
            phase_rows = [row for row in self._rows if row["phase"] == phase]
            if not phase_rows:
                print(f"- {phase}: no samples")
                continue
            print(f"- {phase}:")
            for metric in metrics:
                values = [float(row[metric]) for row in phase_rows if math.isfinite(float(row[metric]))]
                if values:
                    print(f"    {metric}: {statistics.fmean(values):+.6f}")

    def _on_debug_msg(self, name: str, msg: ControlLoopDebug) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        sample = DebugSample(
            stamp_sec=stamp_sec,
            ref_primary=msg.ref_primary,
            now_primary=msg.now_primary,
            ref_secondary=msg.ref_secondary,
            now_secondary=msg.now_secondary,
        )
        with self._lock:
            self._latest[name] = sample

    def _sample(self) -> None:
        with self._lock:
            leg = self._latest.get("leg_length", DebugSample())
            leg_output = self._latest.get("leg_length_output", DebugSample())
            anti_crash = self._latest.get("anti_crash", DebugSample())
            turn = self._latest.get("turn_internal", DebugSample())
            row = {
                "wall_time_sec": f"{time.time():.6f}",
                "phase": self._phase,
                "leg_target_left": leg.ref_primary,
                "leg_now_left": leg.now_primary,
                "leg_target_right": leg.ref_secondary,
                "leg_now_right": leg.now_secondary,
                "leg_left_error": leg.ref_primary - leg.now_primary,
                "leg_right_error": leg.ref_secondary - leg.now_secondary,
                "leg_output_left": leg_output.ref_primary,
                "leg_output_right": leg_output.now_primary,
                "leg_output_diff": leg_output.ref_secondary,
                "leg_output_avg": leg_output.now_secondary,
                "anti_crash_ref": anti_crash.ref_primary,
                "anti_crash_now": anti_crash.now_primary,
                "swerving_speed_ff": turn.now_primary,
                "anti_crash_output": turn.ref_secondary,
                "left_minus_right_hip_torque": turn.now_secondary,
            }
            self._rows.append(row)


def spin_node(node: TurnLegBalanceCapture, stop_event: threading.Event) -> None:
    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except ExternalShutdownException:
        return


def wait_phase(node: TurnLegBalanceCapture, phase: str, duration_sec: float) -> None:
    node.set_phase(phase)
    print(f"\n[{phase}] recording for {duration_sec:.1f}s")
    end_time = time.time() + duration_sec
    while True:
        remaining = end_time - time.time()
        if remaining <= 0.0:
            break
        print(f"  {phase}: {remaining:4.1f}s remaining", end="\r", flush=True)
        time.sleep(min(0.2, remaining))
    print(f"  {phase}: done{' ' * 20}")


def prompt_and_wait_phase(
    node: TurnLegBalanceCapture,
    prompt: str,
    phase: str,
    duration_sec: float,
) -> None:
    input(prompt)
    wait_phase(node, phase, duration_sec)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Capture leg length, leg output, anti-crash feedforward, and anti-crash output while comparing left/right turns."
    )
    parser.add_argument(
        "--output",
        default="/tmp/wheel_leg_turn_leg_balance.csv",
        help="CSV output path.",
    )
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=3.0,
        help="Seconds to hold each stick direction.",
    )
    parser.add_argument(
        "--center-sec",
        type=float,
        default=2.0,
        help="Seconds to hold stick centered between turns.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TurnLegBalanceCapture(Path(args.output))
    stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=spin_node,
        args=(node, stop_event),
        daemon=True,
    )
    spin_thread.start()

    try:
        print("Turn leg balance capture ready.")
        prompt_and_wait_phase(
            node,
            "Center sticks and stabilize, then press Enter to START center_before_right...",
            "center_before_right",
            args.center_sec,
        )
        prompt_and_wait_phase(
            node,
            "Hold RIGHT small turn, then press Enter to START turn_right_small...",
            "turn_right_small",
            args.hold_sec,
        )
        prompt_and_wait_phase(
            node,
            "Hold RIGHT medium turn, then press Enter to START turn_right_medium...",
            "turn_right_medium",
            args.hold_sec,
        )
        prompt_and_wait_phase(
            node,
            "Center sticks, then press Enter to START center_after_right...",
            "center_after_right",
            args.center_sec,
        )
        prompt_and_wait_phase(
            node,
            "Hold LEFT small turn, then press Enter to START turn_left_small...",
            "turn_left_small",
            args.hold_sec,
        )
        prompt_and_wait_phase(
            node,
            "Hold LEFT medium turn, then press Enter to START turn_left_medium...",
            "turn_left_medium",
            args.hold_sec,
        )
        prompt_and_wait_phase(
            node,
            "Center sticks, then press Enter to START center_after_left...",
            "center_after_left",
            args.center_sec,
        )
    except KeyboardInterrupt:
        print("\nInterrupted. Writing any captured samples...")
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        node.write_csv()
        node.print_phase_summary()
        print(f"\nWrote CSV: {args.output}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
