#!/usr/bin/env python3

import argparse
import csv
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from wheel_leg_msgs.msg import ControlLoopDebug


TOPICS = {
    "yaw": "/debug/plot/yaw",
    "yaw_filter": "/debug/plot/ref_filter/yaw_rate",
    "anti_crash": "/debug/plot/anti_crash",
    "wheel_effort": "/debug/plot/wheel_effort",
    "turn_internal": "/debug/plot/turn_internal",
}


@dataclass
class DebugSample:
    stamp_sec: float = math.nan
    ref_primary: float = math.nan
    now_primary: float = math.nan
    ref_secondary: float = math.nan
    now_secondary: float = math.nan


class TurnSignCapture(Node):
    def __init__(self, output_path: Path) -> None:
        super().__init__("turn_sign_capture")
        self._output_path = output_path
        self._lock = threading.Lock()
        self._latest: Dict[str, DebugSample] = {}
        self._phase = "init"
        self._rows = []

        for name, topic in TOPICS.items():
            self.create_subscription(
                ControlLoopDebug,
                topic,
                lambda msg, name=name: self._on_debug_msg(name, msg),
                10,
            )

        self.create_timer(0.02, self._sample)
        self.get_logger().info(
            "Capturing turn sign data. Subscribed to: "
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
                "yaw_ref",
                "yaw_now",
                "yaw_filter_raw",
                "yaw_filter_filtered",
                "anti_crash_ref",
                "anti_crash_now",
                "wheel_right",
                "wheel_left",
                "steer_output",
                "swerving_speed_ff",
                "anti_crash_output",
                "left_minus_right_hip_torque",
            ]
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._rows)

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
            yaw = self._latest.get("yaw", DebugSample())
            yaw_filter = self._latest.get("yaw_filter", DebugSample())
            anti_crash = self._latest.get("anti_crash", DebugSample())
            wheel = self._latest.get("wheel_effort", DebugSample())
            turn_internal = self._latest.get("turn_internal", DebugSample())
            row = {
                "wall_time_sec": f"{time.time():.6f}",
                "phase": self._phase,
                "yaw_ref": yaw.ref_primary,
                "yaw_now": yaw.now_primary,
                "yaw_filter_raw": yaw_filter.ref_primary,
                "yaw_filter_filtered": yaw_filter.now_primary,
                "anti_crash_ref": anti_crash.ref_primary,
                "anti_crash_now": anti_crash.now_primary,
                "wheel_right": wheel.now_primary,
                "wheel_left": wheel.now_secondary,
                "steer_output": turn_internal.ref_primary,
                "swerving_speed_ff": turn_internal.now_primary,
                "anti_crash_output": turn_internal.ref_secondary,
                "left_minus_right_hip_torque": turn_internal.now_secondary,
            }
            self._rows.append(row)


def spin_node(node: TurnSignCapture, stop_event: threading.Event) -> None:
    while rclpy.ok() and not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)


def wait_phase(node: TurnSignCapture, phase: str, duration_sec: float) -> None:
    node.set_phase(phase)
    print(f"\n[{phase}] hold for {duration_sec:.1f}s")
    time.sleep(duration_sec)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Capture turn-direction debug data while the user moves the RC stick."
    )
    parser.add_argument(
        "--output",
        default="/tmp/wheel_leg_turn_sign_capture.csv",
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
    node = TurnSignCapture(Path(args.output))
    stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=spin_node,
        args=(node, stop_event),
        daemon=True,
    )
    spin_thread.start()

    try:
        print("Turn sign capture ready.")
        input("Put the robot in a stable velocity/stand-ready state, center sticks, then press Enter...")
        wait_phase(node, "center_before_right", args.center_sec)
        input("Push turn stick RIGHT and hold it, then press Enter...")
        wait_phase(node, "turn_right", args.hold_sec)
        input("Center the turn stick, then press Enter...")
        wait_phase(node, "center_after_right", args.center_sec)
        input("Push turn stick LEFT and hold it, then press Enter...")
        wait_phase(node, "turn_left", args.hold_sec)
        input("Center the turn stick, then press Enter...")
        wait_phase(node, "center_after_left", args.center_sec)
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        node.write_csv()
        print(f"\nWrote CSV: {args.output}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
