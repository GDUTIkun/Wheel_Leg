#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from wheel_leg_msgs.msg import RcChannelsRaw


class RcChannelWatch(Node):
    def __init__(self) -> None:
        super().__init__("rc_channel_watch")
        self._previous = None
        self.create_subscription(
            RcChannelsRaw,
            "/rc/channels_raw",
            self._on_msg,
            10,
        )
        self.get_logger().info(
            "Watching /rc/channels_raw and printing channels that change by >= 20."
        )

    def _on_msg(self, msg: RcChannelsRaw) -> None:
        current = list(msg.channels)
        if self._previous is None:
            self._previous = current
            self.get_logger().info(
                "Initial channels: "
                + " ".join(f"CH{i + 1}={value}" for i, value in enumerate(current))
            )
            return

        changed = []
        for index, (old, new) in enumerate(zip(self._previous, current), start=1):
            if abs(int(new) - int(old)) >= 20:
                changed.append(f"CH{index}:{old}->{new}")

        if changed:
            self.get_logger().info("Changed: " + " ".join(changed))

        self._previous = current


def main() -> None:
    rclpy.init()
    node = RcChannelWatch()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
