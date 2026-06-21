#!/usr/bin/env python3
"""Configure and loopback-test Raspberry Pi UARTs used by Wheel_Leg."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


CONFIG_CANDIDATES = (Path("/boot/firmware/config.txt"), Path("/boot/config.txt"))
MANAGED_BEGIN = "# BEGIN Wheel_Leg UART loopback config"
MANAGED_END = "# END Wheel_Leg UART loopback config"


def detect_model() -> str:
    model_path = Path("/proc/device-tree/model")
    if not model_path.exists():
        return ""
    return model_path.read_text(errors="ignore").replace("\x00", "").strip()


def selected_overlays() -> tuple[str, str]:
    model = detect_model()
    if "Raspberry Pi 5" in model:
        return ("uart3-pi5", "uart4-pi5")
    return ("uart4", "uart5")


def managed_block() -> str:
    first, second = selected_overlays()
    return f"""{MANAGED_BEGIN}
# GPIO8/GPIO9 and GPIO12/GPIO13 loopback UARTs for Wheel_Leg.
enable_uart=1
dtoverlay={first}
dtoverlay={second}
{MANAGED_END}
"""


def config_path() -> Path:
    for path in CONFIG_CANDIDATES:
        if path.exists():
            return path
    raise FileNotFoundError("No Raspberry Pi config.txt found under /boot")


def require_root() -> None:
    if os.geteuid() != 0:
        raise SystemExit("configure must be run as root, e.g. sudo ./tools/uart_loopback.py configure")


def replace_managed_block(text: str) -> str:
    if MANAGED_BEGIN in text and MANAGED_END in text:
        before, rest = text.split(MANAGED_BEGIN, 1)
        _, after = rest.split(MANAGED_END, 1)
        return before.rstrip() + "\n\n" + managed_block() + after.lstrip("\n")
    return text.rstrip() + "\n\n" + managed_block()


def disable_spi0(text: str) -> str:
    lines = []
    for line in text.splitlines():
        if line.strip() == "dtparam=spi=on":
            lines.append("#dtparam=spi=on  # disabled: GPIO8/9 are used for UART loopback")
        else:
            lines.append(line)
    return "\n".join(lines) + "\n"


def configure(args: argparse.Namespace) -> None:
    require_root()
    path = config_path()
    original = path.read_text()
    updated = replace_managed_block(disable_spi0(original))

    if updated == original:
        print(f"{path} already contains the requested UART config")
    else:
        backup = path.with_suffix(path.suffix + f".bak-{time.strftime('%Y%m%d-%H%M%S')}")
        shutil.copy2(path, backup)
        path.write_text(updated)
        print(f"Updated {path}")
        print(f"Backup saved to {backup}")

    first, second = selected_overlays()
    print(f"Configured GPIO8/9 via dtoverlay={first} and GPIO12/13 via dtoverlay={second}.")
    print("Reboot is required before /dev/ttyAMA3 and /dev/ttyAMA4 appear.")
    if args.reboot:
        subprocess.run(["reboot"], check=False)


def open_serial(port: str, baudrate: int):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: sudo apt install python3-serial") from exc

    return serial.Serial(
        port=port,
        baudrate=baudrate,
        timeout=0.05,
        write_timeout=1,
        exclusive=True,
    )


def test_one(port: str, baudrate: int, count: int) -> bool:
    if not Path(port).exists():
        print(f"FAIL {port}: device node does not exist")
        return False

    ok = True
    with open_serial(port, baudrate) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        for index in range(count):
            payload = f"Wheel_Leg loopback {port} #{index} {time.time_ns()}\n".encode()
            ser.write(payload)
            ser.flush()

            received = bytearray()
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline and len(received) < len(payload):
                chunk = ser.read(len(payload) - len(received))
                if chunk:
                    received.extend(chunk)

            if bytes(received) != payload:
                print(f"FAIL {port}: sent {payload!r}, received {bytes(received)!r}")
                ok = False
                break

    if ok:
        print(f"PASS {port}: {count} loopback frame(s) at {baudrate} baud")
    return ok


def alias_target(name: str) -> str | None:
    path = Path("/proc/device-tree/aliases") / name
    if not path.exists():
        return None
    return path.read_text(errors="ignore").replace("\x00", "").strip()


def detect_ports() -> list[str]:
    preferred = []
    for alias in ("uart3", "uart4"):
        target = alias_target(alias)
        if not target:
            continue
        tty_glob = Path("/sys/firmware/devicetree/base") / target.lstrip("/")
        for tty in Path("/sys/class/tty").glob("ttyAMA*"):
            of_node = tty / "device/of_node"
            if of_node.exists() and of_node.resolve() == tty_glob.resolve():
                preferred.append(f"/dev/{tty.name}")
    if preferred:
        return preferred
    return [port for port in ("/dev/ttyAMA3", "/dev/ttyAMA4") if Path(port).exists()]


def test(args: argparse.Namespace) -> None:
    ports = args.ports or detect_ports()
    if not ports:
        raise SystemExit("No UART loopback devices detected. Reboot after configure and check wiring.")
    results = [test_one(port, args.baudrate, args.count) for port in ports]
    if not all(results):
        raise SystemExit(1)


def status(_: argparse.Namespace) -> None:
    for command in (
        ["ls", "-l", "/dev/ttyAMA3", "/dev/ttyAMA4"],
        ["dtoverlay", "-l"],
        ["pinctrl", "get", "8", "9", "12", "13"],
    ):
        print(f"$ {' '.join(command)}")
        if shutil.which(command[0]) is None:
            print(f"{command[0]}: command not found")
            continue
        subprocess.run(command, check=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    configure_parser = subparsers.add_parser("configure", help="enable the UART overlays in config.txt")
    configure_parser.add_argument("--reboot", action="store_true", help="reboot immediately after editing config.txt")
    configure_parser.set_defaults(func=configure)

    test_parser = subparsers.add_parser("test", help="run TX/RX loopback tests")
    test_parser.add_argument("ports", nargs="*")
    test_parser.add_argument("--baudrate", type=int, default=115200)
    test_parser.add_argument("--count", type=int, default=3)
    test_parser.set_defaults(func=test)

    status_parser = subparsers.add_parser("status", help="show UART-related system state")
    status_parser.set_defaults(func=status)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
