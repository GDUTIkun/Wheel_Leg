#!/usr/bin/env python3
"""Send framed UART traffic for STM32 protocol receive tests."""

from __future__ import annotations

import argparse
import struct
import time


HEAD = b"\xA5\x5A"


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


def build_frame(frame_type: int, seq: int, payload: bytes) -> bytes:
    if not 0 <= frame_type <= 0xFF:
        raise ValueError("frame_type must fit in uint8")
    if len(payload) > 255:
        raise ValueError("payload must fit in uint8 length")

    body = struct.pack("<BBH", frame_type, len(payload), seq & 0xFFFF) + payload
    return HEAD + body + struct.pack("<H", crc16_ccitt(body))


def make_payload(seq: int, payload_len: int) -> bytes:
    return bytes(((seq + index) & 0xFF) for index in range(payload_len))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port, for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--rate-hz", type=float, default=200.0)
    parser.add_argument("--payload-len", type=int, default=32)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--frame-type", type=lambda value: int(value, 0), default=0x01)
    args = parser.parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: pip install pyserial") from exc

    if args.rate_hz <= 0:
        raise SystemExit("--rate-hz must be positive")
    if not 0 <= args.payload_len <= 96:
        raise SystemExit("--payload-len must be in 0..96 for the current STM32 parser")

    period = 1.0 / args.rate_hz
    end_time = time.monotonic() + args.duration
    seq = 0
    sent = 0

    with serial.Serial(args.port, args.baud, timeout=0, write_timeout=1) as ser:
        next_send = time.monotonic()
        while time.monotonic() < end_time:
            payload = make_payload(seq, args.payload_len)
            frame = build_frame(args.frame_type, seq, payload)
            ser.write(frame)
            sent += 1
            seq = (seq + 1) & 0xFFFF

            next_send += period
            sleep_time = next_send - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_send = time.monotonic()

    print(
        f"sent={sent} baud={args.baud} rate_hz={args.rate_hz} "
        f"payload_len={args.payload_len} frame_len={len(build_frame(args.frame_type, 0, make_payload(0, args.payload_len)))}"
    )


if __name__ == "__main__":
    main()
