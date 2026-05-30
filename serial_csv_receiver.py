"""
Receive inverted-pendulum cart data from a serial port and save it as CSV.

Input line format from MCU:
    theta_mrad,theta_dot_mradps,pos_mm,vel_mmps,pwm\r\n

MCU printf format:
    "%ld,%ld,%ld,%ld,%d\r\n"

Default serial port:
    COM10

Output columns:
    timestamp_s,theta_mrad,theta_dot_mradps,pos_mm,vel_mmps,pwm,
    theta_rad,theta_dot_radps,pos_m,vel_mps

The last four columns use SI units and names that lqr_from_csv.py can read:
    theta_rad         rad
    theta_dot_radps   rad/s
    pos_m             m
    vel_mps           m/s

Example:
    python serial_csv_receiver.py

Example with explicit baud rate and output file:
    python serial_csv_receiver.py --port COM10 --baud 115200 --out pendulum_data.csv

Stop:
    Press Ctrl+C.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import serial


OUTPUT_COLUMNS = [
    "timestamp_s",
    "theta_mrad",
    "theta_dot_mradps",
    "pos_mm",
    "vel_mmps",
    "pwm",
    "theta_rad",
    "theta_dot_radps",
    "pos_m",
    "vel_mps",
]


def parse_mcu_line(line: str) -> tuple[int, int, int, int, int] | None:
    line = line.strip()
    if not line:
        return None

    parts = line.split(",")
    if len(parts) != 5:
        return None

    try:
        theta_mrad, theta_dot_mradps, pos_mm, vel_mmps, pwm = [
            int(part.strip()) for part in parts
        ]
    except ValueError:
        return None

    return theta_mrad, theta_dot_mradps, pos_mm, vel_mmps, pwm


def converted_row(
    timestamp_s: float,
    theta_mrad: int,
    theta_dot_mradps: int,
    pos_mm: int,
    vel_mmps: int,
    pwm: int,
) -> dict[str, str]:
    return {
        "timestamp_s": f"{timestamp_s:.6f}",
        "theta_mrad": str(theta_mrad),
        "theta_dot_mradps": str(theta_dot_mradps),
        "pos_mm": str(pos_mm),
        "vel_mmps": str(vel_mmps),
        "pwm": str(pwm),
        "theta_rad": f"{theta_mrad / 1000.0:.10g}",
        "theta_dot_radps": f"{theta_dot_mradps / 1000.0:.10g}",
        "pos_m": f"{pos_mm / 1000.0:.10g}",
        "vel_mps": f"{vel_mmps / 1000.0:.10g}",
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Receive COM serial data and save CSV for lqr_from_csv.py."
    )
    parser.add_argument("--port", default="COM10", help="Serial port, default: COM10.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate, default: 115200.")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout in seconds.")
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output CSV path. Default: pendulum_serial_YYYYmmdd_HHMMSS.csv",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=0,
        help="Stop after this many valid samples. 0 means run until Ctrl+C.",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=50,
        help="Print one status line every N valid samples. 0 disables progress output.",
    )
    parser.add_argument(
        "--send",
        default=None,
        help="Optional text to send once after opening the serial port, for example START.",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()

    out_path = args.out
    if out_path is None:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out_path = Path(f"pendulum_serial_{stamp}.csv")

    valid_count = 0
    bad_count = 0
    start_time = time.monotonic()

    print(f"Opening {args.port} at {args.baud} baud...")
    print(f"Writing CSV to: {out_path.resolve()}")

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()

        if args.send is not None:
            message = args.send
            if not message.endswith("\n"):
                message += "\n"
            ser.write(message.encode("ascii", errors="replace"))
            ser.flush()
            print(f"Sent: {args.send!r}")

        with out_path.open("w", encoding="utf-8-sig", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=OUTPUT_COLUMNS)
            writer.writeheader()

            try:
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue

                    line = raw.decode("ascii", errors="replace")
                    parsed = parse_mcu_line(line)
                    if parsed is None:
                        if line.strip().lower().startswith("theta_mrad,"):
                            continue
                        bad_count += 1
                        if bad_count <= 5:
                            print(f"Skipped malformed line: {line.strip()!r}")
                        continue

                    timestamp_s = time.monotonic() - start_time
                    row = converted_row(timestamp_s, *parsed)
                    writer.writerow(row)
                    valid_count += 1

                    if args.print_every and valid_count % args.print_every == 0:
                        file.flush()
                        print(
                            "samples={samples}, theta={theta} rad, pos={pos} m, pwm={pwm}".format(
                                samples=valid_count,
                                theta=row["theta_rad"],
                                pos=row["pos_m"],
                                pwm=row["pwm"],
                            )
                        )

                    if args.max_samples > 0 and valid_count >= args.max_samples:
                        break

            except KeyboardInterrupt:
                print("\nStopped by Ctrl+C.")

    print(f"Done. Valid samples: {valid_count}, malformed lines: {bad_count}")
    print(f"CSV saved to: {out_path.resolve()}")


if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        sys.exit(1)
