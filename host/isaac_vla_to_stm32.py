#!/usr/bin/env python3

import argparse
import json
import sys
import time

try:
    import serial
except ImportError as e:
    raise SystemExit("missing dependency: run `pip install pyserial`") from e


def _parse_xyz(line: str):
    s = line.strip()
    if not s:
        return None

    try:
        obj = json.loads(s)
        if isinstance(obj, dict):
            x = obj.get("x", obj.get("X"))
            y = obj.get("y", obj.get("Y"))
            z = obj.get("z", obj.get("Z"))
            if x is None or y is None or z is None:
                return None
            return float(x), float(y), float(z)
        if isinstance(obj, (list, tuple)) and len(obj) >= 3:
            return float(obj[0]), float(obj[1]), float(obj[2])
    except json.JSONDecodeError:
        pass

    s = s.replace(",", " ")
    parts = s.split()
    if len(parts) < 3:
        return None
    return float(parts[0]), float(parts[1]), float(parts[2])


def _send(ser: "serial.Serial", msg: str):
    ser.write((msg.strip() + "\n").encode("ascii", errors="ignore"))
    ser.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="serial port, e.g. COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--scale", type=float, default=None, help="world units -> cm (default: keep stm32 scale=1)")
    ap.add_argument("--origin", nargs=3, type=float, default=None, metavar=("OX", "OY", "OZ"))
    ap.add_argument(
        "--map",
        nargs=3,
        type=int,
        default=None,
        metavar=("MX", "MY", "MZ"),
        help="world axis indices for workspace x/y/z (0=x, 1=y, 2=z)",
    )
    ap.add_argument(
        "--sign",
        nargs=3,
        type=int,
        default=None,
        metavar=("SX", "SY", "SZ"),
        help="axis signs (+1 or -1) for workspace x/y/z",
    )
    ap.add_argument("--send-auto", action="store_true", help="send AUTO before streaming targets")
    ap.add_argument("--quiet", action="store_true", help="do not print uart output from the stm32")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(1.0)

    if args.origin is not None:
        ox, oy, oz = args.origin
        _send(ser, f"CFG ORIGIN {ox} {oy} {oz}")

    if args.scale is not None:
        _send(ser, f"CFG SCALE {args.scale}")

    if args.map is not None:
        if args.sign is None:
            raise SystemExit("if --map is set, you also need --sign")
        mx, my, mz = args.map
        sx, sy, sz = args.sign
        _send(ser, f"CFG MAP {mx} {my} {mz} {sx} {sy} {sz}")

    if args.send_auto:
        _send(ser, "AUTO")

    for raw in sys.stdin:
        xyz = _parse_xyz(raw)
        if xyz is None:
            continue
        x, y, z = xyz

        _send(ser, f"T {x} {y} {z}")

        if not args.quiet:
            time.sleep(0.02)
            while ser.in_waiting:
                try:
                    data = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
                    if data:
                        sys.stdout.write(data)
                        sys.stdout.flush()
                except Exception:
                    break


if __name__ == "__main__":
    main()

