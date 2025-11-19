#!/usr/bin/env python3

"""
Utility script to plot circle trajectory from a saved cmdvel_circle_test txt file.
Usage:
    rosrun car_model plot_cmdvel_circle.py _input:=/path/to/circle_20230801.txt [_output:=/path/to/out.png]
"""

import os
import math
import argparse
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _extract_number(val):
    """Extract first float-like number from a string or return original float."""
    if isinstance(val, (int, float)):
        return float(val)
    if not isinstance(val, str):
        return None
    m = re.search(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", val)
    return float(m.group(0)) if m else None


def parse_file(path):
    xs, ys = [], []
    meta = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                parts = line[1:].strip().split(":", 1)
                if len(parts) == 2:
                    meta_key = parts[0].strip()
                    raw_val = parts[1].strip()
                    parsed_val = _extract_number(raw_val)
                    meta[meta_key] = parsed_val if parsed_val is not None else raw_val
                continue
            try:
                _, x, y, _ = line.split()
                xs.append(float(x))
                ys.append(float(y))
            except ValueError:
                continue
    return xs, ys, meta


def plot(xs, ys, meta, output):
    if not xs or not ys:
        raise ValueError("No trajectory points found to plot.")

    radius_cmd = _extract_number(meta.get("radius_cmd")) if meta else None

    plt.figure(figsize=(6, 6))
    plt.plot(xs, ys, label="Recorded path")
    plt.scatter(xs[0], ys[0], c="green", label="Start")
    plt.scatter(xs[-1], ys[-1], c="red", label="End")

    if radius_cmd is not None and len(xs) > 2:
        # crude center estimation: use first point as start, compute circle center using perpendicular bisector
        x0, y0, x1, y1 = xs[0], ys[0], xs[len(xs)//2], ys[len(ys)//2]
        dx, dy = x1 - x0, y1 - y0
        if dx != 0 or dy != 0:
            cx = x0 - dy
            cy = y0 + dx
            theta = [i * 0.01 * 2 * math.pi for i in range(0, 100)]
            circle_x = [cx + radius_cmd * math.cos(t) for t in theta]
            circle_y = [cy + radius_cmd * math.sin(t) for t in theta]
            plt.plot(circle_x, circle_y, "k--", label=f"Theoretical circle R={radius_cmd:.2f}")

    plt.axis("equal")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output)
    plt.close()
    print(f"Saved plot to {output}")


def main():
    parser = argparse.ArgumentParser(description="Plot circle trajectory txt produced by cmdvel_circle_test.py")
    parser.add_argument("--input", "-i", required=True, help="Path to circle_*.txt")
    parser.add_argument("--output", "-o", help="Output png path (default: same dir with .png)")
    args = parser.parse_args()

    input_path = os.path.abspath(args.input)
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"Cannot find input file: {input_path}")

    xs, ys, meta = parse_file(input_path)

    output_path = args.output
    if not output_path:
        base, _ = os.path.splitext(input_path)
        output_path = base + ".png"
    output_path = os.path.abspath(output_path)

    plot(xs, ys, meta, output_path)


if __name__ == "__main__":
    main()
