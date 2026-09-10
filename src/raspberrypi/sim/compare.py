"""Plot simulator truth, simulated odometry, and optional real encoder logs."""

from __future__ import annotations

import argparse
import ast
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def trajectory(samples, wheel_radius=0.046, ticks_per_rev=2220):
    x = y = 0.0
    previous = None
    points = [(x, y)]
    for ticks, angle in samples:
        if previous is None:
            previous = ticks
            continue
        distance = 2 * math.pi * wheel_radius * (ticks - previous) / ticks_per_rev
        previous = ticks
        x += distance * math.cos(math.radians(angle))
        y += distance * math.sin(math.radians(angle))
        points.append((x, y))
    return points


def read_sensor_csv(path):
    with Path(path).open(newline="") as stream:
        rows = csv.DictReader(stream)
        return [(-int(row["encoder_ticks"]), float(row["gyro_deg"])) for row in rows]


def read_real(path):
    path = Path(path)
    if path.suffix.lower() == ".csv":
        with path.open(newline="") as stream:
            rows = csv.DictReader(stream)
            return [(int(row["ticks"]), float(row["angle"])) for row in rows]
    values = []
    for line in path.read_text().splitlines():
        text = line.strip().rstrip(",")
        if text:
            ticks, angle = ast.literal_eval(text)
            values.append((int(ticks), float(angle)))
    return values


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--real-log", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    output = args.output or args.run_directory / "comparison.png"

    fig, ax = plt.subplots(figsize=(7.2, 6.0))
    truth_path = args.run_directory / "ground_truth.csv"
    with truth_path.open(newline="") as stream:
        truth = [(float(row["x_m"]), float(row["y_m"])) for row in csv.DictReader(stream)]
    if truth:
        origin_x, origin_y = truth[0]
        truth = [(x - origin_x, y - origin_y) for x, y in truth]
        ax.plot(*zip(*truth), label="PyBullet ground truth", linewidth=2)
    sim = trajectory(read_sensor_csv(args.run_directory / "sensor.csv"))
    ax.plot(*zip(*sim), label="Simulated odometry", linewidth=1.5)
    if args.real_log:
        real = trajectory(read_real(args.real_log))
        ax.plot(*zip(*real), label="Real odometry", linewidth=1.5)
    ax.set(xlabel="x (m)", ylabel="y (m)", title="Trajectory comparison")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    print(output)


if __name__ == "__main__":
    main()
