"""Standalone exercise for the existing encoder-scripted parking routine."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))
from parking import Parking

from .config import SimConfig
from .runtime import create_simulated_hardware


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run the existing parking-in routine in PyBullet")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--sim-headless", action="store_true")
    parser.add_argument("--encoder-scale", type=float, default=1.3)
    parser.add_argument("--sim-seed", type=int, default=1)
    args = parser.parse_args(argv)

    forwarded = ["--sim-direction", "ccw", "--encoder-scale", str(args.encoder_scale), "--sim-seed", str(args.sim_seed)]
    if args.sim_headless:
        forwarded.append("--sim-headless")
    config = SimConfig.from_argv(forwarded, width=640, height=480)
    bundle = create_simulated_hardware(config, mode="OBSTACLE")
    parking = Parking(
        arduino=bundle.serial,
        parking_speed=22,
        camera_width=640,
        camera_height=480,
        parking_lot_region=[0, 185, 640, 400],
        maxLeft=40,
        maxRight=150,
        STRAIGHT_CONST=95,
        MAX_OFFSET_DEGREE=55,
        REVERSE_REGION=[223, 255, 427, 273],
    )
    try:
        # This method exists in the project but is not called by obstacle_challenge.py.
        parking.process_parking(args.side)
    except KeyboardInterrupt:
        pass
    finally:
        bundle.serial.close()
    print(f"Parking trace written to {bundle.output_directory}")


if __name__ == "__main__":
    main(sys.argv[1:])
