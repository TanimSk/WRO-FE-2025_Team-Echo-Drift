"""Minimal driving view: run the existing controller with saved profiles."""
import argparse
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("open", "obstacle"), default="open")
    args, options = parser.parse_known_args()
    if "--sim-headless" not in options:
        if "--debug" not in options:
            options.append("--debug")
        os.environ.setdefault("MPLBACKEND", "QtAgg")
    script = Path(__file__).resolve().with_name(
        "obstacle_challenge.py" if args.mode == "obstacle" else "open_challenge.py"
    )
    print("Driving view: 3D scene, processing POV and odometry. Space = pause/resume, R = restart, Q = quit.", flush=True)
    os.execv(sys.executable, [sys.executable, str(script), "--sim", *options])


if __name__ == "__main__":
    main()
