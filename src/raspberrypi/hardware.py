"""Single hardware selection boundary for physical and simulated runs."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional


# Capture this before simulator setup changes the working directory to its run
# folder. sys.argv[0] may be a relative path such as raspberrypi/open_challenge.py.
ENTRYPOINT_PATH = Path(sys.argv[0]).resolve()
ENTRYPOINT_DIRECTORY = Path.cwd()


class SimulatorRestartRequested(BaseException):
    """Request a clean process restart from the simulator UI."""


@dataclass
class HardwareBundle:
    camera: Any
    serial: Any
    world: Optional[Any] = None
    output_directory: Optional[Path] = None
    serial_warmup_seconds: float = 2.0
    camera_warmup_seconds: float = 2.0


def create_hardware(
    *, simulated: bool, mode: str, width: int, height: int,
    argv: list[str], rois: Optional[dict] = None,
) -> HardwareBundle:
    """Create hardware without importing platform-specific packages on sim runs."""
    if simulated:
        from sim.config import SimConfig
        from sim.runtime import create_simulated_hardware

        config = SimConfig.from_argv(argv, width=width, height=height, mode=mode, rois=rois)
        return create_simulated_hardware(config, mode=mode)

    import serial
    from picamera2 import Picamera2

    return HardwareBundle(
        camera=Picamera2(),
        serial=serial.Serial(port="/dev/ttyUSB0", baudrate=115200, dsrdtr=True),
    )


def run_entrypoint(main, hardware: HardwareBundle) -> None:
    """Run an entry point and perform a complete restart when the UI requests it."""
    if hardware.world is not None:
        from sim.odometry_settings import apply_odometry_settings

        apply_odometry_settings(main.__globals__, hardware.world.config)
    try:
        main()
    except SimulatorRestartRequested:
        import os

        print("Restarting simulator...")
        os.chdir(ENTRYPOINT_DIRECTORY)
        os.execv(
            sys.executable,
            [sys.executable, str(ENTRYPOINT_PATH), *sys.argv[1:]],
        )
