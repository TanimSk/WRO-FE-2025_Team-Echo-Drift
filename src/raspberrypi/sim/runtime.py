"""Assembly of the simulated world and hardware adapters."""

from pathlib import Path

from hardware import HardwareBundle

from .virtual_camera import VirtualCamera
from .virtual_serial import VirtualSerial
from .world import PyBulletWorld


def create_simulated_hardware(config, mode: str = "NO_OBSTACLE") -> HardwareBundle:
    output_root = Path(__file__).with_name("runs")
    world = PyBulletWorld(config, mode=mode, output_root=output_root)
    camera = VirtualCamera(world, config.camera)
    serial = VirtualSerial(world, config.noise, config.vehicle, config.seed)
    return HardwareBundle(
        camera=camera,
        serial=serial,
        world=world,
        output_directory=world.output_directory,
        serial_warmup_seconds=0.0,
        camera_warmup_seconds=0.0,
    )
