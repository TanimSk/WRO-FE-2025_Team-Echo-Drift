import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

RASPBERRYPI = Path(__file__).parents[2]
sys.path.insert(0, str(RASPBERRYPI))

from sim.config import SimConfig
from sim.runtime import create_simulated_hardware
from sim.settings import save_settings
from sim.ui import _valid_rectangle
import hardware


def test_default_and_cli_configuration():
    config = SimConfig.from_argv(
        ["--sim-headless", "--encoder-scale", "1.0", "--sim-camera-fps", "60", "--random-posts", "3", "--sim-seed", "7"],
        width=320,
        height=240,
    )
    assert not config.gui
    assert config.noise.encoder_scale == 1.0
    assert config.random_posts == 3
    assert config.seed == 7
    assert config.camera.fps == 60
    assert (config.camera.width, config.camera.height) == (320, 240)


def test_control_settings_and_rois_persist(tmp_path, monkeypatch):
    settings = tmp_path / "controls.json"
    monkeypatch.setenv("ECHO_SIM_SETTINGS", str(settings))
    first_rois = {"left": [0, 10, 100, 80], "left_danger": {"x1": 10, "y1": 20, "x2": 30, "y2": 40}}
    first = SimConfig.from_argv([], width=640, height=480, rois=first_rois)
    first.vehicle.max_speed_mps = 0.37
    first.camera.pitch_deg = -24.0
    first.noise.encoder_scale = 1.12
    first.rois["left"][:] = [5, 15, 125, 95]
    first.rois["left_danger"]["x2"] = 44
    save_settings(first)

    second_rois = {"left": [0, 0, 1, 1], "left_danger": {"x1": 0, "y1": 0, "x2": 1, "y2": 1}}
    second = SimConfig.from_argv(
        ["--encoder-scale", "1.0"], width=640, height=480, rois=second_rois
    )
    assert second.vehicle.max_speed_mps == 0.37
    assert second.camera.pitch_deg == -24.0
    assert second.noise.encoder_scale == 1.0
    assert second_rois["left"] == [5, 15, 125, 95]
    assert second_rois["left_danger"]["x2"] == 44


def test_roi_rectangle_is_ordered_and_nonempty():
    assert _valid_rectangle([640, 480, 0, 0], 640, 480) == [0, 0, 640, 480]
    assert _valid_rectangle([640, 480, 640, 480], 640, 480) == [639, 479, 640, 480]


def test_restart_uses_original_absolute_entrypoint(tmp_path, monkeypatch):
    original = Path("/project/src/raspberrypi/open_challenge.py")
    executed = {}
    monkeypatch.setattr(hardware, "ENTRYPOINT_PATH", original)
    monkeypatch.setattr(sys, "argv", ["raspberrypi/open_challenge.py", "--sim", "--debug"])
    monkeypatch.chdir(tmp_path)

    def fake_execv(executable, argv):
        executed["executable"] = executable
        executed["argv"] = argv

    monkeypatch.setattr(os, "execv", fake_execv)

    def request_restart():
        raise hardware.SimulatorRestartRequested

    hardware.run_entrypoint(request_restart, hardware.HardwareBundle(None, None))
    assert executed["argv"] == [sys.executable, str(original), "--sim", "--debug"]


@pytest.mark.parametrize("mode", ["NO_OBSTACLE", "OBSTACLE"])
def test_camera_and_serial_contract(mode):
    pytest.importorskip("pybullet")
    config = SimConfig.from_argv(
        ["--sim-headless", "--encoder-noise-std", "0", "--gyro-noise-std", "0"],
        width=160,
        height=120,
    )
    bundle = create_simulated_hardware(config, mode=mode)
    try:
        assert bundle.serial_warmup_seconds == 0.0
        assert bundle.camera_warmup_seconds == 0.0
        camera = bundle.camera
        camera.configure(camera.create_preview_configuration({"format": "RGB888", "size": (160, 120)}))
        camera.start()
        frame = camera.capture_array()
        assert frame.shape == (120, 160, 3)
        assert frame.dtype == np.uint8
        assert bundle.serial.in_waiting > 0
        assert bundle.serial.readline() == b"START\n"
        assert bundle.serial.write(b"25,-1,95\n") == len(b"25,-1,95\n")
        bundle.serial.write(b"25,20,95\n")
        deadline = time.monotonic() + 1.0
        received = []
        while time.monotonic() < deadline and b"DONE\n" not in received:
            if bundle.serial.in_waiting:
                received.append(bundle.serial.readline())
        assert b"DONE\n" in received
    finally:
        camera.stop()
        bundle.serial.close()


def test_steering_mapping():
    pytest.importorskip("pybullet")
    config = SimConfig.from_argv(["--sim-headless"], width=80, height=60)
    bundle = create_simulated_hardware(config, mode="NO_OBSTACLE")
    try:
        vehicle = bundle.world.vehicle
        left = vehicle._ackermann(math.radians(20))
        right = vehicle._ackermann(math.radians(-20))
        assert left[0] > left[1] > 0
        assert right[1] < right[0] < 0
    finally:
        bundle.serial.close()
