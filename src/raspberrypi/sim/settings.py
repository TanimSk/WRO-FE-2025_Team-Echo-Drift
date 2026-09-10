"""Persistent settings for the interactive simulator controls."""

from __future__ import annotations

import json
import os
from pathlib import Path


SETTINGS_VERSION = 1


def settings_path(mode: str) -> Path:
    override = os.environ.get("ECHO_SIM_SETTINGS")
    if override:
        return Path(override).expanduser().resolve()
    profile = mode.lower().replace("_", "-")
    return Path(__file__).resolve().with_name(f"simulator-settings-{profile}.json")


def load_settings(config) -> None:
    path = Path(config.settings_file)
    if not path.exists():
        return
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError, TypeError) as exc:
        print(f"Ignoring invalid simulator settings file {path}: {exc}")
        return

    _apply_attributes(config.vehicle, data.get("vehicle", {}), {
        "max_speed_mps", "drive_force", "max_steering_deg",
    })
    _apply_attributes(config.camera, data.get("camera", {}), {
        "pitch_deg", "mount_height", "horizontal_fov_deg", "render_scale", "fps",
    })
    _apply_attributes(config.noise, data.get("noise", {}), {
        "encoder_scale", "encoder_noise_std_ticks", "gyro_noise_std_deg",
    })

    stored_rois = data.get("rois", {})
    for name, target in config.rois.items():
        stored = stored_rois.get(name)
        if isinstance(target, list) and isinstance(stored, list) and len(stored) == 4:
            target[:] = [int(value) for value in stored]
        elif isinstance(target, dict) and isinstance(stored, dict):
            for key in ("x1", "y1", "x2", "y2"):
                if key in stored:
                    target[key] = int(stored[key])


def save_settings(config) -> None:
    path = Path(config.settings_file)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "version": SETTINGS_VERSION,
        "vehicle": {
            "max_speed_mps": config.vehicle.max_speed_mps,
            "drive_force": config.vehicle.drive_force,
            "max_steering_deg": config.vehicle.max_steering_deg,
        },
        "camera": {
            "pitch_deg": config.camera.pitch_deg,
            "mount_height": config.camera.mount_height,
            "horizontal_fov_deg": config.camera.horizontal_fov_deg,
            "render_scale": config.camera.render_scale,
            "fps": config.camera.fps,
        },
        "noise": {
            "encoder_scale": config.noise.encoder_scale,
            "encoder_noise_std_ticks": config.noise.encoder_noise_std_ticks,
            "gyro_noise_std_deg": config.noise.gyro_noise_std_deg,
        },
        "rois": {
            name: list(region) if isinstance(region, list) else dict(region)
            for name, region in config.rois.items()
        },
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n")
    temporary.replace(path)


def settings_snapshot(config) -> str:
    values = {
        "vehicle": (
            config.vehicle.max_speed_mps,
            config.vehicle.drive_force,
            config.vehicle.max_steering_deg,
        ),
        "camera": (
            config.camera.pitch_deg,
            config.camera.mount_height,
            config.camera.horizontal_fov_deg,
            config.camera.render_scale,
            config.camera.fps,
        ),
        "noise": (
            config.noise.encoder_scale,
            config.noise.encoder_noise_std_ticks,
            config.noise.gyro_noise_std_deg,
        ),
        "rois": {
            name: tuple(region) if isinstance(region, list) else tuple(sorted(region.items()))
            for name, region in config.rois.items()
        },
    }
    return repr(values)


def _apply_attributes(target, values, allowed) -> None:
    if not isinstance(values, dict):
        return
    for name in allowed:
        value = values.get(name)
        if isinstance(value, (int, float)):
            setattr(target, name, float(value))
