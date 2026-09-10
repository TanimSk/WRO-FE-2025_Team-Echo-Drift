"""Simulator configuration and command-line overrides."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field

from .settings import load_settings, settings_path


@dataclass
class CameraConfig:
    width: int = 640
    height: int = 480
    fps: float = 25.0
    horizontal_fov_deg: float = 150.0
    near: float = 0.02
    far: float = 8.0
    mount_height: float = 0.13
    mount_forward: float = 0.11
    pitch_deg: float = -18.0
    barrel_k1: float = -0.16
    output_bgr: bool = True
    render_scale: float = 0.5


@dataclass
class NoiseConfig:
    encoder_scale: float = 1.3
    encoder_noise_std_ticks: float = 2.0
    gyro_noise_std_deg: float = 0.15
    gyro_bias_walk_std_deg_sqrt_s: float = 0.02
    telemetry_hz: float = 5.0
    watchdog_seconds: float = 1.0


@dataclass
class VehicleConfig:
    length: float = 0.25
    width: float = 0.11
    mass: float = 0.7
    wheel_radius: float = 0.046
    wheelbase: float = 0.18
    track_width: float = 0.105
    max_speed_mps: float = 0.2
    max_steering_deg: float = 30.0
    drive_force: float = 4.0
    ticks_per_revolution: int = 2220


@dataclass
class TrackConfig:
    outer_size: float = 3.0
    island_size: float = 1.0
    wall_height: float = 0.20
    wall_thickness: float = 0.035
    post_radius: float = 0.035
    post_height: float = 0.15


@dataclass
class SimConfig:
    gui: bool = True
    direction: str = "ccw"
    seed: int = 1
    random_posts: int = 0
    duration_seconds: float | None = None
    physics_hz: float = 240.0
    camera: CameraConfig = field(default_factory=CameraConfig)
    noise: NoiseConfig = field(default_factory=NoiseConfig)
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    track: TrackConfig = field(default_factory=TrackConfig)
    rois: dict = field(default_factory=dict)
    settings_file: str = ""

    @classmethod
    def from_argv(
        cls, argv: list[str], *, width: int, height: int,
        mode: str = "NO_OBSTACLE", rois: dict | None = None,
    ) -> "SimConfig":
        parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
        parser.add_argument("--sim-headless", action="store_true")
        parser.add_argument("--sim-direction", choices=("cw", "ccw"), default="ccw")
        parser.add_argument("--sim-seed", type=int, default=1)
        parser.add_argument("--random-posts", type=int, default=0)
        parser.add_argument("--encoder-scale", type=float)
        parser.add_argument("--encoder-noise-std", type=float)
        parser.add_argument("--gyro-noise-std", type=float)
        parser.add_argument("--gyro-bias-walk-std", type=float)
        parser.add_argument("--camera-fov", type=float)
        parser.add_argument("--sim-camera-fps", type=float)
        parser.add_argument("--sim-render-scale", type=float)
        parser.add_argument("--sim-duration", type=float)
        args, _ = parser.parse_known_args(argv)

        config = cls(
            gui=not args.sim_headless,
            direction=args.sim_direction,
            seed=args.sim_seed,
            random_posts=max(0, args.random_posts),
            duration_seconds=args.sim_duration,
            rois=rois or {},
            settings_file=str(settings_path(mode)),
        )
        config.camera.width = width
        config.camera.height = height
        load_settings(config)
        config.camera.render_scale = max(0.25, min(1.0, config.camera.render_scale))
        config.camera.fps = max(5.0, min(120.0, config.camera.fps))
        if args.camera_fov is not None:
            config.camera.horizontal_fov_deg = args.camera_fov
        if args.sim_camera_fps is not None:
            config.camera.fps = max(5.0, min(120.0, args.sim_camera_fps))
        if args.sim_render_scale is not None:
            config.camera.render_scale = max(0.25, min(1.0, args.sim_render_scale))
        if args.encoder_scale is not None:
            config.noise.encoder_scale = args.encoder_scale
        if args.encoder_noise_std is not None:
            config.noise.encoder_noise_std_ticks = max(0.0, args.encoder_noise_std)
        if args.gyro_noise_std is not None:
            config.noise.gyro_noise_std_deg = max(0.0, args.gyro_noise_std)
        if args.gyro_bias_walk_std is not None:
            config.noise.gyro_bias_walk_std_deg_sqrt_s = max(0.0, args.gyro_bias_walk_std)
        return config
