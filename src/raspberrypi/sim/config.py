"""Simulator configuration and command-line overrides."""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass, field
from pathlib import Path

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
    exposure_gain: float = 1.0


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
    track_width: float = 0.122
    max_speed_mps: float = 0.2
    max_steering_deg: float = 30.0
    drive_force: float = 4.0
    ticks_per_revolution: int = 2220
    servo_center_deg: float = 95.0
    servo_span_deg: float = 55.0
    tire_friction: float = 1.2
    steering_force: float = 3.0
    steering_rate: float = 5.0


@dataclass
class TrackConfig:
    outer_size: float = 3.0
    island_size: float = 1.0
    wall_height: float = 0.20
    wall_thickness: float = 0.035
    post_radius: float = 0.035
    post_height: float = 0.15
    shape: str = "square"
    length_scale: float = 1.0
    ambient: float = 0.6
    diffuse: float = 0.4
    light_x: float = -3.0
    light_y: float = -4.0
    light_z: float = 8.0
    shadows: bool = False
    posts: list = field(default_factory=lambda: [
        [-1.0, 0.85, "red"], [1.0, 0.85, "green"],
        [-1.0, -0.65, "green"], [1.0, -0.65, "red"],
    ])


@dataclass
class ParkingConfig:
    custom: bool = False
    center_x: float = -1.05
    center_y: float = -1.075
    width: float = 0.56
    depth: float = 0.38
    rotation_deg: float = 0.0
    start_offset_x: float = 0.0
    start_offset_y: float = -0.085
    heading_deg: float = 0.0


@dataclass
class OdometryConfig:
    # Estimator calibration, independent of simulated wheel geometry/noise.
    start_zone_half_width: float = 0.75
    start_zone_half_height: float = 1.5
    lap_interval_seconds: float = 6.0
    drift_alpha: float = 0.3
    max_drift_threshold: float = 0.5
    estimated_wheel_radius: float = 0.046
    estimated_ticks_per_rev: int = 2220
    estimated_gear_ratio: float = 1.0


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
    odometry: OdometryConfig = field(default_factory=OdometryConfig)
    parking: ParkingConfig = field(default_factory=ParkingConfig)
    rois: dict = field(default_factory=dict)
    settings_file: str = ""
    legacy_controls: bool = False
    preview: bool = False
    processing_view: bool = False
    vehicle_profile: str = ""
    track_profile: str = ""

    @classmethod
    def from_argv(
        cls, argv: list[str], *, width: int, height: int,
        mode: str = "NO_OBSTACLE", rois: dict | None = None,
    ) -> "SimConfig":
        parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
        parser.add_argument("--sim-headless", action="store_true")
        parser.add_argument("--sim-direction", choices=("cw", "ccw"))
        parser.add_argument("--sim-seed", type=int)
        parser.add_argument("--random-posts", type=int)
        parser.add_argument("--encoder-scale", type=float)
        parser.add_argument("--encoder-noise-std", type=float)
        parser.add_argument("--gyro-noise-std", type=float)
        parser.add_argument("--gyro-bias-walk-std", type=float)
        parser.add_argument("--camera-fov", type=float)
        parser.add_argument("--sim-camera-fps", type=float)
        parser.add_argument("--sim-render-scale", type=float)
        parser.add_argument("--sim-duration", type=float)
        parser.add_argument("--vehicle-profile")
        parser.add_argument("--track-profile")
        parser.add_argument("--sim-controls", action="store_true")
        args, _ = parser.parse_known_args(argv)

        config = cls(
            gui=not args.sim_headless,
            direction=args.sim_direction or "ccw",
            seed=args.sim_seed if args.sim_seed is not None else 1,
            random_posts=max(0, args.random_posts or 0),
            duration_seconds=args.sim_duration,
            rois=rois or {},
            settings_file=str(settings_path(mode)),
        )
        config.camera.width = width
        config.camera.height = height
        load_settings(config)
        from .profiles import apply_profiles, default_path
        config.vehicle_profile = str(default_path("vehicle", mode) if args.vehicle_profile is None else Path(args.vehicle_profile).resolve())
        config.track_profile = str(default_path("track", mode) if args.track_profile is None else Path(args.track_profile).resolve())
        apply_profiles(config, required_vehicle=args.vehicle_profile is not None,
                       required_track=args.track_profile is not None)
        if args.vehicle_profile is None and (args.sim_controls or os.environ.get("ECHO_SIM_SETTINGS")):
            load_settings(config)
        config.legacy_controls = args.sim_controls
        config.processing_view = "--debug" in argv and config.gui
        for argument, name in ((args.sim_direction, "direction"), (args.sim_seed, "seed"), (args.random_posts, "random_posts")):
            if argument is not None:
                setattr(config, name, argument)
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
        from .profiles import validate
        validate(config)
        return config
