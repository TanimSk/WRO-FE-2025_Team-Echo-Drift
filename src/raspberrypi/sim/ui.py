"""PyBullet debug-panel controls for interactive simulator runs."""

from .settings import save_settings, settings_snapshot


class SimulatorUI:
    def __init__(self, bullet, client_id, config):
        self.p = bullet
        self.client_id = client_id
        self.config = config
        self.parameters = {
            "max_speed": self._slider("Max speed (m/s)", 0.1, 2.0, config.vehicle.max_speed_mps),
            "drive_force": self._slider("Drive force (N)", 0.2, 8.0, config.vehicle.drive_force),
            "max_steering": self._slider("Max steering (deg)", 15.0, 45.0, config.vehicle.max_steering_deg),
            "camera_tilt": self._slider("Camera tilt (deg)", -45.0, 5.0, config.camera.pitch_deg),
            "camera_height": self._slider("Camera height (m)", 0.05, 0.35, config.camera.mount_height),
            "camera_fov": self._slider("Camera horizontal FOV", 90.0, 170.0, config.camera.horizontal_fov_deg),
            "render_scale": self._slider("POV render scale", 0.25, 1.0, config.camera.render_scale),
            "camera_fps": self._slider("Camera FPS", 5.0, 120.0, config.camera.fps),
            "encoder_scale": self._slider("Encoder scale", 0.5, 1.6, config.noise.encoder_scale),
            "encoder_noise": self._slider("Encoder noise (ticks)", 0.0, 20.0, config.noise.encoder_noise_std_ticks),
            "gyro_noise": self._slider("Gyro noise (deg)", 0.0, 3.0, config.noise.gyro_noise_std_deg),
        }
        self.roi_parameters = {}
        for name, region in config.rois.items():
            coordinates = enumerate(region) if isinstance(region, list) else region.items()
            for coordinate, value in coordinates:
                axis = ("x1", "y1", "x2", "y2")[coordinate] if isinstance(coordinate, int) else coordinate
                maximum = config.camera.width if axis.startswith("x") else config.camera.height
                self.roi_parameters[(name, coordinate)] = self._slider(
                    f"ROI {name.replace('_', ' ').title()} {axis}", 0, maximum, value
                )
        self.buttons = {
            "start": self._button("START / RESUME"),
            "stop": self._button("STOP / PAUSE"),
            "restart": self._button("RESTART RUN"),
        }
        self.button_values = {
            name: self.p.readUserDebugParameter(parameter, physicsClientId=client_id)
            for name, parameter in self.buttons.items()
        }
        self._last_saved_snapshot = settings_snapshot(config)

    def _slider(self, name, minimum, maximum, value):
        return self.p.addUserDebugParameter(
            name, minimum, maximum, value, physicsClientId=self.client_id
        )

    def _button(self, name):
        # PyBullet treats a reversed-range debug parameter as a push button.
        return self.p.addUserDebugParameter(
            name, 1, 0, 0, physicsClientId=self.client_id
        )

    def poll(self):
        read = lambda key: self.p.readUserDebugParameter(
            self.parameters[key], physicsClientId=self.client_id
        )
        self.config.vehicle.max_speed_mps = read("max_speed")
        self.config.vehicle.drive_force = read("drive_force")
        self.config.vehicle.max_steering_deg = read("max_steering")
        self.config.camera.pitch_deg = read("camera_tilt")
        self.config.camera.mount_height = read("camera_height")
        self.config.camera.horizontal_fov_deg = read("camera_fov")
        self.config.camera.render_scale = read("render_scale")
        self.config.camera.fps = read("camera_fps")
        self.config.noise.encoder_scale = read("encoder_scale")
        self.config.noise.encoder_noise_std_ticks = read("encoder_noise")
        self.config.noise.gyro_noise_std_deg = read("gyro_noise")

        updated_rois = {
            name: list(region) if isinstance(region, list) else dict(region)
            for name, region in self.config.rois.items()
        }
        for (name, coordinate), parameter in self.roi_parameters.items():
            value = int(round(self.p.readUserDebugParameter(
                parameter, physicsClientId=self.client_id
            )))
            updated_rois[name][coordinate] = value

        # Rectangular image slices must remain non-empty even if sliders cross.
        for name, values in updated_rois.items():
            region = self.config.rois[name]
            if isinstance(region, list):
                region[:] = _valid_rectangle(
                    values, self.config.camera.width, self.config.camera.height
                )
            else:
                region.update(values)

        snapshot = settings_snapshot(self.config)
        if snapshot != self._last_saved_snapshot:
            save_settings(self.config)
            self._last_saved_snapshot = snapshot

        events = set()
        for name, parameter in self.buttons.items():
            value = self.p.readUserDebugParameter(parameter, physicsClientId=self.client_id)
            if value != self.button_values[name]:
                events.add(name)
                self.button_values[name] = value
        return events

    def save(self):
        save_settings(self.config)
        self._last_saved_snapshot = settings_snapshot(self.config)


def _valid_rectangle(values, width, height):
    x1, y1, x2, y2 = values
    x1, x2 = sorted((max(0, min(width, x1)), max(0, min(width, x2))))
    y1, y2 = sorted((max(0, min(height, y1)), max(0, min(height, y2))))
    if x1 == x2:
        x1, x2 = (width - 1, width) if x1 == width else (x1, x1 + 1)
    if y1 == y2:
        y1, y2 = (height - 1, height) if y1 == height else (y1, y1 + 1)
    return [x1, y1, x2, y2]
