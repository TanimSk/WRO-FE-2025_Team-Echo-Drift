"""Picamera2-compatible POV camera backed by PyBullet rendering."""

from __future__ import annotations

import math
import time

import cv2
import numpy as np


class VirtualCamera:
    def __init__(self, world, config):
        self.world = world
        self.config = config
        self.controls = {}
        self.started = False
        self._last_frame_time = 0.0
        self._maps = None
        self._map_key = None

    def create_preview_configuration(self, main: dict) -> dict:
        return {"main": dict(main)}

    def configure(self, config: dict) -> None:
        main = config.get("main", {})
        if "size" in main:
            self.config.width, self.config.height = map(int, main["size"])

    def set_controls(self, controls: dict) -> None:
        # Stored for interface compatibility. PyBullet has no sensor exposure model.
        self.controls.update(controls)

    def start(self) -> None:
        self.started = True
        self._last_frame_time = time.monotonic() - 1.0 / self.config.fps

    def capture_array(self) -> np.ndarray:
        if not self.started:
            raise RuntimeError("VirtualCamera.capture_array() called before start()")
        wait = (1.0 / self.config.fps) - (time.monotonic() - self._last_frame_time)
        if wait > 0:
            time.sleep(wait)
        # Record the frame start, not its completion. Rendering time now counts
        # toward the camera period instead of being added after a full FPS delay.
        self._last_frame_time = time.monotonic()
        self.world.advance()

        position, yaw = self.world.vehicle.pose()
        cp = math.cos(math.radians(self.config.pitch_deg))
        sp = math.sin(math.radians(self.config.pitch_deg))
        forward = np.array((math.cos(yaw), math.sin(yaw), 0.0))
        eye = np.array(position) + self.config.mount_forward * forward + np.array((0.0, 0.0, self.config.mount_height))
        look = np.array((cp * math.cos(yaw), cp * math.sin(yaw), sp))
        up = np.array((-sp * math.cos(yaw), -sp * math.sin(yaw), cp))
        view = self.world.p.computeViewMatrix(eye, eye + look, up)
        aspect = self.config.width / self.config.height
        vertical_fov = math.degrees(2.0 * math.atan(math.tan(math.radians(self.config.horizontal_fov_deg) / 2.0) / aspect))
        projection = self.world.p.computeProjectionMatrixFOV(
            vertical_fov, aspect, self.config.near, self.config.far
        )
        renderer = self.world.p.ER_BULLET_HARDWARE_OPENGL if self.world.config.gui else self.world.p.ER_TINY_RENDERER
        render_width = max(1, int(round(self.config.width * self.config.render_scale)))
        render_height = max(1, int(round(self.config.height * self.config.render_scale)))
        image = self.world.p.getCameraImage(
            render_width, render_height, view, projection,
            renderer=renderer, physicsClientId=self.world.client_id,
            lightDirection=[self.world.config.track.light_x, self.world.config.track.light_y, self.world.config.track.light_z],
            lightAmbientCoeff=self.world.config.track.ambient,
            lightDiffuseCoeff=self.world.config.track.diffuse,
            lightSpecularCoeff=0.0, shadow=int(self.world.config.track.shadows),
        )
        rgb = np.asarray(image[2], dtype=np.uint8).reshape(render_height, render_width, 4)[..., :3]
        if (render_width, render_height) != (self.config.width, self.config.height):
            rgb = cv2.resize(
                rgb, (self.config.width, self.config.height), interpolation=cv2.INTER_LINEAR
            )
        if self.config.barrel_k1:
            rgb = self._distort(rgb)
        if self.config.exposure_gain != 1:
            rgb = np.clip(rgb.astype(np.float32) * self.config.exposure_gain, 0, 255).astype(np.uint8)
        frame = np.ascontiguousarray(rgb[..., ::-1] if self.config.output_bgr else rgb)
        if self.world.config.gui and not self.world.config.preview and not self.world.config.processing_view:
            cv2.imshow("Robot POV", frame if self.config.output_bgr else frame[..., ::-1])
            if cv2.waitKey(1) & 0xff == ord('q'):
                raise KeyboardInterrupt
        return frame

    def _distort(self, image: np.ndarray) -> np.ndarray:
        key = (image.shape[:2], self.config.barrel_k1)
        if self._maps is None or self._map_key != key:
            self._map_key = key
            h, w = image.shape[:2]
            yy, xx = np.indices((h, w), dtype=np.float32)
            nx = (xx - (w - 1) / 2) / (w / 2)
            ny = (yy - (h - 1) / 2) / (h / 2)
            radius2 = nx * nx + ny * ny
            factor = 1.0 + self.config.barrel_k1 * radius2
            map_x = nx * factor * (w / 2) + (w - 1) / 2
            map_y = ny * factor * (h / 2) + (h - 1) / 2
            self._maps = map_x.astype(np.float32), map_y.astype(np.float32)
        return cv2.remap(image, *self._maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

    def stop(self) -> None:
        self.started = False
