"""Exercise saved editor profiles, generated geometry and desktop interactions."""
import copy
import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[2]))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from sim.config import SimConfig
from sim.profiles import apply, payload, save, standard_config, validate, valid_post
from sim.world import PyBulletWorld
from sim.virtual_camera import VirtualCamera


def test_profile_roundtrip_and_cli_precedence(tmp_path):
    c = standard_config("NO_OBSTACLE")
    c.vehicle.mass = 1.1
    c.vehicle.wheel_radius = .05
    c.camera.pitch_deg = -12
    c.rois["left"] = [0, 150, 230, 240]
    c.track.shape = "rectangle"
    c.track.length_scale = 1.3
    c.track.ambient = .3
    c.direction = "cw"
    save(c, "vehicle", tmp_path / "v.json")
    save(c, "track", tmp_path / "t.json")
    original = standard_config("NO_OBSTACLE").rois
    shared_list = original["left"]
    restored = SimConfig.from_argv(["--sim-headless", "--vehicle-profile", str(tmp_path / "v.json"),
        "--track-profile", str(tmp_path / "t.json"), "--camera-fov", "120", "--sim-direction", "ccw"],
        width=640, height=480, rois=original)
    assert restored.vehicle.mass == 1.1 and restored.vehicle.wheel_radius == .05
    assert restored.track.ambient == .3 and restored.camera.horizontal_fov_deg == 120
    assert shared_list is restored.rois["left"] and shared_list == [0, 150, 230, 240]
    assert restored.direction == "ccw"


def test_geometry_and_lighting_are_real(tmp_path):
    c = standard_config("OBSTACLE")
    c.vehicle.mass = 1.2
    c.vehicle.wheel_radius = .055
    c.vehicle.wheelbase = .2
    c.vehicle.servo_center_deg = 100
    world = PyBulletWorld(c, "OBSTACLE", tmp_path)
    try:
        p, car = world.p, world.vehicle
        total_mass = sum(p.getDynamicsInfo(car.body_id, index, physicsClientId=world.client_id)[0]
                         for index in range(-1, p.getNumJoints(car.body_id, physicsClientId=world.client_id)))
        assert total_mass == pytest.approx(1.2)
        # floor + 8 walls + 8 markers + 3 bay stripes + exactly 4 posts + vehicle
        assert p.getNumBodies(physicsClientId=world.client_id) == 25
        # PyBullet imports URDF cylinders as meshes; their AABB gives actual radius.
        low, high = p.getAABB(car.body_id, car.rear_joints[0], physicsClientId=world.client_id)
        assert high[0] - low[0] == pytest.approx(.11, abs=.0061)
        camera = VirtualCamera(world, c.camera)
        camera.start()
        bright = camera.capture_array()
        c.track.ambient = .1
        c.track.diffuse = .1
        dark = camera.capture_array()
        assert np.mean(dark) < np.mean(bright) * .8
        assert bright.shape == (480, 640, 3)
    finally:
        world.close()


def test_invalid_post_and_roi_rejected():
    c = standard_config("OBSTACLE")
    assert not valid_post(c, 0, 0)
    assert not valid_post(c, 1.5, 1)
    assert not valid_post(c, -1.05, -1.16)
    assert not valid_post(c, 1, .85, [(1, .85)])
    c.rois["left"] = [200, 100, 100, 250]
    with pytest.raises(ValueError, match="x1 < x2"):
        validate(c)


def test_processing_view_observes_masks_without_changing_contours(monkeypatch):
    import img_processing_functions as processing
    from sim.processing_view import ProcessingView
    config = standard_config("NO_OBSTACLE")
    view = ProcessingView(config)
    frame = np.full((480, 640, 3), 255, np.uint8)
    frame[170:225, 5:225] = 0
    lower, upper = np.array([0, 108, 108]), np.array([30, 148, 148])
    expected = processing.find_contours(frame, lower, upper, config.rois["left"])
    monkeypatch.setattr(processing, "mask_observer", view.record)
    actual = processing.find_contours(frame, lower, upper, config.rois["left"])
    assert len(actual) == len(expected)
    for before, after in zip(expected, actual):
        np.testing.assert_array_equal(before, after)
    assert ("left", "black") in view.masks
    mask, _ = view.masks[("left", "black")]
    assert np.count_nonzero(mask) > 0
    assert view.compose(frame).shape == (480, 1120, 3)


@pytest.mark.parametrize("kind", ["vehicle", "track"])
def test_editor_save_preview_reload(kind, tmp_path):
    from PySide6 import QtWidgets as Q
    from sim.editor import Editor
    app = Q.QApplication.instance() or Q.QApplication([])
    window = Editor(kind, "OBSTACLE")
    window.path = tmp_path / (kind + ".json")
    try:
        if kind == "vehicle":
            window.controls[("vehicle", "mass")].setValue(.9)
            window.controls[("camera", "pitch_deg")].setValue(-10)
            window.roi_table.item(0, 1).setText("170")
        else:
            window.controls[("track", "shape")].setCurrentText("rectangle")
            window.controls[("track", "length_scale")].setValue(1.2)
            window.controls[("track", "ambient")].setValue(.25)
            assert "Odometry" in [window.tabs.tabText(i) for i in range(window.tabs.count())]
            window.controls[("odometry", "start_zone_half_width")].setValue(.4)
            window.controls[("odometry", "estimated_wheel_radius")].setValue(.05)
            window.controls[("parking", "custom")].setChecked(True)
            window.controls[("parking", "center_x")].setValue(0)
            window.controls[("parking", "center_y")].setValue(-1.05)
        window.rebuild()
        assert window.save()
        restored = standard_config("OBSTACLE")
        apply(restored, json.loads(window.path.read_text()), kind)
        if kind == "vehicle":
            assert restored.vehicle.mass == .9 and restored.rois["left"][1] == 170
        else:
            assert restored.track.shape == "rectangle" and restored.track.ambient == .25
            assert restored.odometry.start_zone_half_width == .4
            assert restored.odometry.estimated_wheel_radius == .05
            assert restored.parking.custom and restored.parking.center_x == 0
        assert not window.pov.pixmap().isNull()
        assert not window.model_view.pixmap().isNull()
        window.tick()
    finally:
        window.dirty = False
        window.close()


def test_odometry_profile_runtime_and_legacy(tmp_path):
    from types import SimpleNamespace
    from hardware import HardwareBundle, run_entrypoint
    c = standard_config("NO_OBSTACLE")
    c.odometry.start_zone_half_width = .4
    c.odometry.start_zone_half_height = .6
    c.odometry.lap_interval_seconds = 8
    c.odometry.max_drift_threshold = 0
    c.odometry.estimated_wheel_radius = .05
    save(c, "track", tmp_path / "track.json")
    restored = SimConfig.from_argv(["--sim-headless", "--track-profile", str(tmp_path / "track.json")],
                                  width=640, height=480)
    zone = [.75, 1.5]
    namespace = dict(start_zone_rect=zone, tracker=SimpleNamespace(), visualizer=SimpleNamespace())
    exec("def main():\n    assert start_zone_rect == [.4, .6]\n    assert LAP_COUNT_INTERVAL == 8\n", namespace)
    run_entrypoint(namespace["main"], HardwareBundle(None, None, world=SimpleNamespace(config=restored)))
    assert namespace["start_zone_rect"] is zone
    assert namespace["visualizer"].start_zone_rect_y == .6
    assert namespace["tracker"].wheel_radius == .05
    assert namespace["MAX_DRIFT_THRESHOLD"] == 0
    legacy = payload(c, "track")
    del legacy["odometry"]
    apply(restored, legacy, "track")
    assert restored.odometry.start_zone_half_width == .75
    assert restored.odometry.max_drift_threshold == .5


@pytest.mark.parametrize("name,value", [("start_zone_half_width", 0),
    ("estimated_wheel_radius", -1), ("drift_alpha", 1.1),
    ("estimated_ticks_per_rev", 2.5), ("max_drift_threshold", -1)])
def test_invalid_odometry_rejected(name, value):
    c = standard_config("NO_OBSTACLE")
    setattr(c.odometry, name, value)
    with pytest.raises(ValueError, match="Odometry"):
        validate(c)


def test_parking_spawn_geometry_and_validation(tmp_path):
    from sim.profiles import start_pose, parking_geometry
    c = standard_config("OBSTACLE")
    assert start_pose(c, "OBSTACLE")[0] == (-1.05, -1.16, 0)
    c.parking.custom = True
    c.parking.center_x, c.parking.center_y = 1, 0
    c.parking.rotation_deg = 90
    validate(c)
    assert not valid_post(c, 1, 0)
    assert valid_post(c, -1.05, -1.16)
    save(c, "track", tmp_path / "parking.json")
    restored = standard_config("OBSTACLE")
    apply(restored, json.loads((tmp_path / "parking.json").read_text()), "track")
    assert restored.parking.rotation_deg == 90
    world = PyBulletWorld(restored, "OBSTACLE", tmp_path)
    try:
        position, yaw = start_pose(restored, "OBSTACLE")
        actual, _ = world.vehicle.pose()
        np.testing.assert_allclose(actual[:2], position[:2], atol=1e-6)
        assert yaw == pytest.approx(np.pi/2)
        assert position[:2] == pytest.approx((1.085, 0))
        back, _ = world.p.getBasePositionAndOrientation(world.parking_bodies[0], physicsClientId=world.client_id)
        assert back[:2] == pytest.approx((1.19, 0))
    finally:
        world.close()
    restored.parking.center_x = 0
    with pytest.raises(ValueError, match="Parking bay"):
        validate(restored)
    legacy = payload(c, "track")
    del legacy["parking"]
    apply(restored, legacy, "track")
    assert not restored.parking.custom


def test_parking_draft_preview_does_not_freeze_on_invalid_position():
    from PySide6 import QtWidgets as Q
    from sim.editor import Editor
    app = Q.QApplication.instance() or Q.QApplication([])
    w = Editor("track", "NO_OBSTACLE")
    try:
        w.controls[("parking", "custom")].setChecked(True)
        w.controls[("parking", "center_x")].setValue(0)
        before = w.model_view.pixmap().toImage()
        old_world = w.world
        w.controls[("parking", "center_y")].setValue(0)
        # Immediate draft rendering occurs without waiting for the rebuild timer.
        assert w.model_view.pixmap().toImage() != before
        assert "Invalid draft" in w.plan_status.text()
        w.rebuild()
        assert w.world is old_world
        w.tick()
        assert "Invalid draft" in w.plan_status.text()
        w.controls[("parking", "center_y")].setValue(-1.05)
        assert "Live draft" in w.plan_status.text()
        w.controls[("parking", "custom")].setChecked(False)
        assert not w.controls[("parking", "center_x")].isEnabled()
        assert "Default parking" in w.plan_status.text()
    finally:
        w.dirty = False
        w.close()
