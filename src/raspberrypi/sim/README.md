# Vehicle editor, track editor and PyBullet driving view

Run commands from the repository's `src` directory.

```bash
python3 -m pip install -r raspberrypi/sim/requirements.txt
python3 raspberrypi/vehicle_editor.py
python3 raspberrypi/track_editor.py --mode obstacle
python3 raspberrypi/simulator.py
python3 raspberrypi/simulator.py --mode obstacle
```

The editors use Qt and a separate PyBullet DIRECT client for previews. They do
not open the simulator's old slider panel. Use the same Python environment for
all three programs. The supplied `.venv-sim311` has the dependencies installed.

## Vehicle configuration

Use the Vehicle, Camera, Noise and ROIs tabs. Dimensions, mass, wheel radius and
wheelbase rebuild the actual vehicle geometry and inertia. Servo centre/span,
steering torque/rate, tire friction, encoder scale and noise affect the adapters.
Drive force is the per-wheel motor torque in N m (PyBullet revolute joint units).

The model preview has orbit controls and manual motor/servo commands. Those
commands affect only the preview. The POV shows ROI rectangles and danger lines;
the mask checkbox shows the black-wall detector's current colour calibration.
Camera height is measured above the chassis centre of mass, matching the existing
adapter convention. Exposure is a digital gain, not a physical camera exposure.

Save writes a profile; Save as creates another. Closing with unsaved changes
prompts to save. Launch driving view saves first. Existing running simulators
load edited profiles on restart. The standard wheel radius (0.046 m) and encoder
ticks (2220/revolution) match the protected odometry code. Editing either is
useful for sensitivity tests but introduces an odometry mismatch unless the
estimator calibration in Track setup > Odometry is also changed deliberately.

## Track setup

Use the **Parking** tab to move the physical parking/start area. Enable **Use
custom parking / start pose**, then set bay centre X/Y, width/depth and rotation.
Coordinates are absolute world metres from the track centre, not scaled when
track dimensions change. The U-shaped bay opens toward its local +Y axis.
Start offsets are measured from the bay centre in its rotated local axes;
start heading is relative to the bay rotation. Custom heading does not change
automatically with the driving direction. These settings control spawn in both
challenges. The magenta preview shows the bay and the blue arrow shows spawn
heading. Save the track profile and restart to apply. Disable custom placement
to restore the direction-dependent default bay and mode-specific start poses.
Validation rejects wall/island overlaps using conservative rotated bounds and
reserves the bay and spawn against posts. It does not guarantee the unchanged
parking manoeuvre will succeed at every custom location/orientation. The
Odometry tab's start zone is a separate lap-detection region, not this bay.

Choose square or rectangle, dimensions, island size, wall/post dimensions,
ambient/directional lighting, shadow state and light position. Click the plan
to add red/green posts, edit their metre coordinates in the Posts tab, or remove
a selected row. The magenta rectangle shows the parking bay. Placements are
validated against walls, the island, other posts and the initial parking area.
This does not guarantee clearance through the entire parking manoeuvre.
Random posts use the saved seed and count; zero adds none. Posts are active only
in obstacle mode. Arbitrary polygons and track images are not supported.

The separate **Odometry** tab stores start-zone half-width/half-height, minimum
lap interval, drift smoothing and acceptance threshold, and estimator wheel
radius, encoder ticks and gear ratio. Save these with the track profile and
restart the driving view to apply them. Start-zone coordinates are relative to
the initial odometry origin, not the track centre. A half-width of 0.75 m means
the zone spans -0.75 to +0.75 m. This changes lap detection as well as its overlay.
Drift parameters are used by the open challenge only; a threshold of zero
disables acceptance of corrections without changing the correction algorithm.
Old profiles without this section retain the original controller defaults.
The hardware entry point applies these settings once before the control loop;
`odometry.py`, perception and PID code remain unchanged. Physical runs do not
load these simulator settings.

Known visualization issue: both challenge callers pass `gyro_angle + 180`, so
the heading arrow is reversed relative to integration. Its length is hardcoded
to 2.5 m in `odometry.py`. These existing drawing choices are not changed by
the new settings tab. The drawn boundaries remain path-derived estimates.

## Driving view

The 3D scene, processing POV, and odometry windows open automatically. The POV
uses the original controller's ROI/contour overlay, with a panel of actual masks
captured from the workers after colour conversion, filtering and morphology.
Each mask shows its age; asynchronous workers may display different frame ages.
Inactive detectors show no mask. The odometry window uses the existing tracker
and visualizer. There is no duplicate raw-camera window.
With the 3D window focused, Space pauses/resumes the motor and R restarts the
process. Q in the processing window exits. Pause brakes the motor; the controller's
wall-clock timers continue, so use restart for a clean repeat of parking logic.
The launcher enables `--debug` for GUI runs; headless runs omit it by default.

Profiles live in `raspberrypi/sim/profiles/`, separately for each challenge mode.
The old `simulator-settings-*.json` files are retained; normal runs now use the
new profiles. `--sim-controls` restores the legacy sliders and their settings.
An explicit `ECHO_SIM_SETTINGS` also enables legacy overrides. Explicit CLI
options take precedence over saved settings.

```bash
python3 raspberrypi/simulator.py --vehicle-profile /path/vehicle.json --track-profile /path/track.json
```

Standard profiles use 0.20 m/s maximum speed, 0.7 kg mass, 0.18 m wheelbase,
0.122 m wheel spacing, encoder scale 1.0, 25 FPS and full-resolution rendering.
Maximum steering is 30 degrees for open mode and 25 degrees for obstacle mode.
The camera baseline is 110 degrees FOV, -5 degrees pitch, 0.08 m height above
the chassis COM, 0.16 m forward offset, and zero synthetic radial distortion.
The reverse ROI is `[263, 340, 377, 365]`; other ROIs start from the controller's
constants. These are simulator calibration values, not measured camera optics.
See `TEST_REPORT.md` for measured performance and remaining limitations.

For a faster preview use `--sim-render-scale 0.5` (render 320×240 and resize to
640×480), or `--sim-camera-fps 60`. The ROI coordinate system stays 640×480.

## Verification

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest -q raspberrypi/sim/tests
python3 raspberrypi/simulator.py --sim-headless --sim-duration 90
```

The tests cover physical dimensions/mass, lighting changes, post validation,
profile reloads, CLI overrides, ROI object sharing, both editor workflows, and
confirmation that observing masks leaves detector contour outputs unchanged.

PyBullet 3.2.7 may be distributed as source rather than a wheel for newer
Apple Silicon Python installations. The verified fallback is Python 3.11 with
Apple Clang:

```bash
python3.11 -m venv .venv-sim
source .venv-sim/bin/activate
python -m pip install wheel
CC=/usr/bin/clang CXX=/usr/bin/clang++ python -m pip install --no-build-isolation pybullet==3.2.7
python -m pip install -r raspberrypi/sim/requirements.txt
```

Useful options:

```bash
# Deterministic stress test with six additional posts
python3 raspberrypi/obstacle_challenge.py --sim --random-posts 6 --sim-seed 42

# Ideal encoder scale instead of the measured 1.3x scale
python3 raspberrypi/open_challenge.py --sim --encoder-scale 1.0

# Short non-GUI smoke run
python3 raspberrypi/open_challenge.py --sim --sim-headless --sim-duration 5

# Exercise the existing, otherwise unreachable parking-in routine
python3 -m raspberrypi.sim.parking_scenario --side left

# Compare a run with a real tuple log or ticks,angle CSV
python3 -m raspberrypi.sim.compare raspberrypi/sim/runs/RUN_ID --real-log data_analysis/odometry1.csv
```

Each run writes its configuration, commands, telemetry, ground truth and existing
odometry debug output to a unique directory in `raspberrypi/sim/runs/`.
`ground_truth.csv` includes `obstacle_contacts`, excluding floor contacts. Compare
both progress and contacts: a stationary car with zero contacts is not success.

## Reality gap

This minimal world does not model exact fisheye calibration, rolling shutter,
motion blur, automatic exposure, the ND filter, floor glare, tyre deformation,
servo backlash, battery sag, detailed CAD mass distribution, or USB scheduling jitter.
It is intended to test interfaces, control flow, geometry, colour masks and the
vision-odometry handoff, not to predict physical success rates.
