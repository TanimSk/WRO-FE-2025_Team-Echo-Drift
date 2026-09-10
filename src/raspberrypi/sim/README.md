# Minimal PyBullet simulator

Run commands from the repository's `src` directory.

```bash
python3 -m pip install -r raspberrypi/sim/requirements.txt
python3 raspberrypi/open_challenge.py --sim --debug
python3 raspberrypi/obstacle_challenge.py --sim --debug
```

The PyBullet panel contains live controls for maximum speed, drive force,
steering range, camera tilt, camera height, camera FOV, encoder scale, encoder
noise, gyro noise, camera FPS, and POV render scale. GUI runs begin paused:

- `START / RESUME` sends the initial `START` message or resumes a paused run.
- `STOP / PAUSE` brakes the vehicle while leaving the visualization open.
- `RESTART RUN` relaunches the process and resets the vehicle, PID state,
  odometry, lap count, noise state and output logs.

Slider changes take effect during the current run. Headless runs start
automatically because they do not have a control panel.

All slider values are saved immediately in a mode-specific JSON file beside
the simulator source and restored on the next run. Open-track and obstacle-track
settings are kept separate. Set `ECHO_SIM_SETTINGS=/path/to/settings.json` to use
a custom settings location.

The ROI section exposes `x1`, `y1`, `x2`, and `y2` for the left wall, right wall,
lap marker, obstacle, reverse, front-wall, parking, and both danger regions.
Changes update the same mutable region objects used by the existing perception
workers. If the left-wall ROI dimensions are changed, restart once so its cached
area normalization is recalculated from the stored dimensions.

POV render scale defaults to `0.5`: PyBullet renders at 320×240 and OpenCV
restores the image to 640×480 before the unchanged perception code receives it.
Use `--sim-render-scale 1.0` when evaluating pixel-level visual fidelity, or a
lower value for faster control-flow testing.

The simulated camera remains at the physical 25 FPS default. Increase the
`Camera FPS` slider for a more responsive engineering run, or launch with
`--sim-camera-fps 60`. OpenCV is already configured to use all available native
worker threads on the machine.

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
odometry debug output to `raspberrypi/sim/runs/<timestamp>/`.

## Reality gap

This minimal world does not model exact fisheye calibration, rolling shutter,
motion blur, changing exposure, the ND filter, floor glare, tyre deformation,
servo backlash, battery sag, detailed chassis inertia, or USB scheduling jitter.
It is intended to test interfaces, control flow, geometry, colour masks and the
vision-odometry handoff, not to predict physical success rates.
