# Simulator editor and baseline verification

Date: 2026-09-16. Environment: Apple M1, Python 3.11 virtual environment
`.venv-sim311`, PyBullet 3.2.7, Qt/PySide6 6.11.2.

## Application checks

- 13 automated tests passed. They exercise camera/serial compatibility, saved
  profiles, argument precedence, shared ROI objects, actual vehicle mass and
  radius, light response, invalid geometry, both editor save/reload workflows,
  and unchanged contour outputs when mask observation is enabled.
- Both editor windows were rendered and visually inspected. Model and camera
  previews, ROI overlays, calibration inputs and the track plan are present.
- Native PyBullet GUI runs completed with normal shutdown. GUI rendering and
  DIRECT rendering can produce different masks and timing; headless performance
  results below should not be interpreted as GUI success rates.
- The final GUI launcher was also run with debug enabled and visually inspected:
  the 3D scene, actual ROI/contour view with worker masks, and updating odometry
  plot were all present. No duplicate raw-camera window appeared.
- `contour_workers.py`, `odometry.py`, and both challenge control files were not
  edited. The image-processing helper gained optional diagnostic callbacks for
  masks and the rendered debug frame. Thresholding and control outputs are unchanged.

## Measured trials

Contacts below count sampled physics states containing a non-floor contact, not
independent crashes. No claim of a statistical success rate is made from these runs.

| Configuration | Duration | Ground-truth travel | Contact samples | Observed outcome |
|---|---:|---:|---:|---|
| Initial open camera / reverse ROI | 40 s | 5.53 m | 0 | Repeated reversals at first corner |
| Selected open baseline | 90 s | 15.31 m | 0 / 2055 | Traversed all four sides and completed a physical circuit |
| Obstacle, 30-degree steering | 90 s | 6.22 m | 65 / 1723 | Contact during scripted parking-out |
| Selected obstacle, 25-degree steering | 90 s | 6.20 m | 0 / 1791 | Parking-out completed; repeated reversals near first corner |

Open baseline: 110-degree FOV, -5-degree pitch, 0.08 m mount height above the
chassis COM, 0.16 m forward offset, no synthetic radial distortion, reverse ROI
`[263, 340, 377, 365]`. Maximum speed 0.20 m/s, wheel radius 0.046 m,
2220 ticks/revolution, mass 0.7 kg, wheelbase 0.18 m, wheel spacing 0.122 m.
Encoder scale is 1.0; the editor permits 1.3 for systematic-error experiments.
Other ROI coordinates come from the original controller constants for each mode.

Four camera candidates were compared in 35-second open trials. The selected
candidate reached the next straight with no contact, while the other three
reversed repeatedly near the first corner. A longer 90-second run validated its
physical circuit. Three obstacle steering candidates (20, 25, 35 degrees) were
then tested for 35 seconds: 20 and 25 cleared parking-out without contact;
35 recorded 112 contact samples. The 25-degree choice received the longer test.

An initial concurrent comparison was discarded because timestamp-only run names
collided and mixed logs. Run directories now use an atomic unique suffix; all
reported candidate comparisons were rerun with separate logs.

## Reproducible logs

All paths are relative to this `sim` directory:

- Open selected: `runs/20260916_031207_321029_zpozpety/`
- Obstacle 30 degrees: `runs/20260916_031207_377902_1vykyluy/`
- Obstacle selected: `runs/20260916_032926_630001_dfzbnkmu/`

Each includes its exact configuration, commanded steering/speed, noisy sensor
telemetry and ground-truth poses/contact counts. These local run artifacts are
ignored by Git. Repeat with the saved standard profiles and `--sim-duration 90`.

## Limits and remaining work

The open result is one physical circuit, not proof that the unchanged lap counter
or three-lap termination is correct. Obstacle mode is not validated for a complete
lap, avoidance success, or parking-in. Its repeated reversal remains visible and
requires further calibration or separate control-logic investigation.

These profiles are a tested starting point, not a global optimum. Lighting,
custom layouts, randomized posts, clockwise travel and other seeds have not
received full-course validation. Square and rectangular track layouts are
supported; arbitrary track polygons are not. Exposure gain and directional
lighting are simplified models of real camera/lighting behaviour.

Changing physical wheel radius or encoder ticks does not change the protected
odometry constants. Such changes intentionally introduce a calibration mismatch.
