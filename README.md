# carla_path_tracking

Path tracking in the **CARLA** simulator using **Pure Pursuit** for lateral control and **PID** for longitudinal control, with an optional **traffic‐sign detection** loop that modulates the target speed. Utilities are included to record and visualize ego trajectories and to teleoperate with a joystick.

> This README is based on the repository’s current files (`pid.py`, `control_with_vision.py`, `record_ego_path.py`, `plot_ego-path.py`, `compare_plots.py`, `carla_joy_teleop.py`, and the `waypoints/` & `carla_object_detection/` folders).

---

## Table of Contents

- [Overview](#overview)
- [Repository structure](#repository-structure)
- [Requirements](#requirements)
- [Quick start](#quick-start)
- [Controllers](#controllers)
  - [Pure Pursuit (lateral)](#pure-pursuit-lateral)
  - [PID (longitudinal)](#pid-longitudinal)
- [Traffic‑sign–aware speed control](#traffic-signaware-speed-control)
- [Data recording & plotting](#data-recording--plotting)
- [Joystick teleop](#joystick-teleop)
- [carla_object_detection module](#carla_object_detection-module)
- [Configuration variables](#configuration-variables)
- [Tips & troubleshooting](#tips--troubleshooting)
- [License](#license)

---

## Overview

The project demonstrates a clean baseline for **trajectory tracking** in CARLA that you can extend with your own planners or perception modules. It combines:

- **Pure Pursuit** for steering along pre‑sampled waypoints.
- **PID** for speed regulation to a target (possibly modified by traffic signs).
- **Traffic sign detection** to dynamically adjust speed limits inside the control loop.
- **Path utilities** to record the ego path & compare controller runs.

---

## Repository structure

```
carla_path_tracking/
├─ carla_object_detection/      # traffic sign / object detection helpers
├─ waypoints/                   # waypoint CSVs or pickle files (sample paths)
├─ pid.py                       # PID class and tuning helpers
├─ control_with_vision.py       # main loop: controller + traffic sign detection
├─ record_ego_path.py           # record ego (x,y,v,...) along a drive to CSV
├─ plot_ego-path.py             # plot a single run
├─ compare_plots.py             # compare multiple runs (errors, controls, speed)
└─ carla_joy_teleop.py          # optional joystick teleoperation
```

---

## Requirements

- **CARLA** ≥ 0.9.x (Python API installed)
- **Python** 3.8+
- Common packages: `numpy`, `matplotlib`, `pandas`, `opencv-python` (for vision), `pygame` (for joystick), `scipy`
- (Optional) CUDA/cuDNN if you use a DNN‑based detector in `carla_object_detection/`

Set up CARLA env vars (example):
```bash
export CARLA_ROOT=~/CARLA_0.9.X
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-*-py3.*.egg:$CARLA_ROOT/PythonAPI/carla
```

---

## Quick start

1) **Launch CARLA** (server):
```bash
./CarlaUE4.sh -quality-level=Low -prefernvidia
```

2) **Drive & record a path** (creates a CSV in `waypoints/`):
```bash
python3 record_ego_path.py
```

3) **Run controller (pure pursuit + PID)** on recorded waypoints:
```bash
python3 control_with_vision.py
```

4) **Inspect results**:
```bash
python3 plot_ego-path.py
python3 compare_plots.py
```

---

## Controllers

### Pure Pursuit (lateral)

Given a look‑ahead distance \(L_d\) and wheelbase \(L\), steering command \(\delta\) is computed from the curvature to the goal point \(G\) on the path.

Key variables inside `control_with_vision.py`:
- `LOOKAHEAD_DIST`: base look‑ahead distance [m]
- `K_LOOKAHEAD`: scale factor w.r.t. speed
- `WHEELBASE`: vehicle wheelbase [m]
- `MAX_STEER`: steering saturation [deg or rad]

### PID (longitudinal)

A standard PID tracks the desired speed \(v_d\). Typical tunables include proportional, integral, derivative gains, integral guard, and output saturation.

Key variables inside `pid.py`:
- `KP`, `KI`, `KD`
- `THROTTLE_MAX`, `BRAKE_MAX`
- `INTEGRAL_GUARD`

---

## Traffic‑sign–aware speed control

`control_with_vision.py` integrates detection from `carla_object_detection/` to read **speed‑limit** or **stop** signs and adjust the target speed passed to the PID. The loop is:

1. Read current waypoint target speed.
2. Detect traffic signs in the camera frame.
3. If a sign is found (e.g., 30/50/80 km/h), override/limit `v_d` for a configured horizon.
4. Feed `v_d` into PID; feed position to Pure Pursuit.
5. Apply control via CARLA’s vehicle API.

---

## Data recording & plotting

- **`record_ego_path.py`** logs `{t, x, y, yaw, v, a}` while you drive or while a controller runs.
- **`plot_ego-path.py`** visualizes a single run: the path, tracking error, speed & control.
- **`compare_plots.py`** stacks multiple logs to compare gain sets, look‑ahead policies, or detectors.

All logs are CSV for easy analysis in pandas/NumPy.

---

## Joystick teleop

`carla_joy_teleop.py` maps a gamepad to throttle/brake/steer so you can:
- Drive to **record** initial waypoints.
- Take **manual control** if the detector or controller misbehaves.

---

## carla_object_detection module

The `carla_object_detection/` folder contains helpers to process camera images and detect traffic signs or objects in the CARLA world. Typical components include:

- **Model loading**: pretrained detectors for traffic signs (e.g., YOLO/SSD or classical OpenCV approaches).  
- **Image preprocessing**: resizing, color conversions, ROI cropping.  
- **Detection output**: bounding boxes and class IDs for speed signs, stop signs, etc.  
- **Integration hooks**: functions that `control_with_vision.py` calls to get the currently detected speed limit or stop condition.

These detections directly affect the **desired speed variable** before PID control. You can extend this module with your own detectors, different sign classes, or fuse detections with map priors.

---

## Configuration variables

Instead of CLI flags, this repo sets configuration via Python variables/constants inside the scripts. Examples:

- In **`pid.py`**: gains (`KP`, `KI`, `KD`), limits (`THROTTLE_MAX`, `BRAKE_MAX`).  
- In **`control_with_vision.py`**: look‑ahead distance (`LOOKAHEAD_DIST`), wheelbase (`WHEELBASE`), enable/disable vision integration (`USE_VISION = True/False`).  
- In **`record_ego_path.py`**: duration of logging, output file path.  
- In **plotting scripts**: log file paths, output directory, figure options.  

Adjust these variables directly in the source code or centralize them into a config file for cleaner experiments.

---

## Tips & troubleshooting

- **Physics step**: Use fixed time step in CARLA for deterministic runs (`-fps=20`).
- **Vehicle model**: Controllers assume an Ackermann‐style vehicle; wheelbase matters.
- **Look‑ahead tuning**: Increase with speed; too small → oscillations, too big → corner cutting.
- **PID tuning**: Start with P‑only, add I for bias, small D to damp overshoot.
- **Detector latency**: Use a low‑res camera and crop ROIs to keep the loop real‑time.
- **Waypoint density**: 1–2 m spacing is a good starting point.

---

## License

MIT (see `LICENSE`).
