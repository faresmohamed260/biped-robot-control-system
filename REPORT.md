# Biped Robot Project Report

## 1. Project Title

**Biped Robot: A Six-Servo ESP32 Walking Platform with Desktop Control and Color-Based Ball Pursuit**

## 2. Executive Summary

Biped Robot is a six-servo ESP32 platform built around a compact bipedal mechanism. The current project combines:

- embedded motion execution on the ESP32
- a Python desktop control application for calibration and operation
- a Streamlit dashboard for advanced diagnostics and configuration
- a color-based vision pipeline that tracks the robot and a ball from a phone camera feed
- autonomous forward movement until collision distance is reached

The implementation favors a reliable local workflow over theoretical complexity. Walking behavior is driven by manually calibrated flash poses and tuned host-side sequences rather than a fully dynamic gait solver.

## 3. System Objectives

Current goals:

- provide a practical operator interface for the biped robot
- support per-joint calibration and saved pose tuning
- detect the robot and the ball from a phone camera feed
- move the robot toward the ball
- stop automatically when collision distance is reached
- keep screenshots, poster, and demo video organized for submission

## 4. Hardware Configuration

The robot uses six servos and an ESP32. The firmware retains legacy HTTP joint ids, but those ids now map to the biped legs:

| API name | Physical joint |
|---|---|
| `base` | Left Ankle |
| `shoulder` | Left Knee |
| `elbow` | Left Hip |
| `wrist_pitch` | Right Knee |
| `wrist_rotate` | Right Ankle |
| `gripper` | Right Hip |

All six motors are configured as `positional_180`.

Current stand pose:

| Joint | Angle |
|---|---:|
| Left Ankle | 80 |
| Left Knee | 90 |
| Left Hip | 110 |
| Right Knee | 25 |
| Right Ankle | 40 |
| Right Hip | 135 |

## 5. Software Architecture

### 5.1 Firmware

Firmware sketch:

- [`biped_robot.ino`](biped_robot.ino)

Responsibilities:

- joint execution
- pose interpolation
- pose persistence in flash
- Wi-Fi setup
- controller handling
- exposing HTTP APIs such as:
  - `/api/state`
  - `/api/system`
  - `/api/joint`
  - `/api/biped`
  - `/api/ps4`
  - `/api/wifi`

### 5.2 Desktop Control App

Main operator interface:

- [`biped_desktop_app.py`](biped_desktop_app.py)
- launched with [`biped_desktop_app.bat`](biped_desktop_app.bat)

Desktop app features:

- automatic device discovery
- flash save and load
- per-joint editing and calibration
- Wi-Fi management
- controller management
- pose editor
- sequence editor
- live camera view
- autonomous vision run control
- optional video export

### 5.3 Advanced Dashboard

Advanced dashboard:

- [`streamlit_app.py`](streamlit_app.py)

This is no longer the primary workflow, but it remains useful for diagnostics, sequence recording, gait experiments, and direct low-level control visibility.

## 6. Walking and Motion Strategy

Early attempts to force a geometry-first gait model caused unstable motion. The robot often shifted sideways or slid backward because mathematically plausible poses were not necessarily mechanically stable on the real hardware.

The workable strategy became:

1. manually discover stable poses
2. store those poses in flash
3. use the flash poses as motion primitives
4. tune sequence order from the host application

The forward sequence that currently works after manual calibration is:

- `stand,left_forward,right_forward,stand`

This sequence is now the default forward action in both the desktop app and the vision workflow.

## 7. Vision Pipeline

Vision modules:

- [`biped_vision_camera.py`](biped_vision_camera.py)
- [`biped_vision_tracking.py`](biped_vision_tracking.py)

The pipeline is intentionally color-based:

- robot detection uses a `yellow` marker
- ball detection uses a `red` HSV mask

Behavior:

- the robot marker is explicitly drawn on the live feed
- the distance line is anchored to the visible marker when present
- the app computes distance and `dx/dy` between robot and ball

This avoids external model files and keeps tuning practical under local lighting.

## 8. Autonomous Behavior

The desktop app uses a deliberately simple autonomy rule:

- if robot and ball are both visible and the distance is above the collision threshold:
  - run the configured forward sequence
- if collision distance is reached:
  - stop

At the current stage:

- no autonomous turning is performed
- the focus is a predictable forward approach and reliable stop condition

## 9. Repository Cleanup and Refactor

The repository was cleaned to match the current biped design.

Removed:

- unused experiment trees
- ROS support folders no longer relevant to the current deliverables
- temporary tutorial imports
- generated build and runtime caches
- obsolete assets not used by the active biped workflow

Renamed:

- desktop app files to `biped_desktop_app.*`
- vision modules to `biped_vision_*`
- endpoint discovery to `biped_endpoint_discovery.py`
- phase gait helper to `biped_gait_phase.py`
- Streamlit launcher to `biped_streamlit_dashboard.bat`

Retained:

- production firmware
- desktop app
- Streamlit dashboard
- color-based vision modules
- current deliverable assets in `output/`

## 10. Deliverables Status

### 10.1 Source Code

Active source code:

- [`biped_robot.ino`](biped_robot.ino)
- [`biped_desktop_app.py`](biped_desktop_app.py)
- [`streamlit_app.py`](streamlit_app.py)
- [`biped_vision_tracking.py`](biped_vision_tracking.py)
- [`biped_vision_camera.py`](biped_vision_camera.py)
- [`biped_endpoint_discovery.py`](biped_endpoint_discovery.py)
- [`biped_gait_phase.py`](biped_gait_phase.py)

### 10.2 Report

This markdown file is the report source:

- [`REPORT.md`](REPORT.md)

PDF conversion is intentionally deferred to the next step.

### 10.3 Presentation

Deferred for later work.

### 10.4 Marketing Poster

- [`output/poster.jpeg`](output/poster.jpeg)

### 10.5 Demo Video

- [`output/vision_recordings/autonomy_20260513_232641.mp4`](output/vision_recordings/autonomy_20260513_232641.mp4)

### 10.6 Additional Materials

Available screenshots:

- [`output/setup.png`](output/setup.png)
- [`output/joints.png`](output/joints.png)
- [`output/wifi.png`](output/wifi.png)
- [`output/controller.png`](output/controller.png)
- [`output/vision.png`](output/vision.png)
- [`output/state.png`](output/state.png)

## 11. Known Deferred Work

The following are intentionally postponed:

- PDF export of the report
- presentation deck creation
- more advanced autonomous steering behavior

## 12. Conclusion

The repository is now aligned with the current bipedal design rather than the earlier arm-centric project identity. The active codebase is smaller, naming is clearer, and the operator workflow is centered on a practical desktop app plus a stable color-based vision loop. The main next step is converting the report to PDF and finishing the remaining submission materials.
