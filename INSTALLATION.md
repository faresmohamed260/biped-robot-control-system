# Installation Guide

This guide covers the current **Biped Robot** workflow.

Recommended operator flow:

1. compile or upload the ESP32 firmware if needed
2. run the desktop app
3. connect and verify saved flash poses
4. test the live vision feed
5. run autonomous forward-until-collision

The Streamlit dashboard remains available as an advanced secondary interface, but the desktop app is the main day-to-day control tool.

## 1. Requirements

### Hardware

- ESP32 board running [`biped_robot.ino`](biped_robot.ino)
- six 180-degree servos
- USB cable for flashing
- Android phone running IP Webcam or another JPEG feed source
- visible `yellow` robot marker
- visible `red` ball

### Software

- Windows 10 or 11
- Python 3.10
- Arduino IDE or `arduino-cli`
- ESP32 Bluepad32 board package

## 2. Python Setup

From the repository root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r .\requirements.txt
```

Or use the helper script:

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\install_python_deps.ps1
```

## 3. ESP32 / Bluepad32 Setup

Install the ESP32 Bluepad32 core:

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\install_bluepad32_core.ps1
```

## 4. Compile and Flash Firmware

Compile:

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn esp32-bluepad32:esp32:esp32 .
```

Upload:

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' upload -p COM4 --fqbn esp32-bluepad32:esp32:esp32 .
```

Replace `COM4` with the correct port.

Compatibility note:

- the sketch and folder intentionally share the Arduino-compatible name [`biped_robot.ino`](biped_robot.ino)

## 5. Launch the Desktop App

Run:

```powershell
.\biped_desktop_app.bat
```

The app will try to discover the biped robot automatically on startup using the shared endpoint discovery module.

## 6. First Setup in the Desktop App

### Setup tab

Use the `Setup` tab to:

- connect or auto-discover the robot
- load saved flash poses
- edit or capture poses
- save updated poses back to flash
- test the current forward sequence

### Joints tab

Use the `Joints` tab to:

- move a joint directly
- nudge a joint
- home, attach, or detach a joint
- update limits, pulse values, and calibration fields

### Wi-Fi tab

Use the `Wi-Fi` tab to:

- configure hostname
- configure AP and station credentials
- reconnect Wi-Fi
- reboot the ESP32

### Controller tab

Use the `Controller` tab to:

- enable or disable controller input
- enable pairing mode
- set the home-all button
- configure LED, rumble, and deadzone
- perform recovery actions

### Vision tab

Use the `Vision` tab to:

- view the live annotated camera feed
- verify robot and ball detection
- choose the motion sequences used by the pursuit loop
- start autonomous pursuit
- stop at collision distance
- export the last recorded demo video

## 7. Camera Setup

The current vision pipeline expects:

- robot marker color: `Yellow`
- ball color: `Red`

Provide an IP Webcam `shot.jpg` URL in the desktop app.

## 8. Default Motion Logic

The current default forward sequence is:

- `stand,left_forward,right_forward,stand`

Autonomous mode currently:

- keeps moving forward when the robot and ball are both detected
- stops when the collision distance is reached
- does not turn left or right automatically

## 9. Optional Streamlit Dashboard

If you want the advanced dashboard:

```powershell
.\biped_streamlit_dashboard.bat
```

Or directly:

```powershell
python -m streamlit run .\streamlit_app.py
```

## 10. Deliverable Assets

Current submission media is stored in [`output/`](output/).

Important files:

- poster: [`output/poster.jpeg`](output/poster.jpeg)
- demo video: [`output/vision_recordings/autonomy_20260513_232641.mp4`](output/vision_recordings/autonomy_20260513_232641.mp4)

## 11. Validation

Python syntax:

```powershell
python -m py_compile .\biped_desktop_app.py .\streamlit_app.py .\biped_vision_tracking.py .\biped_vision_camera.py .\biped_endpoint_discovery.py .\biped_gait_phase.py
```

Firmware compile:

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn esp32-bluepad32:esp32:esp32 .
```
