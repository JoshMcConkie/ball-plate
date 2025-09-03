# Self-Balancing Table

A computer-vision and control systems project where a tilting table keeps a ball centered on its surface.

The system uses **AprilTags** for table calibration, **OpenCV** for ball tracking, and an **Arduino** for closed-loop control of two servos. A **Kalman filter** for improved state estimation is in progress (WIP).

---

## ✨ Features
- **AprilTag homography** – maps camera pixels to table coordinates.
- **Ball detection** – HSV mask + contour detection.
- **Velocity estimation** – frame differencing with exponential smoothing.
- **50 Hz serial loop** – Python streams `(x, y, vx, vy)` to Arduino.
- **Arduino PD control** – error-based servo commands.
- **Modular structure** – tracker (Python) and controller (Arduino) are independent.

---

## 📂 Repository Layout
```
ball_track.py           # Python: camera, AprilTags, ball tracking, serial streaming
controller.ino          # Arduino: PD control for servos
XY_PID_controller.ipynb # Notebook for controller tuning experiments
sim/                    # Simulation experiments (in progress)
README.md               # Project documentation
```

---

## 🛠 Requirements

### Python
- Python 3.9+
- OpenCV (`cv2`)
- NumPy
- [pupil_apriltags](https://pypi.org/project/pupil-apriltags/)
- pySerial

Install:
```bash
pip install -r requirements.txt
```

### Arduino
- Arduino Uno (or compatible)
- Two hobby servos (tested with high‑torque MG series)
- External 5–7 V supply with enough current for both servos
- **Common ground** between Arduino and servo power

---

## 🚀 Quick Start

1. **Wire hardware** (two servos on pins 9 & 10; dedicated 5–6 V servo supply; common ground).
2. **Upload** `controller.ino` to the Arduino (115200 baud).
3. **Place four AprilTags** (tag36h11 family) at table corners and set their IDs in `TAG_TABLE_COORDS`.
4. **Start the tracker**:
   ```bash
   python ball_track.py
   ```
5. You should see the camera feed and the Arduino echoing parsed packets: `OK,x,y,vx,vy`.

---

## ⚙️ How It Works

1. **Calibration** – Four AprilTags at the table corners define the pixel → table mapping (homography).
2. **Tracking** – Python isolates the ball via HSV color segmentation, finds the centroid, then maps to table coordinates.
3. **Estimation** – Velocity is estimated by frame‑to‑frame differencing (Kalman filter WIP).
4. **Control** – `(x, y, vx, vy)` packets are sent to Arduino at ~50 Hz.
5. **Actuation** – Arduino runs PD control and writes servo angles to tilt the table.

---

## 🔧 Configuration Notes

### Python (`ball_track.py`)
- **Ball color**: change `lower_ball_color`/`upper_ball_color` (HSV) for your ball.
- **Camera**: set `CAMERA_ID`.
- **Serial**: set `SERIAL_ON` and COM port; default is `COM3` at `115200`.
- **Send rate**: `SEND_INTERVAL` (default 1/50 s).

### Arduino (`controller.ino`)
- **Gains**: `Kp`, `Kd` (and `Ki` placeholder) tune response.
- **Servo limits**: `MIN_US_*` / `MAX_US_*` (angles are mapped to known microsecond bounds)
- **Goal**: `goal_x`, `goal_y` center the ball; `goal_window` defines a small dead‑zone.

---

## 🚧 Current Limitations

- **Kalman filter** debugging in progress (simple differencing used).
- **Integer packets** coarsen resolution.

---

## 🗺 Roadmap

- [ ] Integrate Kalman filter for smoother state estimation.
- [ ] Add simulation environment for testing control strategies.
- [ ] Explore model‑based control (LQR, MPC).

---

## 📸 Demo(https://photos.app.goo.gl/n6tguVm4wNqeG5kc6)

---

## 📜 License

MIT License — open for educational and research use.
