<p align="center">
  <img src="https://github.com/user-attachments/assets/678139fc-c481-4e6f-af34-0d96fa4a1ab4" width="100%" alt="VYRA.ADV Banner" />
</p>

<h1 align="center">VYRA.ADV</h1>
<p align="center">
  <strong>Autonomous Driving Robot | Robofest Gujarat 5.0</strong><br>
  <em>Computer Vision + Sensor Fusion + Trajectory Planning</em>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Platform-Raspberry%20Pi%205-c51a4a?style=flat-square&logo=raspberrypi" />
  <img src="https://img.shields.io/badge/Vision-OpenCV%20+%20Pi%20Camera%203-5C3EE8?style=flat-square&logo=opencv" />
  <img src="https://img.shields.io/badge/AI-TensorFlow%20Lite-FF6F00?style=flat-square&logo=tensorflow" />
  <img src="https://img.shields.io/badge/Language-Python%203.11-3776AB?style=flat-square&logo=python" />
  <img src="https://img.shields.io/badge/Stage-Prototype-blue?style=flat-square" />
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/256b1a67-fe0e-418c-883b-6460085999c7" width="100%" alt="VYRA.ADV Build" />
</p>

---

## About

**VYRA.ADV** is a 4WD autonomous driving robot built for **Robofest Gujarat 5.0** — a national-level robotics competition with a **Rs 5,00,000 grand prize**. The robot navigates a 3m x 4m indoor track autonomously, following lanes, responding to traffic signals, avoiding obstacles, and executing reverse parking.

The project combines real-time computer vision, IR sensor fusion, and AI inference on a Raspberry Pi 5 to achieve full autonomy without any remote control.

**Stage 2 (Proof of Concept) secured Rs 1,00,000 prize.** Currently building the final prototype for Stage 3 submission on March 10th, 2026.

---

## Competition Scoring

| Task | Points | Approach |
|:-----|:------:|:---------|
| Lane Following | 30 | Camera CV + 5-channel IR array hybrid |
| Traffic Signals | 10 | TFLite model (Red / Green / Yellow) |
| Obstacle Avoidance | 15 | Dual ultrasonic sensors + dynamic replanning |
| Marker Identification | 10 | YOLO-based visual detection |
| Reverse Parking | 15 | Ultrasonic-guided 90-degree dock |
| Emergency Stop | 10 | Flashing red detection → instant halt |
| Time Bonus | 10 | Speed optimization on straights |
| **Total** | **100** | |

---

## Hardware

### Compute
| Component | Spec |
|:----------|:-----|
| Processor | Raspberry Pi 5 — 8GB RAM |
| Storage | NVMe via M.2 HAT+ (SC1166) |
| Camera | Pi Camera Module 3 (IMX708, 12MP) |
| OS | Raspberry Pi OS (64-bit, Bookworm) |

### Drive System
| Component | Spec |
|:----------|:-----|
| Motors | 4x BO Motors (3-6V DC) |
| Drivers | 2x L298N Dual H-Bridge |
| Wheels | 60mm diameter |
| Config | Skid-steer differential drive |
| Battery | 2x 18650 3S packs (11.1V) in parallel |

### Sensors
| Sensor | Purpose | Interface |
|:-------|:--------|:----------|
| Pi Camera Module 3 | Lane detection, traffic signals, markers | CSI ribbon |
| SmartElex 5-Channel IR Array | Ground-truth line tracking at corners | GPIO digital |
| Front Ultrasonic (HC-SR04) | Obstacle detection | GPIO (Trig/Echo) |
| Rear Ultrasonic (HC-SR04) | Reverse parking alignment | GPIO (Trig/Echo) |

### Chassis
- Custom 3-tier vertical architecture
- Printed in Apple Red Brahma Lab PETG
- Camera mounted at fixed downward angle (ground visible from 20cm ahead)

---

## GPIO Pin Map

```
Raspberry Pi 5 (BCM Mode)
──────────────────────────────────────────────
MOTORS (via L298N)
  Left Side:   IN1=16  IN2=20  EN=21 (PWM)
  Right Side:  IN1=5   IN2=6   EN=12 (PWM)

IR SENSOR ARRAY (5-Channel)
  [Left, L2, Center, R2, Right]
  Pins: [24, 25, 8, 7, 1]

ULTRASONIC (Front)
  TRIG=2  ECHO=3  (Echo via voltage divider)

MOTOR DIRECTION (Forward)
  L_IN1=LOW  L_IN2=HIGH
  R_IN1=LOW  R_IN2=HIGH
──────────────────────────────────────────────
```

> **Note:** Ultrasonic Echo pins use a 1K/2K voltage divider to step 5V → 3.3V for Pi 5 GPIO safety.

---

## Software Architecture

### Lane Following Pipeline (`lane-test-1.py`)

The primary lane-following system uses a priority-fallback architecture with a state machine:

```
Camera Frame (640x360 @ 25fps, BGR)
  │
  ├── Grayscale → Gaussian Blur → Fixed Threshold (70)
  │   (Overexposed camera washes out floor texture,
  │    only black tape lines survive thresholding)
  │
  ├── FAR ROI (rows 198-288, ground 28-45cm ahead)
  │   └── Curvature preview → speed modulation
  │   └── Corner detection → latch turn direction
  │
  ├── NEAR ROI (rows 288-360, ground 20-28cm ahead)
  │   └── Priority scan: row 350 → 320 → 295
  │   └── Column-projection line detection (not contours)
  │   └── PD steering control
  │
  └── State Machine
      ├── TRACKING  → PD control from nearest scan row
      ├── TURN      → Aggressive point-turn (latched direction)
      └── BLIND     → Decayed last-known error continuation
```

**Key design decisions:**
- **Column projection** instead of `findContours` — at right-angle corners, contours merge into one blob, but each horizontal row still crosses the two lines separately
- **Priority fallback** (not weighted averaging) — prevents distant rows from pulling steering toward future turns
- **Camera overexposure** (EV=2.5, Contrast=3.0, Saturation=0.0) — washes out marble floor reflections and shadows, leaving only black tape visible
- **Full sensor FOV** via `raw={"size": (2304, 1296)}` + `ScalerCrop` — default mode crops vertical FOV

### Traffic Signal Detection

TensorFlow Lite model classifies traffic signals into Red, Green, Yellow in real-time. The model runs inference on captured frames and triggers appropriate drive responses.

### Camera Configuration

```python
# picamera2 "RGB888" outputs BGR byte order (DRM naming convention)
# Full vertical FOV requires 2304x1296 raw mode + ScalerCrop
picam2.set_controls({
    "ScalerCrop": (0, 0, 4608, 2592),  # full sensor readout
    "ExposureValue": 2.5,               # overexpose to wash out floor
    "Contrast": 3.0,                    # maximize line visibility
    "Saturation": 0.0,                  # grayscale-like output
})
```

---

## Repository Structure

```
Documents/
├── lane-test-1.py              # Main lane following (CV + state machine)
├── robofest-code-1.py          # Integrated autonomy (CV + IR + ultrasonic + TFLite)
├── line-following-code.py      # IR-only line following (standalone)
├── camera-motor-test.py        # Manual drive + camera dashcam (WASD controls)
├── camera-test.py              # Camera preview + configuration test
├── take-photo.py               # Single photo capture utility
├── obstacle-avoid.py           # Ultrasonic obstacle avoidance
├── obstacle-detection-cv.py    # CV-based obstacle detection
├── full-autonomous-code-v1-*.py   # Autonomy iteration v1 (Gemini-assisted)
├── full-autonomous-code-v2-*.py   # Autonomy iteration v2 (Gemini-assisted)
├── ai.py / ai-us.py           # Traffic signal AI inference
├── only-traffic.py             # Traffic signal detection standalone
├── motor_test.py / motor1_test.py  # Motor driver testing
├── L298n.py                    # L298N basic test
├── LED_code.py                 # LED indicator test
├── ultrasonic-test.py          # Ultrasonic sensor test
├── web_cam.py                  # Web camera feed
├── model_unquant.tflite        # Traffic signal TFLite model
├── labels.txt                  # Model class labels
├── yolov8n.pt                  # YOLOv8 nano weights
├── opencv-images/              # Debug screenshots from CV pipeline
│   ├── VYRA Lane-RightAngleTurn.png
│   ├── Near Binary_RIGHTANGLETURN.png
│   ├── Ground View_screenshot_*.png
│   └── Ground Tracker-*.png
├── green_light/                # Training data — green signals
├── red_light/                  # Training data — red signals
├── yellow_light/               # Training data — yellow signals
├── converted_tflite/           # Converted TFLite model files
├── static/                     # Dashcam captured frames
└── venv/                       # Python virtual environment
```

---

## Running

```bash
# Activate the virtual environment (required for OpenCV)
source venv/bin/activate

# Lane following (main competition code)
python3 lane-test-1.py

# Manual drive with camera (WASD controls)
python3 camera-motor-test.py

# IR-only line following
python3 line-following-code.py

# Camera preview test
python3 camera-test.py
```

> **Important:** The system Python does not have OpenCV installed. Always use the venv, or run directly with `venv/bin/python3`.

---

## Development Log

### What Works
- Straight-line lane following at speed (BASE_SPEED 50%, ~22 cm/s)
- S-curve tracking with curvature-based speed modulation
- U-bend navigation
- Clean binary thresholding on glossy marble floor via overexposure technique
- Full camera FOV utilization (2304x1296 raw → 640x360 main)
- Safe motor shutdown (PWM stop → pins LOW → delay → GPIO cleanup)
- Video recording with debug HUD overlay for analysis

### Active Challenges
- **Right-angle turns**: Camera has a geometric blind spot — lines exit FOV at 90-degree corners. Current mitigation: curvature-based turn latching + blind point-turn execution. Next step: hybrid IR+camera integration.
- **False detections**: Floor objects (slippers, furniture edges) occasionally detected as lane lines in the overexposed view.
- **Far ROI reliability**: The far preview region detects shadows and specular reflections as lines, causing false curvature readings.

### Engineering Discoveries
| Discovery | Impact |
|:----------|:-------|
| picamera2 `RGB888` = BGR byte order | Was using wrong grayscale conversion weights |
| Default 640x360 selects 1536x864 sensor crop | Lost 30% vertical FOV; fixed with explicit raw mode |
| `GPIO.cleanup()` makes pins float → L298N EN pulled HIGH | Motors ran wild after script exit; fixed with explicit LOW + PWM stop |
| `findContours` merges lines at corners into one blob | Switched to column-projection row scanning |
| 6% PWM duty = 0.55V effective → motor stall | Below BO motor operating voltage; need aggressive point-turns |

---

## Next Steps

- [ ] Integrate 5-channel IR array for corner tracking (hybrid camera+IR)
- [ ] Add MPU6050 IMU for precise 90-degree turn measurement
- [ ] Traffic signal response integration with lane following
- [ ] Obstacle avoidance with ultrasonic sensors during lane following
- [ ] Reverse parking routine
- [ ] Full competition run integration
- [ ] Speed optimization for time bonus

---

## Hardware Wiring Diagram

<p align="center">
  <img src="https://github.com/user-attachments/assets/5e0091bb-3c03-49f2-b916-e93c70af18fe" width="100%" alt="Wiring Diagram" />
</p>

---

## Tech Stack

| Layer | Technology |
|:------|:-----------|
| Compute | Raspberry Pi 5 (8GB) + NVMe HAT+ |
| Vision | OpenCV 4.x, picamera2, libcamera |
| AI | TensorFlow Lite, YOLOv8 |
| Control | RPi.GPIO, PWM motor control |
| Language | Python 3.11 |
| Camera | Pi Camera Module 3 (IMX708) |
| Sensors | 5-ch IR array, 2x HC-SR04 ultrasonic |
| Drive | 4x BO motors, 2x L298N, skid-steer |
| Power | 2x 18650 3S (11.1V), parallel config |
| Chassis | PETG 3D-printed, 3-tier vertical |

---

<p align="center">
  <strong>Project VYRA.ADV</strong> — Robofest Gujarat 5.0<br>
  Built with long nights, lots of chai, and 14-hour coding sessions.<br><br>
  <em>Stage 2 Winner (Rs 1,00,000) | Stage 3 Prototype In Progress</em>
</p>
</content>
</invoke>