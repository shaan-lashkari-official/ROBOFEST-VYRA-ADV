#!/usr/bin/env python3
"""
VYRA.ADV - Priority-Fallback Lane Following
Robofest Gujarat 5.0 | Project VYRA.ADV

Architecture:
  Near ROI: steer from CLOSEST row only (350 -> 320 -> 295 fallback).
  Far ROI: curvature preview for speed + turn direction latching.
  Turn latch: when curvature detected before lines vanish, aggressive point-turn.
  3-tier tracking: NEAR (normal) -> TURN (blind corner) -> BLIND (last resort).
  Column-projection line detection (robust at right-angle corners).

NOTE: picamera2 "RGB888" outputs BGR byte order (DRM naming convention).

Run with: venv/bin/python3 lane-test-1.py
"""

import cv2
import numpy as np
from picamera2 import Picamera2
from picamera2.utils import Transform
from libcamera import controls
import RPi.GPIO as GPIO
import time
import traceback

# ======================== FLAGS ========================
SHOW_DEBUG = True   # Set False for competition / headless / SSH

# ======================== MOTOR GPIO ========================
L_IN1, L_IN2, L_EN = 16, 20, 21
R_IN1, R_IN2, R_EN = 5, 6, 12

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in [L_IN1, L_IN2, L_EN, R_IN1, R_IN2, R_EN]:
    GPIO.setup(pin, GPIO.OUT)

l_pwm = GPIO.PWM(L_EN, 1000)
r_pwm = GPIO.PWM(R_EN, 1000)
l_pwm.start(0)
r_pwm.start(0)


def drive(left_speed, right_speed):
    """Drive motors forward. Accepts 0-100."""
    GPIO.output(L_IN1, GPIO.LOW)
    GPIO.output(L_IN2, GPIO.HIGH)
    GPIO.output(R_IN1, GPIO.LOW)
    GPIO.output(R_IN2, GPIO.HIGH)
    l_pwm.ChangeDutyCycle(left_speed)
    r_pwm.ChangeDutyCycle(right_speed)


def stop():
    """Full stop — all pins LOW."""
    GPIO.output(L_IN1, GPIO.LOW)
    GPIO.output(L_IN2, GPIO.LOW)
    GPIO.output(R_IN1, GPIO.LOW)
    GPIO.output(R_IN2, GPIO.LOW)
    l_pwm.ChangeDutyCycle(0)
    r_pwm.ChangeDutyCycle(0)


# ======================== CAMERA ========================
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (640, 360), "format": "RGB888"},
    raw={"size": (2304, 1296)},  # full sensor 2x2 binned — full vertical FOV
    transform=Transform(vflip=True, hflip=True),
    buffer_count=4
)
picam2.configure(config)
picam2.start()
picam2.set_controls({
    "ScalerCrop": (0, 0, 4608, 2592),
    "AfMode": controls.AfModeEnum.Continuous,
    "AfRange": controls.AfRangeEnum.Macro,
    "ExposureValue": 2.5,
    "Brightness": 0.3,
    "Contrast": 3.0,
    "Saturation": 0.0,
    "Sharpness": 1.0,
})
time.sleep(2)

# ======================== GEOMETRY ========================
FRAME_W = 640
FRAME_H = 360
FRAME_CX = FRAME_W // 2  # 320

# ROI boundaries in the full frame
NEAR_TOP = 288   # row 288-360 = bottom 20% (ground ~20-28cm ahead)
NEAR_BOT = 360
FAR_TOP  = 198   # row 198-288 = middle 25% (ground ~28-45cm ahead)
FAR_BOT  = 288

# We threshold one combined region (FAR_TOP to NEAR_BOT) once,
# then scan specific rows within it. Saves a second cvtColor + threshold.
ROI_TOP = FAR_TOP
ROI_BOT = NEAR_BOT

# Priority-fallback scan rows (closest first).
# Use the FIRST row that detects lines — no weighted averaging.
# This prevents distant rows from pulling steering toward future turns.
SCAN_ROWS = [350, 320, 295]  # row 350 (~20cm), 320 (~24cm), 295 (~27cm)

# ======================== CONTROL ========================
KP = 0.15           # correction per pixel of lateral error (high for sharp turns)
KD = 0.10           # correction per pixel-per-frame temporal derivative

# Speed
BASE_SPEED = 50
MIN_SPEED  = 20     # very slow for right-angle turns
MAX_SPEED  = 60
CURVE_BRAKE = 0.35  # aggressive brake per pixel of curvature preview
TURN_SPEED = 32     # point-turn speed for blind 90-degree turns
CURVE_LATCH_THRESH = 60  # curvature px to latch a turn direction

# Detection
BLACK_THRESH = 70
BLUR_KERNEL = (5, 5)
MIN_CONTOUR_AREA = 80   # smaller strips need smaller minimum
MORPH_KERNEL = np.ones((3, 3), np.uint8)
LANE_WIDTH_MIN = 80
LANE_WIDTH_MAX = 550

# Recovery
LOST_DECAY = 0.85
MAX_LOST = 45    # ~1.5s at 30fps — enough to complete a blind 90-degree turn

# ======================== STATE ========================
prev_weighted_error = None
lane_width_history = []
calibrated_half_lane = None
frames_lost = 0
straight_frames = 0
latched_turn_dir = 0   # -1 = left turn pending, 0 = none, +1 = right turn pending


# ======================== DETECTION ========================
def threshold_roi(gray_roi):
    """Blur + fixed threshold -> clean binary mask of black lines."""
    blurred = cv2.GaussianBlur(gray_roi, BLUR_KERNEL, 0)
    _, binary = cv2.threshold(blurred, BLACK_THRESH, 255, cv2.THRESH_BINARY_INV)
    return cv2.morphologyEx(binary, cv2.MORPH_OPEN, MORPH_KERNEL)


def find_center_at_row(binary_roi, row_in_roi, strip_h=5):
    """
    Find lane center by scanning white-pixel RUNS at a specific row.
    Column-projection approach — at corners, contours merge into one
    blob but each horizontal row still crosses the two lines separately.
    Returns (center_x, n_lines) or (None, 0).
    """
    global calibrated_half_lane

    top = max(0, row_in_roi - strip_h // 2)
    bot = min(binary_roi.shape[0], row_in_roi + strip_h // 2 + 1)
    strip = binary_roi[top:bot, :]

    if strip.size == 0:
        return None, 0

    # Column sum: count white pixels per column across the strip
    col_sum = np.sum(strip, axis=0, dtype=np.int32) // 255

    # A column is "active" if at least 2 of the strip rows are white
    active = (col_sum >= 2).astype(np.uint8)

    # Find contiguous runs of active columns (each run = one line crossing)
    padded = np.concatenate([[0], active, [0]])
    diffs = np.diff(padded.astype(np.int8))
    starts = np.where(diffs == 1)[0]
    ends = np.where(diffs == -1)[0]

    # Filter: minimum 5px wide (ignore noise)
    runs = []
    for s, e in zip(starts, ends):
        if e - s >= 5:
            runs.append(((s + e) // 2, e - s))

    if len(runs) >= 2:
        # Two widest runs = the two lane lines
        runs.sort(key=lambda r: r[1], reverse=True)
        cx1, cx2 = runs[0][0], runs[1][0]
        center = (cx1 + cx2) // 2
        width = abs(cx1 - cx2)
        if LANE_WIDTH_MIN < width < LANE_WIDTH_MAX:
            lane_width_history.append(width)
            if len(lane_width_history) > 50:
                lane_width_history.pop(0)
            calibrated_half_lane = int(np.median(lane_width_history)) // 2
        return center, 2

    if len(runs) == 1:
        cx = runs[0][0]
        half = calibrated_half_lane if calibrated_half_lane else 100
        if cx < FRAME_CX:
            return cx + half, 1
        else:
            return cx - half, 1

    return None, 0


def find_far_center(binary_far):
    """Find lane center in the far ROI (for curvature + turn detection)."""
    contours, _ = cv2.findContours(binary_far, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if cv2.contourArea(c) > MIN_CONTOUR_AREA]

    if len(valid) >= 2:
        valid = sorted(valid, key=cv2.contourArea, reverse=True)[:2]
        cxs = []
        for c in valid:
            M = cv2.moments(c)
            if M["m00"] > 0:
                cxs.append(int(M["m10"] / M["m00"]))
        if len(cxs) == 2:
            return (cxs[0] + cxs[1]) // 2

    if len(valid) >= 1:
        biggest = max(valid, key=cv2.contourArea)
        M = cv2.moments(biggest)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            half = calibrated_half_lane if calibrated_half_lane else 100
            return (cx + half) if cx < FRAME_CX else (cx - half)

    return None


# ======================== MAIN LOOP ========================
print("VYRA.ADV Priority-Fallback Lane Following")
print(f"  KP={KP}  KD={KD}  CURVE_BRAKE={CURVE_BRAKE}")
print(f"  BASE_SPEED={BASE_SPEED}  MIN={MIN_SPEED}  MAX={MAX_SPEED}")
print(f"  Scan rows (priority): {SCAN_ROWS}")
print(f"  Debug: {'ON' if SHOW_DEBUG else 'OFF'}")
print("  Ctrl+C to stop | 'q' in window\n")

# ======================== VIDEO RECORDER ========================
REC_PATH = "/home/shaan/Desktop/ROBOFEST/Documents/vyra_run.avi"
fourcc = cv2.VideoWriter_fourcc(*"MJPG")
recorder = cv2.VideoWriter(REC_PATH, fourcc, 25.0, (FRAME_W, FRAME_H))
print(f"  Recording to: {REC_PATH}")

try:
    while True:
        t0 = time.monotonic()
        frame = picam2.capture_array()

        # --- Single grayscale + threshold for entire ROI region ---
        roi_gray = cv2.cvtColor(frame[ROI_TOP:ROI_BOT, :], cv2.COLOR_BGR2GRAY)
        roi_bin = threshold_roi(roi_gray)

        # --- FAR ROI (computed first — used for curvature AND turn detection) ---
        far_bin = roi_bin[0:(FAR_BOT - ROI_TOP), :]
        far_cx = find_far_center(far_bin)

        # --- NEAR: priority-fallback scan (closest row wins) ---
        near_error = None
        scan_src_row = None
        scan_centers = {}

        for row_in_frame in SCAN_ROWS:
            row_in_roi = row_in_frame - ROI_TOP
            center, n = find_center_at_row(roi_bin, row_in_roi)
            if center is not None:
                center = max(0, min(FRAME_W, center))
                scan_centers[row_in_frame] = center
                if near_error is None:
                    near_error = center - FRAME_CX
                    scan_src_row = row_in_frame

        # --- CURVATURE PREVIEW ---
        near_cx = scan_centers.get(SCAN_ROWS[0], FRAME_CX)
        if far_cx is not None:
            curvature_preview = abs(far_cx - near_cx)
        else:
            curvature_preview = 50  # unknown = cautious

        # --- LATCH TURN DIRECTION while near is still tracking ---
        if near_error is not None:
            if curvature_preview > CURVE_LATCH_THRESH and far_cx is not None:
                latched_turn_dir = 1 if (far_cx > near_cx) else -1
            elif curvature_preview < 30:
                latched_turn_dir = 0  # back on straight, reset

        # --- STEERING SOURCE: near > TURN latch > blind continuation ---
        if near_error is not None:
            weighted_error = near_error
            frames_lost = 0
            track_src = f"R{scan_src_row}"
        elif latched_turn_dir != 0:
            # Sharp turn was detected BEFORE lines vanished — aggressive blind turn
            frames_lost += 1
            weighted_error = latched_turn_dir * 120  # placeholder for telemetry
            track_src = "TURN"
        else:
            # No turn latched — keep last direction with decay
            frames_lost += 1
            if prev_weighted_error is not None:
                weighted_error = prev_weighted_error * LOST_DECAY
            else:
                weighted_error = 0.0
            track_src = "BLIND"

        # --- TEMPORAL DERIVATIVE ---
        if prev_weighted_error is not None:
            temporal_d = weighted_error - prev_weighted_error
        else:
            temporal_d = 0.0

        # --- PD CORRECTION (used only in normal tracking) ---
        correction = KP * weighted_error + KD * temporal_d
        prev_weighted_error = weighted_error

        # --- SPEED MODULATION ---
        if curvature_preview < 20:
            straight_frames += 1
            boost = min(straight_frames * 0.5, MAX_SPEED - BASE_SPEED)
            target_speed = BASE_SPEED + boost
        else:
            straight_frames = 0
            target_speed = max(MIN_SPEED, BASE_SPEED - CURVE_BRAKE * curvature_preview)

        target_speed -= 0.15 * abs(weighted_error)
        target_speed = max(MIN_SPEED, min(MAX_SPEED, target_speed))

        # --- MOTOR OUTPUT ---
        if frames_lost >= MAX_LOST:
            left_spd = 0.0
            right_spd = 0.0
            stop()
        elif track_src == "TURN":
            # Aggressive point turn — one wheel spins, other stops
            if latched_turn_dir > 0:
                left_spd = TURN_SPEED
                right_spd = 0.0
            else:
                left_spd = 0.0
                right_spd = TURN_SPEED
            drive(left_spd, right_spd)
        else:
            # Normal PD driving (tracking or mild blind)
            if frames_lost > 0:
                target_speed = MIN_SPEED
            left_spd = max(0.0, min(100.0, target_speed + correction))
            right_spd = max(0.0, min(100.0, target_speed - correction))
            drive(left_spd, right_spd)

        # --- DEBUG OVERLAY + RECORD ---
        for i, row_f in enumerate(SCAN_ROWS):
            g = 255 if i == 0 else 120
            cv2.line(frame, (0, row_f), (FRAME_W, row_f), (0, g, 0), 1)
            if row_f in scan_centers:
                col = (0, 0, 255) if row_f == scan_src_row else (0, 140, 255)
                cv2.circle(frame, (scan_centers[row_f], row_f), 5, col, -1)
        cv2.line(frame, (0, FAR_TOP), (FRAME_W, FAR_TOP), (0, 255, 255), 1)
        cv2.line(frame, (0, FAR_BOT), (FRAME_W, FAR_BOT), (0, 255, 255), 1)
        cv2.line(frame, (FRAME_CX, FAR_TOP), (FRAME_CX, NEAR_BOT), (255, 0, 0), 1)
        if far_cx is not None:
            cv2.circle(frame, (far_cx, (FAR_TOP + FAR_BOT) // 2), 5, (0, 255, 255), -1)

        # Text HUD on frame
        hud = f"[{track_src}] err:{weighted_error:+.0f} spd:{target_speed:.0f} L:{left_spd:.0f} R:{right_spd:.0f}"
        cv2.putText(frame, hud, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Write to video file
        recorder.write(frame)

        if SHOW_DEBUG:
            cv2.imshow("VYRA Lane", frame)
            near_bin_display = roi_bin[(NEAR_TOP - ROI_TOP):, :]
            cv2.imshow("Near Binary", near_bin_display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # --- TELEMETRY ---
        dt_ms = (time.monotonic() - t0) * 1000
        fps = 1000 / dt_ms if dt_ms > 0 else 0
        lw = calibrated_half_lane * 2 if calibrated_half_lane else "---"
        print(f"[{track_src:7s}] err:{weighted_error:+6.1f} "
              f"crv:{curvature_preview:3.0f} spd:{target_speed:4.1f} "
              f"L:{left_spd:5.1f} R:{right_spd:5.1f} lw:{lw} {fps:4.0f}fps   ",
              end="\r")

except KeyboardInterrupt:
    print("\nShutdown requested.")
except Exception as e:
    print(f"\nError: {e}")
    traceback.print_exc()
finally:
    try:
        l_pwm.ChangeDutyCycle(0)
        r_pwm.ChangeDutyCycle(0)
        l_pwm.stop()
        r_pwm.stop()
        for pin in [L_IN1, L_IN2, L_EN, R_IN1, R_IN2, R_EN]:
            GPIO.output(pin, GPIO.LOW)
    except Exception:
        pass
    time.sleep(0.2)
    recorder.release()
    print(f"  Video saved: {REC_PATH}")
    if SHOW_DEBUG:
        cv2.destroyAllWindows()
    picam2.stop()
    GPIO.cleanup()
    print("Cleanup complete.")
