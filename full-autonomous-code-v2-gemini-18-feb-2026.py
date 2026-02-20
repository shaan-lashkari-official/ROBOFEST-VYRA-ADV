import numpy as np
from picamera2 import Picamera2, Preview
from picamera2.utils import Transform
from libcamera import controls
import RPi.GPIO as GPIO
import time

# ================= 1. MOTOR CONFIG (2-Channel Logic) =================
# Left Side
L_IN1, L_IN2, L_EN = 16, 20, 21 
# Right Side
R_IN1, R_IN2, R_EN = 5, 6, 12

ALL_PINS = [L_IN1, L_IN2, L_EN, R_IN1, R_IN2, R_EN]

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in ALL_PINS:
    GPIO.setup(pin, GPIO.OUT)

l_pwm = GPIO.PWM(L_EN, 100)
r_pwm = GPIO.PWM(R_EN, 100)
l_pwm.start(0)
r_pwm.start(0)

def drive(left_speed, right_speed):
    # Left Side: Forward (positive) or Backward (negative)
    GPIO.output(L_IN1, GPIO.HIGH if left_speed >= 0 else GPIO.LOW)
    GPIO.output(L_IN2, GPIO.LOW if left_speed >= 0 else GPIO.HIGH)
    # Right Side: Forward (positive) or Backward (negative)
    GPIO.output(R_IN1, GPIO.HIGH if right_speed >= 0 else GPIO.LOW)
    GPIO.output(R_IN2, GPIO.LOW if right_speed >= 0 else GPIO.HIGH)
    
    l_pwm.ChangeDutyCycle(abs(np.clip(left_speed, 0, 100)))
    r_pwm.ChangeDutyCycle(abs(np.clip(right_speed, 0, 100)))

# ================= 2. CAMERA & VISION CONFIG =================
picam2 = Picamera2()
t = Transform(vflip=True, hflip=True)
config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 360)}, transform=t)
picam2.configure(config)
picam2.start_preview(Preview.QTGL)
picam2.start()

# Cinematic Filters to make lane lines pop
picam2.set_controls({
    "ScalerCrop": (0, 0, 4608, 2592), # IMX708 Max Wide-Angle
    "Contrast": 1.5,
    "Saturation": 1.4,
    "Sharpness": 2.0
})

def get_lane_error(frame_xrgb):
    # ROI: Look at a thin strip at the bottom of the view
    roi = frame_xrgb[300:360, :, :3]
    gray_roi = np.mean(roi, axis=2)
    
    # Thresholding for black lane lines
    line_mask = (gray_roi < 70).astype(np.float32)
    pixel_sums = np.sum(line_mask, axis=0)
    
    left_half = pixel_sums[:320]
    right_half = pixel_sums[320:]
    
    # Check if lines exist in both halves
    if np.max(left_half) > 5 and np.max(right_half) > 5:
        l_line = np.argmax(left_half)
        r_line = np.argmax(right_half) + 320
        lane_center = (l_line + r_line) / 2
        return lane_center - 320 # Error from center (0 = perfectly centered)
    return None

# ================= 3. AUTONOMOUS LOOP =================
BASE_SPEED = 50 # Start slow due to 11.1V battery power
KP = 0.5        # Turn sensitivity (Proportional Gain)

print("🚀 AUTONOMOUS LANE FOLLOWING ACTIVE")
try:
    while True:
        frame = picam2.capture_array()
        error = get_lane_error(frame)
        
        if error is not None:
            # P-Control: Speed up one side, slow down the other
            correction = error * KP
            l_speed = BASE_SPEED + correction
            r_speed = BASE_SPEED - correction
            
            drive(l_speed, r_speed)
            print(f"Status: Tracking | Error: {error:3.0f} | Speeds: L={l_speed:.0f} R={r_speed:.0f}   ", end="\r")
        else:
            drive(0, 0) # Safety stop if lines are lost
            print("Status: NO LANES DETECTED - STOPPING          ", end="\r")

except KeyboardInterrupt:
    print("\n👋 System Shutdown.")
finally:
    drive(0, 0)
    picam2.stop()
    GPIO.cleanup()
