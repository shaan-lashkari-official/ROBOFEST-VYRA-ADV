import RPi.GPIO as GPIO
import sys
import termios
import tty
import time
import threading
import os
import datetime
from picamera2 import Picamera2, Preview
from picamera2.utils import Transform
from libcamera import controls

# ================= GPIO CONFIG ⚙️ =================
L_IN1, L_IN2, L_EN = 16, 20, 21 
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

# ================= MOTOR FUNCTIONS 🏎️ =================
def stop():
    GPIO.output([L_IN1, L_IN2, R_IN1, R_IN2], GPIO.LOW)
    l_pwm.ChangeDutyCycle(0)
    r_pwm.ChangeDutyCycle(0)

def forward(speed=60):
    # Fixed polarity based on your feedback! 🔄
    GPIO.output(L_IN1, GPIO.LOW); GPIO.output(L_IN2, GPIO.HIGH)
    GPIO.output(R_IN1, GPIO.LOW); GPIO.output(R_IN2, GPIO.HIGH)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

def backward(speed=60):
    GPIO.output(L_IN1, GPIO.HIGH); GPIO.output(L_IN2, GPIO.LOW)
    GPIO.output(R_IN1, GPIO.HIGH); GPIO.output(R_IN2, GPIO.LOW)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

def rotate_right(speed=40):
    GPIO.output(L_IN1, GPIO.LOW); GPIO.output(L_IN2, GPIO.HIGH)
    GPIO.output(R_IN1, GPIO.HIGH); GPIO.output(R_IN2, GPIO.LOW)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

def rotate_left(speed=40):
    GPIO.output(L_IN1, GPIO.HIGH); GPIO.output(L_IN2, GPIO.LOW)
    GPIO.output(R_IN1, GPIO.LOW); GPIO.output(R_IN2, GPIO.HIGH)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

# ================= CAMERA WORKER 📸 =================
def start_camera():
    global camera_running
    try:
        # 1. Create the folder if it doesn't exist 📁
        folder_name = "static"
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
            print(f"📂 Created folder: {folder_name}")

        picam2 = Picamera2()
        t = Transform(vflip=True, hflip=True)
        # Using 1280x720 for a better balance of FOV and saving speed 🏎️💨
        config = picam2.create_preview_configuration(
            main={'size': (1280, 720)}, 
            transform=t
        )
        picam2.configure(config)
        picam2.start_preview(Preview.QTGL)
        picam2.start()

        picam2.set_controls({
            "AfMode": controls.AfModeEnum.Continuous,
            "AfRange": controls.AfRangeEnum.Macro,
            "Brightness": 0.0,
            "Contrast": 1.4,
            "Saturation": 1.3,
            "Sharpness": 1.5,
            "AwbMode": controls.AwbModeEnum.Indoor
        })
        print("🎬 CINEMATIC FEED ACTIVE & DASHCAM LOGGING")

        last_capture_time = time.time()

        while camera_running:
            current_time = time.time()
            
            # Snap a photo every 2 seconds 🕒
            if current_time - last_capture_time >= 1.0:
                timestamp = datetime.datetime.now().strftime("%H%M%S")
                filename = f"{folder_name}/snap_{timestamp}.jpg"
                
                picam2.capture_file(filename)
                last_capture_time = current_time
            
            time.sleep(0.1)
            
        picam2.stop()
    except Exception as e:
        print(f"❌ Camera Error: {e}")

# ================= KEYBOARD INPUT ⌨️ =================
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# ================= MAIN EXECUTION 🧠 =================
camera_running = True
cam_thread = threading.Thread(target=start_camera, daemon=True)
cam_thread.start()

print("\n🚀 ROBOT READY!")
print("W/A/S/D to move | Q to quit")
print("Capturing to 'green_light' every 2s...")

try:
    while True:
        key = get_key().lower()
        if key == 'w':
            forward()
            print("Moving Forward   ", end='\r')
        elif key == 's':
            backward()
            print("Moving Backward  ", end='\r')
        elif key == 'a':
            rotate_left()
            print("Rotating LEFT    ", end='\r')
        elif key == 'd':
            rotate_right()
            print("Rotating RIGHT   ", end='\r')
        elif key == 'q':
            print("\n🛑 Shutting down...")
            break
        else:
            stop()
finally:
    camera_running = False 
    stop()
    GPIO.cleanup()
    print("Cleanup complete. Goodbye! 👋")
