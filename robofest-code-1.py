import numpy as np
import RPi.GPIO as GPIO
import time
from ai_edge_litert.interpreter import Interpreter
from picamera2 import Picamera2, Preview
from picamera2.utils import Transform

# --- 1. HARDWARE CONFIGURATION --- ⚙️
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

L_IN1, L_IN2, L_EN = 16, 20, 21
R_IN1, R_IN2, R_EN = 5, 6, 12

IR_PINS = [24, 25, 8, 7, 1] 
TRIG_FRONT, ECHO_FRONT = 2, 3

ALL_MOTORS = [L_IN1, L_IN2, L_EN, R_IN1, R_IN2, R_EN]
for pin in ALL_MOTORS: GPIO.setup(pin, GPIO.OUT)
for pin in IR_PINS: GPIO.setup(pin, GPIO.IN)
GPIO.setup(TRIG_FRONT, GPIO.OUT); GPIO.setup(ECHO_FRONT, GPIO.IN)

l_pwm = GPIO.PWM(L_EN, 100); r_pwm = GPIO.PWM(R_EN, 100)
l_pwm.start(0); r_pwm.start(0)

# Settings
BASE_SPEED = 50     
BOUNCE_SPEED = 80   # Increased for faster "bounce" away from lines
STOP_DISTANCE = 10.0 
LABELS = ["Red", "Green", "Yellow", "None"]

# --- 2. CORE FUNCTIONS --- 🕹️

def get_distance():
    GPIO.output(TRIG_FRONT, GPIO.LOW)
    time.sleep(0.002)
    GPIO.output(TRIG_FRONT, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_FRONT, GPIO.LOW)
    ps, pe = time.time(), time.time()
    to = time.time()
    while GPIO.input(ECHO_FRONT) == 0:
        ps = time.time()
        if ps - to > 0.05: return 999 
    while GPIO.input(ECHO_FRONT) == 1: pe = time.time()
    return round(((pe - ps) * 34300) / 2, 1)

def stop():
    GPIO.output([L_IN1, L_IN2, R_IN1, R_IN2], GPIO.LOW)
    l_pwm.ChangeDutyCycle(0); r_pwm.ChangeDutyCycle(0)

def forward(speed=BASE_SPEED):
    GPIO.output(L_IN1, GPIO.HIGH); GPIO.output(L_IN2, GPIO.LOW)
    GPIO.output(R_IN1, GPIO.HIGH); GPIO.output(R_IN2, GPIO.LOW)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

def rotate_right(speed=BOUNCE_SPEED):
    # Left BACKWARD, Right FORWARD
    GPIO.output(L_IN1, GPIO.LOW); GPIO.output(L_IN2, GPIO.HIGH)
    GPIO.output(R_IN1, GPIO.HIGH); GPIO.output(R_IN2, GPIO.LOW)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

def rotate_left(speed=BOUNCE_SPEED):
    # Left FORWARD, Right BACKWARD
    GPIO.output(L_IN1, GPIO.HIGH); GPIO.output(L_IN2, GPIO.LOW)
    GPIO.output(R_IN1, GPIO.LOW); GPIO.output(R_IN2, GPIO.HIGH)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

# --- 3. VISION --- 🧠
interpreter = Interpreter(model_path="model_unquant.tflite")
interpreter.allocate_tensors()
in_idx = interpreter.get_input_details()[0]['index']
h, w = interpreter.get_input_details()[0]['shape'][1], interpreter.get_input_details()[0]['shape'][2]

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (w, h)}, transform=Transform(vflip=True, hflip=True))
picam2.configure(config)
picam2.start_preview(Preview.NULL)
picam2.start()

# --- 4. MAIN LOOP --- 🚀
print("\n[LANE KEEPING] Goal: Stay BETWEEN two black lines.")
current_dir = "STOP"

try:
    while True:
        dist = get_distance()
        ir = [GPIO.input(p) for p in IR_PINS]
        
        # AI Detection
        frame = picam2.capture_array()
        input_data = np.expand_dims(frame, axis=0).astype(np.float32) / 255.0
        interpreter.set_tensor(in_idx, input_data)
        interpreter.invoke()
        output = interpreter.get_tensor(interpreter.get_output_details()[0]['index'])[0]
        top_idx = np.argmax(output)
        light, conf = LABELS[top_idx], output[top_idx]

        # --- DECISION LOGIC ---
        
        if dist < STOP_DISTANCE or (light == "Red" and conf > 0.85):
            stop()
            current_dir = "STOP"
        
        # 🚦 LANE KEEPING LOGIC (Bouncing away from black lines)
        elif ir[0] == 0:  # If Left sensor hits the black line
            rotate_right() # Pivot RIGHT away from the line
            current_dir = "BOUNCE RIGHT"
            time.sleep(0.12) # Increased nudge to ensure it leaves the line
            
        elif ir[4] == 0:  # If Right sensor hits the black line
            rotate_left()  # Pivot LEFT away from the line
            current_dir = "BOUNCE LEFT"
            time.sleep(0.12) # Increased nudge to ensure it leaves the line
            
        else:             # Both sensors on White surface
            forward()
            current_dir = "FORWARD"

        print(f"DIST: {dist}cm | LIGHT: {light} ({conf:.2f}) | IR: {' '.join(map(str, ir))} | DIR: {current_dir}    ", end='\r')
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n\nShutting down safely...")
finally:
    stop(); picam2.stop(); GPIO.cleanup()