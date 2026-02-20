import numpy as np
import RPi.GPIO as GPIO
import time
from ai_edge_litert.interpreter import Interpreter
from picamera2 import Picamera2, Preview
from picamera2.utils import Transform

# --- 1. HARDWARE CONFIGURATION --- ⚙️
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor Pins (From your setup)
FL_IN1, FL_IN2, FL_EN = 17, 27, 13
RL_IN1, RL_IN2, RL_EN = 22, 23, 18
FR_IN1, FR_IN2, FR_EN = 5, 6, 12
RR_IN1, RR_IN2, RR_EN = 16, 20, 21

# Sensor Pins
IR_PINS = [24, 25, 8, 7, 1]  # [Left, L2, Center, R2, Right]
TRIG_FRONT = 2 
ECHO_FRONT = 3

# Setup GPIO
ALL_MOTORS = [FL_IN1, FL_IN2, FL_EN, RL_IN1, RL_IN2, RL_EN,
              FR_IN1, FR_IN2, FR_EN, RR_IN1, RR_IN2, RR_EN]

for pin in ALL_MOTORS: GPIO.setup(pin, GPIO.OUT)
for pin in IR_PINS: GPIO.setup(pin, GPIO.IN)
GPIO.setup(TRIG_FRONT, GPIO.OUT)
GPIO.setup(ECHO_FRONT, GPIO.IN)

# PWM Setup
fl_pwm = GPIO.PWM(FL_EN, 100); rl_pwm = GPIO.PWM(RL_EN, 100)
fr_pwm = GPIO.PWM(FR_EN, 100); rr_pwm = GPIO.PWM(RR_EN, 100)
for p in [fl_pwm, rl_pwm, fr_pwm, rr_pwm]: p.start(0)

# Settings
BASE_SPEED = 60
TURN_SPEED = 50
STOP_DISTANCE = 20.0 
LABELS = ["Red", "Green", "Yellow", "None"] # Adjust based on your labels.txt

# --- 2. CORE FUNCTIONS --- 🕹️

def get_distance():
    """Calculates distance from front Ultrasonic sensor."""
    GPIO.output(TRIG_FRONT, GPIO.LOW)
    time.sleep(0.002)
    GPIO.output(TRIG_FRONT, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_FRONT, GPIO.LOW)
    
    ps, pe = time.time(), time.time()
    timeout = time.time()
    
    while GPIO.input(ECHO_FRONT) == 0:
        ps = time.time()
        if ps - timeout > 0.05: return 999 
    while GPIO.input(ECHO_FRONT) == 1:
        pe = time.time()
        
    return round(((pe - ps) * 34300) / 2, 1)

def stop():
    GPIO.output([FL_IN1, FL_IN2, RL_IN1, RL_IN2, FR_IN1, FR_IN2, RR_IN1, RR_IN2], GPIO.LOW)
    for p in [fl_pwm, rl_pwm, fr_pwm, rr_pwm]: p.ChangeDutyCycle(0)

def forward(speed=BASE_SPEED):
    # Left Forward, Right Forward
    GPIO.output([FL_IN1, RL_IN1, FR_IN1, RR_IN1], GPIO.HIGH)
    GPIO.output([FL_IN2, RL_IN2, FR_IN2, RR_IN2], GPIO.LOW)
    for p in [fl_pwm, rl_pwm, fr_pwm, rr_pwm]: p.ChangeDutyCycle(speed)

def left_turn(speed=TURN_SPEED):
    # Tank Turn: Left Forward, Right Backward
    GPIO.output([FL_IN1, RL_IN1, FR_IN2, RR_IN2], GPIO.HIGH)
    GPIO.output([FL_IN2, RL_IN2, FR_IN1, RR_IN1], GPIO.LOW)
    for p in [fl_pwm, rl_pwm, fr_pwm, rr_pwm]: p.ChangeDutyCycle(speed)

def right_turn(speed=TURN_SPEED):
    # Tank Turn: Left Backward, Right Forward
    GPIO.output([FL_IN2, RL_IN2, FR_IN1, RR_IN1], GPIO.HIGH)
    GPIO.output([FL_IN1, RL_IN1, FR_IN2, RR_IN2], GPIO.LOW)
    for p in [fl_pwm, rl_pwm, fr_pwm, rr_pwm]: p.ChangeDutyCycle(speed)

# --- 3. VISION CONFIGURATION --- 🧠
interpreter = Interpreter(model_path="model_unquant.tflite")
interpreter.allocate_tensors()
in_idx = interpreter.get_input_details()[0]['index']
h, w = interpreter.get_input_details()[0]['shape'][1], interpreter.get_input_details()[0]['shape'][2]

picam2 = Picamera2()
t = Transform(vflip=True, hflip=True) # Flipped for upside-down mounting

# Headless Configuration
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (w, h)}, transform=t)
picam2.configure(config)
picam2.start_preview(Preview.NULL) # No desktop window
picam2.start()

# --- 4. MAIN INTEGRATED LOOP --- 🚀
print("\n[SYSTEM LIVE] Headless Mode. Monitoring Camera, IR, and Distance...")

try:
    while True:
        # A. SENSOR DATA GATHERING
        dist = get_distance()
        ir_vals = [GPIO.input(p) for p in IR_PINS]
        ir_str = " ".join(map(str, ir_vals))

        # B. VISION PROCESSING
        frame = picam2.capture_array()
        input_data = np.expand_dims(frame, axis=0).astype(np.float32) / 255.0
        interpreter.set_tensor(in_idx, input_data)
        interpreter.invoke()
        output = interpreter.get_tensor(interpreter.get_output_details()[0]['index'])[0]
        
        top_idx = np.argmax(output)
        light, conf = LABELS[top_idx], output[top_idx]

        # C. PRINT STATUS BAR (Real-time telemetry)
        print(f"DIST: {dist}cm | LIGHT: {light} ({conf:.2f}) | IR: [{ir_str}]", end='\r')

        # D. DECISION LOGIC
        # 🛡️ PRIORITY 0: Obstacle Detection
        if dist < STOP_DISTANCE:
            stop()
        
        # 🚦 PRIORITY 1: Red Light Detection
        elif light == "Red" and conf > 0.85:
            stop()
            
        # 🛣️ PRIORITY 2: Line Following
        else:
            if ir_vals[2] == 0:                 # Center on line
                forward()
            elif ir_vals[0] == 0 or ir_vals[1] == 0:  # Deviating Right (Line is Left)
                left_turn()
            elif ir_vals[3] == 0 or ir_vals[4] == 0:  # Deviating Left (Line is Right)
                right_turn()
            else:                               # Off-track
                stop()

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n\nUser Stopped System. Shutting down safely...")
finally:
    stop()
    picam2.stop()
    GPIO.cleanup()