import numpy as np
import RPi.GPIO as GPIO
import time
from ai_edge_litert.interpreter import Interpreter
from picamera2 import Picamera2
from picamera2.utils import Transform

# --- 1. HARDWARE CONFIGURATION --- ⚙️
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin Definitions (From your Keyboard Script)
FL_IN1, FL_IN2, FL_EN = 17, 27, 13
RL_IN1, RL_IN2, RL_EN = 22, 23, 18
FR_IN1, FR_IN2, FR_EN = 5, 6, 12
RR_IN1, RR_IN2, RR_EN = 16, 20, 21
IR_PINS = [24, 25, 8, 7, 1]

ALL_PINS = [FL_IN1, FL_IN2, FL_EN, RL_IN1, RL_IN2, RL_EN,
            FR_IN1, FR_IN2, FR_EN, RR_IN1, RR_IN2, RR_EN]

for pin in ALL_PINS:
    GPIO.setup(pin, GPIO.OUT)
for pin in IR_PINS:
    GPIO.setup(pin, GPIO.IN)

# PWM Setup
fl_pwm = GPIO.PWM(FL_EN, 100)
rl_pwm = GPIO.PWM(RL_EN, 100)
fr_pwm = GPIO.PWM(FR_EN, 100)
rr_pwm = GPIO.PWM(RR_EN, 100)
for pwm in [fl_pwm, rl_pwm, fr_pwm, rr_pwm]:
    pwm.start(0)

# Speed Constants
BASE_SPEED = 60
TURN_SPEED = 50

# --- 2. MOTOR FUNCTIONS (Using your preferred logic) --- 🕹️
def stop():
    for in1, in2, pwm in [(FL_IN1, FL_IN2, fl_pwm), (RL_IN1, RL_IN2, rl_pwm),
                          (FR_IN1, FR_IN2, fr_pwm), (RR_IN1, RR_IN2, rr_pwm)]:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(0)

def forward(speed=BASE_SPEED): #high
    GPIO.output(FL_IN1, GPIO.HIGH); GPIO.output(FL_IN2, GPIO.LOW); fl_pwm.ChangeDutyCycle(speed)
    GPIO.output(RL_IN1, GPIO.HIGH); GPIO.output(RL_IN2, GPIO.LOW); rl_pwm.ChangeDutyCycle(speed)
    GPIO.output(FR_IN1, GPIO.HIGH); GPIO.output(FR_IN2, GPIO.LOW); fr_pwm.ChangeDutyCycle(speed)
    GPIO.output(RR_IN1, GPIO.HIGH); GPIO.output(RR_IN2, GPIO.LOW); rr_pwm.ChangeDutyCycle(speed)

def left_turn(speed=TURN_SPEED):#low
    # Left motors backward, Right motors forward
    GPIO.output(FL_IN1, GPIO.HIGH); GPIO.output(FL_IN2, GPIO.LOW); fl_pwm.ChangeDutyCycle(speed)
    GPIO.output(RL_IN1, GPIO.HIGH); GPIO.output(RL_IN2, GPIO.LOW); rl_pwm.ChangeDutyCycle(speed)
    GPIO.output(FR_IN1, GPIO.LOW); GPIO.output(FR_IN2, GPIO.HIGH); fr_pwm.ChangeDutyCycle(speed)
    GPIO.output(RR_IN1, GPIO.LOW); GPIO.output(RR_IN2, GPIO.HIGH); rr_pwm.ChangeDutyCycle(speed)

def right_turn(speed=TURN_SPEED):#high
    # Left motors forward, Right motors backward
    GPIO.output(FL_IN1, GPIO.LOW); GPIO.output(FL_IN2, GPIO.HIGH); fl_pwm.ChangeDutyCycle(speed)
    GPIO.output(RL_IN1, GPIO.LOW); GPIO.output(RL_IN2, GPIO.HIGH); rl_pwm.ChangeDutyCycle(speed)
    GPIO.output(FR_IN1, GPIO.HIGH); GPIO.output(FR_IN2, GPIO.LOW); fr_pwm.ChangeDutyCycle(speed)
    GPIO.output(RR_IN1, GPIO.HIGH); GPIO.output(RR_IN2, GPIO.LOW); rr_pwm.ChangeDutyCycle(speed)

# --- 3. VISION CONFIGURATION --- 🧠
model_path = "model_unquant.tflite"
interpreter = Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
h, w = input_details[0]['shape'][1], input_details[0]['shape'][2]
input_dtype = input_details[0]['dtype']

picam2 = Picamera2()
t = Transform(vflip=True, hflip=True)
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (w, h)}, transform=t)
picam2.configure(config)
picam2.start()

# --- 4. MAIN CONTROL LOOP --- 🚀
print("AI Integrated System Active. Flipped Camera + IR Line Following.")

try:
    while True:
        # A. Vision Processing (Traffic Light)
        frame = picam2.capture_array()
        input_data = np.expand_dims(frame, axis=0)
        if input_dtype == np.float32:
            input_data = input_data.astype(np.float32) / 255.0
        else:
            input_data = input_data.astype(input_dtype)

        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        output_data = interpreter.get_tensor(interpreter.get_output_details()[0]['index'])[0]
        
        top_idx = np.argmax(output_data)
        confidence = output_data[top_idx] if input_dtype == np.float32 else output_data[top_idx]/255.0

        # B. Decision Priority
        # 🚦 PRIORITY 1: Red Light Detection (Assuming Index 0 is Red)
        if top_idx == 0 and confidence > 0.8:
            print(f"🚦 RED LIGHT ({confidence*100:.1f}%) - STOP")
            stop()
            time.sleep(0.1)
            continue

        # 🛣️ PRIORITY 2: IR Line Following
        ir = [GPIO.input(pin) for pin in IR_PINS]
        
        # ir[2] is Center, ir[0,1] is Left, ir[3,4] is Right
        if ir[2] == 0:
            forward()
        elif ir[0] == 0 or ir[1] == 0:
            left_turn()
        elif ir[3] == 0 or ir[4] == 0:
            right_turn()
        else:
            stop()

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    stop()
    picam2.stop()
    GPIO.cleanup()