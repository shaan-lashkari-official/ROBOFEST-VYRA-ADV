import RPi.GPIO as GPIO
import time
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

def forward(speed=35):
    GPIO.output(L_IN1, GPIO.LOW); GPIO.output(L_IN2, GPIO.HIGH)
    GPIO.output(R_IN1, GPIO.LOW); GPIO.output(R_IN2, GPIO.HIGH)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

def rotate_right(speed=40):
    # Left Side: BACKWARD, Right Side: FORWARD
    GPIO.output(L_IN1, GPIO.LOW); GPIO.output(L_IN2, GPIO.HIGH)
    GPIO.output(R_IN1, GPIO.HIGH); GPIO.output(R_IN2, GPIO.LOW)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

# ================= CAMERA SETUP 📸 =================
NEAR_THRESHOLD = 5.0  # Stop if position >= 5.0
CLEAR_THRESHOLD = 4.0 # Resume if position < 4.0

picam2 = Picamera2()
t = Transform(vflip=True, hflip=True)
config = picam2.create_preview_configuration(main={'size': (640, 480)}, transform=t)
picam2.configure(config)
picam2.start()

picam2.set_controls({
    "AfMode": controls.AfModeEnum.Continuous,
    "AfRange": controls.AfRangeEnum.Normal,
})

# ================= MAIN LOGIC 🧠 =================
print("🚀 AUTOPILOT STARTING...")
try:
    while True:
        metadata = picam2.capture_metadata()
        lens_pos = metadata.get("LensPosition")

        if lens_pos is not None:
            print(f"🔍 Lens: {lens_pos:.2f}", end="\r")

            if lens_pos >= NEAR_THRESHOLD:
                print("\n🚨 OBSTACLE! Stopping and Turning...")
                stop()
                time.sleep(0.1)
                
                # Rotate until clear
                rotate_right()
                time.sleep(0.2)
                #while True:
                #    m = picam2.capture_metadata()
                #    pos = m.get("LensPosition")
                 #   if pos is not None and pos < CLEAR_THRESHOLD:
                #        print(f"\n✅ Path Clear ({pos:.2f})! Resuming...")
                 #       stop()
                 #       time.sleep(0.5)
                 #       break
                 #   time.sleep(0.1)
            
            else:
                # No obstacle? Keep rolling!
                forward()

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n👋 Stopping Robot...")
finally:
    stop()
    picam2.stop()
    GPIO.cleanup()
