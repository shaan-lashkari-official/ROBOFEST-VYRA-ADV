import RPi.GPIO as GPIO
import sys
import termios
import tty

# ================= GPIO CONFIG =================
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

# ================= MOTOR FUNCTIONS =================
def stop():
    GPIO.output([L_IN1, L_IN2, R_IN1, R_IN2], GPIO.LOW)
    l_pwm.ChangeDutyCycle(0)
    r_pwm.ChangeDutyCycle(0)

def forward(speed=40):
    GPIO.output(L_IN1, GPIO.HIGH); GPIO.output(L_IN2, GPIO.LOW)
    GPIO.output(R_IN1, GPIO.HIGH); GPIO.output(R_IN2, GPIO.LOW)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

def backward(speed=40):
    GPIO.output(L_IN1, GPIO.LOW); GPIO.output(L_IN2, GPIO.HIGH)
    GPIO.output(R_IN1, GPIO.LOW); GPIO.output(R_IN2, GPIO.HIGH)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

def rotate_right(speed=40):
    """ Spins the car counter-clockwise in place """
    # Left Side: BACKWARD
    GPIO.output(L_IN1, GPIO.LOW); GPIO.output(L_IN2, GPIO.HIGH)
    # Right Side: FORWARD
    GPIO.output(R_IN1, GPIO.HIGH); GPIO.output(R_IN2, GPIO.LOW)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

def rotate_left(speed=40):
    """ Spins the car clockwise in place """
    # Left Side: FORWARD
    GPIO.output(L_IN1, GPIO.HIGH); GPIO.output(L_IN2, GPIO.LOW)
    # Right Side: BACKWARD
    GPIO.output(R_IN1, GPIO.LOW); GPIO.output(R_IN2, GPIO.HIGH)
    l_pwm.ChangeDutyCycle(speed); r_pwm.ChangeDutyCycle(speed)

# ================= KEYBOARD INPUT =================
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

print("ROTATION MODE: Use W/A/S/D to control, Q to quit.")
try:
    while True:
        key = get_key().lower()
        if key == 'w':
            forward()
            print("Moving Forward ", end='\r')
        elif key == 's':
            backward()
            print("Moving Backward", end='\r')
        elif key == 'a':
            rotate_left()
            print("Rotating LEFT  ", end='\r')
        elif key == 'd':
            rotate_right()
            print("Rotating RIGHT ", end='\r')
        elif key == 'q':
            print("\nExiting...")
            break
        else:
            stop()
finally:
    stop()
    GPIO.cleanup()
