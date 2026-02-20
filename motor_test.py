import RPi.GPIO as GPIO
import keyboard
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ===== LEFT DRIVER (L298N #1) =====
FL_IN1, FL_IN2, FL_EN = 17, 27, 18
RL_IN1, RL_IN2, RL_EN = 22, 23, 13

# ===== RIGHT DRIVER (L298N #2) =====
FR_IN1, FR_IN2, FR_EN = 5, 6, 12
RR_IN1, RR_IN2, RR_EN = 16, 20, 21

PINS = [
    FL_IN1, FL_IN2, FL_EN,
    RL_IN1, RL_IN2, RL_EN,
    FR_IN1, FR_IN2, FR_EN,
    RR_IN1, RR_IN2, RR_EN
]

for p in PINS:
    GPIO.setup(p, GPIO.OUT)

# PWM
FL_PWM = GPIO.PWM(FL_EN, 100)
RL_PWM = GPIO.PWM(RL_EN, 100)
FR_PWM = GPIO.PWM(FR_EN, 100)
RR_PWM = GPIO.PWM(RR_EN, 100)

for pwm in [FL_PWM, RL_PWM, FR_PWM, RR_PWM]:
    pwm.start(0)

# ===== LOW LEVEL =====
def motor_forward(in1, in2, pwm, speed):
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)

def motor_backward(in1, in2, pwm, speed):
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)

def motor_stop(in1, in2, pwm):
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)

# ===== MOVEMENTS =====
def stop():
    motor_stop(FL_IN1, FL_IN2, FL_PWM)
    motor_stop(RL_IN1, RL_IN2, RL_PWM)
    motor_stop(FR_IN1, FR_IN2, FR_PWM)
    motor_stop(RR_IN1, RR_IN2, RR_PWM)

def forward(speed=60):
    motor_forward(FL_IN1, FL_IN2, FL_PWM, speed)
    motor_forward(RL_IN1, RL_IN2, RL_PWM, speed)
    motor_forward(FR_IN1, FR_IN2, FR_PWM, speed)
    motor_forward(RR_IN1, RR_IN2, RR_PWM, speed)

def backward(speed=60):
    motor_backward(FL_IN1, FL_IN2, FL_PWM, speed)
    motor_backward(RL_IN1, RL_IN2, RL_PWM, speed)
    motor_backward(FR_IN1, FR_IN2, FR_PWM, speed)
    motor_backward(RR_IN1, RR_IN2, RR_PWM, speed)

def left(speed=60):
    motor_backward(FL_IN1, FL_IN2, FL_PWM, speed)
    motor_backward(RL_IN1, RL_IN2, RL_PWM, speed)
    motor_forward(FR_IN1, FR_IN2, FR_PWM, speed)
    motor_forward(RR_IN1, RR_IN2, RR_PWM, speed)

def right(speed=60):
    motor_forward(FL_IN1, FL_IN2, FL_PWM, speed)
    motor_forward(RL_IN1, RL_IN2, RL_PWM, speed)
    motor_backward(FR_IN1, FR_IN2, FR_PWM, speed)
    motor_backward(RR_IN1, RR_IN2, RR_PWM, speed)

# ===== MAIN LOOP =====
print("WASD control started")
print("W: forward | S: backward | A: left | D: right | SPACE: stop | Q: quit")

try:
    while True:
        if keyboard.is_pressed(1):
            forward()
        elif keyboard.is_pressed(2):
            backward()
        elif keyboard.is_pressed(3):
            left()
        elif keyboard.is_pressed(4):
            right()
        elif keyboard.is_pressed(5):
            stop()
        elif keyboard.is_pressed(6):
            break
        else:
            stop()

        time.sleep(0.05)

except KeyboardInterrupt:
    pass

finally:
    stop()
    for pwm in [FL_PWM, RL_PWM, FR_PWM, RR_PWM]:
        pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up, exit.")
