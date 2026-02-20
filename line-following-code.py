import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ===== LEFT DRIVER =====
FL_IN1, FL_IN2, FL_EN = 17, 27, 18
RL_IN1, RL_IN2, RL_EN = 22, 23, 13

# ===== RIGHT DRIVER =====
FR_IN1, FR_IN2, FR_EN = 5, 6, 12
RR_IN1, RR_IN2, RR_EN = 16, 20, 21

# ===== IR SENSORS =====
IR_PINS = [24, 25, 8, 7, 1]  
# [Left, L2, Center, R2, Right]

# ===== SETUP =====
motor_pins = [
    FL_IN1, FL_IN2, FL_EN,
    RL_IN1, RL_IN2, RL_EN,
    FR_IN1, FR_IN2, FR_EN,
    RR_IN1, RR_IN2, RR_EN
]

GPIO.setup(motor_pins, GPIO.OUT)

for pin in IR_PINS:
    GPIO.setup(pin, GPIO.IN)

# ===== PWM =====
pwm_FL = GPIO.PWM(FL_EN, 1000)
pwm_RL = GPIO.PWM(RL_EN, 1000)
pwm_FR = GPIO.PWM(FR_EN, 1000)
pwm_RR = GPIO.PWM(RR_EN, 1000)

for pwm in [pwm_FL, pwm_RL, pwm_FR, pwm_RR]:
    pwm.start(0)

BASE_SPEED = 55
TURN_SPEED = 35

# ===== MOTOR FUNCTIONS =====
def forward():
    GPIO.output([FL_IN1, RL_IN1, FR_IN1, RR_IN1], GPIO.HIGH)
    GPIO.output([FL_IN2, RL_IN2, FR_IN2, RR_IN2], GPIO.LOW)
    for pwm in [pwm_FL, pwm_RL, pwm_FR, pwm_RR]:
        pwm.ChangeDutyCycle(BASE_SPEED)

def left():
    GPIO.output([FL_IN1, RL_IN1], GPIO.HIGH)
    GPIO.output([FL_IN2, RL_IN2], GPIO.LOW)
    GPIO.output([FR_IN1, RR_IN1], GPIO.LOW)
    GPIO.output([FR_IN2, RR_IN2], GPIO.HIGH)

    pwm_FL.ChangeDutyCycle(TURN_SPEED)
    pwm_RL.ChangeDutyCycle(TURN_SPEED)
    pwm_FR.ChangeDutyCycle(BASE_SPEED)
    pwm_RR.ChangeDutyCycle(BASE_SPEED)

def right():
    GPIO.output([FL_IN1, RL_IN1], GPIO.LOW)
    GPIO.output([FL_IN2, RL_IN2], GPIO.HIGH)
    GPIO.output([FR_IN1, RR_IN1], GPIO.HIGH)
    GPIO.output([FR_IN2, RR_IN2], GPIO.LOW)

    pwm_FL.ChangeDutyCycle(BASE_SPEED)
    pwm_RL.ChangeDutyCycle(BASE_SPEED)
    pwm_FR.ChangeDutyCycle(TURN_SPEED)
    pwm_RR.ChangeDutyCycle(TURN_SPEED)

def stop():
    for pwm in [pwm_FL, pwm_RL, pwm_FR, pwm_RR]:
        pwm.ChangeDutyCycle(0)
    GPIO.output([
        FL_IN1, FL_IN2, RL_IN1, RL_IN2,
        FR_IN1, FR_IN2, RR_IN1, RR_IN2
    ], GPIO.LOW)

# ===== MAIN LOOP =====
try:
    while True:
        ir = [GPIO.input(pin) for pin in IR_PINS]

        if ir[2] == 0:
            forward()
        elif ir[0] == 0 or ir[1] == 0:
            left()
        elif ir[3] == 0 or ir[4] == 0:
            right()
        else:
            stop()

        time.sleep(0.01)  # fast + stable

except KeyboardInterrupt:
    stop()
    GPIO.cleanup()
