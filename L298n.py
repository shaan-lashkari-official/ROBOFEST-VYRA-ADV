import RPi.GPIO as GPIO
import time


# GPIO Mode
GPIO.setmode(GPIO.BCM)

# Motor A
IN1 = 17
IN2 = 18
ENA = 12

# Motor B
IN3 = 22
IN4 = 23
ENB = 13

# Setup pins
GPIO.setup([IN1, IN2, IN3, IN4], GPIO.OUT)
GPIO.setup([ENA, ENB], GPIO.OUT)

# PWM setup
pwmA = GPIO.PWM(ENA, 1000)  # 1 kHz
pwmB = GPIO.PWM(ENB, 1000)

pwmA.start(0)
pwmB.start(0)

def set_speed(speed):
    """Speed: 0 to 100"""
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)

def forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def left():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def right():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def stop():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)

forward();
time.sleep(5);
stop();
 

 