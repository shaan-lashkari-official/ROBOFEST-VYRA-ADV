import RPi.GPIO as GPIO
import time

# FL Pins
IN1, IN2, EN = 17, 27, 13

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, EN], GPIO.OUT)

pwm = GPIO.PWM(EN, 100)
pwm.start(100) # Full speed

print("Testing FL Motor...")
GPIO.output(IN1, GPIO.HIGH)
GPIO.output(IN2, GPIO.LOW)
time.sleep(3)

print("Stopping...")
GPIO.cleanup()