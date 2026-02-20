from gpiozero import LED
from time import sleep
led = LED(18)
motor1 = LED(15)

while True:
    led.on()
    motor1.on()
    sleep(1)
    led.off()
    motor1.off()
    sleep(1)
    