import RPi.GPIO as GPIO
import time

# --- CONFIGURATION ---
TRIG = 2  # Change to your Trigger pin
ECHO = 3  # Change to your Echo pin

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    # 1. Ensure trigger is low
    GPIO.output(TRIG, GPIO.LOW)
    time.sleep(0.05) 

    # 2. Send a 10us pulse to trigger
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)

    # 3. Measure the time the Echo pin stays HIGH
    pulse_start = time.time()
    pulse_end = time.time()

    # Wait for echo to go HIGH
    timeout = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start - timeout > 0.1: # Timeout after 100ms
            return -1 

    # Wait for echo to go LOW
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # 4. Calculate distance
    # Speed of sound is ~34300 cm/s. 
    # Distance = (time * speed) / 2 (for the out and back trip)
    duration = pulse_end - pulse_start
    distance = (duration * 34300) / 2
    
    return round(distance, 2)

# --- MAIN LOOP ---
try:
    print("Testing Ultrasonic Sensor... Press Ctrl+C to stop.")
    while True:
        dist = get_distance()
        if dist >= 0:
            print(f"Distance: {dist} cm")
        else:
            print("Out of range / Error")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nMeasurement stopped.")
    GPIO.cleanup()