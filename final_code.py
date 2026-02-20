import cv2
import numpy as np
from ultralytics import YOLO
from tkinter import *
from PIL import Image
import threading
import time
import RPi.GPIO as GPIO

# ================= GPIO CONFIG =================
LEFT_IN1  = 17
LEFT_IN2  = 27
RIGHT_IN1 = 22
RIGHT_IN2 = 23
LEFT_EN   = 18
RIGHT_EN  = 13

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in [LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2, LEFT_EN, RIGHT_EN]:
    GPIO.setup(pin, GPIO.OUT)
    print("Succesfully executed")
left_pwm = GPIO.PWM(LEFT_EN, 100)
right_pwm = GPIO.PWM(RIGHT_EN, 100)
left_pwm.start(0)
right_pwm.start(0)

# ================= MOTOR FUNCTIONS =================
def stop_motors():
    GPIO.output([LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2], GPIO.LOW)
    left_pwm.ChangeDutyCycle(0)
    right_pwm.ChangeDutyCycle(0)

def move_forward(l, r):
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN1, GPIO.HIGH)
    GPIO.output(RIGHT_IN2, GPIO.LOW)
    left_pwm.ChangeDutyCycle(l)
    right_pwm.ChangeDutyCycle(r)

def turn_left(speed=45):
    move_forward(30, speed)

def turn_right(speed=45):
    move_forward(speed, 30)

# ================= YOLO CONFIG =================
model = YOLO("yolov8n.pt")
conf_threshold = 0.35

HSV_RANGES = {
    "red1": ((0, 100, 100), (10, 255, 255)),
    "red2": ((160, 100, 100), (179, 255, 255)),
    "yellow": ((15, 100, 100), (35, 255, 255)),
    "green": ((36, 80, 80), (90, 255, 255)),
}

def classify_light(crop):
    if crop is None or crop.size == 0:
        return "unknown"
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    red = cv2.countNonZero(cv2.inRange(hsv, *map(np.array, HSV_RANGES["red1"]))) + \
          cv2.countNonZero(cv2.inRange(hsv, *map(np.array, HSV_RANGES["red2"])))
    yellow = cv2.countNonZero(cv2.inRange(hsv, *map(np.array, HSV_RANGES["yellow"])))
    green = cv2.countNonZero(cv2.inRange(hsv, *map(np.array, HSV_RANGES["green"])))
    data = {"red": red, "yellow": yellow, "green": green}
    color = max(data, key=data.get)
    return color if data[color] > 50 else "unknown"

# ================= LANE DETECTION =================
def lane_steering(frame):
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5,5), 0), 50, 150)

    mask = np.zeros_like(edges)
    roi = np.array([[(0,h), (w,h), (w,int(h*0.6)), (0,int(h*0.6))]], np.int32)
    cv2.fillPoly(mask, roi, 255)
    cropped = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(cropped, 1, np.pi/180, 50, minLineLength=60, maxLineGap=150)

    left_x, right_x = [], []
    if lines is not None:
        for x1,y1,x2,y2 in lines[:,0]:
            slope = (y2-y1)/(x2-x1+1e-6)
            if abs(slope) < 0.4: continue
            if slope < 0: left_x.append((x1+x2)/2)
            else: right_x.append((x1+x2)/2)
            cv2.line(frame, (x1,y1),(x2,y2),(255,255,255),2)

    if not left_x or not right_x:
        return frame, 0

    lane_center = (np.mean(left_x) + np.mean(right_x)) / 2
    frame_center = w / 2
    error = (lane_center - frame_center) / frame_center

    cv2.line(frame, (int(lane_center),h),(int(lane_center),int(h*0.6)),(0,255,0),2)
    cv2.line(frame, (int(frame_center),h),(int(frame_center),int(h*0.6)),(0,0,255),2)

    return frame, error

# ================= GUI APP =================
class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Self Driving Car System")
        self.cap = None
        self.running = False

        self.label = Label(root)
        self.label.pack()

        self.info = Label(root, text="IDLE", font=("Arial",14))
        self.info.pack()

        Button(root, text="START", command=self.start).pack(side=LEFT, padx=10)
        Button(root, text="STOP", command=self.stop).pack(side=LEFT, padx=10)

    def start(self):
        if not self.running:
            self.cap = cv2.VideoCapture(0)
            self.running = True
            threading.Thread(target=self.loop).start()

    def stop(self):
        self.running = False
        stop_motors()
        GPIO.cleanup()
        if self.cap: self.cap.release()

    def loop(self):
        prev = time.time()
        while self.running:
            ret, frame = self.cap.read()
            if not ret: break

            frame, steer_error = lane_steering(frame)
            action = "WAIT"

            results = model.predict(frame, conf=conf_threshold, verbose=False)
            for r in results:
                for box in r.boxes:
                    name = model.names[int(box.cls[0])].lower()
                    if "traffic" in name:
                        x1,y1,x2,y2 = map(int, box.xyxy[0])
                        color = classify_light(frame[y1:y2, x1:x2])
                        action = {"red":"STOP","yellow":"SLOW","green":"GO"}.get(color,"WAIT")
                        cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,255),2)
                        cv2.putText(frame,color,(x1,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,255),2)

            # ================= CONTROL =================
            if action == "STOP":
                stop_motors()
            elif action == "SLOW":
                move_forward(30,30)
            elif action == "GO":
                if steer_error > 0.15:
                    turn_right()
                elif steer_error < -0.15:
                    turn_left()
                else:
                    move_forward(55,55)
            else:
                stop_motors()

            fps = 1/(time.time()-prev+1e-6)
            prev = time.time()
            cv2.putText(frame,f"FPS:{fps:.1f}",(10,25),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,0),2)
            cv2.putText(frame,f"ACTION:{action}",(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2)

            img = Image.PhotoImage(Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)))
            self.label.configure(image=img)
            self.label.image = img
            self.info.configure(text=f"Action: {action} | Steering Error: {steer_error:.2f}")

# ================= RUN =================
root = Tk()
App(root)
root.mainloop()