import cv2
import numpy as np
from ultralytics import YOLO
from tkinter import *
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

left_pwm = GPIO.PWM(LEFT_EN, 100)
right_pwm = GPIO.PWM(RIGHT_EN, 100)
left_pwm.start(0)
right_pwm.start(0)

# ================= MOTOR =================
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

def turn_left():
    move_forward(30, 45)

def turn_right():
    move_forward(45, 30)
# ================= YOLO =================
model = YOLO("yolov8n.pt")
conf_threshold = 0.35
YOLO_SKIP = 5  # run YOLO every 5 frames

# ================= HSV =================
HSV_RANGES = {
    "red1": ((0,100,100),(10,255,255)),
    "red2": ((160,100,100),(179,255,255)),
    "yellow": ((15,100,100),(35,255,255)),
    "green": ((36,80,80),(90,255,255)),
}

def classify_light(crop):
    if crop is None or crop.size == 0:
        return "unknown"
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

    red = cv2.countNonZero(cv2.inRange(hsv,*map(np.array,HSV_RANGES["red1"]))) + \
          cv2.countNonZero(cv2.inRange(hsv,*map(np.array,HSV_RANGES["red2"])))
    yellow = cv2.countNonZero(cv2.inRange(hsv,*map(np.array,HSV_RANGES["yellow"])))
    green = cv2.countNonZero(cv2.inRange(hsv,*map(np.array,HSV_RANGES["green"])))

    data = {"red":red,"yellow":yellow,"green":green}
    color = max(data,key=data.get)
    return color if data[color] > 50 else "unknown"

# ================= LANE =================
def lane_steering(frame):
    h,w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray,(5,5),0),50,150)

    mask = np.zeros_like(edges)
    roi = np.array([[(0,h),(w,h),(w,int(h*0.6)),(0,int(h*0.6))]],np.int32)
    cv2.fillPoly(mask, roi, 255)
    cropped = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(cropped,1,np.pi/180,50,60,150)

    left_x, right_x = [], []
    if lines is not None:
        for x1,y1,x2,y2 in lines[:,0]:
            slope = (y2-y1)/(x2-x1+1e-6)
            if abs(slope)<0.4: continue
            (left_x if slope<0 else right_x).append((x1+x2)/2)

    if not left_x or not right_x:
        return frame, 0

    lane_center = (np.mean(left_x)+np.mean(right_x))/2
    return frame, (lane_center-w/2)/(w/2)

# ================= CV → TK (NO PIL) =================
def cv_to_tk(frame):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    h,w,_ = rgb.shape
    ppm = f"P6 {w} {h} 255 ".encode() + rgb.tobytes()
    return PhotoImage(data=ppm)

# ================= GUI =================
class App:
    def __init__(self, root):
        self.root = root
        self.cap = None
        self.running = False
        self.frame_id = 0
        self.prev = time.time()
        self.action = "WAIT"

        self.label = Label(root)
        self.label.pack()

        self.info = Label(root, text="IDLE", font=("Arial",14))
        self.info.pack()

        Button(root, text="START", command=self.start).pack(side=LEFT, padx=10)
        Button(root, text="STOP", command=self.stop).pack(side=LEFT, padx=10)

    def start(self):
        if not self.running:
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
            self.cap.set(cv2.CAP_PROP_FOURCC,
                         cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FPS,30)

            self.running = True
            threading.Thread(target=self.loop, daemon=True).start()

    def stop(self):
        self.running = False
        stop_motors()
        left_pwm.stop()
        right_pwm.stop()
        GPIO.cleanup()
        if self.cap:
            self.cap.release()

    def loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                break

            self.frame_id += 1
            frame, err = lane_steering(frame)

            if self.frame_id % YOLO_SKIP == 0:
                small = cv2.resize(frame,(320,320))
                results = model.predict(
                    small,
                    conf=conf_threshold,
                    classes=[9],  # traffic light only
                    verbose=False
                )
                self.action = "WAIT"
                for r in results:
                    for box in r.boxes:
                        x1,y1,x2,y2 = map(int,box.xyxy[0])
                        color = classify_light(frame[y1:y2,x1:x2])
                        self.action = {"red":"STOP","yellow":"SLOW","green":"GO"}.get(color,"WAIT")

            if self.action=="STOP":
                stop_motors()
            elif self.action=="SLOW":
                move_forward(30,30)
            elif self.action=="GO":
                if err>0.15: turn_right()
                elif err<-0.15: turn_left()
                else: move_forward(55,55)

            if self.frame_id % 2 == 0:
                img = cv_to_tk(frame)
                self.label.configure(image=img)
                self.label.image = img

            fps = 1/(time.time()-self.prev+1e-6)
            self.prev = time.time()
            self.info.configure(text=f"{self.action} | FPS: {fps:.1f}")

# ================= RUN =================
root = Tk()
App(root)
root.mainloop()
