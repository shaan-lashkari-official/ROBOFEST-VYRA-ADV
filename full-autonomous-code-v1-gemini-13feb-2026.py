import numpy as np
from ai_edge_litert.interpreter import Interpreter
from picamera2 import Picamera2, Preview
from picamera2.utils import Transform
from libcamera import controls
import time
import os

# --- 1. LOAD AI BRAIN ---
labels = []
try:
    with open("labels.txt", "r") as f:
        labels = [line.strip().split(' ', 1)[-1] for line in f.readlines()]
except FileNotFoundError:
    print("❌ labels.txt not found!")
    exit()

model_path = "model_unquant.tflite"
interpreter = Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
ai_h, ai_w = input_details[0]['shape'][1], input_details[0]['shape'][2]
input_dtype = input_details[0]['dtype']

# --- 2. SET UP CAMERA (16:9 Cinematic Wide) ---
picam2 = Picamera2()
t = Transform(vflip=True, hflip=True)

# Optimized for 16:9 Wide Aspect Ratio
config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": (640, 360)},
    transform=t
)
picam2.configure(config)
picam2.start_preview(Preview.QTGL)
picam2.start()

# Applying Cinematic Filters + Unlocking Maximum Physical FOV
picam2.set_controls({
    "ScalerCrop": (0, 0, 4608, 2592), # IMX708 Absolute Wide-Angle
    "AfMode": controls.AfModeEnum.Continuous,
    "AfRange": controls.AfRangeEnum.Macro,
    "Brightness": -0.05,            # Slightly darker for "film" look
    "Contrast": 1.5,                # Makes lane lines pop against asphalt
    "Saturation": 1.4,              # Deep, rich colors
    "Sharpness": 2.0,               # Crisper edges for better AI detection
    "AwbMode": controls.AwbModeEnum.Indoor
})

print("🎬 CINEMATIC 16:9 LANE SYSTEM ACTIVE")
print("Full Sensor FOV Unlocked. Press Ctrl+C to exit.")

# --- 3. DUAL LANE LOGIC ---
def get_lane_status(frame_xrgb):
    # Slice the bottom ROI (The road area in a 360px high frame)
    # We look at the bottom 60 pixels
    roi = frame_xrgb[300:360, :, :3]
    gray_roi = np.mean(roi, axis=2)
    
    # Thresholding: 1 for black line, 0 for floor
    line_mask = (gray_roi < 70).astype(np.float32)
    
    # Sum columns to find vertical density of black pixels
    pixel_sums = np.sum(line_mask, axis=0)
    
    # Split the view to find two distinct lines
    left_half = pixel_sums[:320]
    right_half = pixel_sums[320:]
    
    # Logic: Stay between the peaks of both halves
    if np.max(left_half) > 5 and np.max(right_half) > 5:
        l_line = np.argmax(left_half)
        r_line = np.argmax(right_half) + 320
        
        lane_center = (l_line + r_line) / 2
        error = lane_center - 320
        
        if error > 25: return "👉 MOVE RIGHT", error
        elif error < -25: return "👈 MOVE LEFT", error
        else: return "🟢 CENTERED", error
            
    elif np.max(left_half) > 5:
        return "⚠️ ONLY LEFT LINE", -60 
    elif np.max(right_half) > 5:
        return "⚠️ ONLY RIGHT LINE", 60   
        
    return "❌ NO LANES", 0

# --- 4. MAIN PROCESSING LOOP ---
try:
    while True:
        # Capture the high-contrast XRGB array
        frame = picam2.capture_array()
        
        # --- LANE TRACKING ---
        direction, error = get_lane_status(frame)

        # --- AI OBJECT DETECTION ---
        # Resize logic adjusted for 360p input
        h_step, w_step = 360 // ai_h, 640 // ai_w
        ai_frame = frame[::h_step, ::w_step, :3][:ai_h, :ai_w]
        
        input_data = np.expand_dims(ai_frame, axis=0)
        if input_dtype == np.float32:
            input_data = input_data.astype(np.float32) / 255.0
        else:
            input_data = input_data.astype(input_dtype)

        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        
        output_data = interpreter.get_tensor(output_details[0]['index'])[0]
        top_idx = np.argmax(output_data)
        
        # Displaying the data
        print(f"{direction} | Err: {error:3.0f} | AI: {labels[top_idx]} ({output_data[top_idx]*100:.0f}%)      ", end="\r")

except KeyboardInterrupt:
    print("\n👋 System Shutdown.")
finally:
    if 'picam2' in locals():
        picam2.stop()