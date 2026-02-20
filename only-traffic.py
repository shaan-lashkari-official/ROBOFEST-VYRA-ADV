import numpy as np

from ai_edge_litert.interpreter import Interpreter

from picamera2 import Picamera2

from PIL import Image

import time

import os



# --- 1. LOAD LABELS --- 📋

labels = []

try:

    with open("labels.txt", "r") as f:

        # Removes numbers if labels look like "0 Red", "1 Blue"

        labels = [line.strip().split(' ', 1)[-1] for line in f.readlines()]

    print(f"Loaded {len(labels)} labels: {labels}")

except FileNotFoundError:

    print("❌ Error: labels.txt not found!")

    exit()



# --- 2. SET UP THE BRAIN (TFLite) --- 🧠

model_path = "model_unquant.tflite" # Change if filename changed!



if not os.path.exists(model_path):

    print(f"❌ Error: {model_path} not found!")

    exit()



interpreter = Interpreter(model_path=model_path)

interpreter.allocate_tensors()



input_details = interpreter.get_input_details()

output_details = interpreter.get_output_details()



height = input_details[0]['shape'][1]

width = input_details[0]['shape'][2]

input_dtype = input_details[0]['dtype']



# --- 3. SET UP THE EYES (RGB Mode) --- 📸

picam2 = Picamera2()

config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (width, height)})

picam2.configure(config)

picam2.start()



print(f"🚀 Color Vision Started! ({width}x{height} RGB)")



try:

    while True:

        # Capture raw RGB frame

        frame = picam2.capture_array()

        

        # Pre-process: Add batch dimension (1, H, W, 3)

        input_data = np.expand_dims(frame, axis=0)



        # Handle Data Type

        if input_dtype == np.float32:

            input_data = input_data.astype(np.float32) / 255.0

        else:

            input_data = input_data.astype(input_dtype)



        # RUN INFERENCE

        interpreter.set_tensor(input_details[0]['index'], input_data)

        interpreter.invoke()



        # GET RESULTS

        output_data = interpreter.get_tensor(output_details[0]['index'])[0]

        

        # Find the label with the highest probability

        top_idx = np.argmax(output_data)

        confidence = output_data[top_idx]

        

        # Handle scaling for quantized outputs (if output is 0-255 instead of 0-1)

        if input_dtype != np.float32:

            # Simple conversion to percentage for display

            display_conf = (confidence / 255.0) * 100

        else:

            display_conf = confidence * 100



        print(f"Detected: {labels[top_idx]} | Confidence: {display_conf:.1f}%")

        

        time.sleep(0.1)



except KeyboardInterrupt:

    print("\nStopping... Bye! 👋")

    picam2.stop()



