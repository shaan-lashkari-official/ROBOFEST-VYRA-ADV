from picamera2 import Picamera2
from picamera2.utils import Transform
from libcamera import controls 
import time

# --- CONFIGURATION 🛠️ ---
FILENAME = "robot_pov.jpg"

try:
    picam2 = Picamera2()
    
    # 1. Flip it to match your robot's mounting 🔄
    t = Transform(vflip=True, hflip=True)
    
    # 2. Set for High-Resolution Capture 🌟
    config = picam2.create_still_configuration(
        main={'size': (4608, 2592)}, # Full sensor res for Module 3
        transform=t
    )
    picam2.configure(config)
    picam2.start()

    # 3. Apply your "Cinematic" look 🎨
    picam2.set_controls({
        "AfMode": controls.AfModeEnum.Continuous,
        "Contrast": 1.4,
        "Saturation": 1.3,
        "Sharpness": 1.5,
        "AwbMode": controls.AwbModeEnum.Indoor
    })

    print("📸 Camera waking up...")
    # Give the hardware 2 seconds to adjust exposure and focus! ⏳
    time.sleep(2) 

    print(f"🚀 SNAP! Saving to {FILENAME}...")
    picam2.capture_file(FILENAME)
    print("✅ Done! Photo captured successfully! 🎉")

except Exception as e:
    print(f"❌ Snap failed: {e}")

finally:
    if 'picam2' in locals():
        picam2.stop()
