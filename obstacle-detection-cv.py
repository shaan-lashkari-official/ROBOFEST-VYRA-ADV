from picamera2 import Picamera2, Preview
from picamera2.utils import Transform
from libcamera import controls
import time

# --- THRESHOLD ---
# 0.0 is Infinity (far). 
# Higher numbers mean the lens is moved forward for close-up (Macro).
# For Camera Module 3, ~8.0 to 10.0 is usually around 10cm.
NEAR_THRESHOLD = 5.0

try:
    picam2 = Picamera2()
    
    # 1. Flip for your setup
    t = Transform(vflip=True, hflip=True)
    
    # 2. Configure for POV
    config = picam2.create_preview_configuration(
        main={'size': (1280, 720)}, 
        transform=t
    )
    picam2.configure(config)
    
    # 3. Start the Hardware Preview window
    picam2.start_preview(Preview.QTGL)
    picam2.start()

    # 4. Cinematic & Autofocus Settings
    picam2.set_controls({
        "AfMode": controls.AfModeEnum.Continuous,
        "AfRange": controls.AfRangeEnum.Normal, # Essential for close objects
        "Contrast": 1.4,
        "Saturation": 1.3,
        "Sharpness": 1.5,
        "AwbMode": controls.AwbModeEnum.Indoor
    })

    print("🎬 CAMERA FEED ACTIVE")
    print(f"Tracking Lens Position (Threshold: {NEAR_THRESHOLD})")
    print("Press Ctrl+C to exit")

    while True:
        # Capture metadata from the current stream
        metadata = picam2.capture_metadata()
        
        # Extract the LensPosition (measured in diopters)
        # Note: If the camera is still searching, this value might fluctuate
        lens_pos = metadata.get("LensPosition")

        if lens_pos is not None:
            if lens_pos >= NEAR_THRESHOLD:
                status = "🚨 STATUS: OBJECT NEAR"
            else:
                status = "✅ STATUS: PATH CLEAR"
            
            # Print to CMD with a carriage return to keep it on one line
            print(f"{status} | Lens Position: {lens_pos:.2f}    ", end="\r")
        
        # Small delay to reduce CPU load
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n\n🛑 Stopping...")
except Exception as e:
    print(f"\n❌ Error: {e}")
finally:
    if 'picam2' in locals():
        picam2.stop()
