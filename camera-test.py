from picamera2 import Picamera2, Preview
from picamera2.utils import Transform
from libcamera import controls 
import time

try:
    picam2 = Picamera2()
    
    # 1. Flip for your setup
    t = Transform(vflip=True, hflip=True)
    
    # 2. FORCE FULL SENSOR MODE (4:3 is often more 'filmic' for POV)
    config = picam2.create_preview_configuration(
        main={'size': (1920, 1080)}, 
        transform=t
    )
    picam2.configure(config)
    
    picam2.start_preview(Preview.QTGL)
    picam2.start()

    # 3. THE CINEMATIC "COLOR GRADE"
    picam2.set_controls({
        "ScalerCrop": (0, 0, 4608, 2592),
        "AfMode": controls.AfModeEnum.Continuous,
        "AfRange": controls.AfRangeEnum.Macro,
        
        # --- Graphics & Filters ---
        "Brightness": 0.0,      # Keep at 0 for natural shadows
        "Contrast": 1.4,        # Makes lanes 'pop' and shadows deep
        "Saturation": 1.3,      # Richer colors
        "Sharpness": 1.5,       # Crisper edges for lane lines
        
        # Setting a slightly warm white balance (Cinematic Golden Hour look)
        "AeExposureMode": controls.AeExposureModeEnum.Normal,
        "AwbMode": controls.AwbModeEnum.Indoor # Usually warmer/more orange
    })

    print("🎬 CINEMATIC ROBOT FEED ACTIVE")
    print("Settings: High Contrast | Rich Saturation | Full FOV")
    
    while True:
        time.sleep(1)

except Exception as e:
    print(f"❌ Error: {e}")
finally:
    if 'picam2' in locals():
        picam2.stop()