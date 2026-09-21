import cv2
import numpy as np
import time

# Open the camera (try changing the index if needed)
cap = cv2.VideoCapture(0,cv2.CAP_V4L2)

codec = 0x47504A4D  # MJPG
cap.set(cv2.CAP_PROP_FPS,30.0)
cap.set(cv2.CAP_PROP_FOURCC, codec)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# # # Adjust exposure and brightness
# cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
# # cap.set(cv2.CAP_PROP_EXPOSURE, 100)
# cap.set(cv2.CAP_PROP_AUTO_WB, 1)


# def check_param_support(cap, prop_id):
#     value = cap.get(prop_id)
#     print(f"Property {prop_id}: {value} (supported if not -1)")

# # Check common parameters
# check_param_support(cap, cv2.CAP_PROP_AUTO_EXPOSURE)
# check_param_support(cap, cv2.CAP_PROP_EXPOSURE)
# check_param_support(cap, cv2.CAP_PROP_BRIGHTNESS)

time.sleep(3)

if not cap.isOpened():
    print("Failed to open the camera.")
    exit()

# Verify if the resolution is set correctly
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Resolution set to: {width}x{height}")

i=0

# Capture a frame
for _ in range (60):
    cap.grab()
#time.sleep(1)
ret, frame = cap.read()
if ret:
    filename = "/home/orangepi/MavScripts/photos/captured_image.jpg"
    cv2.imwrite(filename, frame,[cv2.IMWRITE_JPEG_QUALITY, 100])
    print(f"Image saved as {filename}")
else:
    print("Failed to capture image.")