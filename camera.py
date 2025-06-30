#! /usr/bin/env python3
from picamera2 import Picamera2
import cv2
import libcamera
from utils import Logger

logs = Logger(enable=False)

class RealTimeCamera:
    def __init__(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(
            buffer_count=6,
            main={"size": (1280, 960)},
            transform=libcamera.Transform(hflip=1, vflip=1),
            # controls={"FrameDurationLimits": (5000, 5000)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        logs.logger_info("[Camera] PiCamera2 ready")

    def __del__(self):
        self.picam2.close()
        logs.logger_info("[Camera] PiCamera2 closed")

    def capture_frame(self):
        return self.picam2.capture_array()
    
    def stop(self):
        self.picam2.stop()
        logs.logger_info("[Camera] PiCamera2 stopped")

    def close(self):
        self.picam2.close()
        logs.logger_info("[Camera] PiCamera2 stopped")

cam = RealTimeCamera()

def capture_image():
    try:
        frame = cv2.cvtColor(cam.capture_frame(), cv2.COLOR_RGB2BGR)
        return frame           
    except Exception as e:
        logs.logger_error(f"[Camera] Error capturing image: {e}")
        cam.stop()
        cam.close()
        return None
