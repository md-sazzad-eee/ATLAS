"""Frame sources: the Pi camera (Picamera2), a folder of images, or a callable (tests)."""
import glob

import numpy as np


class PiCamera:
    def __init__(self, size=(1536, 864)):
        from picamera2 import Picamera2          # only on the Pi
        self.cam = Picamera2()
        cfg = self.cam.create_video_configuration(main={"size": size, "format": "YUV420"})
        self.cam.configure(cfg)
        self.cam.start()
        self.h = size[1]

    def read(self) -> np.ndarray:
        return self.cam.capture_array("main")[: self.h, :]   # Y plane = greyscale

    def close(self):
        self.cam.stop()


class FileSource:
    def __init__(self, pattern):
        import cv2
        self.files, self.i, self.cv2 = sorted(glob.glob(pattern)), 0, cv2

    def read(self):
        if not self.files:
            return None
        f = self.files[self.i % len(self.files)]; self.i += 1
        return self.cv2.imread(f, self.cv2.IMREAD_GRAYSCALE)

    def close(self):
        pass
