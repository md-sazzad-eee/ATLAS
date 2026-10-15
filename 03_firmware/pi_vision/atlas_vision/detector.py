"""AprilTag (tag36h11) detection and pose, reported in the agent's body frame:
x forward, y left, z up (millimetres), yaw = rotation of the tag's facing direction about z
(0 = tag faces the camera squarely). quality = decision margin scaled to 0..255."""
from dataclasses import dataclass
import math

import numpy as np
from pupil_apriltags import Detector

# Camera Module 3 (66 deg HFOV) in the 1536x864 2x2-binned mode. Replace with a checkerboard
# calibration (camera_calibration.py) on the real unit.
DEFAULT_INTRINSICS = (1180.0, 1180.0, 768.0, 432.0)


@dataclass
class TagObs:
    tag_id: int
    x_mm: float
    y_mm: float
    z_mm: float
    yaw_rad: float
    quality: int
    margin: float


class TagDetector:
    def __init__(self, tag_size_m=0.050, intrinsics=DEFAULT_INTRINSICS, cam_offset_mm=(0.0, 0.0, 0.0),
                 threads=4, decimate=2.0, min_margin=20.0):
        self.det = Detector(families="tag36h11", nthreads=threads, quad_decimate=decimate,
                            refine_edges=True, decode_sharpening=0.25)
        self.tag_size, self.K, self.off, self.min_margin = tag_size_m, intrinsics, cam_offset_mm, min_margin

    def detect(self, gray: np.ndarray):
        out = []
        for d in self.det.detect(gray, estimate_tag_pose=True, camera_params=self.K, tag_size=self.tag_size):
            if d.decision_margin < self.min_margin or d.pose_t is None:
                continue
            t = d.pose_t.ravel() * 1000.0          # camera frame: x right, y down, z forward (mm)
            R = d.pose_R
            yaw = math.atan2(R[0, 2], R[2, 2])    # rotation of the tag normal about the camera's vertical
            out.append(TagObs(int(d.tag_id), t[2] + self.off[0], -t[0] + self.off[1], -t[1] + self.off[2],
                              -yaw, int(min(255, d.decision_margin * 2.0)), float(d.decision_margin)))
        out.sort(key=lambda o: -o.margin)
        return out
