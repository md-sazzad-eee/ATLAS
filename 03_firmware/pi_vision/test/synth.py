"""Render a tag36h11 marker into a pinhole-camera image at a known pose (ground truth for tests)."""
import math

import cv2
import numpy as np


def tag_image(tag_id, px=400):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.generateImageMarker(d, tag_id, px)      # 8x8 cells incl. black border


def render(tag_id, dist_m, lateral_m, yaw_deg, tag_size=0.05, K=(1180.0, 1180.0, 768.0, 432.0),
           size=(1536, 864), noise=4.0, blur=1.0, seed=0):
    """Tag centre at camera-frame (x=lateral right, y=0, z=dist), rotated by yaw about the vertical."""
    fx, fy, cx, cy = K
    tag = tag_image(tag_id)
    n = tag.shape[0]
    pad = n // 4                                               # white quiet zone
    canvas = np.full((n + 2 * pad, n + 2 * pad), 255, np.uint8); canvas[pad:pad + n, pad:pad + n] = tag
    s = tag_size / n                                           # metres per tag pixel
    yaw = math.radians(yaw_deg)
    R = np.array([[math.cos(yaw), 0, math.sin(yaw)], [0, 1, 0], [-math.sin(yaw), 0, math.cos(yaw)]])
    t = np.array([lateral_m, 0.0, dist_m])
    src, dst = [], []
    for u, v in [(0, 0), (canvas.shape[1], 0), (canvas.shape[1], canvas.shape[0]), (0, canvas.shape[0])]:
        P = R @ np.array([(u - canvas.shape[1] / 2) * s, (v - canvas.shape[0] / 2) * s, 0.0]) + t
        src.append([u, v]); dst.append([fx * P[0] / P[2] + cx, fy * P[1] / P[2] + cy])
    H = cv2.getPerspectiveTransform(np.float32(src), np.float32(dst))
    rng = np.random.default_rng(seed)
    bg = np.clip(rng.normal(110, 25, (size[1], size[0])), 0, 255).astype(np.uint8)
    bg = cv2.GaussianBlur(bg, (0, 0), 3)
    warped = cv2.warpPerspective(canvas, H, size, flags=cv2.INTER_LINEAR, borderValue=0)
    mask = cv2.warpPerspective(np.full_like(canvas, 255), H, size, flags=cv2.INTER_NEAREST, borderValue=0)
    img = np.where(mask > 0, warped, bg).astype(np.float32)
    if blur:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    img += rng.normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)
