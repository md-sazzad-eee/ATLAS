"""Accuracy vs range on synthetic renders (50 mm tag36h11, Camera Module 3 binned 1536x864).
python3 test/characterise.py  -> table used in README.md"""
import math, pathlib, sys, time
import numpy as np
HERE = pathlib.Path(__file__).resolve().parent; sys.path[:0] = [str(HERE.parent), str(HERE)]
from synth import render
from atlas_vision.detector import TagDetector

D = TagDetector()
print("| range (m) | detected | range err (mm) | lateral err (mm) | yaw err (deg, tag at 20 deg) | ms/frame (host) |")
print("|---|---|---|---|---|---|")
for d in (0.15, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0):
    re, le, ye, ts = [], [], [], []
    for s in range(6):
        img = render(s + 1, d, 0.03, 20, seed=s); t = time.perf_counter(); o = D.detect(img); ts.append(time.perf_counter() - t)
        if o:
            o = o[0]; re.append(abs(o.x_mm - d * 1000)); le.append(abs(o.y_mm + 30)); ye.append(abs(math.degrees(o.yaw_rad) + 20))
    f = lambda v: f"{np.mean(v):.1f}" if v else "-"
    print(f"| {d:.2f} | {len(re)}/6 | {f(re)} | {f(le)} | {f(ye)} | {np.mean(ts)*1000:.0f} |")
