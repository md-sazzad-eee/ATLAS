import json
import math
import pathlib
import sys

import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
from atlas_vision import proto                        # noqa: E402
from atlas_vision.detector import TagDetector         # noqa: E402
from atlas_vision.service import run                  # noqa: E402
from synth import render                              # noqa: E402

ROOT = HERE.parents[2]
DET = TagDetector()


@pytest.mark.parametrize("tag_id,dist,lat,yaw", [
    (0, 0.25, 0.00, 0), (7, 0.40, 0.05, 20), (12, 0.60, -0.08, -30), (3, 1.00, 0.10, 10), (21, 1.50, 0.0, 0)])
def test_pose_accuracy(tag_id, dist, lat, yaw):
    obs = DET.detect(render(tag_id, dist, lat, yaw, seed=tag_id))
    assert obs, "tag not detected"
    o = obs[0]
    assert o.tag_id == tag_id
    assert abs(o.x_mm - dist * 1000) < 0.02 * dist * 1000 + 3, o        # range within 2 % + 3 mm
    assert abs(o.y_mm - (-lat * 1000)) < 5 + 0.01 * dist * 1000, o      # lateral (left +)
    # body yaw (z up) = -camera yaw. A 50 mm tag gives yaw to 4 deg only up to ~0.6 m; beyond that
    # single-tag yaw is ill-conditioned (~10 deg) while range/bearing stay accurate -> dock uses bearing far out
    assert abs(math.degrees(o.yaw_rad) + yaw) < (4.0 if dist <= 0.6 else 12.0), o
    assert o.quality > 60


def test_no_false_positive_on_clutter():
    import numpy as np
    rng = np.random.default_rng(5)
    img = (rng.random((864, 1536)) * 255).astype("uint8")
    assert DET.detect(img) == []


def test_max_range_spec():
    """Docking alignment needs the dock tag at >= 1.5 m; find where 50 mm tags stop being read."""
    ok = [d for d in (1.5, 2.0, 2.5, 3.0) if DET.detect(render(5, d, 0, 0, seed=1))]
    print("detected at", ok)
    assert 1.5 in ok


def test_proto_copy_identical():
    a = (ROOT / "03_firmware/pi_vision/atlas_vision/proto.py").read_text().split('"""', 2)[2]
    b = (ROOT / "04_ros2_ws/src/atlas_bridge/atlas_bridge/proto.py").read_text().split('"""', 2)[2]
    assert a == b


def test_proto_against_c_vectors():
    v = json.loads((ROOT / "03_firmware/common/test/proto_vectors.json").read_text())
    assert proto.crc16(b"123456789") == v["crc16_123456789"]


class Loop:
    def __init__(self, incoming=b""):
        self.inbox, self.out = bytearray(incoming), bytearray()

    def read(self):
        b = bytes(self.inbox); self.inbox.clear(); return b

    def write(self, b):
        self.out += b


class Frames:
    def __init__(self, imgs):
        self.imgs = list(imgs)

    def read(self):
        return self.imgs.pop(0) if self.imgs else None


def test_service_sends_tag_and_heartbeat():
    link = Loop()
    reason = run(link, Frames([render(9, 0.5, 0.02, 15, seed=3)] * 3), DET, max_frames=3, sleep=lambda s: None)
    msgs = proto.Receiver().feed(bytes(link.out))
    types = [m[0] for m in msgs]
    assert reason == "max_frames" and types.count(proto.MSG_VISION_TAG) == 3 and proto.MSG_HEARTBEAT in types
    tid, x, y, z, yaw, q = proto.VISION_TAG.unpack([m for m in msgs if m[0] == proto.MSG_VISION_TAG][0][2])
    assert tid == 9 and abs(x - 500) < 13 and abs(yaw + 262) < 70


def test_service_shutdown_message():
    link = Loop(proto.frame_build(proto.MSG_SHUTDOWN, 1))
    assert run(link, Frames([]), DET, max_frames=10, shutdown_cmd="true", sleep=lambda s: None) == "shutdown"
