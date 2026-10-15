"""Pi side of the ESP32 <-> Pi UART link (921600 baud, ATLAS frames).

  Pi -> ESP32 : VISION_TAG for the best tag in each processed frame, HEARTBEAT once a second
                (state 0 = camera OK, 1 = camera error)
  ESP32 -> Pi : SHUTDOWN -> clean OS shutdown; the ESP32 cuts 5 V after its 15 s grace period

Run on the Pi:  python3 -m atlas_vision.service --port /dev/serial0
"""
import argparse
import subprocess
import time

from atlas_vision import proto
from atlas_vision.detector import TagDetector


def clamp16(v):
    return max(-32768, min(32767, int(round(v))))


def run(link, source, detector, max_frames=None, shutdown_cmd=None, hz=5.0, clock=time.monotonic, sleep=time.sleep):
    """link: object with read() -> bytes and write(bytes). Returns the reason the loop ended."""
    rx, seq, t_hb, frames, period = proto.Receiver(), 0, -1e9, 0, 1.0 / hz
    while max_frames is None or frames < max_frames:
        t0 = clock()
        for mtype, _, _ in rx.feed(link.read() or b""):
            if mtype == proto.MSG_SHUTDOWN:
                if shutdown_cmd:
                    subprocess.run(shutdown_cmd, shell=True, check=False)
                return "shutdown"
        img, cam_ok = None, True
        try:
            img = source.read()
            cam_ok = img is not None
        except Exception:
            cam_ok = False
        if img is not None:
            obs = detector.detect(img)
            frames += 1
            if obs:
                o = obs[0]; seq = (seq + 1) & 0xFF
                link.write(proto.frame_build(proto.MSG_VISION_TAG, seq, proto.VISION_TAG.pack(
                    o.tag_id, clamp16(o.x_mm), clamp16(o.y_mm), clamp16(o.z_mm), clamp16(o.yaw_rad * 1000), o.quality)))
        if t0 - t_hb >= 1.0:
            t_hb = t0; seq = (seq + 1) & 0xFF
            link.write(proto.frame_build(proto.MSG_HEARTBEAT, seq, proto.HEARTBEAT.pack(int(t0 * 1000) & 0xFFFFFFFF, 0 if cam_ok else 1)))
        dt = clock() - t0
        if dt < period:
            sleep(period - dt)
    return "max_frames"


class SerialLink:
    def __init__(self, port, baud):
        import serial
        self.s = serial.Serial(port, baud, timeout=0)

    def read(self):
        return self.s.read(4096)

    def write(self, b):
        self.s.write(b)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/serial0"); ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--tag-size", type=float, default=0.050); ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--images", help="glob of test images instead of the camera")
    ap.add_argument("--shutdown-cmd", default="sudo /sbin/shutdown -h now")
    a = ap.parse_args(argv)
    if a.images:
        from atlas_vision.camera import FileSource
        src = FileSource(a.images)
    else:
        from atlas_vision.camera import PiCamera
        src = PiCamera()
    try:
        print("atlas_vision:", run(SerialLink(a.port, a.baud), src, TagDetector(a.tag_size), shutdown_cmd=a.shutdown_cmd, hz=a.hz))
    finally:
        src.close()


if __name__ == "__main__":
    main()
