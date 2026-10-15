# Scout vision service (Raspberry Pi Zero 2 W + Camera Module 3)

Runs on the Pi that the ESP32 power-gates (Board B, Q6/Q7). The ESP32 stays in charge: it powers the Pi
for `inspect` tasks and docking, receives tag observations over UART1 (IO48/IO3, 921600 baud, ATLAS
frames), and sends `SHUTDOWN` before cutting 5 V after a 15 s grace period.

| Direction | Message | Content |
|---|---|---|
| Pi → ESP32 | `VISION_TAG` (0x30) | best tag per frame: id, x/y/z mm in the agent body frame (x fwd, y left, z up), yaw mrad, quality 0–255 |
| Pi → ESP32 | `HEARTBEAT` (0x01) | 1 Hz, state 0 = camera OK, 1 = camera error |
| ESP32 → Pi | `SHUTDOWN` (0x40) | clean `shutdown -h now` |

## Install on the Pi (Raspberry Pi OS Lite, Bookworm)

```
sudo raspi-config nonint do_serial_hw 0      # enable UART
sudo raspi-config nonint do_serial_cons 1    # no login console on it
sudo apt install -y python3-picamera2 python3-serial python3-opencv
pip install --break-system-packages pupil-apriltags
sudo mkdir -p /opt/atlas && sudo cp -r pi_vision /opt/atlas/
sudo cp systemd/atlas-vision.service /etc/systemd/system/ && sudo systemctl enable --now atlas-vision
echo "atlas ALL=(root) NOPASSWD: /sbin/shutdown" | sudo tee /etc/sudoers.d/atlas-shutdown
```

## Tests (host, synthetic images)

`python3 -m pytest test` → 11 tests:
- Pose accuracy on rendered tag36h11 scenes.
- No false positives on clutter.
- At least 1.5 m range.
- The protocol copy is identical to the ROS bridge's and matches the C test vectors.
- The service loop sends tags and heartbeats, and handles `SHUTDOWN`.

`python3 test/characterise.py` gives these results (50 mm tag, 6 renders per range, noise σ = 4, blur 1 px):

| range (m) | detected | range err (mm) | lateral err (mm) | yaw err (deg) |
|---|---|---|---|---|
| 0.25 | 6/6 | 0.0 | 0.1 | 0.1 |
| 0.50 | 6/6 | 0.2 | 0.2 | 0.7 |
| 0.75 | 6/6 | 2.5 | 0.4 | 1.9 |
| 1.00 | 6/6 | 21.5 | 1.2 | 24.7 |
| 1.50 | 6/6 | 14.4 | 1.1 | 0.8 |
| 2.00 | 6/6 | 33.4 | 1.0 | 11.7 |
| 3.00 | 2/6 | 28.4 | 1.9 | 17.5 |

### What this means for docking

**Bearing and lateral offset stay accurate to about 2 mm out to 3 m. Yaw from a single 50 mm tag becomes unreliable beyond about 0.75 m** because of the planar pose ambiguity (the 1.0 m and 2.0 m rows).

The dock approach should therefore:
1. Steer on **bearing** when far out.
2. Trust **yaw** only inside about 0.6 m.

Alternatively, use a larger tag (100 mm doubles the range of every row) or a pair of tags on the dock face.

### Caveats

- Renders are ideal pinhole images. Real lens distortion, motion blur and lighting will be worse. Calibrate the camera and repeat the measurements on the bench.
- Host timing is about 50 ms per frame at 1536×864. Expect about 5–8× slower on the Pi Zero 2 W (roughly 3 Hz). If that is too slow, drop to 1/4-resolution detection with `quad_decimate 3`.
