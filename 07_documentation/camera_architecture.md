# Scout camera architecture: ESP32-S3 + Raspberry Pi Zero 2 W

Decision (2026-10-13): every agent runs on an ESP32-S3 (Board B). The Scout adds a
plug-in **Raspberry Pi Zero 2 W + Camera Module 3** vision module on connector J6.
Real-time control and safety stay on the MCU, and Linux only does vision.

## Why not the alternatives

| Option | Vision capability | Problem for ATLAS |
|---|---|---|
| ESP32-S3 + OV2640/OV5640 only | AprilTag on a plain ESP32 takes about 150 ms/frame at VGA (about 6 fps) and about 100 ms at HVGA, with pose [1] | Too slow and low-resolution for docking alignment at range. Heavier processing would have to be streamed over Wi-Fi to the carrier, which adds latency and a link dependency right at the dock |
| Pi as the main brain | Strong | Linux would sit in the motor/safety path, with about 20 s boot and roughly 0.6 W idle draw even when vision isn't needed. The blueprint requires an independent MCU safety layer anyway |
| **ESP32-S3 + Pi Zero 2 W module (chosen)** | Full OpenCV/AprilTag at 640×480 on 4 cores | Adds about 2.3 W while on. The ESP32 can power-gate it (Q6) between inspection tasks |

## Power (fed into `atlas_params.yaml` v0.1.3 → MATLAB sizing)

- Pi Zero 2 W idle: about 120 mA at 5 V (0.6 W) with Raspberry Pi OS Lite; camera capture with H.264 encode adds about 0.73 W [2].
- Under full 4-core load it peaks at about 1.35–1.46 A at 5 V [2]. The 5 V buck (TPS563201, 3 A) and the 220 µF bulk capacitor at J6 are sized for this.
- Scout average sensor load: 0.3 W ToF + about 2.3 W Pi/camera, through a 90% buck, gives **2.9 W**.

## Interfaces on Board B

| J6 pin | Net | Notes |
|---|---|---|
| 1, 2 | PI_5V | Switched by Q6 (AO3401A) from VCC_5V. ESP32 `PI_PWR_EN` (IO42), off by default |
| 3, 4 | GND | |
| 5 | PI_TX | ESP32 IO48 → Pi GPIO15 (RXD) |
| 6 | PI_RX | Pi GPIO14 (TXD) → ESP32 IO3 |

Protocol: framed binary messages (COBS + CRC16) carrying the same fields as the Python `TaskResult` /
`AgentProfile` models. The ESP32 forwards results to the carrier over Wi-Fi, so the Pi never commands motors.
Shutdown: the ESP32 sends `SHUTDOWN`, waits for `ACK` or 10 s, then drops PI_PWR_EN.

## Firmware split

- **ESP32-S3:** speed loops, encoders, watchdog kick, docking FSM, battery and charge management, Wi-Fi link to the carrier.
- **Pi Zero 2 W:** camera capture, AprilTag/ArUco detection (pose relative to the dock or workstation marker), image capture on request.

Sources:
[1] [raspiduino/apriltag-esp32](https://github.com/raspiduino/apriltag-esp32) ·
[2] [CNX Software — Raspberry Pi Zero 2 W power consumption](https://www.cnx-software.com/2021/12/09/raspberry-pi-zero-2-w-power-consumption/)
