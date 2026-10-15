# Firmware

All decision-making code is plain C11 with no hardware calls. It is unit-tested on the host against plant models. The ESP32 ports only read sensors, call `tick()` every 10 ms and drive the pins.

| Folder | What | Host test (`make test`) |
|---|---|---|
| `common/` | Wire protocol (COBS + CRC-16, packed structs), PI controller, docking FSM (C port of the Stateflow chart) | 6022 checks. FSM matches all 8 Stateflow traces sample-for-sample. The PI loop matches Simulink: 4.9 % overshoot vs 4.8 %. Protocol passes fuzz + resync. |
| `agent_core/` | Agent behaviour (Board B) | 42 checks, run against a DC-motor + diff-drive plant (see below) |
| `carrier_core/` | Carrier behaviour (Board A) | 28 checks, run against a skid-steer + 2-bay + charger plant (see below) |
| `carrier_core/sim/` | The carrier core + plant as a shared library | Used by ROS 2 `fake_carrier` (software-in-the-loop) |
| `agent_esp32s3/` | ESP-IDF 5 / PlatformIO port for Board B: LEDC→DRV8871, PCNT encoders, ADC1, Wi-Fi UDP, Pi UART, dock UART | `pio run` (see "Not yet done") |
| `carrier_esp32s3/` | ESP-IDF port for Board A: side PWM, 4 PCNT, ADC1, I²C (TCA9534, 6× INA226, 2× MCP4725), USB-CDC link to ROS, dock UARTs | `pio run` |
| `pi_vision/` | Scout Pi Zero 2 W service: AprilTag → `VISION_TAG` over UART | `python3 -m pytest test`: 11 tests on synthetic renders |

## Agent core results (`agent_core/test`)

- **Navigation:** go-to (1.0, 0.6) m arrives in 7.0 s. True position error is 25–29 mm, inside the 30 mm arrive tolerance. Speed limit holds at 0.201 m/s.
- **Recalibration:** a sensor drifted to read 1.150 on a 1.000 reference reads 1.000 after recalibration, and the calibration epoch increments.
- **Carrier heartbeat lost:** the agent goes to SAFE_STOP after the 2 s timeout and resumes when heartbeats return. At 0.2 m/s the agent travels about 0.4 m before it stops.
  - **Open decision:** shorten the timeout if that distance is too far.
- **Stall:** a jammed wheel triggers FAULT within 300 ms. The task reports FAILED, motor power is cut, and the fault is cleared only by a carrier heartbeat bit.
- **Charging:** enabled only on the dock, only between 0 and 45 °C, and stops at 8.10 V with restart at 7.90 V.
- **DRIVE_CMD dead-man:** a command lasts at most 1 s. The motor then holds zero speed for 300 ms (closed-loop brake) before the motor rail switches off.
- **Inspect:** the Pi is powered on demand. It sends `SHUTDOWN` and the agent cuts power after a 15 s grace period. No tag means INCONCLUSIVE; no camera means FAILED.
- **Tasks:** repeated task IDs are idempotent and a cancel returns ABORTED. `SET_POSE` sets the deploy pose.

## Carrier core results (`carrier_core/test`)

- **Dock, bay 0:** DOCKED at 1.9 s.
  - The 4-wire contact check at 0.5 A measures 30/30 mΩ (true values 30/30), then the bay charges at 1.0 A CC.
  - On undock, the charge switch turns off in the same tick the docking FSM leaves DOCKED.
  - The motors were never enabled while an agent was seated but unlatched.
- **Dirty GND contact (150 mΩ):** rejected at the contact check.
- **Contact degrades to 120 mΩ during charging:** trips in 200 ms.
- **INA226 ALERT:** latched as a charge fault.
- **Taper:** below 50 mA for 10 s ends the charge as DONE.
- **cmd_vel 0.30 m/s:** 90 % reached in 150 ms, steady 0.299 m/s, 0.7 % overshoot, odometry error 3 mm over 0.88 m. A 0.8 rad/s spin also tracks.
- **Stops:** the cmd_vel dead-man (≤ 500 ms) and loss of the ROS heartbeat both stop the carrier.
- **Ignored commands:** cmd_vel is ignored while a bay is in APPROACH.
- **E-stop:** puts both bays in FAULT. Reset is refused while the E-stop is still pressed.
- **Wrong agent ID:** goes to BACKOFF and never reaches DOCKED.

## Protocol additions (all shared C ↔ Python, sizes asserted at compile time)

| Type | Message | Notes |
|---|---|---|
| `0x21` | `SET_POSE` | |
| `0x50` | `DOCK_CMD` | |
| `0x51` | `BAY_STATUS` | 15 B |
| `0x52` | `CARRIER_STATUS` | 23 B |
| `0x53` | `DOCK_RELAY` | Agent frames carried over the dock DATA pad, tagged with the bay |

Carrier heartbeat state bits:
- To the agent: b0 = E-stop, b7 = clear faults.
- ROS → carrier: b0 = soft E-stop, b1/b2 = agent 1/2 link alive.

## Assumptions to replace on the bench

- **Agent motors:** 6 V, 1:150, the same model as `build_speed_loop.m`.
- **Carrier motors:** 12 V, 1:50, 11-CPR hall encoders (R 2 Ω, Kt 0.0155). Carrier PI gains Kp 0.9 and Ki 11.5 were tuned on this model.
- **Re-identify** both motors with a step test before flashing.

## Build status

Both ESP32 ports compile with **0 warnings** using PlatformIO espressif32 6.9.0 / ESP-IDF 5.3.1 (macOS build).

| Firmware | RAM | Flash |
|---|---|---|
| Agent (Board B) | 36.7 kB (11 %) | 839 kB |
| Carrier (Board A) | 14.3 kB (4 %) | 279 kB |

Notes:
- **Partition table:** set to `SINGLE_APP_LARGE` (1.5 MB app). PlatformIO's size bar still assumes 1 MB.
- **Build path:** ESP-IDF refuses paths that contain spaces, so build from a copy, for example `rsync` to `~/tools/fw_build` and run `pio run` there.
- **Fixes made during the first build:**
  - `EXTRA_COMPONENT_DIRS` must be appended, not set, or PlatformIO loses `src/`.
  - `hal_init` clashed with a symbol in the Wi-Fi blob, so the HAL functions are now named `board_b_*` / `board_a_*`.

Still to do: flash the boards and bring them up on hardware.
