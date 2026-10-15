# Electronics

## Board B: ATLAS-S agent controller (shared by S1 and S2), rev A

KiCad 10 project: `agent_controller/atlas_agent_ctrl.kicad_pro`
Design source of truth: `agent_controller/design/board_b_design.py`, which holds every part, value, footprint and pin→net assignment.

| Check | Result |
|---|---|
| Parts / nets | 112 parts, 69 signal nets + 8 intentional no-connects |
| ERC (kicad-cli) | **0 errors, 0 warnings** |
| Schematic vs design spec | `design/verify_netlist.py`: **69/69 nets identical, pin-for-pin** |
| PCB placement | 110 × 90 mm, 2-layer, 4× M3. Courtyard check passes with no overlaps and no off-board parts. ESP32 antenna is flush with the front edge, with a keep-out band |
| Routing | Freerouting 2.1 (local, headless) via `design/run_pcb_pipeline.sh`: 1117 segments, 321 vias, **0 unrouted** |
| DRC (kicad-cli, with schematic parity) | **0 errors, 0 unconnected, 0 parity issues**; 8 cosmetic silkscreen warnings |
| Mechanical fit | Board inside the Scout body (FreeCAD): 5.2 mm above the wheels, 17.4 mm under the lid, 6 mm behind the front wall (`fit_check.json`) |
| Fab outputs | `fab/atlas_agent_ctrl_gerbers.zip` (gerbers + drill), `fab/atlas_agent_ctrl_bom.csv`, `fab/atlas_agent_ctrl_pos.csv`, `atlas_agent_ctrl.step` |

### Architecture

- **Power in:** 2S Li-ion (J1, with NTC), then a 3 A PTC, then a reverse-polarity P-MOSFET (AOD409), then a TVS (SMBJ10CA) and 100 µF bulk.
- **Dock charge path:** J2 pads (VCHG, GND, DATA, ID), then an SS34 diode (no backfeed to the pads), then the charge switch Q2, which the ESP32 can open (`CHG_EN`).
  - The ID resistor R8 sets agent identity: 10 k for S1, 22 k for S2.
  - DATA is a half-duplex UART with 1 k series resistors and ESD protection.
  - Note: the diode drop limits the pack to about 8.0 V (about 90% charge). This is deliberate and conservative.
- **Rails:**
  - **5 V / 3 A:** TPS563201 buck, Vout = 0.768 × (1 + 56k/10k) = 5.07 V. Feeds the Pi module and the sensor connector.
  - **3.3 V / 600 mA:** AP2112K LDO, fed from 5 V or USB VBUS through Schottky OR diodes, so the board can be programmed from USB without a battery.
- **MCU:** ESP32-S3-WROOM-1-N8R8 with native USB-C (ESD-protected), RESET and BOOT buttons, and a UART0 debug header.
  - GPIO35–37 are left unconnected because the N8R8 uses them for octal PSRAM.
  - Strapping pins 45 and 46 are left unconnected.
- **Motors:**
  - Two DRV8871 drivers with ILIM 33 k, giving about 2.0 A current limiting (assumed stall about 1.2 A from the MATLAB sizing).
  - A 20 mΩ high-side shunt feeds an INA180A2 (×50) per motor, so 1 A reads as 1.0 V at the ADC.
  - Encoders have 10 k pull-ups.
- **Independent motor disable:**
  - The motor rail VM_SW is switched by the P-MOSFET Q5.
  - Q5 turns on only when the 74LVC1G08 AND gate sees both `MOT_EN` (firmware) and `WDT_OK` (TPS3823 watchdog, 1.6 s timeout).
  - If the firmware hangs, the motors lose power without any software involved.
- **Scout vision module:** J6 (JST-GH 6P) carries power-gated 5 V and a UART to a Raspberry Pi Zero 2 W + Camera Module 3. See `07_documentation/camera_architecture.md`.
- **Expansion:** Qwiic I2C (J7) for IMU/ToF, and an 8-pin sensor-module connector (J8: 5 V, 3.3 V, I2C, GPIO, ADC) for the Research agent's instrument.
- **Test points:** VBAT, 5 V, 3V3, VM_SW, IMOT_L, IMOT_R, WDT_OK, GND.

### ESP32-S3 pin map

| GPIO | Net | GPIO | Net |
|---|---|---|---|
| 1/2 | I2C SDA/SCL | 15/16 | MOT_L IN1/IN2 |
| 4 | VBAT_SENSE (100k/22k) | 17/18 | LED_STAT / PI_PWR_EN |
| 5/6 | IMOT_L / IMOT_R | 21 | MOT_EN |
| 7 | BATT_NTC | 38 | WDI (watchdog kick) |
| 8 | DOCK_VSENSE | 39/40 | DOCK_TX / DOCK_RX |
| 9/10 | SENS_ADC / SENS_GPIO | 41 | CHG_EN |
| 11/12 | ENC_L A/B | 42 | MOT_R IN2 |
| 13/14 | ENC_R A/B | 47 | MOT_R IN1 |
| 19/20 | USB D−/D+ | 48 / 3 | PI_TX / PI_RX |

*Rev A1: the right-motor inputs moved to right-side module pins (IO47/IO42), swapping with LED_STAT/PI_PWR_EN, because the router could not cross the board from the left-side pins.*

### Rebuilding the board

`design/run_pcb_pipeline.sh` (macOS) runs the whole flow:
1. Place the parts from the spec.
2. Route with Freerouting.
3. Import the routes and add the GND pours and stitching vias.
4. Run up to 3 completion passes for any connection the router missed.
5. Bridge any isolated GND islands.
6. Run DRC.

The spec, the placement and the pipeline are the source. The `.kicad_pcb` is a generated output.
Routing needs a Java 21 runtime; the pipeline uses COMSOL's bundled JRE.

Layout choices:
- **Inset outline for routing.** Freerouting only keeps its 0.2 mm track clearance to the outline, so the DSN is exported with the outline inset 0.4 mm. This gives at least 0.6 mm copper-to-edge on the finished board.
- **GND plane, not routed.** GND is a plane on B.Cu plus a pour on F.Cu, and is not routed as a net.
- **Power class.** Power nets use 0.5 mm tracks. 0.8 mm tracks could not enter SOT-23 pins.

### Known rev-A limitations

- The F.Cu pour uses solid (not thermal-relief) pad connections. This is easier for DRC but harder to hand-solder on large GND pads.
- The schematic is label-driven and block-organised. It is correct (verified), but some value and label text overlaps and could be tidied in Eeschema.
- Motor stall current is assumed, not measured. Measure it before ordering and adjust ILIM (R24/R25).
- Diode-OR losses on the LDO input are about 0.3 V. That's fine at 5 V and should be revisited if the LDO input is ever taken from the battery.

## Shared tool flow (Boards A and C)

All tools are in `02_electronics/tools/`. Each board is defined by one Python spec, and the drawing is generated from it.

1. **`check_design.py`** checks the spec against the KiCad libraries:
   - every symbol exists
   - every spec pin exists on the symbol
   - every symbol pin is assigned
   - every footprint has a pad for every pin
2. **`gen_schematic.py`** writes a label-driven `.kicad_sch` with the same style as Board B. Unused pins get no-connect flags, and PWR_FLAGs are added where needed.
3. **`verify_netlist.py`** compares the `kicad-cli` netlist to the spec, pin-for-pin.
4. **`build_pcb.py`** places the board:
   - edge connectors are fixed
   - the remaining parts are placed greedily next to what they connect to, inside per-channel regions
   - writes a Specctra DSN with a 0.5 mm power class
   - the same commands as Board B handle routing import, GND pours/stitching, island bridging and completion passes
5. **`run_board_flow.sh <board_dir>`** runs the whole sequence: ERC → placement DRC → Freerouting → DRC → gerbers/drill/pos/BOM/STEP.
   - Freerouting is limited to `FR_PASSES` passes and `FR_TIMEOUT` seconds. Freerouting 2.1 never stops on its own while any connection is unroutable.
   - Completion passes route the pads of cut-off GND pour islands as tracks. A completion pass that times out ends the loop (the same input would time out again).
   - `fix_islands` joins GND pins the pours miss: straight/L track, then a grid A* router, then (if the pin is fenced in) A* that crosses a signal track, deletes the crossed piece and locks the new GND copper so the completion pass re-routes the signal.
   - **`route_finish.py`** connects whatever Freerouting leaves open, deterministically: KiCad dumps exact geometry, a numpy grid router (all copper layers, through-vias wherever they fit, net-class widths, clearance, hole spacing, edge and keep-outs) routes each open connection, KiCad applies the result. The flow runs it automatically when `NPY` points at a Python with numpy.
   - `RESUME=1 run_board_flow.sh <board>` restarts at the completion passes on the routed board already on disk.
   - Board config switches: `LAYERS` (2 or 4), `GND_ROUTED` (GND routed as an ordinary net, pours added afterwards), `GND_FANOUT` (pre-routing GND vias, True/False/regex), `GAP` and `PART_GAP` (placement spacing), `TRACK`, `CLEARANCE`, `ZONE_CLEARANCE`.

The flow works with KiCad 7 (Linux) and KiCad 10 (macOS):
- Library differences are aliased: `Q_*MOS` moved from `Device` to `Transistor_FET`, and the USB-C shield pin changed from `S1` to `SH`.
- The TPS3823 symbol is vendored in `tools/atlas_extra.kicad_sym`.

## Board C: dock / charge interface, rev A (`dock_charger/`)

| Check | Result |
|---|---|
| Spec vs libraries | 0 errors |
| Schematic | 131 parts, 63 nets, generated. **Netlist = spec on all nets.** **ERC 0/0 in KiCad 10.** |
| Placement | 90 × 66 mm, 2-layer. Bay 1 on the left half, bay 2 on the right. Placement DRC is clean (copper and edge). |
| Routing | Freerouting routes all signals with 0 DRC errors (KiCad 10). A full DRC on the first complete run gave 0 errors and silkscreen warnings only. |
| **Final** | **ERC 0/0. DRC: 0 errors, 0 unconnected, 0 parity** (warnings are silkscreen only). Fab outputs in `dock_charger/fab/`: gerbers + drill zip, BOM, placement, STEP. |

Fixes made during checking:
- The INA226 footprint is now MSOP-10. KiCad 10 has no `VSSOP-10_3x3mm`; the land pattern is the same.
- 10 Ω input resistors were added on the INA226 inputs. TI recommends them, and they also keep 0.5 mm power tracks off 0.5 mm-pitch pins, which had left 9 connections unroutable.
- Board C uses 0.15 mm clearance. JLCPCB's minimum is 0.127 mm, and the 0.5 mm-pitch MSOP pads have 0.15 mm gaps.
- Mounting holes moved to 4 mm from the edge.
- Symbol/footprint parity: Datasheet and Description fields are now copied, and holes and test points are excluded from the BOM.

### Per-bay circuit

- **CV stage:** TPS54202 buck from the 4S pack. Output is 8.34 V (130k/10.0k, 0.1 %). It is deliberately below 8.40 V, so a shorted agent diode still cannot over-charge the pack.
- **CC loop:** 20 mΩ shunt → INA180A2 → LMV358 integrator against ISET → Schottky + 10k injected into the buck FB node.
  - ISET comes from an MCP4725 DAC through a 2:1 divider, so the hardware ceiling is 1.65 A.
- **Pad switch:** AO3401A. It turns on only when `BAY_EN` (Board A, DOCKED only) AND `KALERT_N` are both true. The INA226 contact-alert trip removes it in hardware.
- **4-wire contact monitor:** each agent pad is touched by a force pin and a separate sense pin. INA226 #2 and #3 read V_force − V_sense = I × R_contact directly, with 2.5 µV resolution. This is the monitor that COMSOL/MATLAB called for.
- **DATA / ID:** half-duplex UART (1k / 10k / ESD) and the ID pull-up for agent identity.

### CC/CV loop simulation (`05_simulation/python/cc_cv_charger_sim.py`, averaged model)

- **Connect into a 40 % pack at ISET = 1 A:** peak 1.12 A, settles in 11 ms.
- **Step 0.5 → 1.0 A:** no overshoot, settles in 0.15 ms. Loop crossover is about 3.4 kHz, well below the buck's own loop.
- **Bad contact (+170 mΩ):** 0.86 A (CV-limited). No runaway.
- **Finding:** the agent's SS34 diode (0.3 V) plus about 0.4 Ω of path resistance ends CC at 62 % SOC. Charging stops at 86 % after 181 min.
  - An ideal-diode controller on Board B rev B would reach 98 % in 175 min. This is recommended.

## Board A: carrier main controller, rev A (`carrier_controller/`)

| Check | Result |
|---|---|
| Spec vs libraries | 0 errors |
| Schematic | 157 parts, 96 nets, generated. **Netlist = spec on 93/93 signal nets.** |
| Placement | 130 × 100 mm. ESP32 body flush with the front edge, with an antenna keep-out on all layers. Wheel channels sit at their connectors. 1.5 mm courtyard gap (2.5 mm around the ESP32) leaves routing channels. Placement DRC is clean. |
| Stack-up | **4 layers** (chosen 2026-10-14): F.Cu signals + GND pour, In1.Cu GND pour, In2.Cu signals, B.Cu signals + GND pour. |
| **Final** | **ERC 0 errors. DRC: 0 errors, 0 unconnected, 0 parity** (11 warnings, all silkscreen-to-edge). Fab outputs in `carrier_controller/fab/`: gerbers (4 copper layers) + drill zip, BOM, placement, STEP. |

Why 4 layers: on 2 layers Freerouting left 3–11 connections open on every run (mostly ESP32 pins) and 1–6 GND pins caged off from the pours by signal tracks, in a different place each run (rounds 9–15). Freerouting 2.1 also fails on a dedicated inner plane (exceptions in its plane search), so all four layers are routed with GND as an ordinary net and the GND pours are filled afterwards. With the wider placement gap the first routing pass finished in 6 minutes with 2 connections open, and one completion pass closed them.

### Main blocks

- **MCU:** ESP32-S3-WROOM-1-**N8**. There is no PSRAM, which frees IO35–37 and lets the board use 33 GPIOs.
- **Power:** XT60 → 15 A fuse → reverse-polarity FET → TVS. TPS54302 makes 5 V / 3 A; AP2112K makes 3.3 V (OR-ed with USB).
- **Wheels:** 4× DRV8871 with ILIM 2.9 A. Each side's two drivers share IN1/IN2, so all four wheels are driven with only 4 PWM pins. Each wheel has an INA180A1 current sense and its own PCNT encoder.
- **Hardware interlock:** the motor rail is on only if `MOT_EN` · `WDT_OK` · `ESTOP_OK` · NAND(SEATED₁, LATCHED_N₁) · NAND(SEATED₂, LATCHED_N₂).
  - The carrier therefore **physically cannot drive while an agent is seated but unlatched**. This is the Stateflow invariant implemented in logic gates.
  - The sensors are wired fail-safe: a cut wire reads as "seated" and "not latched".
- **Latches:** 12 V solenoids on the IO45/IO46 strap pins, which are low at reset, so the latch stays locked by its spring.
- **I/O expander:** TCA9534 reads the bay sensors and Board C alerts, and drives the LED and buzzer.
- **Link to the ROS 2 computer:** USB-C.

Firmware for both boards: `03_firmware/carrier_core` + `carrier_esp32s3`. Board C has no MCU; Board A drives it.
