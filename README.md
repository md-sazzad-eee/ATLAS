# ATLAS — Phase 1 foundation

Modular multi-robot research assistant: a carrier (ATLAS-M) that transports two agents
(ATLAS-S1 Scout, ATLAS-S2 Research), deploys them, collects their results, re-plans when
something fails, and tracks which conclusions depend on which evidence.

Phase 1 is the design and simulation stage. Nothing here is a measured hardware result.
Every number comes from `00_requirements/atlas_params.yaml` and is a **proposed target**.

## One source of truth

```
00_requirements/atlas_params.yaml   <- edit this only
python3 00_requirements/sync_params.py   -> atlas_params.json (+ interface checks)
```

MATLAB, FreeCAD and the Python simulator all read `atlas_params.json`. `sync_params.py`
refuses inconsistent interfaces, for example funnels that overlap, wheels that hit the
bays, or a bay shorter than the agent.

## What is here and how to run it

| Area | Tool | Entry point | Output |
|---|---|---|---|
| Drivetrain + energy sizing | MATLAB | `05_simulation/matlab/atlas_sizing.m` | `results/sizing_report.txt`, plots |
| Wheel-speed control loop | Simulink | `05_simulation/matlab/build_speed_loop.m` | `atlas_wheel_speed_loop.slx`, step/load plot |
| Docking state machine | Stateflow | `build_docking_fsm.m`, then `run_docking_tests.m` | `atlas_docking_fsm.slx`, 8 scenario tests, traces |
| Dock contact heating | MATLAB PDE Toolbox | `05_simulation/matlab/contact_heating.m` | `results/contact_heating.*` |
| Parametric CAD | FreeCAD | `01_mechanical/scripts/build_atlas_cad.py` | `.FCStd`, STEP, test-coupon STL, `cad_check.json` |
| Mission intelligence + fleet sim | Python | `05_simulation/python/run_experiments.py` | `results/summary.md`, plot |
| Tests | pytest | `cd 05_simulation/python && python3 -m pytest` | 33 tests |

To run FreeCAD, execute the script from FreeCAD's Python console:
`exec(open('<ATLAS>/01_mechanical/scripts/build_atlas_cad.py').read(), {'__file__': '<that path>'})`

Python needs `numpy scipy networkx pyyaml pydantic matplotlib pytest`.

## Phase 1 findings (what the models changed)

1. **Skid-steer turning sets the carrier motor size.** An in-place turn needs about 3.7×
   the straight-line torque. Widening the track from 300 to 400 mm cut that requirement
   from 0.80 to 0.64 N·m per wheel. Spec per wheel: at least 0.64 N·m continuous and at
   least 95 rpm loaded.
2. **The dock ramp sets the agent motor size.** Climbing the 12° ramp needs 3× the
   flat-floor torque. Spec per wheel: at least 0.10 N·m continuous and at least 73 rpm.
3. **The interface checks caught three geometry errors:**
   - Carrier wheels intruded into the bays, so the wheels moved outboard.
   - The bay was shorter than the agent, so the guides were lengthened to 120 mm.
   - The side latch hit the chassis wall and the neighbouring bay, so it was redesigned
     as a front catch-tab latch. The new latch needed 9.5 mm of pin travel against 8 mm
     available, so the travel spec is now 10 mm.
4. **Contact heating:** about 48 °C at 1 A with a healthy contact. A contaminated contact
   (200 mΩ) reaches about 177 °C at 2 A. Board C should therefore measure the voltage
   drop across the contacts and inhibit charging above about 100 mΩ. Print the contact
   block in PETG, ASA or nylon, not PLA.
5. **Evidence-aware re-planning** (300 paired Monte-Carlo runs). In runs with sensor
   drift:
   - Wrong conclusions: 21% of runs with reassignment alone, 0% with evidence-aware
     re-planning.
   - Recovery cost: about half the re-measurements of a "redo everything" policy, and
     about 3 minutes shorter missions.
   - The carrier never moved with an unlatched agent and no battery-reserve violations
     occurred across all runs.
6. **The batteries are oversized for a 30-minute target:** the agents have about 5 h of
   runtime and the carrier about 6 h. Either downsize (for example a 2S 1000 mAh agent
   pack) or raise the runtime target. Recharging both agents from the carrier uses 40% of
   the carrier's energy.

## Verification done

- The Stateflow chart passes 8 fault scenarios with 3 safety invariants checked on every sample.
- The Python docking FSM reproduces the Stateflow traces sample-for-sample (parity test).
  A deliberate mutation of the Python FSM makes that test fail, so the test does detect differences.
- The FreeCAD interference and clearance report is in `01_mechanical/docking_system/cad_check.json`.
- The blueprint §11 acceptance tests run as simulation tests in `tests/test_blueprint_section11.py`.

## Subsystems

### Dock contact heating (COMSOL)
- `05_simulation/comsol/ContactHeating.java` reproduces the MATLAB result (θ = 156 K/W, 47.8 °C at 1 A, 177 °C at 2 A / 200 mΩ).
- A realistic model that includes the agent side gives 43.6 °C at the design point and 105.7 °C with a contaminated contact.
- `AddPlots.java` adds the temperature and current-density plots to both saved models.

### Camera
ESP32-S3 on every agent. The Scout adds a power-gated Raspberry Pi Zero 2 W + Camera Module 3
(`07_documentation/camera_architecture.md`). Scout runtime with the camera on is about 3.7 h (`atlas_sizing.m`, params v0.1.3).

### Electronics (KiCad, `02_electronics/`)
Every board is specified in Python; the schematic is generated from the spec and the exported netlist is checked against it pin-for-pin.

| Board | Function | Layers | Result |
|---|---|---|---|
| B: agent controller | ESP32-S3, motors, encoders, camera power | 2 | ERC 0/0, DRC 0 errors, 0 unconnected, fab outputs |
| C: dock / charge interface | CC/CV charger per bay, 4-wire contact-resistance monitor | 2 | ERC 0/0, DRC 0 errors, 0 unconnected, fab outputs |
| A: carrier controller | ESP32-S3, 4 wheel drivers, bay interlock in hardware | 4 | ERC 0 errors, DRC 0 errors, 0 unconnected, fab outputs |

Board A puts the "never move with a seated, unlatched agent" rule into logic gates; Board C measures dock contact
resistance by 4-wire sensing. Details and the PCB flow are in `02_electronics/README.md`.

### Firmware (`03_firmware/`)
Agent and carrier application cores in plain C, host-tested against plant models (6022 + 42 + 28 checks, 0 failures).
ESP-IDF ports for Board B and Board A compile with 0 warnings; the Pi Zero vision service has 11 tests.

### ROS 2
A six-package ROS 2 (Jazzy) workspace exists but is not included in this repository yet.

## Further findings

1. The agent's series diode limits a dock charge to about 86 % SOC. An ideal diode on Board B rev B would reach 98 %.
2. Yaw from a single 50 mm AprilTag is unreliable beyond about 0.75 m. Dock alignment should steer on bearing when far out.
3. A 2 s heartbeat timeout lets an agent travel about 0.4 m blind.

## Next phase (bench work)

- Order and assemble Boards A, B and C; flash the ESP32 firmware.
- Motor identification (the agent and carrier PI gains use assumed motors).
- INA226 / ADC calibration and camera calibration.
- Print the clearance coupons; run 100 docking cycles.
- Charge tests with real contacts.
