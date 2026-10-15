# atlas_params changelog

| Version | Date | Change | Found by |
|---|---|---|---|
| 0.1.0 | 2026-10-13 | Initial targets from blueprint v1.0 (agent 150x130 mm, 1.2 kg, 2S 2500 mAh; carrier 450x350 mm, 4S LiFePO4 10 Ah) | — |
| 0.1.0 | 2026-10-13 | Agent track 112 -> 100 mm: wheels protruded beyond the 130 mm body | sync_params.py |
| 0.1.0 | 2026-10-13 | Bay pitch 165 -> 170 mm, funnel flare 20 -> 8 mm: adjacent funnel mouths overlapped | sync_params.py |
| 0.1.0 | 2026-10-13 | `skid_turn_factor` replaced by `lateral_friction_coeff` + Wong skid-steer moment model | review of atlas_sizing.m |
| 0.1.1 | 2026-10-13 | Carrier track 300 -> 400 mm (wheels outboard, overall width 440 mm): wheels intruded into the rear bays | sync_params.py |
| 0.1.1 | 2026-10-13 | Guide length 90 -> 120 mm: bay was shorter than the agent | sync_params.py |
| 0.1.2 | 2026-10-13 | Side latch -> front catch-tab latch with housing ahead of the stop; front reserve 150 -> 140 mm | build_atlas_cad.py (solenoid hit chassis / other bay) |
| 0.1.2 | 2026-10-13 | Latch pin travel 8 -> 10 mm | cad_check.json (9.5 mm required) |
| 0.1.3 | 2026-10-13 | Scout camera module added (Pi Zero 2 W + Camera Module 3 on Board B J6); scout sensor power 1.8 -> 2.9 W | camera architecture decision |
| 0.1.3 | 2026-10-13 | No parameter change. Board C CV set to 8.34 V (< 8.40 V pack max). Contact-resistance trip set to 80 mOhm (INA226 82 mV range at 1 A) | cc_cv_charger_sim.py / Board C design |
