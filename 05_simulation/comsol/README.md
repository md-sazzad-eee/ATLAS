# COMSOL: dock contact Joule heating (verified 2026-10-13)

`ContactHeating.java` builds two 2D-axisymmetric COMSOL 6.4 models. Each uses Electric Currents and
Heat Transfer in Solids, coupled through Electromagnetic Heating. The script sweeps
I = 0.5–3 A × Rc = 10/30/100/200 mΩ.

```
comsol compile ContactHeating.java
comsol batch -inputfile ContactHeating.class -nosave
```

Outputs: `contact_heating_comsol.txt` (48 runs), `ContactHeating_validation.mph`, `ContactHeating_realistic.mph`.
(COMSOL's security settings block plain Java file I/O, so results are written through a COMSOL Table export.)

## 1. Validation against MATLAB PDE Toolbox: passes

The validation model uses the same assumptions as `05_simulation/matlab/contact_heating.m`: pin and carrier PCB
only, all I²(Rc+Rpin) released at the tip, board edge adiabatic, h = 10 W/m²K, 40 °C ambient.

| Quantity | MATLAB | COMSOL |
|---|---|---|
| Thermal resistance θ (K/W) | 156 | 155.8–156.5 |
| Tmax at 1 A, 30 mΩ | 47.8 °C | 47.8 °C |
| Tmax at 2 A, 200 mΩ | 177.0 °C | 177.1 °C |
| Joule power vs I²R | — | matches to < 0.6 % (energy balance check) |

Two independent FE codes agree to within 0.5%.

## 2. Realistic model: adds the agent pad and PCB

Heat can now also leave through the agent side, and the pin's 20 mΩ is spread along the pin.
This roughly halves the thermal resistance:

| Case | θ (K/W) | Tmax |
|---|---|---|
| Design point: 1 A, 30 mΩ | 73 | **43.6 °C** |
| Pin rating: 2 A, 30 mΩ | 73 | 54.6 °C, at the PLA softening limit |
| Contaminated: 2 A, 200 mΩ | 75 | **105.7 °C**, above PETG/ASA (~80 °C) |
| Worn: 1.5 A, 100 mΩ | 74 | 60.0 °C |

## Conclusions (unchanged from Phase 1, now confirmed)

- At the 1 A design point the contacts run cool (44–48 °C) in either model.
- A contaminated contact still overheats the contact block. Board C must measure contact resistance
  (voltage drop at known current) and inhibit charging above about 100 mΩ.
- Print the contact block in PETG, ASA or nylon (not PLA). Even the realistic model reaches 54.6 °C at 2 A.
