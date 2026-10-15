"""Board C charger: averaged time-domain model of one bay's CC/CV loop (verifies the design before layout).

  buck (TPS54202, current-mode)   i_L tracks Kv*(Vtarget - Vout), 0 .. I_LIM, tau_i
  Vtarget                          from the FB node with the integrator's current injection:
                                   (Vout-Vfb)/Rt + (Vcc-Vd-Vfb)/Rinj = Vfb/Rb, Vfb = 0.596 V
  CC integrator (LMV358)           dVcc/dt = (ISNS - ISET)/(Ri*Cf), clamped 0 .. 3.3 V
  path to the cell                 shunt + switch + 2 contacts + wiring + agent Q2 + SS34 + pack Rint
  2S Li-ion                        OCV(SOC) table, 2.5 Ah
Run: python3 cc_cv_charger_sim.py  -> results/cc_cv_charger.json + .png
"""
import json, pathlib
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
VREF, RT, RB, RINJ, RI, CF, VD = 0.596, 130e3, 10.0e3, 10e3, 15e3, 100e-9, 0.25
COUT, I_LIM, TAU_I, KV = 44e-6, 2.5, 4e-6, 6.0
R_PATH = 0.020 + 0.060 + 2 * 0.030 + 0.020 + 0.050 + 0.080      # shunt, FET, contacts, wire, agent Q2, diode slope
VF_DIODE, R_INT = 0.30, 0.12
SOC_PTS = np.array([0, 0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0])
OCV_PTS = 2 * np.array([3.0, 3.35, 3.5, 3.6, 3.7, 3.8, 3.95, 4.05, 4.18])


def vtarget(vcc):
    inj = max(0.0, (vcc - VD - VREF) / RINJ)
    return RT * (VREF / RB - inj) + VREF


def run(t_end, soc0, iset_fn, dt=0.25e-6, rc_extra=0.0, soc_speedup=1.0, decim=200):
    vout, il, vcc, soc = 0.0, 0.0, 0.0, soc0
    T, I, V, VC, S = [], [], [], [], []
    n = int(t_end / dt)
    for k in range(n):
        t = k * dt
        ocv = np.interp(soc, SOC_PTS, OCV_PTS)
        r = R_PATH + R_INT + rc_extra
        i = max(0.0, (vout - VF_DIODE - ocv) / r)
        icmd = min(max(KV * (vtarget(vcc) - vout), 0.0), I_LIM)
        il += dt * (icmd - il) / TAU_I
        vout += dt * (il - i) / COUT
        vcc = min(max(vcc + dt * (i * 1.0 - iset_fn(t)) / (RI * CF), 0.0), 3.3)   # ISNS = 1 V/A
        soc = min(1.0, soc + dt * soc_speedup * i / (2.5 * 3600))
        if k % decim == 0:
            T.append(t); I.append(i); V.append(vout); VC.append(vcc); S.append(soc)
    return np.array(T), np.array(I), np.array(V), np.array(VC), np.array(S)


def settle(T, I, target, tol=0.02, t0=0.0):
    bad = np.where((T >= t0) & (np.abs(I - target) > tol * max(target, 0.05)))[0]
    return float(T[bad[-1]] - t0) if len(bad) else 0.0


if __name__ == "__main__":
    res = {}
    # 1. connect to a half-charged pack, ISET = 1.0 A (worst inrush: buck already regulating at 8.34 V)
    T, I, V, VC, S = run(0.04, 0.4, lambda t: 1.0)
    res["startup_1A"] = {"peak_A": round(float(I.max()), 3), "final_A": round(float(I[-1]), 3),
                         "settle_2pct_ms": round(settle(T, I, 1.0) * 1e3, 2),
                         "cv_setpoint_V": round(vtarget(0.0), 3)}
    curves = {"startup": (T, I)}
    # 2. ISET step 0.5 -> 1.0 A (contact check -> full charge)
    T2, I2, *_ = run(0.04, 0.4, lambda t: 0.5 if t < 0.02 else 1.0)
    over = float(I2[T2 > 0.02].max())
    res["step_0p5_to_1A"] = {"I_before": round(float(I2[(T2 > 0.015) & (T2 < 0.02)].mean()), 3),
                             "overshoot_pct": round((over - 1.0) * 100, 2), "settle_2pct_ms": round(settle(T2, I2, 1.0, t0=0.02) * 1e3, 2)}
    curves["step"] = (T2, I2)
    # 3. near full: CC -> CV hand-over and taper (SOC accelerated 2e4x so a 30 min taper fits in 0.1 s)
    T3, I3, V3, VC3, S3 = run(0.12, 0.85, lambda t: 1.0, dt=0.5e-6, soc_speedup=2.0e4)
    res["cc_to_cv"] = {"soc_at_cv_entry": round(float(S3[np.argmax(I3 < 0.98)]), 3) if (I3 < 0.98).any() else None,
                       "pack_terminal_max_V": round(float((V3 - VF_DIODE - I3 * (R_PATH)).max()), 3),
                       "final_current_A": round(float(I3[-1]), 3), "cv_out_V": round(float(V3[-1]), 3)}
    curves["taper"] = (S3, I3)
    # 4. bad contact (+170 mOhm): CC still holds 1 A
    T4, I4, *_ = run(0.04, 0.4, lambda t: 1.0, rc_extra=0.17)
    res["bad_contact_170mohm"] = {"final_A": round(float(I4[-1]), 3), "peak_A": round(float(I4.max()), 3)}
    # 5. Bode-style check: loop gain crossover estimate from the linearised model
    gm_path = 1.0 / (R_PATH + R_INT)
    wc = (RT / RINJ) * gm_path / (RI * CF)
    res["cc_loop_crossover_kHz"] = round(wc / 2 / np.pi / 1e3, 2)
    # 6. quasi-static full charge from 10 % (the loop settles in ms, the pack in minutes)
    soc, t, t_cv, cv_soc = 0.10, 0.0, None, None
    while soc < 0.999 and t < 6 * 3600:
        ocv = np.interp(soc, SOC_PTS, OCV_PTS)
        i = min(1.0, max(0.0, (vtarget(0.0) - VF_DIODE - ocv) / (R_PATH + R_INT)))
        if i < 0.999 and t_cv is None:
            t_cv, cv_soc = t, soc
        if i < 0.05:                                     # agent / Board A terminate at C/50
            break
        soc += i * 10 / (2.5 * 3600); t += 10
    res["full_charge_from_10pct"] = {"cc_ends_at_soc": round(cv_soc, 3), "cc_minutes": round(t_cv / 60, 1),
                                      "terminate_at_soc": round(soc, 3), "total_minutes": round(t / 60, 1)}
    res["assumptions"] = "averaged model; TPS54202 current-mode loop approximated by Kv/tau_i; LMV358 ideal within rails"
    out = HERE / "results" / "cc_cv_charger.json"
    out.write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
        ax[0].plot(curves["startup"][0] * 1e3, curves["startup"][1]); ax[0].set(title="Connect at SOC 40 %, ISET 1 A", xlabel="ms", ylabel="A")
        ax[1].plot(curves["step"][0] * 1e3, curves["step"][1]); ax[1].set(title="ISET 0.5 -> 1.0 A", xlabel="ms")
        ax[2].plot(curves["taper"][0] * 100, curves["taper"][1]); ax[2].set(title="CC -> CV taper", xlabel="SOC %")
        for a in ax: a.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(HERE / "results" / "cc_cv_charger.png", dpi=120)
    except Exception as e:
        print("plot skipped:", e)
