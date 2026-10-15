#!/usr/bin/env python3
"""Monte-Carlo comparison of task strategies (blueprint §10-§11).

Each run draws one random fault set (sensor drift, agent failure, comm loss, battery
states, docking misses) and replays it under every strategy, so the comparison is paired.

    python3 run_experiments.py --runs 300
Outputs in results/: runs.csv, summary.csv, summary.md, strategy_comparison.png
"""
import argparse
import copy
import csv
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from atlas.mission import STRATEGIES          # noqa: E402
from atlas.params import load_params          # noqa: E402
from atlas.sim import Faults, FleetSim        # noqa: E402

OUT = pathlib.Path(__file__).resolve().parent / "results"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=300)
    ap.add_argument("--seed", type=int, default=2026)
    args = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    P = load_params()
    rng = np.random.default_rng(args.seed)
    rows = []
    for k in range(args.runs):
        f = Faults.random(rng, ["atlas_scout_01", "atlas_research_02"], "atlas_research_02")
        seed = int(rng.integers(1 << 31))
        for s in STRATEGIES:
            m = FleetSim(s, copy.deepcopy(f), seed=seed, P=P).run()
            m["run"] = k
            rows.append(m)
    keys = list(rows[0].keys())
    with (OUT / "runs.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    def agg(sel, label):
        out = []
        for s in STRATEGIES:
            r = [x for x in rows if x["strategy"] == s and sel(x)]
            if not r:
                continue
            a = lambda key: np.array([x[key] for x in r], float)
            out.append({"subset": label, "strategy": s, "n": len(r),
                        "accuracy_mean": a("accuracy").mean(),
                        "wrong_conclusions_mean": a("wrong").mean(),
                        "runs_with_wrong_conclusion_pct": 100 * (a("wrong") > 0).mean(),
                        "missing_conclusions_mean": a("missing").mean(),
                        "mission_time_min_mean": a("mission_time_s").mean() / 60,
                        "recovery_measurements_mean": a("recovery_measurements").mean(),
                        "energy_Wh_mean": a("energy_Wh").mean(),
                        "carrier_moved_unlatched": int(a("carrier_moved_unlatched").sum()),
                        "reserve_violations": int(a("reserve_violations").sum())})
        return out

    summ = (agg(lambda x: True, "all runs")
            + agg(lambda x: x["fault_bias"], "sensor drift drawn")
            + agg(lambda x: x["fault_fail"] != "", "agent failure drawn")
            + agg(lambda x: not x["fault_bias"] and x["fault_fail"] == "" and not x["fault_comm"], "no fault"))
    with (OUT / "summary.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(summ[0].keys()))
        w.writeheader()
        w.writerows(summ)
    lines = ["| subset | strategy | n | accuracy | runs w/ wrong concl. | missing | time (min) | re-measurements | energy (Wh) |",
             "|---|---|---|---|---|---|---|---|---|"]
    for r in summ:
        lines.append(f"| {r['subset']} | {r['strategy']} | {r['n']} | {r['accuracy_mean']:.3f} | "
                     f"{r['runs_with_wrong_conclusion_pct']:.1f}% | {r['missing_conclusions_mean']:.2f} | "
                     f"{r['mission_time_min_mean']:.1f} | {r['recovery_measurements_mean']:.2f} | {r['energy_Wh_mean']:.2f} |")
    inv = sum(r["carrier_moved_unlatched"] + r["reserve_violations"] for r in summ if r["subset"] == "all runs")
    lines.append(f"\nSafety invariants over all runs: carrier moved with unlatched agent + reserve violations = {inv}")
    (OUT / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    plot(summ)


def plot(summ):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sub = [r for r in summ if r["subset"] == "sensor drift drawn"]
    names = [r["strategy"] for r in sub]
    fig, ax = plt.subplots(1, 3, figsize=(11, 3.4))
    col = "#4a6fa5"
    ax[0].bar(names, [r["runs_with_wrong_conclusion_pct"] for r in sub], color=col)
    ax[0].set_ylabel("% runs with a wrong conclusion")
    ax[1].bar(names, [r["recovery_measurements_mean"] for r in sub], color=col)
    ax[1].set_ylabel("extra measurements per run")
    ax[2].bar(names, [r["mission_time_min_mean"] for r in sub], color=col)
    ax[2].set_ylabel("mission time (min)")
    for a in ax:
        a.tick_params(axis="x", rotation=20)
        a.spines[["top", "right"]].set_visible(False)
    fig.suptitle(f"Runs where sensor drift was drawn (n={sub[0]['n']})", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT / "strategy_comparison.png", dpi=150)


if __name__ == "__main__":
    main()
