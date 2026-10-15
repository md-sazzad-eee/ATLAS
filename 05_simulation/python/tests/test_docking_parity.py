"""The Python docking FSM must reproduce the Stateflow chart sample-for-sample.

Traces are exported by 05_simulation/matlab/run_docking_tests.m (copied into fixtures/).
"""
import csv, pathlib
import pytest
from atlas.docking import DockingFSM, INPUTS, NAMES

FIX = pathlib.Path(__file__).parent / "fixtures" / "docking_traces"
FILES = sorted(FIX.glob("scenario_*.csv"))


@pytest.mark.parametrize("path", FILES, ids=[p.stem for p in FILES])
def test_python_twin_matches_stateflow(path):
    rows = list(csv.DictReader(path.open()))
    fsm = DockingFSM(retry_limit=3)
    mismatches = []
    for r in rows:
        out = fsm.step({k: float(r[k]) > 0.5 for k in INPUTS})
        exp = int(float(r["state_id"]))
        if out["state_id"] != exp or bool(out["move_permit"]) != (float(r["move_permit"]) > 0.5) \
                or float(out["drive_cmd"]) != float(r["drive_cmd"]):
            mismatches.append((r["t"], NAMES.get(exp), NAMES.get(out["state_id"]), r["drive_cmd"], out["drive_cmd"]))
    assert not mismatches, f"{len(mismatches)} mismatching samples, first: {mismatches[:5]}"


def test_traces_present():
    assert len(FILES) == 8
