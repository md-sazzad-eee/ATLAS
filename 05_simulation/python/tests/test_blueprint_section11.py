"""Executable versions of the blueprint §11 acceptance tests (simulation level)."""
import copy
import numpy as np
import pytest

from atlas.sim import Faults, FleetSim, STATIONS

SOC = {"atlas_scout_01": 90, "atlas_research_02": 90}


def run(strategy, **kw):
    f = Faults(soc0=dict(SOC), **kw)
    sim = FleetSim(strategy, f, seed=3)
    return sim, sim.run()


def test_nominal_mission_completes_correctly():
    for s in ("reassign", "full_redo", "evidence"):
        _, m = run(s, dock_fail_prob=0.0)
        assert m["completed"] and m["missing"] == 0 and m["wrong"] == 0, (s, m)


def test_task_reassignment_on_scout_failure():
    """Scout dies early: research agent (slower, riskier inspector) takes over inspections."""
    sim, m = run("evidence", fail_agent="atlas_scout_01", fail_time=60, dock_fail_prob=0.0)
    assert m["missing"] == 0, m
    inspected_by = {t.assigned_to for t in sim.mgr.tasks.values() if t.kind == "inspect" and t.status.value == "done"}
    assert "atlas_research_02" in inspected_by


def test_fixed_plan_cannot_recover_from_failure():
    _, m = run("fixed", fail_agent="atlas_scout_01", fail_time=60, dock_fail_prob=0.0)
    assert m["missing"] > 0


def test_task_declared_unavailable_when_no_capable_agent():
    """Only the research agent can measure: if it dies, measurements are declared unavailable,
    the mission still terminates and both robots are accounted for."""
    sim, m = run("evidence", fail_agent="atlas_research_02", fail_time=60, dock_fail_prob=0.0)
    assert m["completed"]
    assert m["unavailable_tasks"] > 0 and m["missing"] > 0
    assert all("measure" in k or "calcheck" in k or "recal" in k for k in sim.mgr.unavailable)


def test_comm_loss_agent_stops_independently():
    f = Faults(soc0=dict(SOC), comm_agent="atlas_research_02", comm_start=250, comm_duration=120, dock_fail_prob=0.0)
    sim = FleetSim("evidence", f, seed=3)
    positions = []
    orig = sim._mission_tick

    def spy():
        orig()
        if 252 <= sim.t <= 368:
            positions.append(sim.agents["atlas_research_02"].state.position)
    sim._mission_tick = spy
    m = sim.run()
    assert len(set(positions)) == 1, "agent moved while its heartbeat was lost"
    assert m["completed"] and m["missing"] == 0


def test_battery_aware_planning_respects_reserve():
    """Research agent starts just above reserve: it must not be sent on work it cannot return from."""
    f = Faults(soc0={"atlas_scout_01": 90, "atlas_research_02": 25.2}, dock_fail_prob=0.0)
    sim = FleetSim("evidence", f, seed=3)
    m = sim.run()
    assert m["reserve_violations"] == 0
    assert m["min_soc_pct"] >= 25 * 0.5
    assert m["unavailable_tasks"] == 0 or m["missing"] > 0   # declared, not silently dropped


def test_measurement_fault_flags_dependent_conclusions():
    sim, m = run("evidence", bias_agent="atlas_research_02", bias_onset=150, dock_fail_prob=0.0)
    notes = " ".join(n for _, s, n in sim.mgr.log if s == "UPDATE_EVIDENCE")
    assert "invalidated" in notes and "conclusions under review" in notes
    assert m["wrong"] == 0 and m["missing"] == 0, m


def test_evidence_aware_redoes_less_than_full_redo():
    _, me = run("evidence", bias_agent="atlas_research_02", bias_onset=500, dock_fail_prob=0.0)
    _, mf = run("full_redo", bias_agent="atlas_research_02", bias_onset=500, dock_fail_prob=0.0)
    assert me["wrong"] == mf["wrong"] == 0
    assert me["recovery_measurements"] < mf["recovery_measurements"]
    assert me["mission_time_s"] < mf["mission_time_s"]


def test_carrier_never_moves_with_unlatched_agent_monte_carlo():
    rng = np.random.default_rng(7)
    for k in range(40):
        f = Faults.random(rng, ["atlas_scout_01", "atlas_research_02"], "atlas_research_02")
        f.dock_fail_prob = 0.3
        m = FleetSim("evidence", copy.deepcopy(f), seed=k).run()
        assert m["carrier_moved_unlatched"] == 0
        assert m["completed"]
