import pytest
from pydantic import ValidationError

from atlas.allocation import AgentState, assign, eligibility
from atlas.evidence import EvidenceGraph, INVALID, REVIEW, VALID
from atlas.messages import AgentProfile, Health, Task, TaskResult
from atlas.mission import validate_plan

W = {"distance": 1.0, "time": 0.5, "energy": 2.0, "risk": 3.0}


def prof(aid, caps, soc=100.0, cap_Wh=14.8):
    return AgentProfile(agent_id=aid, capabilities=caps, max_speed_mps=0.2, max_payload_g=100,
                        battery_capacity_Wh=cap_Wh, battery_percent=soc, power_idle_W=2.0, power_drive_W=0.2,
                        task_time_s={"measure": 60, "inspect": 30})


def agent(aid, caps, soc=100.0, pos=(0, 0)):
    p = prof(aid, caps, soc)
    return AgentState(p, pos, (0, 0), p.battery_capacity_Wh * soc / 100)


def task(tid, kind="measure", caps=("acquire_sensor_data",), loc=(5, 5), pre=(), **c):
    return Task(task_id=tid, goal=kind, kind=kind, required_capabilities=list(caps), location=loc,
                preconditions=list(pre), constraints=c, timeout_s=300)


# ---------------------------------------------------------------- allocation
def test_capability_mismatch_is_ineligible():
    ok, why = eligibility(agent("scout", ["inspect_visual_markers"]), task("t"), 25)
    assert not ok and "missing" in why


def test_battery_reserve_blocks_assignment():
    low = agent("r", ["acquire_sensor_data"], soc=25.5)       # just above reserve, cannot afford task
    ok, why = eligibility(low, task("t", loc=(20, 20)), 25)
    assert not ok and "battery" in why


def test_cost_never_overrides_hard_constraint():
    near_but_incapable = agent("a", ["inspect_visual_markers"], pos=(5, 5))
    far_but_capable = agent("b", ["acquire_sensor_data"], pos=(0, 0))
    out, _ = assign({"a": near_but_incapable, "b": far_but_capable}, [task("t")], W, 25)
    assert out == {"t": "b"}


def test_unassignable_task_reports_reason():
    out, reasons = assign({"a": agent("a", ["x"])}, [task("t")], W, 25)
    assert out == {} and "t" in reasons


def test_faulted_agent_never_assigned():
    a = agent("a", ["acquire_sensor_data"]); a.profile.health = Health.FAULT
    assert not eligibility(a, task("t"), 25)[0]


def test_pinned_task_only_to_its_agent():
    a, b = agent("a", ["acquire_sensor_data"]), agent("b", ["acquire_sensor_data"])
    t = task("chk", pinned_agent="b")
    out, _ = assign({"a": a, "b": b}, [t], W, 25)
    assert out == {"chk": "b"}


# ---------------------------------------------------------------- plan validation (AI proposals)
def test_validate_plan_rejects_ineligible_and_cycles():
    agents = {"a": agent("a", ["inspect_visual_markers"]), "b": agent("b", ["acquire_sensor_data"])}
    t1, t2 = task("t1", pre=["t2"]), task("t2", pre=["t1"])
    errs = validate_plan([t1, t2], {"t1": "a"}, agents, 25)
    assert any("cycle" in e for e in errs)
    assert any("t1 -> a" in e for e in errs)


def test_validate_plan_rejects_unknown_precondition_and_double_booking():
    agents = {"b": agent("b", ["acquire_sensor_data"])}
    errs = validate_plan([task("t1", pre=["ghost"]), task("t2")], {"t1": "b", "t2": "b"}, agents, 25)
    assert any("unknown precondition" in e for e in errs)
    assert any("more than one" in e for e in errs)


def test_valid_plan_passes():
    agents = {"b": agent("b", ["acquire_sensor_data"])}
    assert validate_plan([task("t1")], {"t1": "b"}, agents, 25) == []


# ---------------------------------------------------------------- messages
def test_malformed_messages_rejected():
    with pytest.raises(ValidationError):
        prof("x", [], soc=140)
    with pytest.raises(ValidationError):
        TaskResult(task_id="t", agent_id="a", status="success", t_start=10, t_end=5)
    with pytest.raises(ValidationError):
        TaskResult(task_id="t", agent_id="a", status="probably fine", t_start=0, t_end=5)


def test_inconclusive_is_a_valid_outcome():
    r = TaskResult(task_id="t", agent_id="a", status="inconclusive", t_start=0, t_end=1)
    assert r.status.value == "inconclusive"


# ---------------------------------------------------------------- evidence graph
def build_graph():
    g = EvidenceGraph()
    g.add_calibration("cal0", "r", 0)
    g.add_calibration("cal1", "r", 100)
    g.add_measurement("M1", 1.2, "cal0", "S1", "r", 10)
    g.add_measurement("M2", 3.4, "cal1", "S2", "r", 110)
    g.add_derived("E1", "estimate", ["M1"], lambda v: v[0] * 2, station="S1")
    g.add_derived("E2", "estimate", ["M2"], lambda v: v[0], station="S2")
    g.add_derived("C1", "conclusion", ["E1"], lambda v: v[0] > 2, station="S1")
    g.add_derived("C12", "conclusion", ["E1", "E2"], lambda v: v[0] < v[1], station="S1+S2")
    return g


def test_blueprint_example_M1_E1_C1():
    g = build_graph()
    rep = g.invalidate("M1", "calibration invalid")
    assert g.node("M1").status == INVALID
    assert g.node("E1").status == REVIEW and g.node("C1").status == REVIEW and g.node("C12").status == REVIEW
    assert g.node("M2").status == VALID and g.node("E2").status == VALID     # untouched branch
    assert rep["redo_measurements"] == ["M1"]
    assert set(rep["affected_conclusions"]) == {"C1", "C12"}


def test_failed_calibration_invalidates_only_its_epoch():
    g = build_graph()
    rep = g.invalidate("cal1", "check failed")
    assert rep["redo_measurements"] == ["M2"]
    assert g.stations_to_remeasure(rep) == ["S2"]
    assert g.node("C1").status == VALID and g.node("C12").status == REVIEW


def test_measurement_under_bad_calibration_starts_under_review():
    g = EvidenceGraph()
    g.add_calibration("c", "r", 0, passed=False)
    g.add_measurement("m", 1.0, "c", "S", "r", 1)
    assert g.node("m").status == REVIEW


def test_duplicate_and_dangling_ids_rejected():
    g = build_graph()
    with pytest.raises(KeyError):
        g.add_measurement("M1", 0, "cal0", "S", "r", 0)
    with pytest.raises(KeyError):
        g.add_derived("X", "estimate", ["nope"], lambda v: v)
