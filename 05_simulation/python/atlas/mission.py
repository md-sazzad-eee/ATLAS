"""Deterministic mission manager (blueprint §7.3, §8).

The manager is a finite-state machine. Any externally proposed plan (e.g. from an LLM
planner) goes through `validate_plan` before it can be executed; the manager never runs
an unvalidated plan.

Strategies (for the §11 comparison):
  fixed          one allocation at t=0; no reassignment; ignores calibration faults
  reassign       event-triggered reassignment; recalibrates on a failed check but KEEPS
                 data already collected (no provenance)
  full_redo      reassign + recalibrate + re-measure EVERY station measured so far
  evidence       reassign + recalibrate + re-measure only stations whose evidence the
                 graph invalidated (data since the last passing calibration check)
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import networkx as nx

from .allocation import AgentState, assign, eligibility
from .evidence import VALID, EvidenceGraph
from .messages import Health, ResultStatus, Task, TaskResult, TaskStatus

STRATEGIES = ("fixed", "reassign", "full_redo", "evidence")

# mission-manager FSM states (blueprint §7.3 diagram)
RECEIVE_GOAL, PLAN, CHECK_PLAN, ASSIGN, MONITOR, VALIDATE, UPDATE_EVIDENCE, CONTAIN_FAULT, \
    CHECK_GOAL, REEVALUATE, REPORT = ("RECEIVE_GOAL", "PLAN", "CHECK_PLAN", "ASSIGN", "MONITOR", "VALIDATE",
                                      "UPDATE_EVIDENCE", "CONTAIN_FAULT", "CHECK_GOAL", "REEVALUATE", "REPORT")


# ------------------------------------------------------------------ plan validation
def validate_plan(tasks: List[Task], assignment: Dict[str, str], agents: Dict[str, AgentState],
                  reserve_pct: float) -> List[str]:
    """Return a list of violations (empty == plan may execute)."""
    errors = []
    ids = {t.task_id for t in tasks}
    g = nx.DiGraph()
    for t in tasks:
        g.add_node(t.task_id)
        for p in t.preconditions:
            if p not in ids:
                errors.append(f"{t.task_id}: unknown precondition {p}")
            g.add_edge(p, t.task_id)
    if not nx.is_directed_acyclic_graph(g):
        errors.append("task dependencies contain a cycle")
    by_id = {t.task_id: t for t in tasks}
    for tid, aid in assignment.items():
        if tid not in by_id:
            errors.append(f"assignment to unknown task {tid}")
            continue
        if aid not in agents:
            errors.append(f"{tid}: unknown agent {aid}")
            continue
        ok, why = eligibility(agents[aid], by_id[tid], reserve_pct)
        if not ok:
            errors.append(f"{tid} -> {aid}: {why}")
    per_agent = [a for a in assignment.values()]
    for a in set(per_agent):
        if per_agent.count(a) > 1:
            errors.append(f"{a} assigned more than one concurrent task")
    return errors


@dataclass
class StationTruth:
    name: str
    location: Tuple[float, float]


@dataclass
class MissionManager:
    strategy: str
    stations: Dict[str, Tuple[float, float]]
    ref_location: Tuple[float, float]
    ref_value: float
    spec_limit: float
    calib_threshold: float
    calib_every: int
    weights: Dict[str, float]
    reserve_pct: float
    timeout_factor: float = 2.0
    state: str = RECEIVE_GOAL
    tasks: Dict[str, Task] = field(default_factory=dict)
    evidence: EvidenceGraph = field(default_factory=EvidenceGraph)
    log: List[Tuple[float, str, str]] = field(default_factory=list)
    fixed_plan: Dict[str, List[str]] = field(default_factory=dict)   # 'fixed' strategy queues
    cal_node: Dict[str, str] = field(default_factory=dict)           # agent -> current calibration node
    meas_since_check: Dict[str, int] = field(default_factory=dict)
    naive_values: Dict[str, float] = field(default_factory=dict)     # non-evidence strategies' data store
    measure_tasks_created: int = 0
    unavailable: Dict[str, str] = field(default_factory=dict)
    recalibrating: set = field(default_factory=set)
    _ctr: itertools.count = field(default_factory=itertools.count)
    t_started: Dict[str, float] = field(default_factory=dict)

    def __post_init__(self):
        assert self.strategy in STRATEGIES

    # -------------------------------------------------------------- logging / fsm
    def _to(self, t, s, note=""):
        self.state = s
        self.log.append((round(t, 1), s, note))

    def _new_id(self, prefix):
        return f"{prefix}_{next(self._ctr)}"

    # -------------------------------------------------------------- planning
    def start(self, t: float, agents: Dict[str, AgentState], task_time: Dict[str, float]):
        self._to(t, RECEIVE_GOAL, f"characterise {len(self.stations)} stations vs spec {self.spec_limit}")
        self._to(t, PLAN)
        for aid, a in agents.items():
            cid = f"cal_{aid}_0"
            self.evidence.add_calibration(cid, aid, t, True)
            self.cal_node[aid] = cid
            self.meas_since_check[aid] = 0
        for s, loc in self.stations.items():
            self._add_task(f"inspect_{s}", "inspect", ["navigate_indoor", "inspect_visual_markers"], loc, s, [], task_time)
            self._add_measure(s, [f"inspect_{s}"], task_time)
        self._to(t, CHECK_PLAN)
        if self.strategy == "fixed":
            self._make_fixed_plan(agents)

    def _add_task(self, tid, kind, caps, loc, station, pre, task_time, **constraints):
        self.tasks[tid] = Task(task_id=tid, goal=f"{kind} {station or ''}".strip(), kind=kind,
                               required_capabilities=caps, location=loc, station=station,
                               preconditions=pre, constraints=constraints,
                               timeout_s=self.timeout_factor * (task_time.get(kind, 60) + 200))
        return tid

    def _add_measure(self, s, pre, task_time):
        self.measure_tasks_created += 1
        tid = self._new_id(f"measure_{s}")
        return self._add_task(tid, "measure", ["navigate_indoor", "acquire_sensor_data"],
                              self.stations[s], s, pre, task_time)

    def _make_fixed_plan(self, agents):
        """Static plan: each task goes to the cheapest capable agent at t=0, then never changes."""
        self.fixed_plan = {a: [] for a in agents}
        for t in self.tasks.values():
            best, bc = None, None
            for aid, a in agents.items():
                ok, _ = eligibility(a, t, self.reserve_pct)
                if ok:
                    from .allocation import cost
                    c = cost(a, t, self.weights)
                    if bc is None or c < bc:
                        best, bc = aid, c
            if best:
                self.fixed_plan[best].append(t.task_id)
                t.assigned_to = best
            else:
                t.status = TaskStatus.UNAVAILABLE
                self.unavailable[t.task_id] = "no eligible agent at plan time"

    # -------------------------------------------------------------- dispatch
    def ready_tasks(self) -> List[Task]:
        out = []
        for t in self.tasks.values():
            if t.status != TaskStatus.PENDING:
                continue
            if all(self.tasks[p].status == TaskStatus.DONE for p in t.preconditions):
                out.append(t)
        return out

    def dispatch(self, t: float, agents: Dict[str, AgentState], idle: List[str]) -> Dict[str, str]:
        """Return {agent_id: task_id} for idle agents."""
        self._propagate_unavailable()
        if not idle:
            return {}
        self._to(t, ASSIGN)
        out = {}
        if self.strategy == "fixed":
            for aid in idle:
                q = self.fixed_plan.get(aid, [])
                while q:
                    tk = self.tasks[q[0]]
                    if tk.status in (TaskStatus.DONE, TaskStatus.UNAVAILABLE, TaskStatus.CANCELLED):
                        q.pop(0)
                        continue
                    if tk.status == TaskStatus.PENDING and all(self.tasks[p].status == TaskStatus.DONE for p in tk.preconditions):
                        out[aid] = q.pop(0)
                    break
            # recalibration tasks are never created in the fixed strategy
        else:
            ready = self.ready_tasks()
            # 1) pinned self-checks / recalibration pre-empt normal work for their agent
            assignment = {}
            for aid in idle:
                pinned = [r for r in ready if r.constraints.get("pinned_agent") == aid and r.task_id not in assignment]
                if pinned:
                    a1, _ = assign({aid: agents[aid]}, pinned[:1], self.weights, self.reserve_pct)
                    assignment.update(a1)
            busy = set(assignment.values())
            # 2) an agent awaiting recalibration takes nothing else
            normal = {a: agents[a] for a in idle if a not in busy and a not in self.recalibrating}
            rest = [r for r in ready if r.task_id not in assignment and not r.constraints.get("pinned_agent")]
            a2, reasons = assign(normal, rest, self.weights, self.reserve_pct)
            assignment.update(a2)
            errs = validate_plan(list(self.tasks.values()), assignment, agents, self.reserve_pct)
            assert not errs, errs                      # allocator output must always validate
            out = {aid: tid for tid, aid in assignment.items()}
            self._declare_unavailable(t, ready, agents)
        for aid, tid in out.items():
            tk = self.tasks[tid]
            tk.status, tk.assigned_to = TaskStatus.RUNNING, aid
            tk.attempts += 1
            self.t_started[tid] = t
        self._to(t, MONITOR)
        return out

    def _propagate_unavailable(self):
        changed = True
        while changed:
            changed = False
            for tk in self.tasks.values():
                if tk.status == TaskStatus.PENDING and any(
                        self.tasks[p].status in (TaskStatus.UNAVAILABLE, TaskStatus.CANCELLED) for p in tk.preconditions):
                    tk.status = TaskStatus.UNAVAILABLE
                    self.unavailable[tk.task_id] = "precondition unavailable"
                    changed = True

    def _declare_unavailable(self, t, ready, agents):
        """A task is unavailable when NO non-faulted agent in the fleet has the capabilities."""
        alive = [a for a in agents.values() if a.profile.health not in (Health.FAULT,)]
        for tk in ready:
            pinned = tk.constraints.get("pinned_agent")
            capable = [a for a in alive if set(tk.required_capabilities) <= set(a.profile.capabilities)
                       and (not pinned or a.profile.agent_id == pinned)]
            if not capable:
                tk.status = TaskStatus.UNAVAILABLE
                self.unavailable[tk.task_id] = "no capable agent remaining"
                self.log.append((round(t, 1), CONTAIN_FAULT, f"{tk.task_id} unavailable"))
                # dependants are unavailable too
                for other in self.tasks.values():
                    if tk.task_id in other.preconditions and other.status == TaskStatus.PENDING:
                        other.status = TaskStatus.UNAVAILABLE
                        self.unavailable[other.task_id] = f"depends on unavailable {tk.task_id}"

    # -------------------------------------------------------------- events
    def on_agent_lost(self, t: float, aid: str, reason: str):
        self._to(t, REEVALUATE, f"{aid} lost: {reason}")
        for tk in self.tasks.values():
            if tk.assigned_to == aid and tk.status == TaskStatus.RUNNING:
                if self.strategy == "fixed":
                    tk.status = TaskStatus.UNAVAILABLE          # fixed plan has no recovery path
                    self.unavailable[tk.task_id] = f"assigned agent {aid} lost"
                else:
                    tk.status, tk.assigned_to = TaskStatus.PENDING, None
        if self.strategy == "fixed":
            for tid in self.fixed_plan.get(aid, []):
                if self.tasks[tid].status == TaskStatus.PENDING:
                    self.tasks[tid].status = TaskStatus.UNAVAILABLE
                    self.unavailable[tid] = f"assigned agent {aid} lost"
            self.fixed_plan[aid] = []
        self._to(t, MONITOR)

    def check_timeouts(self, t: float) -> List[Tuple[str, str]]:
        """Return [(task_id, agent_id)] that exceeded their timeout (to be cancelled on the agent)."""
        out = []
        for tid, tk in self.tasks.items():
            if tk.status == TaskStatus.RUNNING and t - self.t_started[tid] > tk.timeout_s:
                out.append((tid, tk.assigned_to))
                self._to(t, REEVALUATE, f"{tid} timed out on {tk.assigned_to}")
                if self.strategy == "fixed":
                    tk.status = TaskStatus.PENDING             # same agent keeps it
                    self.fixed_plan[tk.assigned_to].insert(0, tid)
                else:
                    tk.status, tk.assigned_to = TaskStatus.PENDING, None
        return out

    def on_result(self, t: float, r: TaskResult):
        tk = self.tasks[r.task_id]
        if tk.status != TaskStatus.RUNNING or tk.assigned_to != r.agent_id:
            return                                         # stale result from a cancelled assignment
        self._to(t, VALIDATE, f"{r.task_id} {r.status.value}")
        if r.status != ResultStatus.SUCCESS:
            tk.status = TaskStatus.PENDING if self.strategy != "fixed" else TaskStatus.UNAVAILABLE
            tk.assigned_to = None if self.strategy != "fixed" else tk.assigned_to
            self._to(t, MONITOR)
            return
        tk.status = TaskStatus.DONE
        aid = r.agent_id
        if tk.kind == "measure":
            self._to(t, UPDATE_EVIDENCE)
            mid = f"m_{r.task_id}"
            self.evidence.add_measurement(mid, r.measurements["value"], self.cal_node[aid], tk.station, aid, t)
            self._recompute_station(tk.station)
            self.naive_values[tk.station] = r.measurements["value"]
            self.meas_since_check[aid] += 1
            if self.meas_since_check[aid] >= self.calib_every:
                self._schedule_cal_check(aid)
        elif tk.kind == "calibration_check":
            self._handle_cal_check(t, aid, r.measurements["value"])
        elif tk.kind == "recalibrate":
            cid = self._new_id(f"cal_{aid}")
            self.evidence.add_calibration(cid, aid, t, True)
            self.cal_node[aid] = cid
            self.meas_since_check[aid] = 0
            self.recalibrating.discard(aid)
        self._to(t, CHECK_GOAL)
        self._to(t, MONITOR)

    def _schedule_cal_check(self, aid):
        if self.strategy == "fixed":
            return                                         # static plan has no self-checks
        if any(tk.kind == "calibration_check" and tk.constraints.get("pinned_agent") == aid
               and tk.status in (TaskStatus.PENDING, TaskStatus.RUNNING) for tk in self.tasks.values()):
            return
        tid = self._new_id(f"calcheck_{aid}")
        self._add_task(tid, "calibration_check", ["acquire_sensor_data"], self.ref_location, "REF", [], {},
                       pinned_agent=aid)
        self.meas_since_check[aid] = 0

    def schedule_final_checks(self, agents):
        """Before declaring the goal met, every agent with unchecked measurements closes its
        calibration epoch with a check (otherwise late drift would go undetected)."""
        for aid in agents:
            open_meas = [m for m in self.evidence.g.successors(self.cal_node[aid])
                         if self.evidence.node(m).kind == "measurement"]
            if open_meas:
                self._schedule_cal_check(aid)

    def _handle_cal_check(self, t, aid, value):
        ok = abs(value - self.ref_value) <= self.calib_threshold
        cid = self._new_id(f"chk_{aid}")
        if ok:
            # check passed: close the epoch, start a new calibration node
            self.evidence.add_calibration(cid, aid, t, True)
            self.cal_node[aid] = cid
            return
        self._to(t, CONTAIN_FAULT, f"calibration check failed on {aid}: {value:.3f} vs {self.ref_value}")
        if self.strategy == "fixed":
            return
        # recalibrate before further measurements
        self.recalibrating.add(aid)
        self._add_task(self._new_id(f"recal_{aid}"), "recalibrate", ["acquire_sensor_data"],
                       self.ref_location, "REF", [], {}, pinned_agent=aid)
        if self.strategy == "reassign":
            return
        if self.strategy == "full_redo":
            stations = sorted({n.meta["station"] for n in self.evidence.nodes("measurement")})
            for n in self.evidence.nodes("measurement"):
                n.status = "invalid"
        else:  # evidence-aware
            rep = self.evidence.invalidate(self.cal_node[aid], f"calibration check failed at t={t:.0f}")
            stations = self.evidence.stations_to_remeasure(rep)
            self.log.append((round(t, 1), UPDATE_EVIDENCE,
                             f"invalidated {len(rep['redo_measurements'])} measurements, "
                             f"{len(rep['affected_conclusions'])} conclusions under review"))
        for s in stations:
            # do not duplicate an already pending re-measure
            if not any(tk.kind == "measure" and tk.station == s and tk.status in (TaskStatus.PENDING, TaskStatus.RUNNING)
                       for tk in self.tasks.values()):
                recal = [tid for tid, tk in self.tasks.items() if tk.kind == "recalibrate"
                         and tk.constraints.get("pinned_agent") == aid and tk.status != TaskStatus.DONE]
                self._add_measure(s, recal[-1:], {})

    def _recompute_station(self, s):
        g = self.evidence
        valid = [n for n in g.nodes("measurement", VALID) if n.meta["station"] == s]
        if not valid:
            return
        latest = max(valid, key=lambda n: n.meta["t"])
        eid = self._new_id(f"est_{s}")
        g.add_derived(eid, "estimate", [latest.id], lambda v: v[0], station=s)
        g.add_derived(self._new_id(f"concl_{s}"), "conclusion", [eid],
                      lambda v, lim=self.spec_limit: bool(v[0] <= lim), station=s)

    # -------------------------------------------------------------- goal / report
    def conclusions(self) -> Dict[str, Optional[bool]]:
        """Final within-spec verdict per station (None = no valid conclusion)."""
        out: Dict[str, Optional[bool]] = {s: None for s in self.stations}
        if self.strategy == "evidence":
            for n in self.evidence.nodes("conclusion", VALID):
                s = n.meta["station"]
                out[s] = n.value  # later nodes overwrite earlier (ids increase with time)
            # ensure "latest valid" semantics
            latest = {}
            for n in self.evidence.nodes("conclusion", VALID):
                s = n.meta["station"]
                if s not in latest or int(n.id.rsplit("_", 1)[1]) > int(latest[s].id.rsplit("_", 1)[1]):
                    latest[s] = n
            out.update({s: n.value for s, n in latest.items()})
        elif self.strategy == "full_redo":
            for s in self.stations:
                valid = [n for n in self.evidence.nodes("measurement", VALID) if n.meta["station"] == s]
                if valid:
                    out[s] = bool(max(valid, key=lambda n: n.meta["t"]).value <= self.spec_limit)
        else:
            for s, v in self.naive_values.items():
                out[s] = bool(v <= self.spec_limit)
        return out

    def work_remaining(self) -> bool:
        return any(tk.status in (TaskStatus.PENDING, TaskStatus.RUNNING) for tk in self.tasks.values())
