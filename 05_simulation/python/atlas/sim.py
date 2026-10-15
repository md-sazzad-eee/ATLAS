"""Time-stepped fleet simulator: one carrier, two agents, fault injection (blueprint §10).

Abstract on purpose: no physics engine. It exists to test task logic, recovery and the
value of evidence-aware replanning before any hardware or Gazebo work.
Energy, speeds and power figures come from atlas_params.json so the simulation and the
MATLAB sizing describe the same robots.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .allocation import AgentState, dist
from .messages import AgentProfile, Health, ResultStatus, TaskResult
from .mission import MissionManager
from .params import load_params

# ------------------------------------------------------------------ scenario
STATIONS = {          # name: (x, y, true value)   spec limit 1.00; three are close to it
    "S_A": (4.0, 12.0, 0.70),
    "S_B": (9.0, 13.5, 0.86),
    "S_C": (15.0, 12.5, 0.93),
    "S_D": (18.0, 7.0, 0.97),
    "S_E": (16.0, 2.5, 1.04),
    "S_F": (3.0, 6.0, 1.15),
}
REF = ((10.0, 8.0), 0.80)   # reference station with a known value
SPEC_LIMIT = 1.00
DEPLOY_POINT = (10.0, 1.0)
HOME = (0.0, 0.0)


@dataclass
class Faults:
    bias_agent: Optional[str] = None
    bias_onset: float = math.inf
    bias_value: float = 0.10
    fail_agent: Optional[str] = None
    fail_time: float = math.inf
    comm_agent: Optional[str] = None
    comm_start: float = math.inf
    comm_duration: float = 0.0
    dock_fail_prob: float = 0.05
    soc0: Dict[str, float] = field(default_factory=dict)

    @staticmethod
    def random(rng: np.random.Generator, agents: List[str], research_agent: str) -> "Faults":
        f = Faults()
        if rng.random() < 0.5:
            f.bias_agent, f.bias_onset = research_agent, rng.uniform(150, 1200)
        if rng.random() < 0.25:
            f.fail_agent, f.fail_time = agents[rng.integers(len(agents))], rng.uniform(100, 1500)
        if rng.random() < 0.3:
            f.comm_agent = agents[rng.integers(len(agents))]
            f.comm_start, f.comm_duration = rng.uniform(100, 1500), rng.uniform(30, 600)
        f.soc0 = {a: rng.uniform(40, 100) for a in agents}
        return f


@dataclass
class SimAgent:
    state: AgentState
    mode: str = "docked"            # docked | undocking | idle | moving | working | returning | docking | failed | stopped
    task_id: Optional[str] = None
    target: Optional[Tuple[float, float]] = None
    work_left: float = 0.0
    t_task_start: float = 0.0
    bias: float = 0.0
    comm_ok: bool = True
    timer: float = 0.0
    dock_attempts: int = 0
    dock_failures: int = 0
    energy_used_Wh: float = 0.0
    min_soc: float = 100.0


def make_profiles(P) -> Dict[str, AgentProfile]:
    a = P["agent"]
    cap_Wh = a["battery"]["nominal_voltage"] * a["battery"]["capacity_mAh"] / 1000 * a["battery"]["usable_fraction"]
    e = a["electronics_power_W"]
    g, crr = P["environment"]["gravity"], P["environment"]["rolling_resistance_coeff"]
    p_drive = a["mass_loaded_kg"] * g * crr * a["max_speed_mps"] / (a["gearbox_efficiency"] * 0.6)
    base = e["mcu_and_radio"] + e["motor_drivers_idle"]
    scout = AgentProfile(agent_id="atlas_scout_01",
                         capabilities=["navigate_indoor", "map_environment", "detect_obstacles", "inspect_visual_markers"],
                         max_speed_mps=a["max_speed_mps"], max_payload_g=100, battery_capacity_Wh=cap_Wh,
                         battery_percent=100, power_idle_W=base + e["sensors_scout"], power_drive_W=p_drive,
                         task_time_s={"inspect": 30}, task_risk={"inspect": 0.05})
    research = AgentProfile(agent_id="atlas_research_02",
                            capabilities=["navigate_indoor", "acquire_sensor_data", "inspect_visual_markers"],
                            max_speed_mps=a["max_speed_mps"], max_payload_g=150, battery_capacity_Wh=cap_Wh,
                            battery_percent=100, power_idle_W=base + e["sensors_research"], power_drive_W=p_drive,
                            task_time_s={"inspect": 60, "measure": 60, "calibration_check": 30, "recalibrate": 90},
                            task_risk={"inspect": 0.25, "measure": 0.05})
    return {scout.agent_id: scout, research.agent_id: research}


class FleetSim:
    def __init__(self, strategy: str, faults: Faults, seed: int = 0, P=None, dt: float = 1.0, t_max: float = 3 * 3600):
        self.P = P or load_params()
        self.rng = np.random.default_rng(seed)
        self.dt, self.t_max, self.t = dt, t_max, 0.0
        self.f = faults
        profiles = make_profiles(self.P)
        self.research_id = "atlas_research_02"
        self.agents: Dict[str, SimAgent] = {}
        for aid, prof in profiles.items():
            soc = faults.soc0.get(aid, 100.0)
            prof.battery_percent = soc
            st = AgentState(prof, DEPLOY_POINT, DEPLOY_POINT, prof.battery_capacity_Wh * soc / 100)
            self.agents[aid] = SimAgent(st, min_soc=soc)
        m = self.P["mission"]
        self.mgr = MissionManager(strategy=strategy, stations={k: v[:2] for k, v in STATIONS.items()},
                                  ref_location=REF[0], ref_value=REF[1], spec_limit=SPEC_LIMIT,
                                  calib_threshold=0.05, calib_every=2, weights=m["cost_weights"],
                                  reserve_pct=m["battery_return_reserve_pct"], timeout_factor=m["task_timeout_factor"])
        self.events: List[Tuple[float, str]] = []
        self.phase = "transit_out"
        self.carrier_pos = HOME
        self.carrier_move_while_unlatched = 0
        self.reserve_violations = 0
        self.final_checks_scheduled = False
        self.sigma = 0.01
        a_p = self.P["agent"]["battery"]
        self.charge_W = a_p["max_voltage"] * self.P["carrier"]["agent_charge_current_A"] * 0.85

    # -------------------------------------------------------------- helpers
    def _states(self):
        return {a: s.state for a, s in self.agents.items()}

    def _drain(self, a: SimAgent, driving: bool):
        p = a.state.profile
        P = p.power_idle_W + (p.power_drive_W if driving else 0.0)
        e = P * self.dt / 3600
        a.state.battery_Wh = max(0.0, a.state.battery_Wh - e)
        a.energy_used_Wh += e
        a.state.profile.battery_percent = a.state.soc_pct
        a.min_soc = min(a.min_soc, a.state.soc_pct)

    def _move(self, a: SimAgent, target) -> bool:
        x, y = a.state.position
        d = dist((x, y), target)
        step = a.state.profile.max_speed_mps * self.dt
        if d <= step:
            a.state.position = tuple(target)
            return True
        a.state.position = (x + (target[0] - x) * step / d, y + (target[1] - y) * step / d)
        return False

    def _dock_attempt(self, a: SimAgent) -> bool:
        a.dock_attempts += 1
        ok = self.rng.random() > self.f.dock_fail_prob
        if not ok:
            a.dock_failures += 1
        return ok

    # -------------------------------------------------------------- main loop
    def run(self) -> dict:
        task_time = {}
        for prof in (s.state.profile for s in self.agents.values()):
            for k, v in prof.task_time_s.items():
                task_time[k] = max(task_time.get(k, 0), v)
        carrier_v = self.P["carrier"]["max_speed_mps"]
        while self.t < self.t_max:
            self._inject_faults()
            if self.phase == "transit_out":
                # carrier may only move with both agents docked and latched (move_permit)
                if any(a.mode not in ("docked", "failed") for a in self.agents.values()):
                    self.carrier_move_while_unlatched += 1
                d = dist(self.carrier_pos, DEPLOY_POINT)
                step = carrier_v * self.dt
                if d <= step:
                    self.carrier_pos = DEPLOY_POINT
                    self.phase = "deploy"
                    self.mgr.start(self.t, self._states(), task_time)
                    for a in self.agents.values():
                        a.mode, a.timer = "undocking", 10.0
                else:
                    self.carrier_pos = (self.carrier_pos[0] + (DEPLOY_POINT[0] - self.carrier_pos[0]) * step / d,
                                        self.carrier_pos[1] + (DEPLOY_POINT[1] - self.carrier_pos[1]) * step / d)
            elif self.phase in ("deploy", "mission"):
                self.phase = "mission"
                self._mission_tick()
            elif self.phase == "recover":
                self._recover_tick()
            elif self.phase == "transit_home":
                if any(a.mode not in ("docked", "failed", "stranded") for a in self.agents.values()):
                    self.carrier_move_while_unlatched += 1
                d = dist(self.carrier_pos, HOME)
                if d <= carrier_v * self.dt:
                    self.carrier_pos = HOME
                    break
                self.carrier_pos = (self.carrier_pos[0] - self.carrier_pos[0] * carrier_v * self.dt / d,
                                    self.carrier_pos[1] - self.carrier_pos[1] * carrier_v * self.dt / d)
            self.t += self.dt
        return self.metrics()

    def _inject_faults(self):
        f = self.f
        if f.fail_agent and self.t >= f.fail_time and self.agents[f.fail_agent].mode != "failed":
            a = self.agents[f.fail_agent]
            a.mode, a.task_id = "failed", None
            a.state.profile.health = Health.FAULT
            self.events.append((self.t, f"{f.fail_agent} hardware failure"))
            if self.phase == "mission":
                self.mgr.on_agent_lost(self.t, f.fail_agent, "hardware failure")
        if f.bias_agent and self.t >= f.bias_onset and self.agents[f.bias_agent].bias == 0.0 \
                and not getattr(self, "_bias_applied", False):
            self.agents[f.bias_agent].bias = f.bias_value
            self._bias_applied = True
            self.events.append((self.t, f"{f.bias_agent} sensor drift +{f.bias_value}"))
        if f.comm_agent:
            a = self.agents[f.comm_agent]
            on = f.comm_start <= self.t < f.comm_start + f.comm_duration
            if on and a.comm_ok:
                a.comm_ok = False
                self.events.append((self.t, f"{f.comm_agent} comm lost"))
            elif not on and not a.comm_ok:
                a.comm_ok = True
                self.events.append((self.t, f"{f.comm_agent} comm restored"))

    def _mission_tick(self):
        mgr = self.mgr
        # 1) agent physics / task progress
        for aid, a in self.agents.items():
            if a.mode == "failed":
                continue
            if not a.comm_ok:
                # independent safety: heartbeat lost -> agent stops on its own, keeps power
                self._drain(a, False)
                continue
            if a.mode == "undocking":
                a.timer -= self.dt
                self._drain(a, False)
                if a.timer <= 0:
                    a.mode = "idle"
            elif a.mode == "moving":
                self._drain(a, True)
                if self._move(a, a.target):
                    a.mode = "working"
            elif a.mode == "working":
                self._drain(a, False)
                a.work_left -= self.dt
                if a.work_left <= 0:
                    self._finish_task(aid, a)
            elif a.mode == "idle":
                self._drain(a, False)
                # battery-aware: below reserve (+5 % margin) -> go back and charge on the carrier
                if a.state.soc_pct < self.P["mission"]["battery_return_reserve_pct"] + 5:
                    a.mode, a.target = "to_charge", DEPLOY_POINT
                    self.events.append((self.t, f"{aid} returning to charge at {a.state.soc_pct:.0f}%"))
            elif a.mode == "to_charge":
                self._drain(a, True)
                if self._move(a, DEPLOY_POINT):
                    a.mode, a.timer = "charge_docking", 20.0
            elif a.mode == "charge_docking":
                self._drain(a, False)
                a.timer -= self.dt
                if a.timer <= 0:
                    a.timer = 0.0 if self._dock_attempt(a) else 25.0
                    if a.timer == 0.0:
                        a.mode = "charging"
            elif a.mode == "charging":
                a.state.battery_Wh = min(a.state.profile.battery_capacity_Wh,
                                         a.state.battery_Wh + self.charge_W * self.dt / 3600)
                a.state.profile.battery_percent = a.state.soc_pct
                if a.state.soc_pct >= 90:
                    a.mode, a.timer = "undocking", 10.0
            if a.state.soc_pct < self.P["mission"]["battery_return_reserve_pct"] * 0.5:
                self.reserve_violations += 1
        # 2) timeouts -> cancel on the agent
        for tid, aid in mgr.check_timeouts(self.t):
            a = self.agents.get(aid)
            if a and a.task_id == tid:
                a.task_id, a.mode = None, ("idle" if a.mode != "failed" else "failed")
        # 3) dispatch idle, reachable agents
        idle = [aid for aid, a in self.agents.items() if a.mode == "idle" and a.comm_ok and a.task_id is None]
        for aid, tid in mgr.dispatch(self.t, self._states(), idle).items():
            a, tk = self.agents[aid], mgr.tasks[tid]
            a.task_id, a.target, a.mode, a.t_task_start = tid, tk.location, "moving", self.t
            a.work_left = a.state.profile.task_time_s.get(tk.kind, 60.0)
        # 4) goal check
        if not mgr.work_remaining():
            if not self.final_checks_scheduled:
                self.final_checks_scheduled = True
                mgr.schedule_final_checks([a for a, s in self.agents.items() if s.mode != "failed"])
                if mgr.work_remaining():
                    return
            self.phase = "recover"
            for a in self.agents.values():
                if a.mode not in ("failed",):
                    a.mode, a.target = "returning", DEPLOY_POINT

    def _finish_task(self, aid, a: SimAgent):
        tk = self.mgr.tasks[a.task_id]
        meas, unc = {}, {}
        if tk.kind in ("measure", "calibration_check"):
            true = REF[1] if tk.kind == "calibration_check" else STATIONS[tk.station][2]
            meas["value"] = true + a.bias + self.rng.normal(0, self.sigma)
            unc["value"] = self.sigma
        if tk.kind == "recalibrate":
            a.bias = 0.0
        r = TaskResult(task_id=tk.task_id, agent_id=aid, status=ResultStatus.SUCCESS,
                       t_start=a.t_task_start, t_end=self.t, measurements=meas, uncertainty=unc)
        self.mgr.on_result(self.t, r)
        a.task_id, a.mode = None, "idle"
        a.state.home = DEPLOY_POINT

    def _recover_tick(self):
        done = True
        for a in self.agents.values():
            if a.mode in ("failed", "docked", "stranded"):
                continue
            done = False
            if not a.comm_ok:
                self._drain(a, False)
                continue
            if a.mode == "returning":
                self._drain(a, True)
                if self._move(a, DEPLOY_POINT):
                    a.mode, a.timer = "docking", 20.0
            elif a.mode == "docking":
                self._drain(a, False)
                a.timer -= self.dt
                if a.timer <= 0:
                    if self._dock_attempt(a):
                        a.mode = "docked"
                    elif a.dock_failures % 3 == 0:
                        a.mode = "stranded"           # 3 failures -> request human assistance
                        self.events.append((self.t, "docking failed 3x: assistance requested"))
                    else:
                        a.timer = 20.0 + 5.0          # back off and retry
        if done:
            self.phase = "transit_home"

    # -------------------------------------------------------------- metrics
    def metrics(self) -> dict:
        concl = self.mgr.conclusions()
        truth = {s: v[2] <= SPEC_LIMIT for s, v in STATIONS.items()}
        correct = sum(1 for s in STATIONS if concl[s] is not None and concl[s] == truth[s])
        wrong = sum(1 for s in STATIONS if concl[s] is not None and concl[s] != truth[s])
        missing = sum(1 for s in STATIONS if concl[s] is None)
        return {
            "strategy": self.mgr.strategy,
            "mission_time_s": self.t,
            "completed": self.t < self.t_max,
            "correct": correct, "wrong": wrong, "missing": missing,
            "accuracy": correct / len(STATIONS),
            "measure_tasks": self.mgr.measure_tasks_created,
            "recovery_measurements": self.mgr.measure_tasks_created - len(STATIONS),
            "unavailable_tasks": len(self.mgr.unavailable),
            "energy_Wh": sum(a.energy_used_Wh for a in self.agents.values()),
            "min_soc_pct": min(a.min_soc for a in self.agents.values()),
            "dock_attempts": sum(a.dock_attempts for a in self.agents.values()),
            "dock_failures": sum(a.dock_failures for a in self.agents.values()),
            "stranded": sum(a.mode == "stranded" for a in self.agents.values()),
            "failed_agents": sum(a.mode == "failed" for a in self.agents.values()),
            "carrier_moved_unlatched": self.carrier_move_while_unlatched,
            "reserve_violations": self.reserve_violations,
            "fault_bias": self.f.bias_agent is not None,      # drawn for this run (paired across strategies)
            "fault_fail": self.f.fail_agent or "",
            "fault_comm": self.f.comm_agent is not None,
        }
