"""Capability- and battery-aware task allocation (blueprint §8, Level 1).

Hard constraints decide ELIGIBILITY; the weighted cost
    J_ij = w_d*d_ij + w_t*T_ij + w_e*E_ij + w_r*R_ij
only ranks eligible robots. A cheap score can never buy back a violated constraint.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

from .messages import AgentProfile, Health, Task

INF = 1e12


@dataclass
class AgentState:
    profile: AgentProfile
    position: Tuple[float, float]
    home: Tuple[float, float]                   # where it must be able to return to (the carrier)
    battery_Wh: float
    busy_until_task: Optional[str] = None

    @property
    def soc_pct(self) -> float:
        return 100.0 * self.battery_Wh / self.profile.battery_capacity_Wh


def dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def task_energy_Wh(agent: AgentState, task: Task) -> Tuple[float, float, float]:
    """(energy to do the task AND get home, travel distance, total time) — conservative."""
    p = agent.profile
    d_go = dist(agent.position, task.location)
    d_back = dist(task.location, agent.home)
    v = p.max_speed_mps
    t_drive = (d_go + d_back) / v
    t_work = p.task_time_s.get(task.kind, 60.0)
    E_J = (p.power_idle_W + p.power_drive_W) * t_drive + p.power_idle_W * t_work
    return E_J / 3600.0, d_go, t_drive + t_work


def eligibility(agent: AgentState, task: Task, reserve_pct: float) -> Tuple[bool, str]:
    p = agent.profile
    if p.health not in (Health.READY, Health.BUSY):
        return False, f"health={p.health.value}"
    pinned = task.constraints.get("pinned_agent")
    if pinned and pinned != p.agent_id:
        return False, f"task pinned to {pinned}"
    missing = set(task.required_capabilities) - set(p.capabilities)
    if missing:
        return False, f"missing capabilities {sorted(missing)}"
    E, _, _ = task_energy_Wh(agent, task)
    reserve = reserve_pct / 100.0 * p.battery_capacity_Wh
    if agent.battery_Wh - E < reserve:
        return False, f"battery: needs {E:.2f} Wh + reserve {reserve:.2f} Wh, has {agent.battery_Wh:.2f} Wh"
    return True, "ok"


def cost(agent: AgentState, task: Task, w: Dict[str, float]) -> float:
    E, d, T = task_energy_Wh(agent, task)
    R = agent.profile.task_risk.get(task.kind, 0.1)
    # normalise so weights are comparable: metres, minutes, Wh*10, risk*100
    return w["distance"] * d + w["time"] * T / 60.0 + w["energy"] * E * 10 + w["risk"] * R * 100


def assign(agents: Dict[str, AgentState], tasks: List[Task], weights: Dict[str, float],
           reserve_pct: float) -> Tuple[Dict[str, str], Dict[str, str]]:
    """Assign at most one task per free agent (optimal for this round, Hungarian method).

    Returns (assignment task_id -> agent_id, reasons task_id -> why it could not be assigned
    to ANY agent). Tasks with no eligible agent at all are reported so the manager can
    declare them unavailable instead of silently waiting forever.
    """
    a_ids = list(agents)
    if not tasks or not a_ids:
        return {}, {}
    M = np.full((len(tasks), len(a_ids)), INF)
    reasons: Dict[str, str] = {}
    for i, t in enumerate(tasks):
        why = []
        for j, aid in enumerate(a_ids):
            ok, r = eligibility(agents[aid], t, reserve_pct)
            if ok:
                M[i, j] = cost(agents[aid], t, weights)
            else:
                why.append(f"{aid}: {r}")
        if np.all(M[i] >= INF):
            reasons[t.task_id] = "; ".join(why)
    rows, cols = linear_sum_assignment(M)
    out = {}
    for i, j in zip(rows, cols):
        if M[i, j] < INF:
            out[tasks[i].task_id] = a_ids[j]
    return out, reasons
