"""Structured messages between the mission manager and agents (blueprint §7.1-7.2).

Agents never send free text that the planner has to interpret: every message is one of
these validated models. Malformed messages raise pydantic.ValidationError and are rejected.
"""
from __future__ import annotations

from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from pydantic import BaseModel, Field, field_validator


class Health(str, Enum):
    READY = "ready"
    BUSY = "busy"
    DEGRADED = "degraded"
    FAULT = "fault"
    OFFLINE = "offline"


class AgentProfile(BaseModel):
    """Advertised on connection. Must match the hardware actually installed."""
    agent_id: str
    capabilities: List[str]
    max_speed_mps: float = Field(gt=0, le=1.0)
    max_payload_g: float = Field(ge=0)
    battery_capacity_Wh: float = Field(gt=0)
    battery_percent: float = Field(ge=0, le=100)
    power_idle_W: float = Field(gt=0)            # electronics + sensors while working
    power_drive_W: float = Field(ge=0)           # extra while driving at max speed
    health: Health = Health.READY
    # per-capability expected execution time (s) and risk (0..1) for the cost function
    task_time_s: Dict[str, float] = {}
    task_risk: Dict[str, float] = {}


class TaskStatus(str, Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    RUNNING = "running"
    DONE = "done"
    UNAVAILABLE = "unavailable"     # no eligible agent: declared, not hidden
    CANCELLED = "cancelled"


class Task(BaseModel):
    task_id: str
    goal: str
    kind: str                                   # inspect | measure | calibration_check | recalibrate
    required_capabilities: List[str]
    location: Tuple[float, float]
    station: Optional[str] = None
    preconditions: List[str] = []               # task_ids that must be DONE first
    constraints: Dict[str, Any] = {}            # e.g. {'pinned_agent': 'atlas_research_02'}
    success_criteria: str = ""
    timeout_s: float = Field(gt=0)
    fallback: str = "return_to_carrier"
    status: TaskStatus = TaskStatus.PENDING
    assigned_to: Optional[str] = None
    attempts: int = 0


class ResultStatus(str, Enum):
    SUCCESS = "success"
    INCONCLUSIVE = "inconclusive"               # valid scientific outcome
    MEASUREMENT_INVALID = "measurement_invalid" # valid scientific outcome
    FAILED = "failed"
    ABORTED = "aborted"


class TaskResult(BaseModel):
    task_id: str
    agent_id: str
    status: ResultStatus
    t_start: float
    t_end: float
    measurements: Dict[str, float] = {}
    uncertainty: Dict[str, float] = {}
    calibration_epoch: Optional[int] = None     # provenance: which calibration this data relies on
    error_codes: List[str] = []
    files: List[str] = []

    @field_validator("t_end")
    @classmethod
    def _ordered(cls, v, info):
        if "t_start" in info.data and v < info.data["t_start"]:
            raise ValueError("t_end before t_start")
        return v
