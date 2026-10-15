"""Docking state machine — Python twin of 05_simulation/matlab/atlas_docking_fsm.slx.

Same states, transition order and timers as the Stateflow chart, so the simulator,
the MATLAB model and (later) the MCU firmware can be checked against one another.
Executed at a fixed 10 ms tick, like the chart.

AI planning never touches this: the mission manager can only *request* dock/undock.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict

IDLE, APPROACH, LATCHING, VERIFY, DOCKED, UNLATCHING, RELEASING, BACKOFF, FAULT = 0, 1, 2, 3, 4, 5, 6, 7, 9
NAMES = {IDLE: "IDLE", APPROACH: "APPROACH", LATCHING: "LATCHING", VERIFY: "VERIFY", DOCKED: "DOCKED",
         UNLATCHING: "UNLATCHING", RELEASING: "RELEASING", BACKOFF: "BACKOFF", FAULT: "FAULT"}
INPUTS = ("cmd_dock", "cmd_undock", "seated", "latch_locked", "id_ok", "comm_ok",
          "carrier_stationary", "agent_clear", "estop", "reset")


@dataclass
class DockingFSM:
    retry_limit: int = 3
    dt: float = 0.01
    t: float = 0.0
    state: int = -1                      # -1: not yet initialised (first tick runs default transition)
    t_entry: float = 0.0
    retries: int = 0
    drive_cmd: float = 0.0
    latch_open_cmd: bool = False
    move_permit: bool = False
    log: list = field(default_factory=list)

    # ------------------------------------------------------------ helpers
    def _after(self, sec: float) -> bool:
        return self.t - self.t_entry >= sec - 1e-9

    def _enter(self, s: int):
        self.state, self.t_entry = s, self.t
        if s == IDLE:
            self.drive_cmd, self.latch_open_cmd, self.move_permit = 0, False, False
        elif s == APPROACH:
            self.drive_cmd, self.latch_open_cmd, self.move_permit = 1, True, False
        elif s == LATCHING:
            self.drive_cmd, self.latch_open_cmd = 0, False
        elif s == VERIFY:
            self.drive_cmd = 0
        elif s == DOCKED:
            self.drive_cmd, self.latch_open_cmd = 0, False
        elif s == UNLATCHING:
            self.latch_open_cmd, self.move_permit = True, False
        elif s == RELEASING:
            self.drive_cmd = -1
        elif s == BACKOFF:
            self.latch_open_cmd, self.drive_cmd = True, 0
            self.retries += 1
        elif s == FAULT:
            self.drive_cmd, self.latch_open_cmd, self.move_permit = 0, False, False
        self.log.append((round(self.t, 3), NAMES[s]))

    def _exit(self, s: int):
        if s == DOCKED:
            self.move_permit = False

    def _go(self, s: int):
        self._exit(self.state)
        self._enter(s)

    # ------------------------------------------------------------ step
    def step(self, u: Dict[str, bool]) -> Dict[str, object]:
        u = {k: bool(u.get(k, False)) for k in INPUTS}
        if self.state == -1:
            self._enter(IDLE)
            self.t += self.dt
            return self.outputs()
        s = self.state
        # parent (ACTIVE) transition has priority over children, like Stateflow
        if s != FAULT and u["estop"]:
            self._go(FAULT)
        elif s == FAULT:
            if u["reset"] and not u["estop"]:
                self._go(IDLE)
        elif s == IDLE:
            if u["cmd_dock"] and u["carrier_stationary"]:
                self.retries = 0
                self._go(APPROACH)
        elif s == APPROACH:
            if u["seated"]:
                self._go(LATCHING)
            elif self._after(15):
                self._go(BACKOFF)
        elif s == LATCHING:
            if u["seated"] and u["latch_locked"]:
                self._go(VERIFY)
            elif (not u["seated"]) or self._after(2):
                self._go(BACKOFF)
        elif s == VERIFY:
            if u["seated"] and u["latch_locked"] and u["id_ok"] and u["comm_ok"]:
                self._go(DOCKED)
            elif self._after(3):
                self._go(BACKOFF)
        elif s == DOCKED:
            if u["cmd_undock"] and u["carrier_stationary"]:
                self._go(UNLATCHING)
            elif (not u["seated"]) or (not u["latch_locked"]):
                self._go(FAULT)
            else:
                self.move_permit = u["seated"] and u["latch_locked"]
        elif s == UNLATCHING:
            if not u["latch_locked"]:
                self._go(RELEASING)
            elif self._after(2):
                self._go(FAULT)
        elif s == RELEASING:
            if (not u["seated"]) and u["agent_clear"]:
                self._go(IDLE)
            elif self._after(10):
                self._go(FAULT)
        elif s == BACKOFF:
            if (not u["seated"]) and self._after(3) and self.retries >= self.retry_limit:
                self._go(FAULT)
            elif (not u["seated"]) and self._after(3) and self.retries < self.retry_limit:
                self._go(APPROACH)
            elif self._after(10):
                self._go(FAULT)
            else:
                self.drive_cmd = -1 if not u["latch_locked"] else 0
        self.t += self.dt
        return self.outputs()

    def outputs(self):
        return {"state_id": self.state, "drive_cmd": self.drive_cmd,
                "latch_open_cmd": self.latch_open_cmd, "move_permit": self.move_permit}
