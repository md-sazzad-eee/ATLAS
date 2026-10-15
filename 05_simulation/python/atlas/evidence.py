"""Evidence graph with provenance and dependency propagation (blueprint §8, Level 3).

Nodes:  calibration -> measurement -> estimate -> conclusion
Edges point from evidence to what depends on it. Each node has a status:
    valid | under_review | invalid
Invalidating a node marks every descendant `under_review` (it may still be true, but its
support is gone) and returns the minimum set of *measurements* that must be redone so that
every affected conclusion can be recomputed. Nothing here relies on a language model
"remembering" that a result was questionable: it is a graph traversal.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Set

import networkx as nx

VALID, REVIEW, INVALID = "valid", "under_review", "invalid"


@dataclass
class Node:
    id: str
    kind: str                     # calibration | measurement | estimate | conclusion
    value: object = None
    status: str = VALID
    meta: dict = None


class EvidenceGraph:
    def __init__(self):
        self.g = nx.DiGraph()

    # ---------------------------------------------------------------- building
    def _add(self, nid: str, kind: str, value=None, parents: Iterable[str] = (), **meta):
        if nid in self.g:
            raise KeyError(f"duplicate evidence id {nid}")
        for p in parents:
            if p not in self.g:
                raise KeyError(f"unknown parent {p} for {nid}")
        self.g.add_node(nid, node=Node(nid, kind, value, VALID, meta))
        for p in parents:
            self.g.add_edge(p, nid)
        return nid

    def add_calibration(self, cid: str, agent: str, t: float, passed: bool = True):
        nid = self._add(cid, "calibration", passed, agent=agent, t=t)
        if not passed:
            self.node(cid).status = INVALID
        return nid

    def add_measurement(self, mid: str, value: float, calibration: str, station: str, agent: str, t: float):
        nid = self._add(mid, "measurement", value, [calibration], station=station, agent=agent, t=t)
        if self.node(calibration).status != VALID:
            self.node(mid).status = REVIEW
        return nid

    def add_derived(self, nid: str, kind: str, parents: List[str], fn: Callable[[List[object]], object], **meta):
        """Estimate / conclusion computed from VALID parents only."""
        vals = [self.node(p).value for p in parents]
        self._add(nid, kind, fn(vals), parents, fn=fn, **meta)
        if any(self.node(p).status != VALID for p in parents):
            self.node(nid).status = REVIEW
        return nid

    # ---------------------------------------------------------------- queries
    def node(self, nid: str) -> Node:
        return self.g.nodes[nid]["node"]

    def nodes(self, kind: Optional[str] = None, status: Optional[str] = None) -> List[Node]:
        out = [d["node"] for _, d in self.g.nodes(data=True)]
        if kind:
            out = [n for n in out if n.kind == kind]
        if status:
            out = [n for n in out if n.status == status]
        return out

    def supporting_measurements(self, nid: str) -> Set[str]:
        return {a for a in nx.ancestors(self.g, nid) if self.node(a).kind == "measurement"}

    # ---------------------------------------------------------------- invalidation
    def invalidate(self, nid: str, reason: str) -> Dict[str, object]:
        """Mark `nid` invalid, propagate `under_review` to descendants.

        Returns {'invalidated': [...], 'under_review': [...], 'redo_measurements': [...],
                 'affected_conclusions': [...]}
        """
        n = self.node(nid)
        n.status = INVALID
        n.meta = {**(n.meta or {}), "invalid_reason": reason}
        invalid = {nid}
        # a failed calibration invalidates the measurements taken under it
        if n.kind == "calibration":
            for ch in self.g.successors(nid):
                if self.node(ch).kind == "measurement":
                    self.node(ch).status = INVALID
                    invalid.add(ch)
        review = set()
        for x in list(invalid):
            for d in nx.descendants(self.g, x):
                if d not in invalid and self.node(d).status == VALID:
                    self.node(d).status = REVIEW
                    review.add(d)
        conclusions = sorted(d for d in review if self.node(d).kind == "conclusion")
        redo = sorted(m for m in invalid if self.node(m).kind == "measurement")
        return {"invalidated": sorted(invalid), "under_review": sorted(review),
                "redo_measurements": redo, "affected_conclusions": conclusions}

    def stations_to_remeasure(self, report: Dict[str, object]) -> List[str]:
        return sorted({self.node(m).meta["station"] for m in report["redo_measurements"]})
