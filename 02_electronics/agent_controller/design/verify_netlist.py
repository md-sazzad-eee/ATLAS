#!/usr/bin/env python3
"""Compare the KiCad-exported netlist with board_b_design.py (pin-for-pin)."""
import pathlib, re, sys
sys.path.insert(0, str(pathlib.Path(__file__).parent))
import board_b_design as b

net_file = pathlib.Path(__file__).parents[1] / "atlas_agent_ctrl.net"
txt = net_file.read_text()
kicad = {}
body = txt[txt.index("(nets"):]
for blk in re.split(r"\n\s*\(net\b", body)[1:]:
    name = re.search(r'\(name "([^"]*)"\)', blk).group(1).lstrip("/")
    nodes = re.findall(r'\(ref "([^"]+)"\)\s*\(pin "([^"]+)"\)', blk)
    pins = frozenset((r, p) for r, p in nodes if not r.startswith("#"))
    if pins:
        kicad[pins] = name
spec = {frozenset(v): k for k, v in b.NETS.items() if not k.startswith("NC_")}
missing = [n for s, n in spec.items() if s not in kicad]
extra = [n for s, n in kicad.items() if s not in spec and len(s) > 1]
singles = [n for s, n in kicad.items() if len(s) == 1 and not n.startswith("unconnected")]
print(f"spec nets: {len(spec)}  kicad multi-pin nets: {sum(len(s) > 1 for s in kicad)}")
print("nets in spec not reproduced exactly:", missing or "none")
print("unexpected kicad nets:", extra or "none")
print("single-pin named nets:", singles or "none")
sys.exit(1 if missing or extra else 0)
