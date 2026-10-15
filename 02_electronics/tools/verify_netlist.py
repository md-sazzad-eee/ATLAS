#!/usr/bin/env python3
"""Compare a KiCad-exported netlist with a board design spec, pin-for-pin.
usage: verify_netlist.py <design.py> <board.net>"""
import importlib.util, re, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import symlib


def main(design, net_file):
    spec = importlib.util.spec_from_file_location("d", design)
    b = importlib.util.module_from_spec(spec); spec.loader.exec_module(b)
    txt = open(net_file).read()
    kicad = {}
    body = txt[txt.index("(nets"):]
    for blk in re.split(r"\n\s*\(net\b", body)[1:]:
        name = re.search(r'\(name "([^"]*)"\)', blk).group(1).lstrip("/")
        nodes = re.findall(r'\(ref "([^"]+)"\)\s*\(pin "([^"]+)"\)', blk)
        pins = frozenset((r, p) for r, p in nodes if not r.startswith("#"))
        if pins:
            kicad[pins] = name
    rev = {a: k for k, al in symlib.ALIASES.items() for a in al}
    norm = lambda s: frozenset((r, rev.get(p, p)) for r, p in s)
    kicad = {norm(s): n for s, n in kicad.items()}
    spec_nets = {}
    for n, v in b.NETS.items():
        if n.startswith("NC_"):
            continue
        spec_nets[frozenset(v)] = n
    # hidden stacked pins (e.g. USB-C GND A12/B1/B12) come along in KiCad; compare on spec pins only
    missing, extra = [], []
    kicad_by_pin = {}
    for s, n in kicad.items():
        for rp in s:
            kicad_by_pin[rp] = s
    for s, n in spec_nets.items():
        groups = {kicad_by_pin.get(rp) for rp in s}       # every spec pin must sit in one and the same KiCad net
        if len(groups) != 1 or None in groups:
            missing.append(n)
    spec_all = {rp for s in spec_nets for rp in s}
    for s, n in kicad.items():
        named = s & spec_all
        if len(named) > 0 and not any(named <= t for t in spec_nets):
            extra.append(n)
    print(f"spec nets: {len(spec_nets)}  kicad nets with spec pins: {sum(1 for s in kicad if s & spec_all)}")
    print("nets in spec not reproduced exactly:", missing or "none")
    print("kicad nets merging spec nets:", extra or "none")
    return 1 if missing or extra else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
