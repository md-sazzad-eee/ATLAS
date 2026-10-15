"""Check a board design spec against the KiCad libraries before anything is drawn:
every symbol exists, every spec pin exists on the symbol, every symbol pin is assigned (or the
part is a connector/mechanical), every footprint exists and has a pad for every spec pin.
usage: python3 check_design.py <path/to/board_x_design.py>"""
import importlib.util, os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import symlib

FPDIRS = [d for d in (os.environ.get("KICAD_FOOTPRINT_DIR"),
                      "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints",
                      "/usr/share/kicad/footprints") if d and os.path.isdir(d)]


def fp_pads(fpid):
    lib, name = fpid.split(":")
    for d in FPDIRS:
        f = os.path.join(d, lib + ".pretty", name + ".kicad_mod")
        if os.path.exists(f):
            return set(re.findall(r'\(pad "?([^" ]*)"? ', open(f).read()))
    return None


def load_spec(path):
    spec = importlib.util.spec_from_file_location("design", path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m


def check(path):
    d = load_spec(path)
    errors, warns = [], []
    for ref, lib, val, fp, x, y, rot, pins in d.P:
        try:
            sym = symlib.load(lib)
        except Exception as e:
            errors.append(f"{ref}: symbol {lib}: {e}"); continue
        spins = {p[1] for p in symlib.pins(sym)}
        for p in pins:
            if p not in spins and not any(a in spins for a in symlib.ALIASES.get(p, ())):
                errors.append(f"{ref} ({lib}): spec pin {p} not on symbol")
        unassigned = spins - set(pins) - {a for p in pins for a in symlib.ALIASES.get(p, ())}
        if unassigned and not lib.startswith(("Connector", "Mechanical")):
            warns.append(f"{ref} ({lib}): symbol pins not in spec: {sorted(unassigned)}")
        fpid = fp or symlib.prop(sym, "Footprint")
        if not fpid:
            errors.append(f"{ref}: no footprint"); continue
        pads = fp_pads(fpid)
        if pads is None:
            errors.append(f"{ref}: footprint {fpid} not found"); continue
        for p in pins:
            if p not in pads and not (p == "SH" and any(q.startswith("S") for q in pads)):
                errors.append(f"{ref}: pin {p} has no pad in {fpid}")
    return errors, warns


if __name__ == "__main__":
    e, w = check(sys.argv[1])
    for x in w: print("WARN ", x)
    for x in e: print("ERROR", x)
    print(f"{len(e)} errors, {len(w)} warnings")
    sys.exit(1 if e else 0)
