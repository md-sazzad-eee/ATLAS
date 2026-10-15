"""KiCad symbol library access: load a symbol by lib_id (resolving `extends`), list its pins per unit."""
import os
import sexpr

SYMDIRS = [d for d in (os.environ.get("KICAD_SYMBOL_DIR"),
                       "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols",
                       "/usr/share/kicad/symbols") if d and os.path.isdir(d)]
EXTRA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "atlas_extra.kicad_sym")
_cache = {}
# pin names that differ between KiCad library versions (KiCad 10 name -> older names)
ALIASES = {"SH": ("S1",)}


def _lib(name):
    if name not in _cache:
        path = EXTRA if name == "atlas_extra" else next(
            (os.path.join(d, name + ".kicad_sym") for d in SYMDIRS if os.path.exists(os.path.join(d, name + ".kicad_sym"))), None)
        if path is None:
            raise FileNotFoundError(f"symbol library {name} not found in {SYMDIRS}")
        tree = sexpr.parse(open(path, encoding="utf-8").read())
        _cache[name] = {s[1]: s for s in sexpr.find(tree, "symbol")}
    return _cache[name]


# symbols that moved between KiCad library versions (KiCad 7 <-> 8+)
MOVED = {"Device:Q_NMOS_GSD": "Transistor_FET:Q_NMOS_GSD", "Device:Q_PMOS_GDS": "Transistor_FET:Q_PMOS_GDS",
         "Device:Q_PMOS_GSD": "Transistor_FET:Q_PMOS_GSD", "Device:Q_NMOS_GDS": "Transistor_FET:Q_NMOS_GDS"}
MOVED.update({v: k for k, v in list(MOVED.items())})


def load(lib_id):
    """Return the symbol s-expr (a deep copy, `extends` flattened) for 'Lib:Name'."""
    import copy
    lib, name = lib_id.split(":", 1)
    try:
        syms = _lib(lib)
    except FileNotFoundError:
        syms = {}
    if name not in syms:
        if lib_id in MOVED:
            return load_exact(MOVED[lib_id])
        raise KeyError(f"{lib_id} not in library")
    return _flatten(syms, name)


def resolve(lib_id):
    """The lib_id under which this KiCad installation actually has the symbol."""
    try:
        load_exact(lib_id); return lib_id
    except (KeyError, FileNotFoundError):
        return MOVED.get(lib_id, lib_id)


def load_exact(lib_id):
    lib, name = lib_id.split(":", 1)
    syms = _lib(lib)
    if name not in syms:
        raise KeyError(f"{lib_id} not in library")
    return _flatten(syms, name)


def _flatten(syms, name):
    import copy
    s = copy.deepcopy(syms[name])
    ext = sexpr.find1(s, "extends")
    if ext:
        parent = copy.deepcopy(syms[ext[1]])
        props = {p[1]: p for p in sexpr.find(s, "property")}
        out = [e for e in parent if not (isinstance(e, list) and e and e[0] == "property")]
        out[1] = sexpr.Q(name)
        pprops = [p for p in sexpr.find(parent, "property")]
        merged = [props.pop(p[1], p) for p in pprops] + list(props.values())
        for e in out:                                   # rename sub-units Parent_x_y -> Name_x_y
            if isinstance(e, list) and e and e[0] == "symbol":
                e[1] = sexpr.Q(name + e[1][len(ext[1]):])
        s = out[:2] + merged + out[2:]
    return s


def pins(sym):
    """[(unit, number, name, x, y, angle, length, etype)] — unit 0 = common to all units."""
    out = []
    for u in sexpr.find(sym, "symbol"):
        unit = int(u[1].rsplit("_", 2)[-2])
        for p in sexpr.find(u, "pin"):
            at = sexpr.find1(p, "at")
            ln = sexpr.find1(p, "length")
            out.append((unit, sexpr.find1(p, "number")[1], sexpr.find1(p, "name")[1],
                        float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0.0,
                        float(ln[1]) if ln else 0.0, p[1]))
    return out


def prop(sym, key):
    p = [x for x in sexpr.find(sym, "property") if x[1] == key]
    return p[0][2] if p else None


if __name__ == "__main__":
    import sys
    for lid in sys.argv[1:]:
        s = load(lid)
        ps = pins(s)
        print(f"== {lid}  fp={prop(s, 'Footprint')}  units={sorted({p[0] for p in ps})}")
        print("   " + "  ".join(f"{p[1]}:{p[2]}" + (f"(u{p[0]})" if p[0] > 1 else "") for p in sorted(ps, key=lambda p: (len(p[1]), p[1]))))
