"""Generate a label-driven KiCad schematic from an ATLAS board design spec.

Every symbol is placed on a non-overlapping grid (in spec order, so blocks stay together) and every
pin carries a local net label at its connection point, exactly like Board B. Unused pins on
NC_ nets get no-connect flags; nets with power-input pins but no power source get a PWR_FLAG.
The result is checked afterwards by exporting the netlist with kicad-cli and diffing it against the
spec pin-for-pin (verify_netlist.py), so the drawing cannot silently differ from the design file.

usage: python3 gen_schematic.py <design.py> <out.kicad_sch> "<title>" [rev]
"""
import importlib.util
import math
import os
import sys
import uuid as _uuid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sexpr
import symlib
from sexpr import Q

GRID = 2.54
CHAR_W = 1.0            # approximate label character width (mm) at 1.27 mm font


def uid(seed):
    return Q(str(_uuid.uuid5(_uuid.NAMESPACE_URL, "atlas/" + seed)))   # deterministic: stable diffs


def snap(v):
    return round(v / GRID) * GRID


def load_design(path):
    spec = importlib.util.spec_from_file_location("design", path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m


def rot(px, py, r):
    a = math.radians(r)
    return px * math.cos(a) - py * math.sin(a), px * math.sin(a) + py * math.cos(a)


def unit_pins(sym, unit):
    return [p for p in symlib.pins(sym) if p[0] in (0, unit)]


def bbox(sym, unit, r, label_len):
    """Extent of pins + labels around the origin (schematic coords, y down)."""
    xs, ys = [0.0], [0.0]
    for _, num, name, px, py, ang, ln, et in unit_pins(sym, unit):
        x, y = rot(px, py, r)
        xs.append(x); ys.append(-y)
        # label extends away from the body
        la = math.radians(ang + 180 + r)
        xs.append(x + math.cos(la) * label_len); ys.append(-(y + math.sin(la) * label_len))
    for g in sexpr.find(sym, "symbol"):
        for rect in sexpr.find(g, "rectangle"):
            s, e = sexpr.find1(rect, "start"), sexpr.find1(rect, "end")
            for (a, b) in ((float(s[1]), float(s[2])), (float(e[1]), float(e[2]))):
                x, y = rot(a, b, r); xs.append(x); ys.append(-y)
    return min(xs), min(ys), max(xs), max(ys)


def fp_board_only(fpid):
    """True when the footprint itself is excluded from the BOM (mounting holes, test pads)."""
    import check_design
    if not fpid or ":" not in fpid:
        return False
    lib, name = fpid.split(":")
    for d in check_design.FPDIRS:
        f = os.path.join(d, lib + ".pretty", name + ".kicad_mod")
        if os.path.exists(f):
            return "exclude_from_bom" in open(f).read()
    return False


def flag(sym, key):
    """in_bom / on_board as the library symbol declares them (test points, holes are not in the BOM)."""
    f = sexpr.find1(sym, key)
    return f[1] if f else "yes"


def lib_symbol_entry(lib_id, sym):
    s = [e for e in sym]
    s[1] = Q(lib_id)
    return s


def generate(design_path, out_path, title, rev="A", page_w=594.0):
    d = load_design(design_path)
    project = os.path.splitext(os.path.basename(out_path))[0]
    root_uuid = uid(project + "/root")
    lib_syms, placed, labels, ncs, flags = {}, [], [], [], []
    errors = []
    # ---------------------------------------------------------------- symbol instances to place
    items = []
    for ref, lib, val, fp, x, y, r, pins in d.P:
        lib = symlib.resolve(lib)
        sym = symlib.load(lib)
        lib_syms[lib] = sym
        units = sorted({p[0] for p in symlib.pins(sym) if p[0] > 0}) or [1]
        # only place units that carry at least one spec pin
        for u in units:
            if any(p[1] in pins or any(a in pins for a in [k for k, v in symlib.ALIASES.items() if p[1] in v])
                   for p in unit_pins(sym, u)):
                items.append((ref, lib, val, fp, r, pins, sym, u))
    # ---------------------------------------------------------------- grid packing
    x_cur, y_cur, row_h, margin = 25.4, 30.48, 0.0, 5.08
    for ref, lib, val, fp, r, pins, sym, u in items:
        longest = max([len(n) for n in pins.values()] + [3])
        bx0, by0, bx1, by1 = bbox(sym, u, r, longest * CHAR_W + 2)
        w, h = bx1 - bx0 + margin, by1 - by0 + margin + 6
        if x_cur + w > page_w - 25:
            x_cur, y_cur, row_h = 25.4, y_cur + row_h, 0.0
        ox, oy = snap(x_cur - bx0), snap(y_cur - by0 + 3)
        placed.append((ref, lib, val, fp, r, pins, sym, u, ox, oy))
        x_cur += w; row_h = max(row_h, h)
    page_h = y_cur + row_h + 25
    # ---------------------------------------------------------------- pin -> net labels
    nets_with_source, nets_with_sink, label_at = set(), {}, {}
    for ref, lib, val, fp, r, pins, sym, u, ox, oy in placed:
        by_loc = {}
        for _, num, name, px, py, ang, ln, et in unit_pins(sym, u):
            net = pins.get(num)
            if net is None:
                for k, alts in symlib.ALIASES.items():
                    if num in alts and k in pins:
                        net = pins[k]
            x, y = rot(px, py, r)
            loc = (round(ox + x, 2), round(oy - y, 2))
            by_loc.setdefault(loc, []).append((num, name, ang, et, net))
        for loc, plist in by_loc.items():
            nets = {p[4] for p in plist if p[4] is not None}
            if len(nets) > 1:
                errors.append(f"{ref}: stacked pins at {loc} map to different nets {nets}")
                continue
            if not nets:
                if all(p[3] == "no_connect" for p in plist):
                    continue
                errors.append(f"{ref}: pin(s) {[p[0] for p in plist]} not in spec"); continue
            net = nets.pop()
            if net.startswith("NC_"):
                ncs.append(loc); continue
            if loc in label_at and label_at[loc] != net:
                errors.append(f"{ref}: pin location {loc} collides with net {label_at[loc]}")
            label_at[loc] = net
            ang = [p[2] for p in plist][0]
            la = (ang + 180 + r) % 360
            labels.append((net, loc, la))
            for p in plist:
                if p[3] in ("power_out",):
                    nets_with_source.add(net)
                if p[3] == "power_in":
                    nets_with_sink.setdefault(net, loc)
    for net, loc in nets_with_sink.items():
        if net not in nets_with_source:
            flags.append((net, loc))
    if errors:
        raise SystemExit("schematic generation failed:\n  " + "\n  ".join(errors))
    # ---------------------------------------------------------------- write
    pwr = symlib.load("power:PWR_FLAG")
    root = ["kicad_sch", ["version", "20230121"], ["generator", "atlas_gen_schematic"],
            ["uuid", root_uuid], ["paper", "User", f"{page_w:.0f}", f"{max(page_h, 297):.0f}"],
            ["title_block", ["title", Q(title)], ["rev", Q(rev)], ["company", Q("ATLAS")],
             ["comment", "1", Q("Generated from " + os.path.basename(design_path) + " by gen_schematic.py - edit the spec, not this drawing")]]]
    ls = ["lib_symbols"] + [lib_symbol_entry(k, v) for k, v in sorted(lib_syms.items())]
    if flags:
        ls.append(lib_symbol_entry("power:PWR_FLAG", pwr))
    root.append(ls)

    def eff(hide=False, justify=None):
        e = ["effects", ["font", ["size", "1.27", "1.27"]]]
        if justify:
            e.append(["justify"] + justify)
        if hide:
            e.append("hide")
        return e

    for ref, lib, val, fp, r, pins, sym, u, ox, oy in placed:
        fpid = fp or symlib.prop(sym, "Footprint") or ""
        ds = symlib.prop(sym, "Datasheet") or ""
        desc = symlib.prop(sym, "Description") or symlib.prop(sym, "ki_description") or ""   # KiCad 8+ / 7
        inst_id = uid(f"{project}/{ref}/{u}")
        s = ["symbol", ["lib_id", Q(lib)], ["at", f"{ox:.2f}", f"{oy:.2f}", str(int(r))], ["unit", str(u)],
             ["in_bom", "no" if fp_board_only(fpid) else flag(sym, "in_bom")], ["on_board", flag(sym, "on_board")], ["dnp", "no"], ["uuid", inst_id],
             ["property", Q("Reference"), Q(ref), ["at", f"{ox:.2f}", f"{oy - 3:.2f}", "0"], eff()],
             ["property", Q("Value"), Q(val), ["at", f"{ox:.2f}", f"{oy + 3:.2f}", "0"], eff()],
             ["property", Q("Footprint"), Q(fpid), ["at", f"{ox:.2f}", f"{oy:.2f}", "0"], eff(hide=True)],
             ["property", Q("Datasheet"), Q(ds), ["at", f"{ox:.2f}", f"{oy:.2f}", "0"], eff(hide=True)],
             ["property", Q("Description"), Q(desc), ["at", f"{ox:.2f}", f"{oy:.2f}", "0"], eff(hide=True)]]
        for p in unit_pins(sym, u):
            s.append(["pin", Q(p[1]), ["uuid", uid(f"{project}/{ref}/{u}/pin{p[1]}")]])
        s.append(["instances", ["project", Q(project), ["path", Q("/" + root_uuid), ["reference", Q(ref)], ["unit", str(u)]]]])
        root.append(s)
    for i, (net, (x, y), a) in enumerate(labels):
        just = {0: ["left", "bottom"], 90: ["left", "bottom"], 180: ["right", "bottom"], 270: ["right", "bottom"]}[int(a) % 360]
        root.append(["label", Q(net), ["at", f"{x:.2f}", f"{y:.2f}", str(int(a))], eff(justify=just),
                     ["uuid", uid(f"{project}/label{i}/{net}/{x}/{y}")]])
    for i, (x, y) in enumerate(ncs):
        root.append(["no_connect", ["at", f"{x:.2f}", f"{y:.2f}"], ["uuid", uid(f"{project}/nc{i}")]])
    for i, (net, (x, y)) in enumerate(flags):
        ref = f"#FLG{i+1:02d}"
        fx, fy = snap(x) + 0.0, y
        s = ["symbol", ["lib_id", Q("power:PWR_FLAG")], ["at", f"{x:.2f}", f"{y:.2f}", "0"], ["unit", "1"],
             ["in_bom", "yes"], ["on_board", "yes"], ["dnp", "no"], ["uuid", uid(f"{project}/{ref}")],
             ["property", Q("Reference"), Q(ref), ["at", f"{x:.2f}", f"{y - 4:.2f}", "0"], eff(hide=True)],
             ["property", Q("Value"), Q("PWR_FLAG"), ["at", f"{x:.2f}", f"{y - 5:.2f}", "0"], eff(hide=True)],
             ["property", Q("Footprint"), Q(""), ["at", f"{x:.2f}", f"{y:.2f}", "0"], eff(hide=True)],
             ["property", Q("Datasheet"), Q(""), ["at", f"{x:.2f}", f"{y:.2f}", "0"], eff(hide=True)],
             ["pin", Q("1"), ["uuid", uid(f"{project}/{ref}/pin")]],
             ["instances", ["project", Q(project), ["path", Q("/" + root_uuid), ["reference", Q(ref)], ["unit", "1"]]]]]
        root.append(s)
    root.append(["sheet_instances", ["path", Q("/"), ["page", Q("1")]]])
    with open(out_path, "w") as fh:
        fh.write(sexpr.dump(root) + "\n")
    print(f"{out_path}: {len(placed)} symbol units, {len(labels)} labels, {len(ncs)} no-connects, {len(flags)} PWR_FLAGs")
    return out_path


if __name__ == "__main__":
    generate(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4] if len(sys.argv) > 4 else "A")
