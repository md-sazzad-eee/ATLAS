"""Generic ATLAS PCB flow (KiCad 7 or 10 pcbnew Python), generalised from Board B's build script.

  python3 build_pcb.py <board_cfg.py> place            footprints from the spec, nets from the schematic
                                                        netlist, edge connectors fixed, everything else
                                                        placed greedily next to what it connects to;
                                                        writes .kicad_pcb + Specctra .dsn (power class patched in)
  python3 build_pcb.py <board_cfg.py> import_ses <ses>  routed session -> board, GND pours, stitching
  python3 build_pcb.py <board_cfg.py> export_routed <dsn>
  python3 build_pcb.py <board_cfg.py> bridge            join isolated GND pour islands with vias
  python3 build_pcb.py <board_cfg.py> unconnected

A board config module defines: NAME, DESIGN (path), X0, Y0, W, H, HOLES, FIXED {ref: (x, y, rot)},
POWER_NETS, optional KEEPOUTS [(x1, y1, x2, y2)], OFFBOARD_OK {refs}, ANTENNA_REF (ESP32 at front edge).
"""
import importlib.util, json, math, os, re, sys
import pcbnew

MM = pcbnew.FromMM
FPDIRS = [d for d in (os.environ.get("KICAD_FOOTPRINT_DIR"),
                      "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints",
                      "/usr/share/kicad/footprints") if d and os.path.isdir(d)]
SKIP_NETS_FOR_PLACEMENT = {"GND", "VCC_3V3", "VCC_5V", "VBAT", "VIN", "VM_SW"}


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m


CFG_PATH = os.path.abspath(sys.argv[1])
cfg = load_module(CFG_PATH, "cfg")
HERE = os.path.dirname(CFG_PATH)
spec = load_module(os.path.join(HERE, cfg.DESIGN), "design")
PROJ = os.path.dirname(HERE)
PCB = os.path.join(PROJ, cfg.NAME + ".kicad_pcb")
DSN = os.path.join(PROJ, cfg.NAME + ".dsn")
NET = os.path.join(PROJ, cfg.NAME + ".net")
X0, Y0, W, H = cfg.X0, cfg.Y0, cfg.W, cfg.H
# signal class: 0.5 mm-pitch parts (INA226 MSOP-10) need 0.15 mm clearance; JLCPCB 2-layer minimum is 0.127 mm
TRACK, CLEAR = getattr(cfg, "TRACK", 0.25), getattr(cfg, "CLEARANCE", 0.2)
# LAYERS = 4: In1.Cu is a solid GND plane ("power" layer: no signals, every GND pin reaches it through a via
# the router places). In2.Cu is a POWER_PLANE plane, or a third signal layer when POWER_PLANE is None
LAYERS = getattr(cfg, "LAYERS", 2)
POWER_PLANE = getattr(cfg, "POWER_PLANE", "VCC_3V3")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import symlib


def default_fp(lib):
    return symlib.prop(symlib.load(lib), "Footprint")


def netlist_info():
    txt = open(NET).read()
    info = {}
    for blk in re.split(r"\(comp\s", txt)[1:]:
        ref = re.search(r'\(ref "([^"]+)"\)', blk).group(1)
        uids = [u for u in re.findall(r'\(tstamps "([^"]+)"\)', blk) if u != "/"]
        desc = re.search(r'\(description "([^"]*)"\)', blk)
        ds = re.search(r'\(name "Datasheet"\) "([^"]*)"', blk) or re.search(r'\(datasheet "([^"]*)"\)', blk)
        info[ref] = (uids[-1] if uids else None, desc.group(1) if desc else "", ds.group(1) if ds else "",
                     "exclude_from_bom" in blk, "exclude_from_board" in blk)
    nc = {}
    body = txt[txt.index("(nets"):]
    for blk in re.split(r"\n\s*\(net\b", body)[1:]:
        name = re.search(r'\(name "([^"]*)"\)', blk).group(1)
        if name.startswith("unconnected-("):
            for r, p in re.findall(r'\(ref "([^"]+)"\)\s*\(pin "([^"]+)"\)', blk):
                nc[(r, p)] = name
    return info, nc


def load_fp(fpid):
    lib, name = fpid.split(":")
    for d in FPDIRS:
        p = os.path.join(d, lib + ".pretty")
        if os.path.exists(os.path.join(p, name + ".kicad_mod")):
            fp = pcbnew.FootprintLoad(p, name)
            fp.SetFPID(pcbnew.LIB_ID(lib, name))
            return fp
    raise RuntimeError(f"footprint not found: {fpid}")


def outline(board, inset=0.0):
    i = inset
    pts = [(X0 + i, Y0 + i), (X0 + W - i, Y0 + i), (X0 + W - i, Y0 + H - i), (X0 + i, Y0 + H - i)]
    for a, b in zip(pts, pts[1:] + pts[:1]):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pcbnew.VECTOR2I(MM(a[0]), MM(a[1]))); s.SetEnd(pcbnew.VECTOR2I(MM(b[0]), MM(b[1])))
        s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(MM(0.1))
        board.Add(s)


def set_outline_inset(board, i):
    for sgm in [d for d in board.Drawings() if d.GetLayer() == pcbnew.Edge_Cuts]:
        a, b = sgm.GetStart(), sgm.GetEnd()
        mx, my = pcbnew.ToMM((a.x + b.x) // 2), pcbnew.ToMM((a.y + b.y) // 2)
        if abs(a.y - b.y) < 10:
            yv = Y0 + i if my < Y0 + H / 2 else Y0 + H - i
            sgm.SetStart(pcbnew.VECTOR2I(MM(X0 + i), MM(yv))); sgm.SetEnd(pcbnew.VECTOR2I(MM(X0 + W - i), MM(yv)))
        else:
            xv = X0 + i if mx < X0 + W / 2 else X0 + W - i
            sgm.SetStart(pcbnew.VECTOR2I(MM(xv), MM(Y0 + i))); sgm.SetEnd(pcbnew.VECTOR2I(MM(xv), MM(Y0 + H - i)))


def rules(board):
    ds = board.GetDesignSettings()
    ds.SetCopperLayerCount(LAYERS)
    ds.m_MinThroughDrill = MM(0.2)
    ds.m_HoleClearance = MM(0.15)
    ds.m_CopperEdgeClearance = MM(0.5)
    ns = ds.m_NetSettings
    try:                                                     # KiCad 8+
        d = ns.GetDefaultNetclass()
        d.SetTrackWidth(MM(TRACK)); d.SetClearance(MM(CLEAR)); d.SetViaDiameter(MM(0.6)); d.SetViaDrill(MM(0.3))
        pw = pcbnew.NETCLASS("Power")
        pw.SetTrackWidth(MM(0.5)); pw.SetClearance(MM(CLEAR));   # <15 V nets: signal clearance is enough
        pw.SetViaDiameter(MM(0.8)); pw.SetViaDrill(MM(0.4))
        ns.SetNetclass("Power", pw)
        for n in cfg.POWER_NETS:
            ns.SetNetclassPatternAssignment("/" + n, "Power")
    except AttributeError:                                   # KiCad 7: default class only; DSN is patched
        d = ns.m_DefaultNetClass
        d.SetTrackWidth(MM(TRACK)); d.SetClearance(MM(CLEAR)); d.SetViaDiameter(MM(0.6)); d.SetViaDrill(MM(0.3))


def strip_dsn_planes(dsn):
    """Drop the (plane ...) blocks so the router treats GND as an ordinary net and ties every GND pin with
    real tracks (cfg.GND_ROUTED). The pours are still added afterwards; they only add copper."""
    t = open(dsn).read()
    n = 0
    while True:
        i = t.find("(plane ")
        if i < 0:
            break
        depth, j = 0, i
        while True:
            if t[j] == "(":
                depth += 1
            elif t[j] == ")":
                depth -= 1
                if depth == 0:
                    break
            j += 1
        t = t[:i] + t[j + 1:]; n += 1
    open(dsn, "w").write(t)
    return n


def patch_dsn_power(dsn):
    """Put POWER_NETS into a 0.5 mm class in the Specctra file (version-independent)."""
    t = open(dsn).read()
    names = ['"/' + n + '"' if '"/' + n + '"' in t else "/" + n for n in cfg.POWER_NETS]
    names = [n for n in names if n in t]
    if not names or "(class Power" in t:
        return
    for n in names:                                          # remove from default class lists
        t = re.sub(r"(\(class kicad_default[^()]*?)\s" + re.escape(n) + r"(?=[\s)])", r"\1", t, count=1, flags=re.S)
    cls = ("    (class Power " + " ".join(names) +
           "\n      (circuit (use_via \"Via[0-1]_800:400_um\"))\n      (rule (width 500) (clearance %d)))\n" % round(CLEAR * 1000))
    i = t.rindex("(class kicad_default")
    depth, j = 0, i
    while True:
        if t[j] == "(":
            depth += 1
        elif t[j] == ")":
            depth -= 1
            if depth == 0:
                break
        j += 1
    t = t[:j + 1] + "\n" + cls + t[j + 1:]
    if "Via[0-1]_800:400_um" not in t.split("(network")[0]:
        t = t.replace("(padstack \"Via[0-1]_600:300_um\"", "(padstack \"Via[0-1]_800:400_um\"\n      (shape (circle F.Cu 800))\n      (shape (circle B.Cu 800))\n      (attach off)\n    )\n    (padstack \"Via[0-1]_600:300_um\"", 1)
        t = t.replace("(via \"Via[0-1]_600:300_um\"", "(via \"Via[0-1]_600:300_um\" \"Via[0-1]_800:400_um\"", 1)
    open(dsn, "w").write(t)


def fill(board):
    board.BuildConnectivity()
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())


def fanout_gnd(board, w=0.3, only=None):
    """Pre-routing GND fan-out: every top-side SMD GND pad gets a short track and a via into the B.Cu plane
    before the router runs, so no GND pin can be boxed in by signal tracks (Board C INA226 pin 7 was).
    Positions are checked against every other-net pad with KiCad's shape collision."""
    gnd = board.GetNetcodeFromNetname("/GND")
    clr = MM(CLEAR)
    others = [p for p in board.GetPads() if p.GetNetCode() != gnd]
    vias, n, skipped = [], 0, []
    edge = (MM(X0 + 1.0), MM(Y0 + 1.0), MM(X0 + W - 1.0), MM(Y0 + H - 1.0))
    keep = [z for z in board.Zones() if z.GetIsRuleArea()]
    sel = re.compile(only) if only else None          # e.g. r"^U" = ICs only
    todo = [(fp, p) for fp in board.GetFootprints() for p in fp.Pads()
            if p.GetNetCode() == gnd and not p.HasHole() and p.IsOnLayer(pcbnew.F_Cu)
            and (sel is None or sel.match(fp.GetReference()))]
    # adjacent GND pins of one part (e.g. TCA9534 A0..A2) are tied pad-to-pad first: one via then serves
    # the whole run, and a pin whose own fan-out has no room still reaches GND through its neighbour
    joined = 0
    for ref in {f.GetReference() for f, _ in todo}:
        gp = [p for f, p in todo if f.GetReference() == ref]
        for i in range(len(gp)):
            for j in range(i + 1, len(gp)):
                a, b = gp[i].GetPosition(), gp[j].GetPosition()
                if (a.x - b.x) ** 2 + (a.y - b.y) ** 2 <= MM(0.7) ** 2:
                    t = pcbnew.PCB_TRACK(board); t.SetStart(a); t.SetEnd(b); t.SetWidth(MM(0.25))
                    t.SetLayer(pcbnew.F_Cu); t.SetNetCode(gnd); board.Add(t); joined += 1
    if joined:
        print(f"GND pad-to-pad joins: {joined}")
    for fp, pad in todo:
        c, fc = pad.GetPosition(), fp.GetPosition()
        bb = pad.GetBoundingBox()
        half = max(bb.GetWidth(), bb.GetHeight()) / 2
        # preferred direction: away from the footprint centre, then the other three axes
        dx, dy = c.x - fc.x, c.y - fc.y
        dirs = [(1 if dx >= 0 else -1, 0), (0, 1 if dy >= 0 else -1)] if abs(dx) >= abs(dy) else [(0, 1 if dy >= 0 else -1), (1 if dx >= 0 else -1, 0)]
        dirs += [(-a, -b) for a, b in dirs]
        done = False
        for ux, uy in dirs:
            for extra in (0.55, 0.75, 1.0, 1.3):
                d = half + MM(extra)
                v = pcbnew.VECTOR2I(int(c.x + ux * d), int(c.y + uy * d))
                if not (edge[0] < v.x < edge[2] and edge[1] < v.y < edge[3]):
                    continue
                if any(z.Outline().Contains(v) for z in keep):
                    continue
                via_sh = pcbnew.SHAPE_CIRCLE(v, MM(0.3))
                seg = pcbnew.SHAPE_SEGMENT(c, v, MM(w))
                if any(o.GetEffectiveShape(pcbnew.F_Cu).Collide(via_sh, clr) or o.GetEffectiveShape(pcbnew.F_Cu).Collide(seg, clr)
                       or o.GetEffectiveShape(pcbnew.B_Cu).Collide(via_sh, clr) for o in others):
                    continue
                if any((q.x - v.x) ** 2 + (q.y - v.y) ** 2 < MM(0.8) ** 2 for q in vias):
                    continue
                t = pcbnew.PCB_TRACK(board); t.SetStart(c); t.SetEnd(v); t.SetWidth(MM(w)); t.SetLayer(pcbnew.F_Cu); t.SetNetCode(gnd)
                vi = pcbnew.PCB_VIA(board); vi.SetPosition(v); vi.SetWidth(MM(0.6)); vi.SetDrill(MM(0.3)); vi.SetNetCode(gnd)
                board.Add(t); board.Add(vi); vias.append(v); n += 1; done = True
                break
            if done:
                break
        if not done:
            skipped.append(f"{fp.GetReference()}.{pad.GetNumber()}")
    print(f"GND fan-out: {n} pads, no room for {skipped}")


def gnd_zone(board, layer, net="/GND"):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNetCode(board.GetNetcodeFromNetname(net))
    # pour clearance: 0.3 mm normally; boards with 0.5 mm-pitch parts need 0.2 mm or the pour cannot
    # reach a GND pin between two neighbours (Board C INA226 pin 7)
    z.SetLocalClearance(MM(getattr(cfg, "ZONE_CLEARANCE", 0.3))); z.SetMinThickness(MM(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)   # copper islands with no GND item are dropped
    ol = z.Outline(); ol.NewOutline()
    for x, y in [(X0 + .5, Y0 + .5), (X0 + W - .5, Y0 + .5), (X0 + W - .5, Y0 + H - .5), (X0 + .5, Y0 + H - .5)]:
        ol.Append(MM(x), MM(y))
    board.Add(z)


def rule_area(board, x1, y1, x2, y2, fills=True):
    for layer in ((pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu) if LAYERS == 4 else (pcbnew.F_Cu, pcbnew.B_Cu)):
        k = pcbnew.ZONE(board)
        k.SetIsRuleArea(True)
        k.SetDoNotAllowTracks(True); k.SetDoNotAllowVias(True)
        (getattr(k, "SetDoNotAllowZoneFills", None) or k.SetDoNotAllowCopperPour)(fills)   # KiCad 10 / 7
        k.SetLayer(layer)
        ol = k.Outline(); ol.NewOutline()
        for x, y in [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]:
            ol.Append(MM(x), MM(y))
        board.Add(k)


def fp_box(fp):
    """Courtyard bbox (mm) relative to the footprint origin at its current orientation."""
    poly = fp.GetCourtyard(pcbnew.F_CrtYd)
    if poly.OutlineCount():
        bb = poly.BBox()
    else:
        bb = fp.GetBoundingBox(False, False)
    p = fp.GetPosition()
    return (pcbnew.ToMM(bb.GetLeft() - p.x), pcbnew.ToMM(bb.GetTop() - p.y),
            pcbnew.ToMM(bb.GetRight() - p.x), pcbnew.ToMM(bb.GetBottom() - p.y))


def place():
    board = pcbnew.BOARD()
    if LAYERS == 4:
        board.SetCopperLayerCount(4)
        # GND_ROUTED: In1.Cu is routed too (GND as tracks on all layers); its GND pour is filled afterwards
        board.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_SIGNAL if getattr(cfg, "GND_ROUTED", False) else pcbnew.LT_POWER)
        board.SetLayerType(pcbnew.In2_Cu, pcbnew.LT_POWER if POWER_PLANE else pcbnew.LT_SIGNAL)
    outline(board, inset=0.4)
    rules(board)
    info, nc = netlist_info()
    nets = {}
    for n in spec.NETS:
        if not n.startswith("NC_"):
            ni = pcbnew.NETINFO_ITEM(board, "/" + n); board.Add(ni); nets[n] = ni
    rep = {"unmatched_spec_pins": [], "unplaced": []}
    fps = {}
    for ref, lib, val, fpid, x, y, rot, pins in spec.P:
        fp = load_fp(fpid or default_fp(lib))
        fp.SetReference(ref); fp.SetValue(val)
        uid, desc, dsheet, no_bom, no_board = info.get(ref, (None, "", "", False, False))
        if uid:
            fp.SetPath(pcbnew.KIID_PATH("/" + uid))
        try:                                              # KiCad 8+: footprint fields mirror the symbol (parity)
            for fld in fp.GetFields():
                if fld.GetName() == "Description":
                    fld.SetText(desc)
                elif fld.GetName() == "Datasheet":
                    fld.SetText(dsheet)
        except AttributeError:
            pass
        a = fp.GetAttributes()
        a = (a | pcbnew.FP_EXCLUDE_FROM_BOM) if no_bom else (a & ~pcbnew.FP_EXCLUDE_FROM_BOM)
        fp.SetAttributes(a)
        board.Add(fp)
        pads = {}
        for pad in fp.Pads():
            pads.setdefault(pad.GetNumber(), []).append(pad)
        for pin, net in pins.items():
            if net.startswith("NC_"):
                nm = nc.get((ref, pin))
                if nm:
                    ni = pcbnew.NETINFO_ITEM(board, nm); board.Add(ni)
                    for p in pads.get(pin, []):
                        p.SetNet(ni)
                continue
            tg = [p for k, v in pads.items() if k.startswith("S") for p in v] if pin == "SH" else pads.get(pin, [])
            if not tg:
                rep["unmatched_spec_pins"].append(f"{ref}.{pin}")
            for p in tg:
                p.SetNet(nets[net])
        fps[ref] = fp
    # ------------------------------------------------ placement
    occ = []                                              # placed rectangles (x1, y1, x2, y2) in mm
    for (hx, hy) in cfg.HOLES:
        occ.append((hx - 3.5, hy - 3.5, hx + 3.5, hy + 3.5))
    for k in getattr(cfg, "KEEPOUTS", []):
        occ.append(k)
    holes = {f"H{i+1}": h for i, h in enumerate(cfg.HOLES)}
    gap = getattr(cfg, "GAP", 0.6)

    part_gap = [(re.compile(rx), g) for rx, g in getattr(cfg, "PART_GAP", [])]

    def gap_of(ref):
        return next((g for rx, g in part_gap if rx.match(ref)), gap)

    def put(fp, x, y, r):
        fp.SetOrientationDegrees(r)
        fp.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        bx = fp_box(fp)
        g = gap_of(fp.GetReference())                     # PART_GAP applies to fixed parts too
        occ.append((x + bx[0] - g / 2, y + bx[1] - g / 2, x + bx[2] + g / 2, y + bx[3] + g / 2))

    for ref, (hx, hy) in holes.items():
        if ref in fps:
            fps[ref].SetPosition(pcbnew.VECTOR2I(MM(hx), MM(hy)))
    placed = set(holes) & set(fps)
    ant = getattr(cfg, "ANTENNA_REF", None)
    for ref, (x, y, r) in cfg.FIXED.items():
        if ref == ant:                                    # module body (F.Fab) flush with the front edge
            u = fps[ref]
            u.SetOrientationDegrees(r); u.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
            fab = [g.GetBoundingBox() for g in u.GraphicalItems() if g.GetLayer() == pcbnew.F_Fab]
            y += pcbnew.ToMM(MM(Y0) - min(bb.GetTop() for bb in fab))
        n_before = len(occ)
        put(fps[ref], x, y, r); placed.add(ref)
        new = occ[-1]
        hit = [i for i, o in enumerate(occ[:n_before]) if not (new[2] <= o[0] or new[0] >= o[2] or new[3] <= o[1] or new[1] >= o[3])]
        if hit:
            rep.setdefault("fixed_overlaps", []).append(ref)
    sig = {}
    for ref, lib, val, fpid, x, y, rot, pins in spec.P:
        for pin, net in pins.items():
            if net not in SKIP_NETS_FOR_PLACEMENT | set(getattr(cfg, "PLACEMENT_IGNORE", ())) \
                    and not net.startswith("NC_") and len(spec.NETS[net]) <= 12:
                sig.setdefault(ref, set()).add(net)

    def net_pos(net):
        pts = [fps[r].GetPosition() for r, _ in spec.NETS[net] if r in placed]
        return pts

    regions = [(re.compile(rx), box) for rx, box in getattr(cfg, "REGIONS", [])]

    def region_of(ref):
        for rx, box in regions:
            if rx.match(ref):
                return box
        return None

    def overlaps(r):
        if r[0] < X0 + 0.8 or r[1] < Y0 + 0.8 or r[2] > X0 + W - 0.8 or r[3] > Y0 + H - 0.8:
            return True
        return any(not (r[2] <= o[0] or r[0] >= o[2] or r[3] <= o[1] or r[1] >= o[3]) for o in occ)

    order = [p[0] for p in spec.P if p[0] not in placed]
    while order:
        # most connections to already-placed parts first (spec order breaks ties)
        best = max(order, key=lambda r: (sum(1 for n in sig.get(r, ()) for q, _ in spec.NETS[n] if q in placed), -order.index(r)))
        order.remove(best)
        fp = fps[best]
        pts = [p for n in sig.get(best, ()) for p in net_pos(n)]
        if pts:
            tx = sum(pcbnew.ToMM(p.x) for p in pts) / len(pts); ty = sum(pcbnew.ToMM(p.y) for p in pts) / len(pts)
        else:
            tx, ty = X0 + W / 2, Y0 + H / 2
        reg = region_of(best)
        if reg:
            tx = min(max(tx, reg[0]), reg[2]); ty = min(max(ty, reg[1]), reg[3])
        done = False
        for rad in [i * 0.5 for i in range(0, int(max(W, H) * 2))]:
            cands = [(tx, ty)] if rad == 0 else [
                (tx + rad * math.cos(a), ty + rad * math.sin(a)) for a in [k * 2 * math.pi / max(8, int(rad * 6)) for k in range(max(8, int(rad * 6)))]]
            for cx, cy in cands:
                cx, cy = round(cx * 4) / 4, round(cy * 4) / 4
                for r in (0, 90):
                    fp.SetOrientationDegrees(r)
                    fp.SetPosition(pcbnew.VECTOR2I(MM(cx), MM(cy)))
                    bx = fp_box(fp)
                    gp = gap_of(best)                     # fine-pitch parts get extra room for fan-out
                    rect = (cx + bx[0] - gp / 2, cy + bx[1] - gp / 2, cx + bx[2] + gp / 2, cy + bx[3] + gp / 2)
                    if reg and not (reg[0] <= cx <= reg[2] and reg[1] <= cy <= reg[3]):
                        continue
                    if not overlaps(rect):
                        occ.append(rect); placed.add(best); done = True
                        break
                if done:
                    break
            if done:
                break
        if not done:
            rep["unplaced"].append(best)
            fp.SetPosition(pcbnew.VECTOR2I(MM(X0 + W + 20), MM(Y0)))
    if ant and ant in fps:                                 # ESP32 antenna keep-out band (no copper)
        u = fps[ant]
        o = u.GetCourtyard(pcbnew.F_CrtYd).Outline(0)
        ys = sorted({o.CPoint(i).y for i in range(o.PointCount())})
        crt = u.GetCourtyard(pcbnew.F_CrtYd).BBox()
        rule_area(board, max(pcbnew.ToMM(crt.GetLeft()), X0), Y0, min(pcbnew.ToMM(crt.GetRight()), X0 + W), pcbnew.ToMM(ys[1]))
    fo = getattr(cfg, "GND_FANOUT", True)
    if fo:
        fanout_gnd(board, only=None if fo is True else fo)
    if LAYERS == 4:                                       # planes only; outer GND pours are added after routing
        gnd_zone(board, pcbnew.In1_Cu)
        if POWER_PLANE:
            gnd_zone(board, pcbnew.In2_Cu, "/" + POWER_PLANE)
    else:
        gnd_zone(board, pcbnew.B_Cu)
    board.Save(PCB)                                       # KiCad 7: zone filling needs a loaded board
    board = pcbnew.LoadBoard(PCB)
    fill(board)
    set_outline_inset(board, 0.4)
    rep["dsn_exported"] = bool(pcbnew.ExportSpecctraDSN(board, DSN))
    patch_dsn_power(DSN)
    if getattr(cfg, "GND_ROUTED", False):
        rep["dsn_planes_removed"] = strip_dsn_planes(DSN)
    set_outline_inset(board, 0.0)
    fill(board)
    board.Save(PCB)
    rep["placed"] = len(placed)
    print(json.dumps(rep, indent=1))


def via_clear(board, pt, r=0.3):
    """True when a GND via of radius r at pt keeps CLEAR from every other-net pad/track on both layers."""
    gnd = board.GetNetcodeFromNetname("/GND")
    sh = pcbnew.SHAPE_CIRCLE(pt, MM(r))
    for L in ((pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu) if LAYERS == 4 else (pcbnew.F_Cu, pcbnew.B_Cu)):
        for o in list(board.GetPads()) + list(board.GetTracks()):          # through-vias touch every layer
            if o.GetNetCode() != gnd and o.IsOnLayer(L) and o.GetEffectiveShape(L).Collide(sh, MM(CLEAR)):
                return False
    return True


def stitch_and_fill(board):
    gnd = board.GetNetcodeFromNetname("/GND")
    zones = {z.GetLayer(): z for z in board.Zones() if z.GetNetCode() == gnd and not z.GetIsRuleArea()}
    keep = [z for z in board.Zones() if z.GetIsRuleArea()]
    holes = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA"]
    holes += [p.GetPosition() for p in board.GetPads() if p.HasHole()]
    added = []

    def ok(x, y):
        pt = pcbnew.VECTOR2I(MM(x), MM(y))
        probes = [pcbnew.VECTOR2I(MM(x + dx), MM(y + dy)) for dx, dy in
                  ((0, 0), (.4, 0), (-.4, 0), (0, .4), (0, -.4), (.3, .3), (-.3, -.3), (.3, -.3), (-.3, .3))]
        if not all(zones[l].HitTestFilledArea(l, q, 0) for l in (pcbnew.F_Cu, pcbnew.B_Cu) for q in probes):
            return None
        if any(k.Outline().Contains(pt) for k in keep):
            return None
        if any((h.x - pt.x) ** 2 + (h.y - pt.y) ** 2 < MM(1.2) ** 2 for h in holes + added):
            return None
        return pt
    x = X0 + 2.5
    while x < X0 + W - 2:
        y = Y0 + 2.5
        while y < Y0 + H - 2:
            pt = ok(x, y)
            if pt and via_clear(board, pt):
                v = pcbnew.PCB_VIA(board); v.SetPosition(pt); v.SetWidth(MM(0.6)); v.SetDrill(MM(0.3)); v.SetNetCode(gnd)
                board.Add(v); added.append(pt)
            y += 5
        x += 5
    print("GND stitching vias:", len(added))


def import_ses(ses):
    txt = open(ses).read()
    if "GND_FIX_" in txt:                                 # temporary nets from export_routed -> GND
        ses = ses + ".gnd.ses"
        open(ses, "w").write(re.sub(r'"?GND_FIX_\d+"?', "/GND", txt))
    board = pcbnew.LoadBoard(PCB)
    has = {z.GetLayer() for z in board.Zones() if not z.GetIsRuleArea()}
    print("ses imported:", pcbnew.ImportSpecctraSES(board, ses))
    for L in (pcbnew.F_Cu, pcbnew.B_Cu):
        if L not in has:
            gnd_zone(board, L)
    f = pcbnew.ZONE_FILLER(board); f.Fill(board.Zones())
    stitch_and_fill(board); f.Fill(board.Zones())
    board.Save(PCB)


def export_routed(out):
    """Re-export the routed board (tracks kept) so Freerouting only completes what is missing.
    GND pads that no pour reaches (KiCad connectivity) are moved, together with the nearest well-connected
    GND pad, into a temporary net GND_FIX_n so the router draws a real track for them; import_ses renames
    those tracks back to /GND."""
    board = pcbnew.LoadBoard(PCB)
    set_outline_inset(board, 0.4)
    print("dsn exported:", pcbnew.ExportSpecctraDSN(board, out))
    patch_dsn_power(out)
    gnd = board.GetNetcodeFromNetname("/GND")
    strays = stray_pad_clusters(board, gnd)
    if not strays:
        return
    stray_ids = {q.m_Uuid.AsString() for cl in strays for q in cl}   # (SWIG proxies differ: compare UUIDs)
    main = [p for p in board.GetPads() if p.GetNetCode() == gnd and p.m_Uuid.AsString() not in stray_ids
            and not p.GetParentFootprint().GetReference().startswith(("H", "TP"))]
    pin = lambda p: f"{p.GetParentFootprint().GetReference()}-{p.GetNumber()}"
    pairs, used = [], set()
    for cl in strays:
        a = cl[0]
        cands = [q for q in main if pin(q) not in used and (q.HasHole() or q.GetLayerSet().Contains(pcbnew.F_Cu) == a.GetLayerSet().Contains(pcbnew.F_Cu))]
        t = min(cands, key=lambda q: (q.GetPosition().x - a.GetPosition().x) ** 2 + (q.GetPosition().y - a.GetPosition().y) ** 2)
        used.add(pin(t)); pairs.append((pin(a), pin(t)))
    t = open(out).read()
    m = re.search(r"\(net \"?/GND\"?\s*\(pins ([^)]*)\)", t)
    body = m.group(1)
    for a, b in pairs:
        body = re.sub(r"(?<![\w-])" + re.escape(a) + r"(?![\w@-])", "", body)
        body = re.sub(r"(?<![\w-])" + re.escape(b) + r"(?![\w@-])", "", body)
    t = t[:m.start(1)] + body + t[m.end(1):]
    extra = "".join(f"    (net GND_FIX_{k}\n      (pins {a} {b})\n    )\n" for k, (a, b) in enumerate(pairs))
    ci = t.index("(class kicad_default")
    t = t[:ci] + extra.lstrip() + "    " + t[ci:]
    t = t.replace("(class kicad_default", "(class kicad_default " + " ".join(f"GND_FIX_{k}" for k in range(len(pairs))), 1)
    open(out, "w").write(t)
    print("stray GND pads routed as tracks:", pairs)


def gnd_groups(board, gnd, Z):
    """Union-find of GND pour fragments (both layers) joined by GND vias / plated GND holes."""
    nodes = []
    for L in (pcbnew.F_Cu, pcbnew.B_Cu):
        f = Z[L].GetFilledPolysList(L)
        for i in range(f.OutlineCount()):
            P = pcbnew.SHAPE_POLY_SET(); P.AddOutline(f.Outline(i)); nodes.append((L, P, f.Outline(i).BBox()))
    par = list(range(len(nodes)))

    def fd(x):
        while par[x] != x:
            x = par[x]
        return x
    hs = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA" and v.GetNetCode() == gnd]
    hs += [p.GetPosition() for p in board.GetPads() if p.HasHole() and p.GetNetCode() == gnd]
    for h in hs:
        ids = [k for k, n in enumerate(nodes) if n[1].Contains(h)]
        for k in ids[1:]:
            par[fd(k)] = fd(ids[0])
    groups = {}
    for k in range(len(nodes)):
        groups.setdefault(fd(k), []).append(k)
    main = max(groups.values(), key=lambda ks: sum(nodes[k][1].Area() for k in ks))
    return nodes, groups, main


def stray_pad_clusters(board, gnd):
    """GND pads that KiCad's connectivity does not join to the main GND cluster, grouped by cluster."""
    board.BuildConnectivity()
    c = board.GetConnectivity()
    pads = [p for p in board.GetPads() if p.GetNetCode() == gnd]
    seen, clusters = set(), []
    for p in pads:
        k = p.m_Uuid.AsString()
        if k in seen:
            continue
        ids = {k} | {x.m_Uuid.AsString() for x in c.GetConnectedItems(p)}
        cl = [q for q in pads if q.m_Uuid.AsString() in ids]
        seen |= {q.m_Uuid.AsString() for q in cl}
        clusters.append(cl)
    clusters.sort(key=len, reverse=True)
    return clusters[1:]


def main_gnd_items(board, gnd):
    """Tracks, vias and pads (not zones) of the largest connected GND cluster."""
    board.BuildConnectivity()
    c = board.GetConnectivity()
    pads = [p for p in board.GetPads() if p.GetNetCode() == gnd]
    seen, best = set(), set()
    for p in pads:
        k = p.m_Uuid.AsString()
        if k in seen:
            continue
        ids = {k} | {x.m_Uuid.AsString() for x in c.GetConnectedItems(p)}
        seen |= ids
        if len(ids) > len(best):
            best = ids
    return [o for o in list(board.GetTracks()) + pads if o.GetNetCode() == gnd and o.m_Uuid.AsString() in best]


def astar_connect(board, pad, gnd, main_polys, win=12.0, res=0.1, w=0.25, main_items=(), rip=False):
    """Tiny two-layer grid router for one stray GND pad: A* from the pad to any point solidly inside the
    main GND pour, or touching a track/via/pad of the main GND cluster (either layer); vias allowed
    (cost 25 cells). Obstacles are other-net pads/tracks/vias, tested with KiCad's own shape collision
    at track half-width + clearance.
    rip=True: when the pad is fenced in by signal tracks on both layers, those tracks may be crossed at a high
    cost; the crossed segments are deleted (the completion pass re-routes them) and the new GND copper is
    locked so the router keeps it. Returns the set of ripped net names (or True/False)."""
    import heapq
    c0 = pad.GetPosition()
    n = int(2 * win / res) + 1
    ox, oy = c0.x - MM(win), c0.y - MM(win)
    cell = lambda i, j: pcbnew.VECTOR2I(ox + MM(i * res), oy + MM(j * res))
    layers = (pcbnew.F_Cu, pcbnew.B_Cu)
    rt = MM(w / 2) + MM(CLEAR)
    rv = MM(0.3) + MM(CLEAR)
    bbox = pcbnew.BOX2I(pcbnew.VECTOR2I(ox, oy), pcbnew.VECTOR2I(MM(2 * win), MM(2 * win)))
    blocked = [bytearray(n * n) for _ in layers]
    soft = [bytearray(n * n) for _ in layers]             # rip mode: cells blocked only by signal tracks
    viablk = bytearray(n * n)
    items = [o for o in list(board.GetPads()) + list(board.GetTracks())
             if o.GetNetCode() != gnd and o.GetBoundingBox().Intersects(bbox)]
    for li, L in enumerate(layers):
        for o in items:
            if not o.IsOnLayer(L):
                continue
            sh = o.GetEffectiveShape(L)
            bb = o.GetBoundingBox()
            grid = soft[li] if rip and o.GetClass() in ("PCB_TRACK", "PCB_ARC") and not o.IsLocked() else blocked[li]
            i0 = max(0, int((bb.GetLeft() - rv - ox) / MM(res))); i1 = min(n - 1, int((bb.GetRight() + rv - ox) / MM(res)) + 1)
            j0 = max(0, int((bb.GetTop() - rv - oy) / MM(res))); j1 = min(n - 1, int((bb.GetBottom() + rv - oy) / MM(res)) + 1)
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    k = j * n + i
                    if grid[k] and viablk[k]:
                        continue
                    p = cell(i, j)
                    if not grid[k] and sh.Collide(pcbnew.SHAPE_CIRCLE(p, rt), 0):
                        grid[k] = 1
                    if not viablk[k] and sh.Collide(pcbnew.SHAPE_CIRCLE(p, rv), 0):
                        viablk[k] = 1
    for L in ((pcbnew.In1_Cu, pcbnew.In2_Cu) if LAYERS == 4 else ()):   # a new via also crosses inner tracks
        for o in items:
            if not o.IsOnLayer(L):
                continue
            sh = o.GetEffectiveShape(L)
            bb = o.GetBoundingBox()
            i0 = max(0, int((bb.GetLeft() - rv - ox) / MM(res))); i1 = min(n - 1, int((bb.GetRight() + rv - ox) / MM(res)) + 1)
            j0 = max(0, int((bb.GetTop() - rv - oy) / MM(res))); j1 = min(n - 1, int((bb.GetBottom() + rv - oy) / MM(res)) + 1)
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    if not viablk[j * n + i] and sh.Collide(pcbnew.SHAPE_CIRCLE(cell(i, j), rv), 0):
                        viablk[j * n + i] = 1
    edge = (MM(X0 + 0.8), MM(Y0 + 0.8), MM(X0 + W - 0.8), MM(Y0 + H - 0.8))
    holes = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA"] + [q.GetPosition() for q in board.GetPads() if q.HasHole()]
    for j in range(n):
        for i in range(n):
            p = cell(i, j)
            if not (edge[0] < p.x < edge[2] and edge[1] < p.y < edge[3]):
                blocked[0][j * n + i] = blocked[1][j * n + i] = viablk[j * n + i] = 1
    hr = int(0.6 / res)                                   # drill-to-drill: new via >= 0.6 mm from any hole
    for h in holes:
        hi, hj = int((h.x - ox) / MM(res)), int((h.y - oy) / MM(res))
        for di in range(-hr, hr + 1):
            for dj in range(-hr, hr + 1):
                if di * di + dj * dj <= hr * hr and 0 <= hi + di < n and 0 <= hj + dj < n:
                    viablk[(hj + dj) * n + hi + di] = 1
    touch = [bytearray(n * n) for _ in layers]            # cells where a track end joins the main cluster
    for li, L in enumerate(layers):
        for o in main_items:
            if not o.IsOnLayer(L) or not o.GetBoundingBox().Intersects(bbox):
                continue
            sh = o.GetEffectiveShape(L)
            bb = o.GetBoundingBox()
            i0 = max(0, int((bb.GetLeft() - ox) / MM(res))); i1 = min(n - 1, int((bb.GetRight() - ox) / MM(res)) + 1)
            j0 = max(0, int((bb.GetTop() - oy) / MM(res))); j1 = min(n - 1, int((bb.GetBottom() - oy) / MM(res)) + 1)
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    if not touch[li][j * n + i] and sh.Collide(pcbnew.SHAPE_CIRCLE(cell(i, j), MM(w / 4)), 0):
                        touch[li][j * n + i] = 1

    def goal(li, i, j):
        if touch[li][j * n + i] and not blocked[li][j * n + i]:
            return True
        if i % 2 or j % 2:
            return False
        p = cell(i, j)
        pts = [p] + [pcbnew.VECTOR2I(p.x + MM(dx), p.y + MM(dy)) for dx, dy in ((0.3, 0), (-0.3, 0), (0, 0.3), (0, -0.3))]
        return any(all(P.Contains(q) for q in pts) for P in main_polys[layers[li]])
    si = sj = n // 2
    start = [(0, si, sj)] if pad.IsOnLayer(pcbnew.F_Cu) else []
    if pad.IsOnLayer(pcbnew.B_Cu):
        start.append((1, si, sj))
    dist, prev, pq = {}, {}, []
    for st in start:
        dist[st] = 0; heapq.heappush(pq, (0, st))
    found = None
    while pq:
        d, (li, i, j) = heapq.heappop(pq)
        if d > dist.get((li, i, j), 1e18):
            continue
        if d > 0 and goal(li, i, j):
            found = (li, i, j); break
        for di, dj, cst in ((1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)):
            a, b = i + di, j + dj
            if 0 <= a < n and 0 <= b < n and (not blocked[li][b * n + a] or (abs(a - si) + abs(b - sj) < 6)):
                nd = d + cst + (40 if soft[li][b * n + a] else 0)
                if nd < dist.get((li, a, b), 1e18):
                    dist[(li, a, b)] = nd; prev[(li, a, b)] = (li, i, j); heapq.heappush(pq, (nd, (li, a, b)))
        k = j * n + i
        if not viablk[k] and abs(i - si) + abs(j - sj) >= 6:
            o = 1 - li
            nd = d + 25 + (40 if soft[o][k] else 0)
            if not blocked[o][k] and nd < dist.get((o, i, j), 1e18):
                dist[(o, i, j)] = nd; prev[(o, i, j)] = (li, i, j); heapq.heappush(pq, (nd, (o, i, j)))
    if not found:
        return False
    path = [found]
    while path[-1] in prev:
        path.append(prev[path[-1]])
    path.reverse()
    # emit: merge collinear same-layer steps into segments, a via at every layer change
    new = []

    def add_seg(a, b, li):
        t = pcbnew.PCB_TRACK(board); t.SetStart(cell(*a)); t.SetEnd(cell(*b))
        t.SetWidth(MM(w)); t.SetLayer(layers[li]); t.SetNetCode(gnd); board.Add(t)
        new.append((layers[li], pcbnew.SHAPE_SEGMENT(cell(*a), cell(*b), MM(w))))
    run_start, run_dir = path[0], None
    for k in range(1, len(path)):
        a, b = path[k - 1], path[k]
        if a[0] != b[0]:                                  # layer change at this cell
            if (run_start[1], run_start[2]) != (a[1], a[2]):
                add_seg((run_start[1], run_start[2]), (a[1], a[2]), a[0])
            v = pcbnew.PCB_VIA(board); v.SetPosition(cell(a[1], a[2])); v.SetWidth(MM(0.6)); v.SetDrill(MM(0.3)); v.SetNetCode(gnd); board.Add(v)
            new += [(L, pcbnew.SHAPE_CIRCLE(cell(a[1], a[2]), MM(0.3))) for L in layers]
            if rip:
                v.SetLocked(True)
            run_start, run_dir = b, None
            continue
        d = (b[1] - a[1], b[2] - a[2])
        if run_dir is not None and d != run_dir:
            add_seg((run_start[1], run_start[2]), (a[1], a[2]), a[0]); run_start = a
        run_dir = d
    last = path[-1]
    if (run_start[1], run_start[2]) != (last[1], last[2]):
        add_seg((run_start[1], run_start[2]), (last[1], last[2]), last[0])
    if not rip:
        return True
    for t in board.GetTracks():                           # new GND copper is kept by the completion router
        if t.GetNetCode() == gnd and t.GetClass() in ("PCB_TRACK", "PCB_VIA") and not t.IsLocked():
            if any(t.IsOnLayer(L) and t.GetEffectiveShape(L).Collide(sh, 0) for L, sh in new):
                t.SetLocked(True)
    ripped, dead = set(), []
    for t in board.GetTracks():
        if t.GetNetCode() == gnd or t.GetClass() not in ("PCB_TRACK", "PCB_ARC") or t.IsLocked():
            continue
        if any(t.IsOnLayer(L) and t.GetEffectiveShape(L).Collide(sh, MM(CLEAR)) for L, sh in new):
            dead.append(t); ripped.add(t.GetNetname())
    for t in dead:
        board.Delete(t)
    return ripped or True


def fix_islands(max_r=15.0, w=0.25):
    """For every GND pour group the bridge could not join: from one of its GND pads run a short track
    (straight or L-shaped) either to a solid part of the main pour on the same layer, or to a new via
    dropped into the main pour on the other layer. Clearance is checked against every other-net item
    with KiCad's own shape collision."""
    board = pcbnew.LoadBoard(PCB)
    board.BuildConnectivity()
    gnd = board.GetNetcodeFromNetname("/GND")
    Z = {z.GetLayer(): z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == gnd}
    for z in Z.values():
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        z.SetLocalClearance(MM(getattr(cfg, "ZONE_CLEARANCE", 0.3))); z.SetMinThickness(MM(0.2))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    # stitching vias that only join two cut-off pour fragments (no GND pad in the group) keep useless
    # islands alive: drop them so island removal can clear that copper
    nodes, groups, main = gnd_groups(board, gnd, Z)
    gpads = [p.GetPosition() for p in board.GetPads() if p.GetNetCode() == gnd]
    orphan = []
    for ks in groups.values():
        if ks is main or any(nodes[k][1].Contains(q) for k in ks for q in gpads):
            continue
        orphan += [v for v in board.GetTracks() if v.GetClass() == "PCB_VIA" and v.GetNetCode() == gnd
                   and any(nodes[k][1].Contains(v.GetPosition()) for k in ks)]
    for v in orphan:
        board.Delete(v)
    if orphan:
        print("orphan stitching vias removed:", len(orphan))
        board.BuildConnectivity()
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    clr = MM(CLEAR)
    keep = [z for z in board.Zones() if z.GetIsRuleArea()]
    fixed = 0
    for it in range(16):
        nodes, groups, main = gnd_groups(board, gnd, Z)
        if not stray_pad_clusters(board, gnd):
            break
        mitems = main_gnd_items(board, gnd)
        mainL = {L: [nodes[m][1] for m in main if nodes[m][0] == L] for L in (pcbnew.F_Cu, pcbnew.B_Cu)}
        others = {L: [t for t in board.GetTracks() if t.GetNetCode() != gnd and t.IsOnLayer(L)] +
                     [p for p in board.GetPads() if p.GetNetCode() != gnd and p.IsOnLayer(L)] for L in (pcbnew.F_Cu, pcbnew.B_Cu)}
        holes = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA"] + [p.GetPosition() for p in board.GetPads() if p.HasHole()]

        def free(shape, L):
            return not any(o.GetEffectiveShape(L).Collide(shape, clr) for o in others[L])

        def solid(polys, v, r=0.35):
            pts = [v] + [pcbnew.VECTOR2I(v.x + MM(dx), v.y + MM(dy)) for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r))]
            return any(all(P.Contains(q) for q in pts) for P in polys)

        def path_free(c, v, L):
            for mid in (None, pcbnew.VECTOR2I(v.x, c.y), pcbnew.VECTOR2I(c.x, v.y)):
                segs = [(c, v)] if mid is None else [(c, mid), (mid, v)]
                if all(free(pcbnew.SHAPE_SEGMENT(a, b, MM(w)), L) for a, b in segs):
                    return segs
            return None

        def add_track(segs, L):
            for a, b in segs:
                t = pcbnew.PCB_TRACK(board); t.SetStart(a); t.SetEnd(b); t.SetWidth(MM(w)); t.SetLayer(L); t.SetNetCode(gnd)
                board.Add(t)
        progress = ripped_any = False
        for pads in stray_pad_clusters(board, gnd):
            ok = False
            for pad in pads:
                for L in (pcbnew.F_Cu, pcbnew.B_Cu):
                    if not pad.IsOnLayer(L) or ok:
                        continue
                    other = pcbnew.B_Cu if L == pcbnew.F_Cu else pcbnew.F_Cu
                    c = pad.GetPosition()
                    for r in [x * 0.25 for x in range(3, int(max_r * 4))]:
                        n = max(16, int(r * 12))
                        for k in range(n):
                            a = 2 * math.pi * k / n
                            v = pcbnew.VECTOR2I(c.x + MM(r * math.cos(a)), c.y + MM(r * math.sin(a)))
                            if solid(mainL[L], v):                       # same-layer: track into main pour
                                segs = path_free(c, v, L)
                                if segs:
                                    add_track(segs, L); ok = True; how = "track"
                            elif solid(mainL[other], v) and not any(z.Outline().Contains(v) for z in keep) \
                                    and all((h.x - v.x) ** 2 + (h.y - v.y) ** 2 >= MM(0.9) ** 2 for h in holes):
                                via_sh = pcbnew.SHAPE_CIRCLE(v, MM(0.3))
                                if free(via_sh, pcbnew.F_Cu) and free(via_sh, pcbnew.B_Cu) and (LAYERS == 2 or via_clear(board, v)):
                                    segs = path_free(c, v, L)
                                    if segs:
                                        add_track(segs, L)
                                        vi = pcbnew.PCB_VIA(board); vi.SetPosition(v); vi.SetWidth(MM(0.6)); vi.SetDrill(MM(0.3)); vi.SetNetCode(gnd)
                                        board.Add(vi); ok = True; how = "track+via"
                            if ok:
                                fixed += 1
                                print(f"island fix ({how}): {pad.GetParentFootprint().GetReference()}.{pad.GetNumber()} on "
                                      f"{board.GetLayerName(L)} -> ({pcbnew.ToMM(v.x):.2f}, {pcbnew.ToMM(v.y):.2f})")
                                break
                        if ok:
                            break
                if ok:
                    break
            if not ok:
                for pad in pads[:3]:
                    if astar_connect(board, pad, gnd, mainL, main_items=mitems):
                        ok = True; fixed += 1
                        print(f"island fix (grid route): {pad.GetParentFootprint().GetReference()}.{pad.GetNumber()}")
                        break
            if not ok:                                    # fenced in: cross signal tracks, re-route them later
                for pad in pads[:3]:
                    res_ = astar_connect(board, pad, gnd, mainL, main_items=mitems, rip=True)
                    if res_:
                        ok = True; fixed += 1
                        print(f"island fix (grid route, ripped {sorted(res_) if res_ is not True else []}): "
                              f"{pad.GetParentFootprint().GetReference()}.{pad.GetNumber()}")
                        ripped_any = True
                        break
            progress = progress or ok
            if ripped_any:                                # deleted tracks: rebuild every item list first
                break
        board.BuildConnectivity()
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        if not progress:
            break
    nodes, groups, main = gnd_groups(board, gnd, Z)
    board.BuildConnectivity()
    left = stray_pad_clusters(board, gnd)
    print(f"island fixes: {fixed}, stray GND pads left: {[q.GetParentFootprint().GetReference() + '.' + q.GetNumber() for cl in left for q in cl]}, "
          f"unconnected: {board.GetConnectivity().GetUnconnectedCount(True)}")
    board.Save(PCB)


def bridge(probe=0.4, hole_gap=1.0, step=0.2):
    """Union-find GND pour fragments through GND vias / holes; drop a via joining each stray group.
    A second, tighter search (smaller keep-clear) runs for groups the first one cannot join."""
    board = pcbnew.LoadBoard(PCB)
    gnd = board.GetNetcodeFromNetname("/GND")
    if LAYERS == 4:                                       # the In1.Cu plane joins every GND via: nothing to bridge
        print("bridge: 4-layer board, GND plane on In1.Cu")
        return
    Z = {z.GetLayer(): z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == gnd}
    for it in range(8):
        nodes = []
        for L in (pcbnew.F_Cu, pcbnew.B_Cu):
            f = Z[L].GetFilledPolysList(L)
            for i in range(f.OutlineCount()):
                P = pcbnew.SHAPE_POLY_SET(); P.AddOutline(f.Outline(i)); nodes.append((L, P, f.Outline(i).BBox()))
        par = list(range(len(nodes)))

        def fd(x):
            while par[x] != x:
                x = par[x]
            return x
        hs = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA" and v.GetNetCode() == gnd]
        hs += [p.GetPosition() for p in board.GetPads() if p.HasHole() and p.GetNetCode() == gnd]
        allh = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA"] + [p.GetPosition() for p in board.GetPads() if p.HasHole()]
        for h in hs:
            ids = [k for k, n in enumerate(nodes) if n[1].Contains(h)]
            for k in ids[1:]:
                par[fd(k)] = fd(ids[0])
        groups = {}
        for k in range(len(nodes)):
            groups.setdefault(fd(k), []).append(k)
        if len(groups) <= 1:
            print("GND pour groups: 1"); break
        main = max(groups.values(), key=lambda ks: sum(nodes[k][1].Area() for k in ks))
        added = 0
        tight = it >= 2                                    # relax the keep-clear after two passes
        pr_r, gap, st = (probe * 0.75, hole_gap * 0.8, step / 2) if tight else (probe, hole_gap, step)
        for ks in groups.values():
            if ks is main:
                continue
            done = False
            for k in ks:
                L, P, bb = nodes[k]
                others = [nodes[m][1] for m in main if nodes[m][0] != L]
                y = pcbnew.ToMM(bb.GetTop())
                while y <= pcbnew.ToMM(bb.GetBottom()) and not done:
                    x = pcbnew.ToMM(bb.GetLeft())
                    while x <= pcbnew.ToMM(bb.GetRight()) and not done:
                        pr = [pcbnew.VECTOR2I(MM(x + dx), MM(y + dy)) for dx, dy in ((0, 0), (pr_r, 0), (-pr_r, 0), (0, pr_r), (0, -pr_r))]
                        if all(P.Contains(q) for q in pr) and any(all(O.Contains(q) for q in pr) for O in others) \
                                and all((h.x - pr[0].x) ** 2 + (h.y - pr[0].y) ** 2 > MM(gap) ** 2 for h in allh) \
                                and via_clear(board, pr[0], 0.25 if tight else 0.3):
                            v = pcbnew.PCB_VIA(board); v.SetPosition(pr[0]); v.SetNetCode(gnd)
                            v.SetWidth(MM(0.6 if not tight else 0.5)); v.SetDrill(MM(0.3 if not tight else 0.25))
                            board.Add(v); done = True; added += 1
                        x += st
                    y += st
                if done:
                    break
        print(f"pass {it}: {len(groups)} GND groups, vias added {added}{' (tight)' if tight else ''}")
        board.BuildConnectivity()
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        if not added and tight:
            break
    board.Save(PCB)


def sync_fields():
    """Copy Description/Datasheet/BOM flags from the schematic netlist onto a routed board (parity)."""
    board = pcbnew.LoadBoard(PCB)
    info, _ = netlist_info()
    n = 0
    for fp in board.GetFootprints():
        uid, desc, dsheet, no_bom, no_board = info.get(fp.GetReference(), (None, "", "", False, False))
        for fld in fp.GetFields():
            if fld.GetName() == "Description" and fld.GetText() != desc:
                fld.SetText(desc); n += 1
            elif fld.GetName() == "Datasheet" and fld.GetText() != dsheet:
                fld.SetText(dsheet); n += 1
        a = fp.GetAttributes()
        fp.SetAttributes((a | pcbnew.FP_EXCLUDE_FROM_BOM) if no_bom else (a & ~pcbnew.FP_EXCLUDE_FROM_BOM))
    board.Save(PCB)
    print("fields synced:", n)


if __name__ == "__main__":
    cmd = sys.argv[2] if len(sys.argv) > 2 else "place"
    if cmd == "place":
        place()
    elif cmd == "import_ses":
        import_ses(sys.argv[3])
    elif cmd == "export_routed":
        export_routed(sys.argv[3])
    elif cmd == "bridge":
        bridge()
        fix_islands()
    elif cmd == "fix_islands":
        fix_islands()
    elif cmd == "sync_fields":
        sync_fields()
    elif cmd == "unconnected":
        b = pcbnew.LoadBoard(PCB); b.BuildConnectivity()
        print("UNCONNECTED", b.GetConnectivity().GetUnconnectedCount(True))
