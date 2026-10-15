"""Build the ATLAS Board B PCB from board_b_design.py with KiCad's pcbnew Python API.

Run with KiCad's bundled Python (macOS):
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 \
      build_board_b_pcb.py [place|import_ses <file.ses>|pour]

  place       new board: outline from atlas_params, footprints from the spec, nets, net classes,
              placement, then writes the .kicad_pcb and a Specctra .dsn for Freerouting
  import_ses  import a routed Freerouting session, then add GND pours, fill, save
Footprint pad numbers are matched to the spec's pin numbers; any spec pin with no pad
(or vice versa on a signal pad) is reported, so a symbol/footprint mismatch cannot hide.
"""
import json, math, os, sys
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import board_b_design as spec

PROJ = os.path.dirname(HERE)
PCB = os.path.join(PROJ, "atlas_agent_ctrl.kicad_pcb")
DSN = os.path.join(PROJ, "atlas_agent_ctrl.dsn")
FPDIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
DEFAULT_FP = {"Driver_Motor:DRV8871DDA":
              "Package_SO:Texas_HTSOP-8-1EP_3.9x4.9mm_P1.27mm_EP2.95x4.9mm_Mask2.4x3.1mm_ThermalVias"}
MM = pcbnew.FromMM

# ------------------------------------------------------------------ board geometry
X0, Y0, W, H = 100.0, 100.0, 110.0, 90.0          # front edge = top (y = 100)
HOLES = [(104, 104), (206, 111), (104, 186), (206, 186)]

# ------------------------------------------------------------------ placement (x, y, rot, bottom?)
PL = {
    # front edge: dock pads, sensor module, Pi/camera link, ESP32 (antenna flush with edge)
    "J2": (118, 104.5, 0), "J8": (138, 103.8, 180), "J6": (156, 103.8, 180), "U4": (190, 113.5, 0),
    "D2": (113, 112, 90), "Q2": (120, 112, 0), "R4": (125, 110, 90), "Q3": (130, 112, 0), "R5": (134, 110, 90),
    "R6": (113, 118, 90), "R7": (116, 118, 90), "C4": (119, 118, 90), "R8": (122, 118, 90),
    "R9": (125, 118, 90), "R10": (128, 118, 90), "R11": (131, 118, 90), "D3": (135, 118, 90),
    # Pi power switch next to J6
    "Q6": (152, 111, 0), "Q7": (158, 112, 0), "R32": (148, 111, 90), "R33": (161, 112, 90), "C26": (153, 118, 0),
    "J7": (206.5, 132, 90),
    # ESP32 support, left of the module
    "C13": (176, 109, 0), "C14": (176, 112, 0), "R17": (176, 118, 90), "C15": (176, 122, 0),
    "SW1": (169, 112, 0), "R18": (176, 126, 90), "SW2": (169, 121, 0),
    "R34": (203, 119, 90), "R35": (206, 119, 90), "D6": (184, 130, 0), "R36": (188, 130, 0),
    # left motor channel
    "J4": (103.5, 150, 90), "U9": (113, 150, 90), "C19": (113, 141, 0), "C20": (117, 143, 90), "R24": (118, 155, 90),
    "R22": (113, 131, 0), "U7": (119, 131, 0), "R26": (124, 131, 90), "C23": (127, 131, 90),
    "R28": (108, 160, 90), "R29": (111, 160, 90), "C25": (114, 160, 90),
    # right motor channel
    "J5": (206.5, 152, 270), "U10": (197, 152, 90), "C21": (197, 143, 0), "C22": (193, 144, 90), "R25": (192, 157, 90),
    "R23": (191, 138, 0), "U8": (197, 138, 0), "R27": (202, 140, 90), "C24": (205, 140, 90),
    "R30": (199, 162, 90), "R31": (202, 162, 90),
    # motor power cut + watchdog, centre
    "Q5": (155, 137, 0), "C18": (155, 148, 0), "R21": (145, 133, 90), "Q4": (145, 140, 0), "R20": (148, 142, 90),
    "U6": (138, 140, 0), "C17": (138, 145, 0), "R19": (134, 144, 90), "U5": (138, 132, 0), "C16": (134, 130, 90),
    "TP4": (162, 143, 0), "TP7": (130, 137, 0), "TP5": (128, 150, 0), "TP6": (183, 150, 0),
    # 5 V buck + 3.3 V LDO
    "U1": (166, 160, 0), "C5": (160, 157, 90), "C6": (160, 162, 90), "R12": (162, 166, 90), "C7": (166, 155, 0),
    "L1": (174, 160, 0), "R13": (180, 165, 90), "R14": (183, 165, 90), "C8": (180, 157, 90), "C9": (183, 157, 90),
    "D4": (176, 170, 0), "D5": (176, 175, 0), "U2": (184, 172, 0), "C10": (181, 179, 90), "C11": (188, 168, 90),
    "C12": (191, 168, 90), "TP2": (170, 168, 0), "TP3": (193, 176, 0),
    # USB-C on the rear edge
    "J3": (196, 186.5, 0), "U3": (196, 179, 0), "R15": (189, 181, 90), "R16": (203, 181, 90),
    # power entry on the rear edge
    "J1": (118, 185, 180), "F1": (128, 184, 90), "Q1": (138, 181, 270), "D1": (148, 185, 0), "C1": (148, 177, 0),
    "R1": (127, 175, 90), "R2": (130, 175, 90), "C2": (133, 175, 90), "R3": (114, 177, 90), "C3": (117, 177, 90),
    "TP1": (156, 183, 0), "TP8": (160, 183, 0),
    # debug, LED, test
    "J9": (110, 170, 90), "D7": (122, 166, 0), "R37": (126, 166, 0),
}
for i, h in enumerate(HOLES):
    PL[f"H{i+1}"] = (h[0], h[1], 0)

POWER_NETS = {"VBAT_RAW", "VBAT_F", "VBAT", "VM_SW", "VM_L", "VM_R", "MOT_L_P", "MOT_L_N", "MOT_R_P",
              "MOT_R_N", "VCHG", "VCHG_D", "VCC_5V", "SW_5V", "PI_5V"}   # GND is a plane (B.Cu) + pour (F.Cu)


def netlist_info():
    """ref -> (symbol uuid, description) from the schematic netlist, so footprints link to symbols."""
    import re
    txt = open(os.path.join(PROJ, "atlas_agent_ctrl.net")).read()
    info = {}
    for blk in re.split(r"\(comp\s", txt)[1:]:
        ref = re.search(r'\(ref "([^"]+)"\)', blk).group(1)
        uids = [u for u in re.findall(r'\(tstamps "([^"]+)"\)', blk) if u != "/"]
        desc = re.search(r'\(description "([^"]*)"\)', blk)
        ds = re.search(r'\(name "Datasheet"\) "([^"]*)"', blk)
        info[ref] = (uids[-1] if uids else None, desc.group(1) if desc else "",
                     ds.group(1) if ds else "", "exclude_from_bom" in blk, "exclude_from_board" in blk)
    return info


def schematic_nc_nets():
    import re
    txt = open(os.path.join(PROJ, "atlas_agent_ctrl.net")).read()
    out = {}
    body = txt[txt.index("(nets"):]
    for blk in re.split(r"\n\s*\(net\b", body)[1:]:
        name = re.search(r'\(name "([^"]*)"\)', blk).group(1)
        if name.startswith("unconnected-("):
            for r, p in re.findall(r'\(ref "([^"]+)"\)\s*\(pin "([^"]+)"\)', blk):
                out[(r, p)] = name
    return out


def load_fp(fpid):
    lib, name = fpid.split(":")
    fp = pcbnew.FootprintLoad(os.path.join(FPDIR, lib + ".pretty"), name)
    if fp is None:
        raise RuntimeError(f"footprint not found: {fpid}")
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def add_outline(board, inset=0.0):
    i = inset
    pts = [(X0 + i, Y0 + i), (X0 + W - i, Y0 + i), (X0 + W - i, Y0 + H - i), (X0 + i, Y0 + H - i)]
    shapes = []
    for a, b in zip(pts, pts[1:] + pts[:1]):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pcbnew.VECTOR2I(MM(a[0]), MM(a[1])))
        s.SetEnd(pcbnew.VECTOR2I(MM(b[0]), MM(b[1])))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(MM(0.1))
        board.Add(s)
        shapes.append(s)
    return shapes


def setup_rules(board):
    ds = board.GetDesignSettings()
    ds.SetCopperLayerCount(2)
    ds.m_MinThroughDrill = MM(0.2)          # ESP32 EP thermal vias are 0.2 mm (JLCPCB min 0.2)
    ds.m_HoleClearance = MM(0.15)           # USB-C NPTH pegs sit 0.19 mm from their GND pads by design
    ds.m_CopperEdgeClearance = MM(0.5)
    ns = ds.m_NetSettings
    dflt = ns.GetDefaultNetclass()
    dflt.SetTrackWidth(MM(0.25)); dflt.SetClearance(MM(0.2)); dflt.SetViaDiameter(MM(0.6)); dflt.SetViaDrill(MM(0.3))
    pw = pcbnew.NETCLASS("Power")
    pw.SetTrackWidth(MM(0.5)); pw.SetClearance(MM(0.2)); pw.SetViaDiameter(MM(0.8)); pw.SetViaDrill(MM(0.4))  # 0.8 mm could not enter SOT-23 pins
    ns.SetNetclass("Power", pw)
    for n in POWER_NETS:
        ns.SetNetclassPatternAssignment("/" + n, "Power")


def place():
    board = pcbnew.BOARD()
    # Freerouting keeps only its 0.2 mm track clearance to the outline, so the DSN is exported with an
    # outline inset by 0.4 mm (=> >= 0.6 mm copper-to-edge); the true outline is restored afterwards.
    dsn_outline = add_outline(board, inset=0.4)
    setup_rules(board)
    nets = {}
    for n in spec.NETS:
        if n.startswith("NC_"):
            continue
        ni = pcbnew.NETINFO_ITEM(board, "/" + n if n != "GND" else "/GND")
        board.Add(ni)
        nets[n] = ni
    report = {"unmatched_spec_pins": [], "unconnected_signal_pads": [], "unplaced": []}
    info = netlist_info()
    nc_nets = schematic_nc_nets()
    for ref, lib, val, fpid, x, y, rot, pins in spec.P:
        fpid = fpid or DEFAULT_FP[lib]
        fp = load_fp(fpid)
        fp.SetReference(ref)
        fp.SetValue(val)
        uid, desc, dsheet, no_bom, no_board = info.get(ref, (None, "", "", False, False))
        if uid:
            fp.SetPath(pcbnew.KIID_PATH("/" + uid))
        for f in fp.GetFields():
            if f.GetName() == "Description":
                f.SetText(desc)
            elif f.GetName() == "Datasheet":
                f.SetText(dsheet)
        attrs = fp.GetAttributes()
        attrs = (attrs | pcbnew.FP_EXCLUDE_FROM_BOM) if no_bom else (attrs & ~pcbnew.FP_EXCLUDE_FROM_BOM)
        attrs = (attrs | pcbnew.FP_EXCLUDE_FROM_POS_FILES) if no_board else attrs
        fp.SetAttributes(attrs)
        board.Add(fp)
        if ref in PL:
            px, py, prot = PL[ref]
            fp.SetPosition(pcbnew.VECTOR2I(MM(px), MM(py)))
            fp.SetOrientationDegrees(prot)
        else:
            report["unplaced"].append(ref)
        pads_by_num = {}
        for pad in fp.Pads():
            pads_by_num.setdefault(pad.GetNumber(), []).append(pad)
        for pin, net in pins.items():
            if net.startswith("NC_"):
                nm = nc_nets.get((ref, pin))
                if nm:                                   # keep KiCad's own 'unconnected-(...)' net name
                    ni = pcbnew.NETINFO_ITEM(board, nm); board.Add(ni)
                    for p in pads_by_num.get(pin, []):
                        p.SetNet(ni)
                continue
            num = pin
            if ref == "J3" and pin == "SH":
                targets = [p for k, v in pads_by_num.items() if k.startswith("S") for p in v]
            else:
                targets = pads_by_num.get(num, [])
            if not targets:
                report["unmatched_spec_pins"].append(f"{ref}.{pin}")
            for p in targets:
                p.SetNet(nets[net])
        for num, pads in pads_by_num.items():
            if num and num not in pins and not num.startswith(("MP", "S")) and fp.GetReference() != "U4":
                report["unconnected_signal_pads"].append(f"{ref}.{num}")
    # ESP32: module body (F.Fab) top flush with the front edge; the footprint's 15 mm antenna
    # keep-out courtyard then lies off-board, and only its lower 6 mm band remains on the board.
    u4 = board.FindFootprintByReference("U4")
    fab = [g.GetBoundingBox() for g in u4.GraphicalItems() if g.GetLayer() == pcbnew.F_Fab]
    u4.Move(pcbnew.VECTOR2I(0, MM(Y0) - min(bb.GetTop() for bb in fab)))
    # antenna keep-out: no copper under/beside the ESP32 antenna (front 6.5 mm of the module)
    crt = u4.GetCourtyard(pcbnew.F_CrtYd).BBox()
    band_bottom = None
    o = u4.GetCourtyard(pcbnew.F_CrtYd).Outline(0)
    ys = sorted({o.CPoint(i).y for i in range(o.PointCount())})
    band_bottom = pcbnew.ToMM(ys[1])                     # bottom of the wide antenna band
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        k = pcbnew.ZONE(board)
        k.SetIsRuleArea(True)
        k.SetDoNotAllowZoneFills(True); k.SetDoNotAllowTracks(True); k.SetDoNotAllowVias(True)
        k.SetLayer(layer)
        ol = k.Outline(); ol.NewOutline()
        x1, x2 = max(pcbnew.ToMM(crt.GetLeft()), X0), min(pcbnew.ToMM(crt.GetRight()), X0 + W)
        for x, y in [(x1, Y0), (x2, Y0), (x2, band_bottom), (x1, band_bottom)]:
            ol.Append(MM(x), MM(y))
        board.Add(k)
    ring = pcbnew.ZONE(board)
    ring.SetIsRuleArea(True)
    ring.SetDoNotAllowTracks(True); ring.SetDoNotAllowVias(True)
    ring.SetDoNotAllowZoneFills(False); ring.SetDoNotAllowPads(False); ring.SetDoNotAllowFootprints(False)
    ring.SetLayerSet(pcbnew.LSET.AllCuMask())
    ol = ring.Outline()
    e = 0.6
    for (xa, ya, xb, yb) in [(X0, Y0, X0 + W, Y0 + e), (X0, Y0 + H - e, X0 + W, Y0 + H),
                             (X0, Y0, X0 + e, Y0 + H), (X0 + W - e, Y0, X0 + W, Y0 + H)]:
        ol.NewOutline()
        for x, y in [(xa, ya), (xb, ya), (xb, yb), (xa, yb)]:
            ol.Append(MM(x), MM(y))
    board.Add(ring)
    report["courtyard_overlaps"] = overlaps(board)
    gnd_zone(board, pcbnew.B_Cu)               # bottom GND plane: the router drops vias into it
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    ok = pcbnew.ExportSpecctraDSN(board, DSN)
    report["dsn_exported"] = bool(ok)
    for sh in dsn_outline:
        board.Remove(sh)
    add_outline(board)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(PCB)
    print(json.dumps(report, indent=1))


def overlaps(board):
    """Courtyard polygon intersections (front), plus parts whose courtyard leaves the board."""
    items = []
    for fp in board.GetFootprints():
        poly = fp.GetCourtyard(pcbnew.F_CrtYd)
        if poly.OutlineCount():
            items.append((fp.GetReference(), poly))
    out = []
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a = pcbnew.SHAPE_POLY_SET(items[i][1])
            a.BooleanIntersection(items[j][1])
            if a.OutlineCount() and a.Area() > MM(0.1) * MM(0.1):
                out.append(f"{items[i][0]}-{items[j][0]}")
    edge = pcbnew.SHAPE_POLY_SET()
    edge.NewOutline()
    for x, y in [(X0, Y0), (X0 + W, Y0), (X0 + W, Y0 + H), (X0, Y0 + H)]:
        edge.Append(MM(x), MM(y))
    for ref, poly in items:
        if ref in ("U4", "J3"):        # antenna band and USB-C lip overhang the edge by design
            continue
        a = pcbnew.SHAPE_POLY_SET(poly)
        a.BooleanSubtract(edge)
        if a.OutlineCount() and a.Area() > MM(0.1) * MM(0.1):
            out.append(f"{ref}-OFFBOARD")
    return out


def gnd_zone(board, layer):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNetCode(board.GetNetcodeFromNetname("/GND"))
    z.SetLocalClearance(MM(0.3))
    z.SetMinThickness(MM(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)   # avoids starved thermals on small pads (rev A)
    if layer == pcbnew.F_Cu:
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)   # top pour is optional copper; B.Cu is the plane
    ol = z.Outline()
    ol.NewOutline()
    for x, y in [(X0 + 0.5, Y0 + 0.5), (X0 + W - 0.5, Y0 + 0.5), (X0 + W - 0.5, Y0 + H - 0.5), (X0 + 0.5, Y0 + H - 0.5)]:
        ol.Append(MM(x), MM(y))
    board.Add(z)
    return z


def nudge_edge_tracks(board, margin=0.65):
    """Freerouting only keeps track clearance (0.2 mm) to the outline. Pull any track vertex that is
    closer than `margin` to an edge back inside; shared vertices move identically, so connectivity holds."""
    lo_x, hi_x, lo_y, hi_y = MM(X0 + margin), MM(X0 + W - margin), MM(Y0 + margin), MM(Y0 + H - margin)
    moved = 0
    for t in board.GetTracks():
        if t.GetClass() != "PCB_TRACK":
            continue
        for get, setp in ((t.GetStart, t.SetStart), (t.GetEnd, t.SetEnd)):
            p = get()
            hw = t.GetWidth() // 2
            nx = min(max(p.x, lo_x + hw), hi_x - hw)
            ny = min(max(p.y, lo_y + hw), hi_y - hw)
            if (nx, ny) != (p.x, p.y):
                setp(pcbnew.VECTOR2I(nx, ny)); moved += 1
    print("edge vertices nudged:", moved)


def stitch_gnd(board, pitch=5.0):
    """GND stitching: a coarse grid of vias where both pours are solid, then one extra via in every
    top-layer GND island that still has none, so no F.Cu island floats."""
    gnd = board.GetNetcodeFromNetname("/GND")
    zones = {z.GetLayer(): z for z in board.Zones() if z.GetNetCode() == gnd and not z.GetIsRuleArea()}
    keep = [z for z in board.Zones() if z.GetIsRuleArea() and z.GetDoNotAllowVias() and z.GetDoNotAllowZoneFills()]
    holes = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA"]
    holes += [p.GetPosition() for p in board.GetPads() if p.HasHole()]
    gnd_holes = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA" and v.GetNetCode() == gnd]
    gnd_holes += [p.GetPosition() for p in board.GetPads() if p.HasHole() and p.GetNetCode() == gnd]
    added = []

    def can_place(x, y):
        pt = pcbnew.VECTOR2I(MM(x), MM(y))
        probes = [pcbnew.VECTOR2I(MM(x + dx), MM(y + dy)) for dx, dy in
                  ((0, 0), (0.4, 0), (-0.4, 0), (0, 0.4), (0, -0.4), (0.3, 0.3), (-0.3, -0.3), (0.3, -0.3), (-0.3, 0.3))]
        if not all(zones[l].HitTestFilledArea(l, q, 0) for l in (pcbnew.F_Cu, pcbnew.B_Cu) for q in probes):
            return None
        if any(k.Outline().Contains(pt) for k in keep):
            return None
        if any((h.x - pt.x) ** 2 + (h.y - pt.y) ** 2 < MM(1.2) ** 2 for h in holes + added):
            return None
        return pt

    def add(pt):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pt); v.SetWidth(MM(0.6)); v.SetDrill(MM(0.3)); v.SetNetCode(gnd)
        board.Add(v); added.append(pt)

    x = X0 + 2.5
    while x < X0 + W - 2:
        y = Y0 + 2.5
        while y < Y0 + H - 2:
            pt = can_place(x, y)
            if pt:
                add(pt)
            y += pitch
        x += pitch
    # islands without a via
    filled = zones[pcbnew.F_Cu].GetFilledPolysList(pcbnew.F_Cu)
    fixed = 0
    for i in range(filled.OutlineCount()):
        poly = pcbnew.SHAPE_POLY_SET(); poly.AddOutline(filled.Outline(i))
        if any(poly.Contains(h) for h in gnd_holes + added):
            continue
        bb = filled.Outline(i).BBox()
        done = False
        yy = pcbnew.ToMM(bb.GetTop())
        while yy <= pcbnew.ToMM(bb.GetBottom()) and not done:
            xx = pcbnew.ToMM(bb.GetLeft())
            while xx <= pcbnew.ToMM(bb.GetRight()) and not done:
                pt = can_place(xx, yy)
                if pt and poly.Contains(pt):
                    add(pt); done = True; fixed += 1
                xx += 0.25
            yy += 0.25
    print("GND stitching vias:", len(added), "| islands given a via:", fixed)


def import_ses(ses):
    board = pcbnew.LoadBoard(PCB)
    has_top = any(not z.GetIsRuleArea() and z.GetLayer() == pcbnew.F_Cu for z in board.Zones())
    ok = pcbnew.ImportSpecctraSES(board, ses)
    print("ses imported:", ok)
    if not has_top:
        gnd_zone(board, pcbnew.F_Cu)           # top GND pour (bottom plane already exists)
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    stitch_gnd(board)
    filler.Fill(board.Zones())
    board.Save(PCB)
    print("pours added and filled")


def set_outline_inset(board, inset):
    """Move the existing Edge.Cuts rectangle in place (pcbnew Remove() is unreliable from Python)."""
    i = inset
    corners = [(X0 + i, Y0 + i), (X0 + W - i, Y0 + i), (X0 + W - i, Y0 + H - i), (X0 + i, Y0 + H - i)]
    segs = [d for d in board.Drawings() if d.GetLayer() == pcbnew.Edge_Cuts]
    for k, sgm in enumerate(sorted(segs, key=lambda d: (d.GetStart().y, d.GetStart().x))):
        pass
    # match each segment to a side by its midpoint
    for sgm in segs:
        a, b = sgm.GetStart(), sgm.GetEnd()
        mx, my = pcbnew.ToMM((a.x + b.x) // 2), pcbnew.ToMM((a.y + b.y) // 2)
        if abs(a.y - b.y) < 10:            # horizontal
            yv = Y0 + i if my < Y0 + H / 2 else Y0 + H - i
            sgm.SetStart(pcbnew.VECTOR2I(MM(X0 + i), MM(yv))); sgm.SetEnd(pcbnew.VECTOR2I(MM(X0 + W - i), MM(yv)))
        else:
            xv = X0 + i if mx < X0 + W / 2 else X0 + W - i
            sgm.SetStart(pcbnew.VECTOR2I(MM(xv), MM(Y0 + i))); sgm.SetEnd(pcbnew.VECTOR2I(MM(xv), MM(Y0 + H - i)))


def export_routed(dsn_out):
    """Re-export the routed board (tracks kept) so Freerouting only completes what is missing."""
    board = pcbnew.LoadBoard(PCB)
    set_outline_inset(board, 0.4)
    print("dsn exported:", pcbnew.ExportSpecctraDSN(board, dsn_out))    # board itself is not saved


def bridge_gnd_islands():
    """Union-find the GND pour fragments (both layers) through GND vias / plated GND holes; for every
    group not connected to the main group, drop a via where one of its fragments overlaps a main-group
    fragment on the other layer."""
    board = pcbnew.LoadBoard(PCB)
    gnd = board.GetNetcodeFromNetname("/GND")
    Z = {z.GetLayer(): z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == gnd}
    for it in range(5):
        nodes = []
        for L in (pcbnew.F_Cu, pcbnew.B_Cu):
            f = Z[L].GetFilledPolysList(L)
            for i in range(f.OutlineCount()):
                P = pcbnew.SHAPE_POLY_SET(); P.AddOutline(f.Outline(i))
                nodes.append((L, P, f.Outline(i).BBox()))
        par = list(range(len(nodes)))

        def fd(x):
            while par[x] != x:
                x = par[x]
            return x
        hs = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA" and v.GetNetCode() == gnd]
        hs += [p.GetPosition() for p in board.GetPads() if p.HasHole() and p.GetNetCode() == gnd]
        allholes = [v.GetPosition() for v in board.GetTracks() if v.GetClass() == "PCB_VIA"]
        allholes += [p.GetPosition() for p in board.GetPads() if p.HasHole()]
        for h in hs:
            ids = [k for k, n in enumerate(nodes) if n[1].Contains(h)]
            for k in ids[1:]:
                par[fd(k)] = fd(ids[0])
        groups = {}
        for k in range(len(nodes)):
            groups.setdefault(fd(k), []).append(k)
        if len(groups) <= 1:
            print("GND pour groups: 1 (all connected)")
            break
        main = max(groups.values(), key=lambda ks: sum(nodes[k][1].Area() for k in ks))
        added = 0
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
                        probes = [pcbnew.VECTOR2I(MM(x + dx), MM(y + dy)) for dx, dy in
                                  ((0, 0), (0.4, 0), (-0.4, 0), (0, 0.4), (0, -0.4), (0.3, 0.3), (-0.3, -0.3), (0.3, -0.3), (-0.3, 0.3))]
                        if all(P.Contains(q) for q in probes) and any(all(O.Contains(q) for q in probes) for O in others) \
                                and all((h.x - probes[0].x) ** 2 + (h.y - probes[0].y) ** 2 > MM(1.0) ** 2 for h in allholes):
                            v = pcbnew.PCB_VIA(board)
                            v.SetPosition(probes[0]); v.SetWidth(MM(0.6)); v.SetDrill(MM(0.3)); v.SetNetCode(gnd)
                            board.Add(v); done = True; added += 1
                        x += 0.2
                    y += 0.2
                if done:
                    break
        print(f"pass {it}: {len(groups)} GND pour groups, bridging vias added: {added}")
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        if not added:
            break
    board.Save(PCB)


def unconnected_count():
    board = pcbnew.LoadBoard(PCB)
    board.BuildConnectivity()
    return board.GetConnectivity().GetUnconnectedCount(True)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "place"
    if cmd == "place":
        place()
    elif cmd == "import_ses":
        import_ses(sys.argv[2])
    elif cmd == "export_routed":
        export_routed(sys.argv[2])
    elif cmd == "bridge":
        bridge_gnd_islands()
    elif cmd == "unconnected":
        print("UNCONNECTED", unconnected_count())
