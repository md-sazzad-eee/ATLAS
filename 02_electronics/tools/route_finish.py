"""Finish the last few open connections that Freerouting leaves on a dense board.

Freerouting 2.x reliably routes ~99 % of Board A and then stalls on a handful of nets around the ESP32 module,
in a different place every run. This tool connects what is left with a small, deterministic grid router
(Dijkstra/A* over all copper layers, through-vias anywhere they fit), using exact KiCad geometry:

  KiCad-python route_finish.py <board_cfg.py> dump  <geom.json>     board -> geometry + open clusters
  python3 (numpy) route_finish.py route <geom.json> <routes.json>   grid routing, no KiCad needed
  KiCad-python route_finish.py <board_cfg.py> apply <routes.json>   tracks/vias -> board, zones refilled

Rules it keeps: net-class track width, CLEARANCE to every other-net copper on the same layer (vias: on every
layer), 0.6 mm hole-to-hole spacing for new vias, 0.5 mm copper-to-edge, rule-area keep-outs. Track pieces
that belong to no pad (router leftovers) are deleted in `apply`. GND is not routed here (planes/pours).
"""
import json, math, os, sys

SAFETY = 0.02           # mm added to every clearance on top of grid quantisation margin


# ----------------------------------------------------------------------------------------------- KiCad side
def _kicad_init(cfgp):
    sys.argv = [sys.argv[0], cfgp]
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import build_pcb as B
    return B


def _layers(B, pcbnew):
    return [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu] if B.LAYERS == 4 else [pcbnew.F_Cu, pcbnew.B_Cu]


def _via_width(v, pcbnew):
    try:
        return pcbnew.ToMM(v.GetWidth(pcbnew.F_Cu))
    except TypeError:
        return pcbnew.ToMM(v.GetWidth())


def _pad_poly(p, L, pcbnew):
    try:
        ps = p.GetEffectivePolygon(L, pcbnew.ERROR_INSIDE)
    except TypeError:
        ps = p.GetEffectivePolygon(L)
    except AttributeError:
        ps = p.GetEffectivePolygon()
    out = []
    for k in range(ps.OutlineCount()):
        o = ps.Outline(k)
        out.append([(pcbnew.ToMM(o.CPoint(i).x), pcbnew.ToMM(o.CPoint(i).y)) for i in range(o.PointCount())])
    return out


def dump(cfgp, out):
    B = _kicad_init(cfgp)
    import pcbnew
    b = pcbnew.LoadBoard(B.PCB)
    b.BuildConnectivity()
    conn = b.GetConnectivity()
    Ls = _layers(B, pcbnew)
    lname = {L: b.GetLayerName(L) for L in Ls}
    gnd = b.GetNetcodeFromNetname("/GND")
    items = []                                          # every copper item, with a uuid
    for t in b.GetTracks():
        c = t.GetClass()
        rec = {"uuid": t.m_Uuid.AsString(), "net": t.GetNetname(), "code": t.GetNetCode()}
        if c == "PCB_VIA":
            rec.update(kind="via", x=pcbnew.ToMM(t.GetPosition().x), y=pcbnew.ToMM(t.GetPosition().y),
                       d=_via_width(t, pcbnew), drill=pcbnew.ToMM(t.GetDrillValue()), layers=[lname[L] for L in Ls])
        else:
            pts = [t.GetStart(), t.GetEnd()]
            if c == "PCB_ARC":
                pts = [t.GetStart(), t.GetMid(), t.GetEnd()]
            rec.update(kind="track", w=pcbnew.ToMM(t.GetWidth()), layer=lname.get(t.GetLayer()),
                       pts=[(pcbnew.ToMM(q.x), pcbnew.ToMM(q.y)) for q in pts])
            if rec["layer"] is None:
                continue
        items.append(rec)
    for p in b.GetPads():
        fp = p.GetParentFootprint()
        rec = {"uuid": p.m_Uuid.AsString(), "net": p.GetNetname(), "code": p.GetNetCode(), "kind": "pad",
               "ref": fp.GetReference() + "." + p.GetNumber(),
               "polys": {lname[L]: _pad_poly(p, L, pcbnew) for L in Ls if p.IsOnLayer(L)}}
        if p.HasHole():
            ds = p.GetDrillSize()
            rec["hole"] = (pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y), pcbnew.ToMM(max(ds.x, ds.y)))
        items.append(rec)
    keep = []
    for z in b.Zones():
        if z.GetIsRuleArea() and (z.GetDoNotAllowTracks() or z.GetDoNotAllowVias()):
            bb = z.Outline().BBox()
            keep.append({"box": [pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()), pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())],
                         "layers": [lname[L] for L in Ls if z.GetLayerSet().Contains(L)],
                         "tracks": bool(z.GetDoNotAllowTracks()), "vias": bool(z.GetDoNotAllowVias())})
    # connected components per net (pads, tracks, vias); nets with more than one component are open
    by_uuid = {}
    for t in list(b.GetTracks()) + list(b.GetPads()):
        by_uuid[t.m_Uuid.AsString()] = t
    open_nets = {}
    netcodes = {r["code"] for r in items if r["code"] > 0 and r["code"] != gnd}
    for code in sorted(netcodes):
        mem = [r["uuid"] for r in items if r["code"] == code]
        if len(mem) < 2:
            continue
        seen, comps = set(), []
        for u in mem:
            if u in seen:
                continue
            ids = {u} | {x.m_Uuid.AsString() for x in conn.GetConnectedItems(by_uuid[u])}
            ids &= set(mem)
            seen |= ids
            comps.append(sorted(ids))
        if len(comps) > 1:
            name = next(r["net"] for r in items if r["code"] == code)
            open_nets[name] = comps
    power = {"/" + n for n in B.cfg.POWER_NETS}
    geom = {"X0": B.X0, "Y0": B.Y0, "W": B.W, "H": B.H, "clear": B.CLEAR, "track": B.TRACK,
            "edge": 0.5, "layers": [lname[L] for L in Ls], "items": items, "keepouts": keep,
            "open": open_nets, "power": sorted(power)}
    json.dump(geom, open(out, "w"))
    print(f"dump: {len(items)} items, open nets: {len(open_nets)} "
          f"{ {k: len(v) for k, v in open_nets.items()} }")


def apply(cfgp, routes):
    B = _kicad_init(cfgp)
    import pcbnew
    MM = pcbnew.FromMM
    r = json.load(open(routes))
    b = pcbnew.LoadBoard(B.PCB)
    Ls = _layers(B, pcbnew)
    lid = {b.GetLayerName(L): L for L in Ls}
    dead = set(r.get("delete", []))
    gone = 0
    for t in list(b.GetTracks()):
        if t.m_Uuid.AsString() in dead:
            b.Delete(t); gone += 1
    added = 0
    for rt in r["routes"]:
        code = b.GetNetcodeFromNetname(rt["net"])
        for s in rt["segs"]:
            t = pcbnew.PCB_TRACK(b)
            t.SetStart(pcbnew.VECTOR2I(MM(s[1]), MM(s[2]))); t.SetEnd(pcbnew.VECTOR2I(MM(s[3]), MM(s[4])))
            t.SetWidth(MM(rt["w"])); t.SetLayer(lid[s[0]]); t.SetNetCode(code); b.Add(t); added += 1
        for v in rt["vias"]:
            vi = pcbnew.PCB_VIA(b); vi.SetPosition(pcbnew.VECTOR2I(MM(v[0]), MM(v[1])))
            vi.SetWidth(MM(rt["via_d"])); vi.SetDrill(MM(rt["via_drill"])); vi.SetNetCode(code); b.Add(vi); added += 1
    b.BuildConnectivity()
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    b.Save(B.PCB)
    b = pcbnew.LoadBoard(B.PCB); b.BuildConnectivity()
    print(f"apply: {gone} leftover track pieces deleted, {added} tracks/vias added, failed: {r.get('failed', [])}, "
          f"unconnected now: {b.GetConnectivity().GetUnconnectedCount(True)}")


# ------------------------------------------------------------------------------------------- numpy router
def route(geom_path, out):
    import heapq
    import numpy as np
    g = json.load(open(geom_path))
    LN = g["layers"]; nL = len(LN); li = {n: k for k, n in enumerate(LN)}
    clear, edge = g["clear"], g["edge"]
    X0, Y0, W, H = g["X0"], g["Y0"], g["W"], g["H"]
    items = g["items"]
    by_uuid = {it["uuid"]: it for it in items}
    power = set(g["power"])
    keepouts = g["keepouts"]

    def seg_dist(px, py, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 == 0:
            return np.hypot(px - ax, py - ay)
        t = np.clip(((px - ax) * dx + (py - ay) * dy) / L2, 0, 1)
        return np.hypot(px - (ax + t * dx), py - (ay + t * dy))

    def poly_dist(px, py, poly):
        """distance to the polygon outline, 0 inside"""
        d = np.full(px.shape, np.inf)
        inside = np.zeros(px.shape, bool)
        n = len(poly)
        for k in range(n):
            ax, ay = poly[k]; bx, by = poly[(k + 1) % n]
            d = np.minimum(d, seg_dist(px, py, ax, ay, bx, by))
            cond = ((ay > py) != (by > py))
            with np.errstate(divide="ignore", invalid="ignore"):
                xint = ax + (py - ay) * (bx - ax) / (by - ay)
            inside ^= cond & (px < xint)
        d[inside] = 0
        return d

    def shapes(it):
        """(layer index or None for all, kind, params, bbox) copper shapes of one item"""
        if it["kind"] == "track":
            out = []
            p = it["pts"]
            for a, b_ in zip(p, p[1:]):
                out.append((li[it["layer"]], "seg", (a[0], a[1], b_[0], b_[1], it["w"] / 2)))
            return out
        if it["kind"] == "via":
            return [(None, "circ", (it["x"], it["y"], it["d"] / 2))]
        out = []
        for ln, polys in it["polys"].items():
            for poly in polys:
                out.append((li[ln], "poly", poly))
        return out

    def bbox_of(kind, prm):
        if kind == "seg":
            x1, y1, x2, y2, r = prm
            return min(x1, x2) - r, min(y1, y2) - r, max(x1, x2) + r, max(y1, y2) + r
        if kind == "circ":
            x, y, r = prm
            return x - r, y - r, x + r, y + r
        xs = [q[0] for q in prm]; ys = [q[1] for q in prm]
        return min(xs), min(ys), max(xs), max(ys)

    def dist_field(win, res, sel, grow):
        """per layer: distance (mm) from each cell to the nearest copper of the selected items (inf if > grow)"""
        x0, y0, nx, ny = win
        D = np.full((nL, ny, nx), np.inf)
        for it in sel:
            for L, kind, prm in shapes(it):
                bx0, by0, bx1, by1 = bbox_of(kind, prm)
                i0 = max(0, int((bx0 - grow - x0) / res)); i1 = min(nx, int((bx1 + grow - x0) / res) + 2)
                j0 = max(0, int((by0 - grow - y0) / res)); j1 = min(ny, int((by1 + grow - y0) / res) + 2)
                if i0 >= i1 or j0 >= j1:
                    continue
                xs = x0 + np.arange(i0, i1) * res; ys = y0 + np.arange(j0, j1) * res
                PX, PY = np.meshgrid(xs, ys)
                if kind == "seg":
                    d = seg_dist(PX, PY, *prm[:4]) - prm[4]
                elif kind == "circ":
                    d = np.hypot(PX - prm[0], PY - prm[1]) - prm[2]
                else:
                    d = poly_dist(PX, PY, prm)
                d = np.maximum(d, 0)
                for k in (range(nL) if L is None else [L]):
                    D[k, j0:j1, i0:i1] = np.minimum(D[k, j0:j1, i0:i1], d)
        return D

    holes = []
    for it in items:
        if it["kind"] == "via":
            holes.append((it["x"], it["y"], it["drill"]))
        elif it.get("hole"):
            holes.append(tuple(it["hole"]))

    routes, failed, delete = [], [], []
    extra = []                                          # copper added by this run (obstacles for later nets)
    # leftover pieces: components without a pad are deleted, not routed
    jobs = []
    for net, comps in g["open"].items():
        withpad = [c for c in comps if any(by_uuid[u]["kind"] == "pad" for u in c)]
        for c in comps:
            if c not in withpad:
                delete += [u for u in c if by_uuid[u]["kind"] != "pad"]
        if len(withpad) > 1:
            withpad.sort(key=lambda c: -sum(by_uuid[u]["kind"] == "pad" for u in c))
            for c in withpad[1:]:
                jobs.append((net, c, withpad[0]))
    dead = set(delete)

    def cbox(c):
        bs = [bbox_of(k, p) for u in c for (_, k, p) in shapes(by_uuid[u])]
        return min(b[0] for b in bs), min(b[1] for b in bs), max(b[2] for b in bs), max(b[3] for b in bs)

    def area(job):
        a, b_ = cbox(job[1]), cbox(job[2])
        return (max(a[2], b_[2]) - min(a[0], b_[0])) * (max(a[3], b_[3]) - min(a[1], b_[1]))
    jobs.sort(key=area)
    for net, src, dst in jobs:
        pw = net in power
        w = 0.5 if pw else g["track"]
        ok = False
        for margin, res, vd, vdr in ((4.0, 0.1, 0.6, 0.3), (10.0, 0.1, 0.6, 0.3), (10.0, 0.1, 0.5, 0.25), (25.0, 0.15, 0.5, 0.25)):
            if pw:
                vd, vdr = 0.8, 0.4
            a, b_ = cbox(src), cbox(dst)
            wx0 = max(X0, min(a[0], b_[0]) - margin); wy0 = max(Y0, min(a[1], b_[1]) - margin)
            wx1 = min(X0 + W, max(a[2], b_[2]) + margin); wy1 = min(Y0 + H, max(a[3], b_[3]) + margin)
            nx, ny = int((wx1 - wx0) / res) + 1, int((wy1 - wy0) / res) + 1
            win = (wx0, wy0, nx, ny)
            q = 0.01 + SAFETY       # a straight move between two clear cell centres dips < 0.01 mm toward round copper
            rt, rv = w / 2 + clear + q, vd / 2 + clear + q
            others = [it for it in items if it["net"] != net and it["uuid"] not in dead] + [e for e in extra if e["net"] != net]
            Do = dist_field(win, res, others, rv + 0.1)
            own_s = dist_field(win, res, [by_uuid[u] for u in src], 0.2)
            own_d = dist_field(win, res, [by_uuid[u] for u in dst] + [e for e in extra if e["net"] == net], 0.2)
            blocked = Do < rt
            viaok = (Do.min(axis=0) >= rv)
            xs = wx0 + np.arange(nx) * res; ys = wy0 + np.arange(ny) * res
            PX, PY = np.meshgrid(xs, ys)
            inb = (PX > X0 + edge + w / 2) & (PX < X0 + W - edge - w / 2) & (PY > Y0 + edge + w / 2) & (PY < Y0 + H - edge - w / 2)
            blocked |= ~inb[None]
            viaok &= (PX > X0 + edge + vd / 2) & (PX < X0 + W - edge - vd / 2) & (PY > Y0 + edge + vd / 2) & (PY < Y0 + H - edge - vd / 2)
            for hx, hy, hd in holes + [(e["x"], e["y"], e["drill"]) for e in extra if e["kind"] == "via"]:
                if wx0 - 2 < hx < wx1 + 2 and wy0 - 2 < hy < wy1 + 2:
                    viaok &= np.hypot(PX - hx, PY - hy) >= hd / 2 + vdr / 2 + 0.25 + SAFETY
            for k in keepouts:
                x1, y1, x2, y2 = k["box"]
                m = (PX > x1 - rt) & (PX < x2 + rt) & (PY > y1 - rt) & (PY < y2 + rt)
                if k["tracks"]:
                    for ln in k["layers"]:
                        blocked[li[ln]] |= m
                if k["vias"]:
                    viaok &= ~((PX > x1 - rv) & (PX < x2 + rv) & (PY > y1 - rv) & (PY < y2 + rv))
            start = own_s <= 0.0
            goal = own_d <= 0.0
            passable = ~blocked | start | goal             # own copper may be crossed (it is the same net)
            gj, gi = np.nonzero(goal.any(axis=0))
            if not len(gi) or not start.any():
                continue
            gx0, gx1, gy0, gy1 = gi.min(), gi.max(), gj.min(), gj.max()
            N = nL * ny * nx
            dist = np.full(N, np.inf, dtype=np.float64)
            prev = np.full(N, -1, dtype=np.int64)
            pq = []
            for k, j, i in zip(*np.nonzero(start)):
                s = (k * ny + j) * nx + i
                dist[s] = 0
                heapq.heappush(pq, (0.0, s))
            goal_f = goal.ravel(); pass_f = passable.ravel(); via_f = viaok.ravel()
            VIA_COST = 30.0
            found = -1
            nb = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142))
            while pq:
                d, s = heapq.heappop(pq)
                if d > dist[s]:
                    continue
                if goal_f[s] and d > 0:
                    found = s; break
                k, rem = divmod(s, ny * nx); j, i = divmod(rem, nx)
                for di, dj, c in nb:
                    a_, b2 = i + di, j + dj
                    if 0 <= a_ < nx and 0 <= b2 < ny:
                        t = (k * ny + b2) * nx + a_
                        if pass_f[t]:
                            if di and dj and not (pass_f[(k * ny + j) * nx + a_] and pass_f[(k * ny + b2) * nx + i]):
                                continue                    # no corner cutting
                            nd = d + c
                            if nd < dist[t]:
                                dist[t] = nd; prev[t] = s
                                hx = max(gx0 - a_, 0, a_ - gx1); hy = max(gy0 - b2, 0, b2 - gy1)
                                heapq.heappush(pq, (nd + max(hx, hy), t))      # Chebyshev: admissible
                if via_f[rem]:
                    for k2 in range(nL):
                        if k2 != k:
                            t = (k2 * ny + j) * nx + i
                            if pass_f[t] and d + VIA_COST < dist[t]:
                                dist[t] = d + VIA_COST; prev[t] = s
                                hx = max(gx0 - i, 0, i - gx1); hy = max(gy0 - j, 0, j - gy1)
                                heapq.heappush(pq, (d + VIA_COST + max(hx, hy), t))
            if found < 0:
                continue
            path = [found]
            while prev[path[-1]] >= 0:
                path.append(int(prev[path[-1]]))
            path.reverse()
            pts = []
            for s in path:
                k, rem = divmod(s, ny * nx); j, i = divmod(rem, nx)
                pts.append((k, round(wx0 + i * res, 4), round(wy0 + j * res, 4)))
            segs, vias = [], []
            run = [pts[0]]
            for p in pts[1:]:
                if p[0] != run[-1][0]:
                    if len(run) > 1:
                        segs.append((run[0], run[-1]))
                    vias.append((p[1], p[2]))
                    run = [p]
                    continue
                if len(run) >= 2:
                    d1 = (run[-1][1] - run[-2][1], run[-1][2] - run[-2][2]); d2 = (p[1] - run[-1][1], p[2] - run[-1][2])
                    if abs(d1[0] * d2[1] - d1[1] * d2[0]) > 1e-9:
                        segs.append((run[0], run[-1])); run = [run[-1]]
                run.append(p)
            if len(run) > 1:
                segs.append((run[0], run[-1]))
            segs = [(LN[a[0]], a[1], a[2], b3[1], b3[2]) for a, b3 in segs if (a[1], a[2]) != (b3[1], b3[2])]
            routes.append({"net": net, "w": w, "via_d": vd, "via_drill": vdr, "segs": segs, "vias": vias,
                           "len": round(sum(math.hypot(s[3] - s[1], s[4] - s[2]) for s in segs), 2)})
            for s in segs:
                extra.append({"kind": "track", "net": net, "layer": s[0], "w": w, "pts": [(s[1], s[2]), (s[3], s[4])], "uuid": "new"})
            for v in vias:
                extra.append({"kind": "via", "net": net, "x": v[0], "y": v[1], "d": vd, "drill": vdr, "uuid": "new"})
            print(f"routed {net}: {len(segs)} segments, {len(vias)} vias, {routes[-1]['len']} mm (window +{margin} mm)")
            ok = True
            break
        if not ok:
            failed.append(net)
            print(f"FAILED {net}")
    json.dump({"routes": routes, "failed": failed, "delete": delete}, open(out, "w"))
    print(f"route: {len(routes)} connections routed, {len(failed)} failed, {len(delete)} leftover pieces to delete")


if __name__ == "__main__":
    if sys.argv[1] == "route":
        route(sys.argv[2], sys.argv[3])
    elif sys.argv[2] == "dump":
        dump(sys.argv[1], sys.argv[3])
    elif sys.argv[2] == "apply":
        apply(sys.argv[1], sys.argv[3])
