"""Diagnose GND pads cut off from the main GND cluster.
usage: KiCad-python diag_gnd_strays.py <board_cfg.py> <board.kicad_pcb>"""
import sys, math
cfgp, pcbp = sys.argv[1], sys.argv[2]
sys.argv = [sys.argv[0], cfgp]
import os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_pcb as B
import inspect
_src = inspect.getsource(B.astar_connect).replace("    if not found:\n        return False",
 "    print('    n', n, 'reached', len(dist), 'blocked F/B', sum(blocked[0]), sum(blocked[1]), 'viablk', sum(viablk), 'goal cells F/B', sum(goal(0,i,j) for i in range(0,n,2) for j in range(0,n,2)), sum(goal(1,i,j) for i in range(0,n,2) for j in range(0,n,2)))\n    if not found:\n        return False")
exec(_src, B.__dict__)
import pcbnew
MM = pcbnew.FromMM
b = pcbnew.LoadBoard(pcbp)
gnd = b.GetNetcodeFromNetname("/GND")
Z = {z.GetLayer(): z for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == gnd}
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
nodes, groups, main = B.gnd_groups(b, gnd, Z)
mainL = {L: [nodes[m][1] for m in main if nodes[m][0] == L] for L in (pcbnew.F_Cu, pcbnew.B_Cu)}
print("main pour fragments F/B:", len(mainL[pcbnew.F_Cu]), len(mainL[pcbnew.B_Cu]), " groups:", len(groups))
for cl in B.stray_pad_clusters(b, gnd):
    print("cluster:", [p.GetParentFootprint().GetReference() + "." + p.GetNumber() for p in cl])
    for p in cl:
        c = p.GetPosition(); fp = p.GetParentFootprint()
        s = f"  {fp.GetReference()}.{p.GetNumber()} at ({pcbnew.ToMM(c.x):.2f},{pcbnew.ToMM(c.y):.2f}) rot {fp.GetOrientationDegrees():.0f} layers F={p.IsOnLayer(pcbnew.F_Cu)} B={p.IsOnLayer(pcbnew.B_Cu)}"
        for L in (pcbnew.F_Cu, pcbnew.B_Cu):
            best = None
            for r10 in range(1, 300):
                r = r10 * 0.1
                for k in range(max(8, int(r * 10))):
                    a = 2 * math.pi * k / max(8, int(r * 10))
                    v = pcbnew.VECTOR2I(c.x + MM(r * math.cos(a)), c.y + MM(r * math.sin(a)))
                    if any(P.Contains(v) for P in mainL[L]):
                        best = (r, pcbnew.ToMM(v.x), pcbnew.ToMM(v.y)); break
                if best: break
            s += f" | nearest main {b.GetLayerName(L)}: {best}"
        # any own-pour fragment touching the pad?
        own = [k for k, n in enumerate(nodes) if n[1].Contains(c)]
        s += f" | in pour fragments {own}"
        print(s)
    for win in (12.0,):
        t = pcbnew.LoadBoard(pcbp)
        pcbnew.ZONE_FILLER(t).Fill(t.Zones())
        tn, tg, tm = B.gnd_groups(t, gnd, {z.GetLayer(): z for z in t.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == gnd})
        tL = {L: [tn[m][1] for m in tm if tn[m][0] == L] for L in (pcbnew.F_Cu, pcbnew.B_Cu)}
        tp = [q for q in t.GetPads() if q.m_Uuid.AsString() == cl[0].m_Uuid.AsString()][0]
        print(f"  astar win={win}:", B.astar_connect(t, tp, gnd, tL, win=win))
