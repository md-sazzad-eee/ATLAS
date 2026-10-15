"""Debug the stray-GND grid router: dumps a PPM of the search grid per stray cluster.
usage: KiCad-python diag_astar.py <cfg> <board copy> <outdir>"""
import sys, os, inspect
cfgp, pcbp, out = sys.argv[1], sys.argv[2], sys.argv[3]
sys.argv = [sys.argv[0], cfgp]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_pcb as B
import pcbnew
src = inspect.getsource(B.astar_connect).replace("    if not found:\n        return False",
"""    img = bytearray()
    for jj in range(n):
        for ii in range(n):
            k = jj * n + ii
            r = 255 if blocked[0][k] else 0
            g = 255 if touch[0][k] or touch[1][k] else (120 if goal(0, ii - ii % 2, jj - jj % 2) or goal(1, ii - ii % 2, jj - jj % 2) else 0)
            b = 255 if blocked[1][k] else 0
            if (0, ii, jj) in dist or (1, ii, jj) in dist:\n                r = g = b = (255 if not viablk[k] else (170 if not blocked[1][k] else 80))
            img += bytes((r, g, b))
    fn = os.path.join(OUT, pad.GetParentFootprint().GetReference() + "_" + pad.GetNumber() + ".ppm")
    open(fn, "wb").write(b"P6 %d %d 255\\n" % (n, n) + bytes(img))
    print('   ', fn, 'reached', len(dist), 'touch', sum(touch[0]), sum(touch[1]), 'viablk', sum(viablk), 'found', found)
    if not found:
        return False""")
B.OUT = out
exec(src, B.__dict__)
b = pcbnew.LoadBoard(pcbp)
gnd = b.GetNetcodeFromNetname("/GND")
Z = {z.GetLayer(): z for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == gnd}
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
nodes, groups, main = B.gnd_groups(b, gnd, Z)
mainL = {L: [nodes[m][1] for m in main if nodes[m][0] == L] for L in (pcbnew.F_Cu, pcbnew.B_Cu)}
mi = B.main_gnd_items(b, gnd)
print("main items", len(mi))
for cl in B.stray_pad_clusters(b, gnd):
    B.astar_connect(b, cl[0], gnd, mainL, win=6.0, main_items=mi)
