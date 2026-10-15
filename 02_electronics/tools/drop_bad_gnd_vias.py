"""Remove GND vias that short / crowd other-net copper on any layer (repair for boards stitched before
via_clear checked inner layers). usage: KiCad-python drop_bad_gnd_vias.py <cfg> [board]"""
import sys, os
cfgp = sys.argv[1]; pcbp = sys.argv[2] if len(sys.argv) > 2 else None
sys.argv = [sys.argv[0], cfgp]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_pcb as B
import pcbnew
path = pcbp or B.PCB
b = pcbnew.LoadBoard(path)
gnd = b.GetNetcodeFromNetname("/GND")
bad = [v for v in b.GetTracks() if v.GetClass() == "PCB_VIA" and v.GetNetCode() == gnd and not B.via_clear(b, v.GetPosition(), 0.3)]
# stitching vias that landed on an inner track can come back carrying that track's net: a via exactly on
# the 5 mm stitching grid with no track ending at its centre is one of those
ends = {(t.GetStart().x, t.GetStart().y) for t in b.GetTracks() if t.GetClass() != "PCB_VIA"} | \
       {(t.GetEnd().x, t.GetEnd().y) for t in b.GetTracks() if t.GetClass() != "PCB_VIA"}
def on_grid(v):
    x, y = pcbnew.ToMM(v.GetPosition().x) - B.X0 - 2.5, pcbnew.ToMM(v.GetPosition().y) - B.Y0 - 2.5
    return abs(x / 5 - round(x / 5)) < 1e-4 and abs(y / 5 - round(y / 5)) < 1e-4
stray = [v for v in b.GetTracks() if v.GetClass() == "PCB_VIA" and v.GetNetCode() != gnd and on_grid(v)
         and (v.GetPosition().x, v.GetPosition().y) not in ends]
print("stitching vias with a foreign net removed:", [(v.GetNetname(), round(pcbnew.ToMM(v.GetPosition().x), 2), round(pcbnew.ToMM(v.GetPosition().y), 2)) for v in stray])
for v in bad + stray:
    b.Delete(v)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(path)
print("bad GND vias removed:", len(bad))
