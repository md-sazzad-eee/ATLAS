"""Run fix_islands on a copy of a board (debug): KiCad-python try_fix_islands.py <cfg> <copy.kicad_pcb>"""
import sys, os
cfgp, pcbp = sys.argv[1], sys.argv[2]
sys.argv = [sys.argv[0], cfgp]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build_pcb as B
B.PCB = pcbp
B.fix_islands()
