"""PCB configuration for Board A (used by ../../tools/build_pcb.py)."""
NAME = "atlas_carrier_ctrl"
DESIGN = "board_a_design.py"
X0, Y0, W, H = 100.0, 100.0, 130.0, 100.0         # front edge (y = 100): ESP32 antenna, bay harnesses
HOLES = [(104.0, 104.0), (226.0, 104.0), (104.0, 196.0), (226.0, 196.0)]   # pad edge >= 0.8 mm from the outline
ANTENNA_REF = "U4"
FIXED = {
    "U4": (165.0, 113.0, 0),                       # moved so its body is flush with the front edge
    # front edge: bay sensors + latch solenoids
    "J6": (115.0, 104.0, 0), "J8": (128.0, 104.5, 0),
    "J7": (200.0, 104.0, 0), "J9": (213.0, 104.5, 0),
    # left edge: battery, fuse, left wheels
    "J1": (104.5, 150.0, 90), "J12": (104.0, 120.0, 90), "J16": (103.5, 134.0, 90),
    "J13": (104.0, 168.0, 90), "J17": (103.5, 182.0, 90),
    # right edge: right wheels, Qwiic
    "J14": (226.0, 120.0, 270), "J18": (226.5, 134.0, 270),
    "J15": (226.0, 168.0, 270), "J19": (226.5, 182.0, 270), "J10": (225.5, 155.0, 270),
    # rear edge: USB-C to the ROS computer, Board C link + power, SBC power, E-stop
    "J4": (150.0, 196.5, 0), "J11": (180.0, 195.0, 90), "J3": (134.5, 195.5, 0), "J2": (119.0, 195.5, 0),
    "J5": (211.0, 196.0, 0),
}
POWER_NETS = {"VBAT_RAW", "VBAT_F", "VBAT", "VM_SW", "SW_5V", "VCC_5V",
              "VM_LF", "VM_LR", "VM_RF", "VM_RR", "MOT_LF_P", "MOT_LF_N", "MOT_LR_P", "MOT_LR_N",
              "MOT_RF_P", "MOT_RF_N", "MOT_RR_P", "MOT_RR_N", "SOL_1", "SOL_2"}
PLACEMENT_IGNORE = {"I2C_SDA", "I2C_SCL", "MOT_L_IN1", "MOT_L_IN2", "MOT_R_IN1", "MOT_R_IN2"}
REGIONS = [
    (r"^(R4\d|U11|U15|C3[0-3])$", (100.0, 112.0, 150.0, 150.0)),     # LF wheel
    (r"^(R5\d|U12|U16|C3[4-7])$", (100.0, 150.0, 150.0, 192.0)),     # LR wheel
    (r"^(R6\d|U13|U17|C3[89]|C4[01])$", (180.0, 112.0, 230.0, 150.0)),  # RF wheel
    (r"^(R7\d|U14|U18|C4[2-5])$", (180.0, 150.0, 230.0, 192.0)),     # RR wheel
]
OFFBOARD_OK = {"J1", "J4", "U4"}

# GND fan-out off: on this board the extra vias blocked the 0.5 mm VBAT routes (47 / 15 unrouted);
# the few GND pads the pours miss are connected afterwards by fix_islands (grid router)
GND_FANOUT = False
# GND_ROUTED: GND routed as an ordinary net in the first routing pass (plane removed from the DSN), so every GND pin
# is tied by tracks and no pin can be caged off from the pour by signal tracks (rounds 9-13 left 1-6 such
# pins, at a different place each run); the pours are added afterwards as usual
# (GND_ROUTED = True on 2 layers, round 14: the router no longer converged, 15-20 open)
# keep a 2.5 mm escape channel around the ESP32 module: with U10/U9 placed right under its pad row the
# router left 8-11 U4 nets open (round 11)
PART_GAP = [(r"^U4$", 5.0)]

# 4-layer (round 16, chosen 2026-10-14): on 2 layers Freerouting left 3-6 signals open on every run and GND
# pins were caged off from the pours. In1.Cu = GND plane; In2.Cu = third signal layer (a VCC_3V3 plane there
# left 140+ connections unrouted in Freerouting 2.1)
# Freerouting 2.1 also mishandles the In1.Cu GND plane (NullPointerException in the plane search, 87 open),
# so all four layers are routed with GND as an ordinary net; GND pours then fill every layer around the tracks
LAYERS = 4
POWER_PLANE = None
GND_ROUTED = True

# placement spacing: the greedy placer's default 0.6 mm courtyard gap packed U5/U9/U10 into a clump where
# neither Freerouting nor a grid router could escape their pins (round 18); 1.5 mm leaves routing channels
GAP = 1.5
