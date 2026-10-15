"""PCB configuration for Board C (used by ../../tools/build_pcb.py)."""
NAME = "atlas_dock_charger"
DESIGN = "board_c_design.py"
X0, Y0, W, H = 100.0, 100.0, 90.0, 66.0          # front edge (y = 100) faces the two dock blocks
HOLES = [(104.0, 104.0), (186.0, 104.0), (104.0, 162.0), (186.0, 162.0)]   # pad edge >= 0.8 mm from the outline
TRACK, CLEARANCE = 0.2, 0.15
ZONE_CLEARANCE = 0.2
FIXED = {
    "J2": (113.0, 104.5, 0),                      # bay 1 dock harness
    "J3": (163.0, 104.5, 0),                      # bay 2 dock harness
    "J1": (104.0, 133.0, 90),                     # 4S power in (left edge)
    "J4": (145.0, 160.0, 90),                     # 2x8 to Board A (rear edge)
}
POWER_NETS = {"VIN_RAW", "VIN_F", "VIN", "SW_1", "SW_2", "VOUT_1", "VOUT_2", "VSH_1", "VSH_2",
              "VCHG_F_1", "VCHG_F_2"}
PLACEMENT_IGNORE = {"I2C_SDA", "I2C_SCL"}
# bay 1 channel (refs x1xx) left half, bay 2 (x2xx) right half, shared parts anywhere
REGIONS = [(r"^[A-Z]+1\d\d$", (100.0, 100.0, 148.0, 166.0)), (r"^[A-Z]+2\d\d$", (142.0, 100.0, 190.0, 166.0))]
OFFBOARD_OK = {"J1"}
# INA226 (MSOP-10, 0.5 mm pitch): 2 mm keep-clear so every pin, GND pin 7 included, can fan out
PART_GAP = [(r"^U[12]0[5-7]$", 2.0)]
