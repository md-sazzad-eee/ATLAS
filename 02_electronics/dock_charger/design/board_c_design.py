"""ATLAS Board C (dock / charge interface on the carrier) — design source of truth.

Two identical bay channels, each:
  CV source   TPS54202 buck, 4S LiFePO4 (10-14.6 V) -> 8.34 V (130k/10.0k 0.1 %, Vref 0.596 V).
              Kept below 8.40 V so a shorted agent diode can never over-charge the 2S pack.
  CC loop     20 mOhm shunt -> INA180A2 (x50, 1 V/A) -> LMV358 integrator vs ISET -> Schottky + 10k
              into the buck FB node. ISET from an MCP4725 DAC through a 2:1 divider, so a runaway DAC
              can ask for at most 1.65 A (buck limit ~2 A). CC/CV loop simulated in
              05_simulation/python/cc_cv_charger_sim.py.
  Pad switch  AO3401A high-side, off unless EN_n = BAY_EN_n (Board A, only in DOCKED) AND
              KALERT_N_n (Kelvin over-resistance trip) -> buck EN driven by the same signal.
  Telemetry   INA226 #1 on the shunt (I, V at the pad); INA226 #2 / #3 measure the drop across the
              VCHG and GND contacts by 4-wire (Kelvin) sensing: each agent pad is touched by a force pin
              and a separate sense pin, so V_force - V_sense = I * R_contact exactly.
              COMSOL/MATLAB: > ~100 mOhm overheats the contact block -> INA226 ALERT trips EN in hardware.
  Data / ID   half-duplex UART on DATA (1k series, 10k pull-up, ESD), ID pull-up 10k -> ADC on Board A
              (agent R8: S1 10k -> 1.65 V, S2 22k -> 2.27 V, empty bay 3.3 V).
No MCU: Board A (carrier controller) owns BAY_EN (docking FSM), the I2C bus and the UARTs.
"""
from collections import defaultdict

R = "Resistor_SMD:R_0603_1608Metric"
R12 = "Resistor_SMD:R_1206_3216Metric"
C = "Capacitor_SMD:C_0603_1608Metric"
C12 = "Capacitor_SMD:C_1206_3216Metric"
SOT23 = "Package_TO_SOT_SMD:SOT-23"
SOT5 = "Package_TO_SOT_SMD:SOT-23-5"
SOT6 = "Package_TO_SOT_SMD:SOT-23-6"
DPAK = "Package_TO_SOT_SMD:TO-252-2"
LED = "LED_SMD:LED_0603_1608Metric"
TP = "TestPoint:TestPoint_Pad_D1.5mm"
SOD323 = "Diode_SMD:D_SOD-323"

P = []
def part(ref, lib, val, fp, x, y, rot=0, **pins):
    P.append((ref, lib, val, fp, x, y, rot, {k.lstrip("_"): v for k, v in pins.items()}))
def res(ref, val, x, y, a, b, fp=R):
    part(ref, "Device:R", val, fp, x, y, 0, _1=a, _2=b)
def cap(ref, val, x, y, a, b, fp=C):
    part(ref, "Device:C", val, fp, x, y, 0, _1=a, _2=b)

# ============================== 1. power entry (carrier 4S LiFePO4)
part("J1", "Connector_Generic:Conn_01x02", "VIN 4S (+,-)", "Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal",
     30, 40, 0, _1="VIN_RAW", _2="GND")
part("F1", "Device:Polyfuse", "3A hold PTC", "Fuse:Fuse_1812_4532Metric", 55, 40, 0, _1="VIN_RAW", _2="VIN_F")
part("Q1", "Device:Q_PMOS_GDS", "AOD409 (rev-pol)", DPAK, 80, 40, 0, _1="GND", _2="VIN_F", _3="VIN")
part("D1", "Device:D_TVS", "SMBJ16CA", "Diode_SMD:D_SMB", 100, 40, 0, _1="VIN", _2="GND")
part("C1", "Device:C_Polarized", "100uF 25V", "Capacitor_SMD:CP_Elec_8x10", 115, 40, 0, _1="VIN", _2="GND")
part("J4", "Connector_Generic:Conn_02x08_Odd_Even", "TO BOARD A", "Connector_PinHeader_2.54mm:PinHeader_2x08_P2.54mm_Vertical",
     160, 40, 0, _1="VCC_3V3", _2="GND", _3="I2C_SDA", _4="I2C_SCL", _5="BAY_EN_1", _6="BAY_EN_2",
     _7="DATA_TX_1", _8="DATA_RX_1", _9="DATA_TX_2", _10="DATA_RX_2", _11="ID_1", _12="ID_2",
     _13="KALERT_N_1", _14="KALERT_N_2", _15="GND", _16="GND")
res("R1", "2.2k", 190, 30, "VCC_3V3", "I2C_SDA")
res("R2", "2.2k", 200, 30, "VCC_3V3", "I2C_SCL")
cap("C2", "10uF", 210, 30, "VCC_3V3", "GND", "Capacitor_SMD:C_0805_2012Metric")

# INA226 address straps (A1, A0): 0x40 GND/GND, 0x46 VS/SDA, 0x44 VS/GND, 0x45 VS/VS, 0x42 GND/SDA, 0x43 GND/SCL
# (bay-1 contact monitor moved from 0x41 to 0x46: its A1 GND strap was the one pin the router could not reach)
ADDR = {1: [("GND", "GND"), ("VCC_3V3", "I2C_SDA"), ("VCC_3V3", "GND")],
        2: [("VCC_3V3", "VCC_3V3"), ("GND", "I2C_SDA"), ("GND", "I2C_SCL")]}

# ============================== 2. two bay channels
for n in (1, 2):
    y0 = 90 + (n - 1) * 150
    k = 100 * n                                  # ref offset: bay 1 -> 1xx, bay 2 -> 2xx
    V, SW, BST, FB, EN = f"VOUT_{n}", f"SW_{n}", f"BST_{n}", f"FB_{n}", f"EN_{n}"
    VSH, GATE, GN = f"VSH_{n}", f"PG_{n}", f"PGN_{n}"
    VF, VS, GS = f"VCHG_F_{n}", f"VCHG_S_{n}", f"GND_S_{n}"
    # --- buck (CV)
    part(f"U{k+1}", "Regulator_Switching:TPS54202DDC", "TPS54202DDC", SOT6, 40, y0, 0,
         _1="GND", _2=SW, _3="VIN", _4=FB, _5=EN, _6=BST)
    cap(f"C{k+1}", "10uF 25V", 20, y0, "VIN", "GND", C12)
    cap(f"C{k+2}", "100nF", 25, y0 + 15, "VIN", "GND")
    cap(f"C{k+3}", "100nF", 60, y0 - 15, BST, SW)
    part(f"L{k+1}", "Device:L", "22uH 2.5A", "Inductor_SMD:L_Bourns_SRR1260", 70, y0, 0, _1=SW, _2=V)
    cap(f"C{k+4}", "22uF 25V", 85, y0, V, "GND", C12)
    cap(f"C{k+5}", "22uF 25V", 92, y0, V, "GND", C12)
    res(f"R{k+1}", "130k 0.1%", 100, y0, V, FB)
    res(f"R{k+2}", "10.0k 0.1%", 100, y0 + 15, FB, "GND")
    res(f"R{k+3}", "100k", 30, y0 + 30, EN, "GND")
    # --- shunt + current sense
    part(f"R{k+4}", "Device:R", "20m 1%", R12, 115, y0, 0, _1=V, _2=VSH)
    part(f"U{k+2}", "Amplifier_Current:INA180A2", "INA180A2", SOT5, 125, y0 + 25, 0,
         _1=f"ISNS_{n}", _2="GND", _3=V, _4=VSH, _5="VCC_3V3")
    cap(f"C{k+6}", "100nF", 140, y0 + 25, "VCC_3V3", "GND")
    # --- CC integrator (LMV358 unit n) + FB injection
    ua = n                                       # unit 1 for bay 1, unit 2 for bay 2
    pins = {1: dict(_1=f"CCOUT_1", _2="CCINV_1", _3="ISNS_1"), 2: dict(_7="CCOUT_2", _6="CCINV_2", _5="ISNS_2")}[n]
    if n == 1:
        part("U3", "Amplifier_Operational:LMV358", "LMV358", "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm", 160, 60 + 150, 0,
             _4="GND", _8="VCC_3V3", _1="CCOUT_1", _2="CCINV_1", _3="ISNS_1",
             _7="CCOUT_2", _6="CCINV_2", _5="ISNS_2")
        cap("C3", "100nF", 175, 210, "VCC_3V3", "GND")
    res(f"R{k+5}", "10k", 150, y0 + 45, f"ISET_{n}", f"CCINV_{n}")
    cap(f"C{k+7}", "100nF", 165, y0 + 45, f"CCINV_{n}", f"CCOUT_{n}")
    part(f"D{k+1}", "Device:D_Schottky", "BAT54J", SOD323, 180, y0 + 45, 0, _1=f"CCINJ_{n}", _2=f"CCOUT_{n}")
    res(f"R{k+6}", "10k", 190, y0 + 45, f"CCINJ_{n}", FB)
    # --- ISET DAC (MCP4725, A0 strap = bay) with 2:1 divider (hard cap 1.65 A)
    part(f"U{k+3}", "Analog_DAC:MCP4725xxx-xCH", "MCP4725A0T-E/CH", SOT6, 210, y0 + 25, 0,
         _1=f"DAC_{n}", _2="GND", _3="VCC_3V3", _4="I2C_SDA", _5="I2C_SCL", _6=("GND" if n == 1 else "VCC_3V3"))
    cap(f"C{k+8}", "100nF", 225, y0 + 25, "VCC_3V3", "GND")
    res(f"R{k+7}", "10k", 210, y0 + 45, f"DAC_{n}", f"ISET_{n}")
    res(f"R{k+8}", "10k", 220, y0 + 45, f"ISET_{n}", "GND")
    cap(f"C{k+9}", "1uF", 230, y0 + 45, f"ISET_{n}", "GND")
    # --- pad switch: AO3401A high side, 2N7002 level shift, EN = BAY_EN AND KALERT_N
    part(f"Q{k+1}", "Transistor_FET:AO3401A", "AO3401A", SOT23, 135, y0, 0, _1=GATE, _2=VSH, _3=VF)
    res(f"R{k+9}", "100k", 135, y0 - 15, VSH, GATE)
    res(f"R{k+10}", "10k", 145, y0 - 15, GATE, GN)
    part(f"Q{k+2}", "Device:Q_NMOS_GSD", "2N7002", SOT23, 150, y0, 0, _1=EN, _2="GND", _3=GN)
    part(f"U{k+4}", "74xGxx:74LVC1G08", "74LVC1G08", SOT5, 60, y0 + 30, 0,
         _1=f"BAY_EN_{n}", _2=f"KALERT_N_{n}", _3="GND", _4=EN, _5="VCC_3V3")
    cap(f"C{k+10}", "100nF", 75, y0 + 30, "VCC_3V3", "GND")
    res(f"R{k+11}", "100k", 45, y0 + 45, f"BAY_EN_{n}", "GND")
    res(f"R{k+12}", "10k", 55, y0 + 45, "VCC_3V3", f"KALERT_N_{n}")
    part(f"D{k+2}", "Device:LED", "AMBER charging", "LED_SMD:LED_0603_1608Metric", 85, y0 + 45, 0, _1=f"LEDK_{n}", _2=EN)
    res(f"R{k+13}", "1k", 95, y0 + 45, f"LEDK_{n}", "GND")
    # --- telemetry + Kelvin contact measurement
    a1, a2, a3 = ADDR[n]
    # INA226 inputs through 10 ohm (TI input filter); also keeps the 0.5 mm power tracks off the 0.5 mm-pitch pins
    part(f"U{k+5}", "Sensor_Energy:INA226", "INA226 (I, V)", "Package_SO:MSOP-10_3x3mm_P0.5mm", 250, y0, 0,
         _1=a1[0], _2=a1[1], _3=f"KALERT_N_{n}", _4="I2C_SDA", _5="I2C_SCL", _6="VCC_3V3", _7="GND",
         _8=f"VBS_{n}", _9=f"SHN_{n}", _10=f"SHP_{n}")
    res(f"R{k+20}", "10R", 240, y0 - 10, V, f"SHP_{n}")
    res(f"R{k+21}", "10R", 245, y0 - 10, VSH, f"SHN_{n}")
    res(f"R{k+22}", "10R", 250, y0 - 10, VF, f"VBS_{n}")
    cap(f"C{k+15}", "100nF", 255, y0 - 10, f"SHP_{n}", f"SHN_{n}")
    part(f"U{k+6}", "Sensor_Energy:INA226", "INA226 (VCHG contact)", "Package_SO:MSOP-10_3x3mm_P0.5mm", 250, y0 + 40, 0,
         _1=a2[0], _2=a2[1], _3=f"KALERT_N_{n}", _4="I2C_SDA", _5="I2C_SCL", _6="VCC_3V3", _7="GND",
         _8=VS, _9=VS, _10=f"VFK_{n}")
    res(f"R{k+23}", "10R", 240, y0 + 30, VF, f"VFK_{n}")      # Kelvin force tap (input bias x 10R = 0.1 mOhm error)
    part(f"U{k+7}", "Sensor_Energy:INA226", "INA226 (GND contact)", "Package_SO:MSOP-10_3x3mm_P0.5mm", 250, y0 + 80, 0,
         _1=a3[0], _2=a3[1], _3=f"KALERT_N_{n}", _4="I2C_SDA", _5="I2C_SCL", _6="VCC_3V3", _7="GND",
         _8=GS, _9="GND", _10=GS)
    for i in range(3):
        cap(f"C{k+11+i}", "100nF", 270, y0 + 40 * i, "VCC_3V3", "GND")
    res(f"R{k+14}", "100k", 280, y0 + 40, VF, VS)         # sense pin open (no agent) -> reads 0 V drop
    res(f"R{k+15}", "100k", 280, y0 + 80, GS, "GND")
    part(f"D{k+3}", "Device:D_TVS", "SMAJ12CA", "Diode_SMD:D_SMA", 160, y0 + 10, 0, _1=VF, _2="GND")
    # --- dock connector: force + sense per power pad, DATA, ID
    part(f"J{n+1}", "Connector_Generic:Conn_01x06", f"BAY {n} DOCK (VF,VS,GF,GS,DATA,ID)",
         "Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical", 300, y0 + 40, 0,
         _1=VF, _2=VS, _3="GND", _4=GS, _5=f"DOCK_DATA_{n}", _6=f"ID_{n}")
    res(f"R{k+16}", "1k", 320, y0 + 30, f"DATA_TX_{n}", f"DOCK_DATA_{n}")
    res(f"R{k+17}", "1k", 330, y0 + 30, f"DOCK_DATA_{n}", f"DATA_RX_{n}")
    res(f"R{k+18}", "10k", 340, y0 + 30, "VCC_3V3", f"DOCK_DATA_{n}")
    res(f"R{k+19}", "10k", 320, y0 + 55, "VCC_3V3", f"ID_{n}")
    cap(f"C{k+14}", "100nF", 330, y0 + 55, f"ID_{n}", "GND")
    part(f"D{k+4}", "Device:D_TVS", "PESD5V0S1BL", SOD323, 350, y0 + 30, 0, _1=f"DOCK_DATA_{n}", _2="GND")
    part(f"D{k+5}", "Device:D_TVS", "PESD5V0S1BL", SOD323, 350, y0 + 55, 0, _1=f"ID_{n}", _2="GND")

# ============================== 3. test points, mounting
for i, net in enumerate(("VIN", "VOUT_1", "VOUT_2", "VCHG_F_1", "VCHG_F_2", "ISNS_1", "ISNS_2", "GND")):
    part(f"TP{i+1}", "Connector:TestPoint", net, TP, 30 + 12.7 * i, 400, 0, _1=net)
for i in range(4):
    part(f"H{i+1}", "Mechanical:MountingHole_Pad", "M3", "MountingHole:MountingHole_3.2mm_M3_Pad_Via",
         150 + 12.7 * i, 400, 0, _1="GND")

# ---------------------------------------------------------------- derived
NETS = defaultdict(list)
for ref, lib, val, fp, x, y, rot, pins in P:
    for pin, net in pins.items():
        NETS[net].append((ref, pin))

if __name__ == "__main__":
    print(f"{len(P)} parts, {len(NETS)} nets")
    single = [n for n, v in NETS.items() if len(v) == 1 and not n.startswith("NC_")]
    print("single-pin nets (check!):", single)
