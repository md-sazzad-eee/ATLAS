"""ATLAS Board B (agent controller) — design source of truth for the KiCad schematic.

Every part, value, footprint and pin->net assignment lives here. The KiCad schematic
was generated from this file (label-driven: each pin carries a net label), so a design
review can diff this file instead of the drawing. Run it to print a BOM + net summary:
    python3 board_b_design.py

Requirements traced to: 05_simulation/matlab/results/sizing_report.txt,
speed_loop_results.json, contact_heating.json and 02_electronics/README.md.
"""
from collections import defaultdict

R = "Resistor_SMD:R_0603_1608Metric"
C = "Capacitor_SMD:C_0603_1608Metric"
C8 = "Capacitor_SMD:C_0805_2012Metric"
SOT23 = "Package_TO_SOT_SMD:SOT-23"
DPAK = "Package_TO_SOT_SMD:TO-252-2"
SMA = "Diode_SMD:D_SMA"
LED = "LED_SMD:LED_0603_1608Metric"
TP = "TestPoint:TestPoint_Pad_D1.5mm"

# (ref, lib_id, value, footprint, x, y, rotation, {pin: net})
P = []
def part(ref, lib, val, fp, x, y, rot=0, **pins):
    P.append((ref, lib, val, fp, x, y, rot, {k.lstrip("_"): v for k, v in pins.items()}))

def res(ref, val, x, y, a, b, fp=R):   # vertical resistor, pin1 top
    part(ref, "Device:R", val, fp, x, y, 0, _1=a, _2=b)

def cap(ref, val, x, y, a, b, fp=C):
    part(ref, "Device:C", val, fp, x, y, 0, _1=a, _2=b)

# ============================== 1. battery input + protection (x 25-140, y 30-75)
part("J1", "Connector_Generic:Conn_01x03", "BATT 2S (+,-,NTC)", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical",
     30, 50, 0, _1="VBAT_RAW", _2="GND", _3="BATT_NTC")
part("F1", "Device:Polyfuse", "3A hold PTC", "Fuse:Fuse_1812_4532Metric", 55, 50, 0, _1="VBAT_RAW", _2="VBAT_F")
part("Q1", "Transistor_FET:Q_PMOS_GDS", "AOD409 (rev-pol)", DPAK, 80, 50, 0, _1="GND", _2="VBAT_F", _3="VBAT")
part("D1", "Device:D_TVS", "SMBJ10CA", "Diode_SMD:D_SMB", 100, 50, 0, _1="VBAT", _2="GND")
part("C1", "Device:C_Polarized", "100uF 16V", "Capacitor_SMD:CP_Elec_6.3x7.7", 115, 50, 0, _1="VBAT", _2="GND")
res("R1", "100k", 130, 50, "VBAT", "VBAT_SENSE")
res("R2", "22k", 142.7, 50, "VBAT_SENSE", "GND")
cap("C2", "100nF", 155.4, 50, "VBAT_SENSE", "GND")
res("R3", "10k", 168.1, 50, "VCC_3V3", "BATT_NTC")
cap("C3", "100nF", 180.8, 50, "BATT_NTC", "GND")

# ============================== 2. dock charge path (x 25-180, y 95-140)
part("J2", "Connector_Generic:Conn_01x04", "DOCK PADS (VCHG,GND,DATA,ID)", "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical",
     30, 110, 0, _1="VCHG", _2="GND", _3="DOCK_DATA", _4="DOCK_ID")
part("D2", "Device:D_Schottky", "SS34 (no backfeed)", SMA, 55, 110, 0, _1="VCHG_D", _2="VCHG")   # 1=K 2=A
part("Q2", "Transistor_FET:Q_PMOS_GSD", "AO3401A (charge sw)", SOT23, 80, 110, 0, _1="CHG_G", _2="VCHG_D", _3="VBAT")
res("R4", "100k", 95, 110, "VCHG_D", "CHG_G")
part("Q3", "Transistor_FET:Q_NMOS_GSD", "2N7002", SOT23, 110, 110, 0, _1="CHG_EN", _2="GND", _3="CHG_G")
res("R5", "100k", 125, 110, "CHG_EN", "GND")
res("R6", "100k", 140, 110, "VCHG", "DOCK_VSENSE")
res("R7", "22k", 152.7, 110, "DOCK_VSENSE", "GND")
cap("C4", "100nF", 165.4, 110, "DOCK_VSENSE", "GND")
res("R8", "10k", 55, 135, "DOCK_ID", "GND")                       # S1 = 10k, S2 = 22k (agent identity)
res("R9", "1k", 70, 135, "DOCK_TX", "DOCK_DATA")
res("R10", "1k", 85, 135, "DOCK_DATA", "DOCK_RX")
res("R11", "10k", 100, 135, "VCC_3V3", "DOCK_DATA")
part("D3", "Device:D_TVS", "PESD3V3S1BL", "Diode_SMD:D_SOD-323", 115, 135, 0, _1="DOCK_DATA", _2="GND")

# ============================== 3. 5 V buck (x 210-330, y 30-80)
part("U1", "Regulator_Switching:TPS563201", "TPS563201", "Package_TO_SOT_SMD:SOT-23-6",
     230, 50, 0, _1="GND", _2="SW_5V", _3="VBAT", _4="FB_5V", _5="BUCK_EN", _6="BST_5V")
cap("C5", "10uF 25V", 205, 50, "VBAT", "GND", C8)
cap("C6", "10uF 25V", 205, 70, "VBAT", "GND", C8)
res("R12", "100k", 215, 70, "VBAT", "BUCK_EN")
cap("C7", "100nF", 255, 35, "BST_5V", "SW_5V")
part("L1", "Device:L", "2.2uH 4A (SRN6045TA-2R2)", "Inductor_SMD:L_Bourns_SRN6045TA", 270, 50, 0, _1="SW_5V", _2="VCC_5V")
res("R13", "56k", 285, 50, "VCC_5V", "FB_5V")
res("R14", "10k", 285, 70, "FB_5V", "GND")
cap("C8", "22uF 10V", 300, 50, "VCC_5V", "GND", C8)
cap("C9", "22uF 10V", 312.7, 50, "VCC_5V", "GND", C8)

# ============================== 4. 3.3 V LDO + USB (x 210-330, y 95-175)
part("D4", "Device:D_Schottky", "SS14", SMA, 225, 100, 0, _1="V_LDO_IN", _2="VCC_5V")
part("D5", "Device:D_Schottky", "SS14", SMA, 225, 115, 0, _1="V_LDO_IN", _2="VBUS")
part("U2", "Regulator_Linear:AP2112K-3.3", "AP2112K-3.3", "Package_TO_SOT_SMD:SOT-23-5",
     260, 105, 0, _1="V_LDO_IN", _2="GND", _3="V_LDO_IN", _4="NC_LDO", _5="VCC_3V3")
cap("C10", "1uF", 245, 125, "V_LDO_IN", "GND")
cap("C11", "10uF", 280, 125, "VCC_3V3", "GND", C8)
cap("C12", "1uF", 292.7, 125, "VCC_3V3", "GND")
part("J3", "Connector:USB_C_Receptacle_USB2.0_16P", "USB-C (program/debug)",
     "Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal", 225, 160, 0,
     A1="GND", A12="GND", B1="GND", B12="GND", SH="GND", A4="VBUS", A9="VBUS", B4="VBUS", B9="VBUS",
     A5="CC1", B5="CC2", A6="USB_DP", B6="USB_DP", A7="USB_DN", B7="USB_DN", A8="NC_SBU1", B8="NC_SBU2")
res("R15", "5.1k", 255, 150, "CC1", "GND")
res("R16", "5.1k", 267.7, 150, "CC2", "GND")
part("U3", "Power_Protection:USBLC6-2SC6", "USBLC6-2SC6", "Package_TO_SOT_SMD:SOT-23-6",
     290, 160, 0, _1="USB_DP", _2="GND", _3="USB_DN", _4="USB_DN", _5="VBUS", _6="USB_DP")

# ============================== 5. MCU (x 370-470, y 60-170)
esp = dict(_1="GND", _40="GND", _41="GND", _2="VCC_3V3", _3="ESP_EN",
           _4="VBAT_SENSE", _5="IMOT_L", _6="IMOT_R", _7="BATT_NTC", _12="DOCK_VSENSE",
           _39="I2C_SDA", _38="I2C_SCL", _17="SENS_ADC", _18="SENS_GPIO",
           _19="ENC_L_A", _20="ENC_L_B", _21="ENC_R_A", _22="ENC_R_B",
           _8="MOT_L_IN1", _9="MOT_L_IN2", _10="LED_STAT", _11="PI_PWR_EN",   # rev A1: right-motor inputs moved to right-side pins
           _23="MOT_EN", _31="WDI", _32="DOCK_TX", _33="DOCK_RX", _34="CHG_EN", _35="MOT_R_IN2",
           _24="MOT_R_IN1", _25="PI_TX", _15="PI_RX", _27="ESP_IO0", _13="USB_DN", _14="USB_DP",
           _37="DBG_TX", _36="DBG_RX",
           _16="NC_IO46", _26="NC_IO45", _28="NC_IO35", _29="NC_IO36", _30="NC_IO37")
part("U4", "RF_Module:ESP32-S3-WROOM-1", "ESP32-S3-WROOM-1-N8R8", "RF_Module:ESP32-S3-WROOM-1", 420, 110, 0, **esp)
cap("C13", "10uF", 380, 55, "VCC_3V3", "GND", C8)
cap("C14", "100nF", 392.7, 55, "VCC_3V3", "GND")
res("R17", "10k", 360, 75, "VCC_3V3", "ESP_EN")
cap("C15", "1uF", 360, 95, "ESP_EN", "GND")
part("SW1", "Switch:SW_Push", "RESET", "Button_Switch_SMD:SW_SPST_TL3342", 340, 95, 90, _1="ESP_EN", _2="GND")
res("R18", "10k", 360, 120, "VCC_3V3", "ESP_IO0")
part("SW2", "Switch:SW_Push", "BOOT", "Button_Switch_SMD:SW_SPST_TL3342", 340, 130, 90, _1="ESP_IO0", _2="GND")

# ============================== 6. motor power cut + watchdog (x 25-200, y 200-260)
part("U5", "Power_Supervisor:TPS3823-xxDBV", "TPS3823-33DBV", "Package_TO_SOT_SMD:SOT-23-5",
     40, 220, 0, _1="WDT_OK", _2="GND", _3="VCC_3V3", _4="WDI", _5="VCC_3V3")
cap("C16", "100nF", 25, 240, "VCC_3V3", "GND")
part("U6", "74xGxx:74LVC1G08", "74LVC1G08", "Package_TO_SOT_SMD:SOT-23-5",
     85, 220, 0, _1="MOT_EN", _2="WDT_OK", _3="GND", _4="MOT_ENABLE", _5="VCC_3V3")
cap("C17", "100nF", 85, 245, "VCC_3V3", "GND")
res("R19", "100k", 105, 240, "MOT_EN", "GND")
part("Q4", "Transistor_FET:Q_NMOS_GSD", "2N7002", SOT23, 120, 220, 0, _1="MOT_ENABLE", _2="GND", _3="VM_G")
res("R20", "100k", 135, 240, "MOT_ENABLE", "GND")
res("R21", "100k", 150, 205, "VBAT", "VM_G")
part("Q5", "Transistor_FET:Q_PMOS_GDS", "AOD409 (motor cut)", DPAK, 170, 220, 0, _1="VM_G", _2="VM_SW", _3="VBAT")
part("C18", "Device:C_Polarized", "100uF 16V", "Capacitor_SMD:CP_Elec_6.3x7.7", 190, 230, 0, _1="VM_SW", _2="GND")

# ============================== 7. motor drivers + current sense (x 25-330, y 280-370)
for side, x0, a1, a2, enc_a, enc_b, jref, base in (
        ("L", 25, "MOT_L_IN1", "MOT_L_IN2", "ENC_L_A", "ENC_L_B", "J4", 0),
        ("R", 185, "MOT_R_IN1", "MOT_R_IN2", "ENC_R_A", "ENC_R_B", "J5", 1)):
    n = lambda k: f"{k}{side}"
    rs, ui, ud, rlim, cvm, cvm2, rf, cf = (
        [("R22", "U7", "U9", "R24", "C19", "C20", "R26", "C23"), ("R23", "U8", "U10", "R25", "C21", "C22", "R27", "C24")][base])
    res(rs, "20m 1206", x0, 290, "VM_SW", f"VM_{side}", "Resistor_SMD:R_1206_3216Metric")
    part(ui, "Amplifier_Current:INA180A2", "INA180A2", "Package_TO_SOT_SMD:SOT-23-5", x0 + 25, 290, 0,
         _1=f"ISNS_{side}_RAW", _2="GND", _3="VM_SW", _4=f"VM_{side}", _5="VCC_3V3")
    res(rf, "1k", x0 + 45, 290, f"ISNS_{side}_RAW", f"IMOT_{side}")
    cap(cf, "100nF", x0 + 57.7, 290, f"IMOT_{side}", "GND")
    part(ud, "Driver_Motor:DRV8871DDA", "DRV8871DDA", None, x0 + 25, 335, 0,
         _1="GND", _7="GND", _9="GND", _2=a2, _3=a1, _4=f"ILIM_{side}", _5=f"VM_{side}", _6=f"MOT_{side}_P", _8=f"MOT_{side}_N")
    res(rlim, "33k (2.0A trip)", x0 + 50, 340, f"ILIM_{side}", "GND")
    cap(cvm, "10uF 25V", x0, 335, f"VM_{side}", "GND", C8)
    cap(cvm2, "100nF", x0 + 10, 350, f"VM_{side}", "GND")
    part(jref, "Connector_Generic:Conn_01x06", f"MOTOR+ENC {side}", "Connector_JST:JST_PH_B6B-PH-K_1x06_P2.00mm_Vertical",
         x0 + 80, 335, 0, _1=f"MOT_{side}_P", _2=f"MOT_{side}_N", _3="VCC_3V3", _4="GND", _5=enc_a, _6=enc_b)
# encoder pull-ups
for i, net in enumerate(("ENC_L_A", "ENC_L_B", "ENC_R_A", "ENC_R_B")):
    res(f"R{28+i}", "10k", 30 + 12.7 * i, 390, "VCC_3V3", net)
cap("C25", "100nF", 85, 390, "VCC_3V3", "GND")

# ============================== 8. Pi Zero 2 W camera module power + link (x 370-480, y 200-260)
part("Q6", "Transistor_FET:Q_PMOS_GSD", "AO3401A (Pi power)", SOT23, 390, 215, 0, _1="PI_G", _2="VCC_5V", _3="PI_5V")
res("R32", "100k", 375, 205, "VCC_5V", "PI_G")
part("Q7", "Transistor_FET:Q_NMOS_GSD", "2N7002", SOT23, 410, 235, 0, _1="PI_PWR_EN", _2="GND", _3="PI_G")
res("R33", "100k", 425, 245, "PI_PWR_EN", "GND")
part("C26", "Device:C_Polarized", "220uF 10V", "Capacitor_SMD:CP_Elec_6.3x7.7", 440, 220, 0, _1="PI_5V", _2="GND")
part("J6", "Connector_Generic:Conn_01x06", "PI ZERO 2W (5V,5V,GND,GND,TX,RX)",
     "Connector_JST:JST_GH_SM06B-GHS-TB_1x06-1MP_P1.25mm_Horizontal", 465, 225, 0,
     _1="PI_5V", _2="PI_5V", _3="GND", _4="GND", _5="PI_TX", _6="PI_RX")

# ============================== 9. expansion, debug, status (x 370-560, y 280-400)
part("J7", "Connector_Generic:Conn_01x04", "QWIIC I2C (GND,3V3,SDA,SCL)",
     "Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal", 380, 300, 0,
     _1="GND", _2="VCC_3V3", _3="I2C_SDA", _4="I2C_SCL")
res("R34", "4.7k", 400, 290, "VCC_3V3", "I2C_SDA")
res("R35", "4.7k", 412.7, 290, "VCC_3V3", "I2C_SCL")
part("J8", "Connector_Generic:Conn_01x08", "SENSOR MODULE",
     "Connector_JST:JST_GH_SM08B-GHS-TB_1x08-1MP_P1.25mm_Horizontal", 450, 310, 0,
     _1="VCC_5V", _2="VCC_3V3", _3="GND", _4="I2C_SDA", _5="I2C_SCL", _6="SENS_GPIO", _7="SENS_ADC", _8="GND")
part("J9", "Connector_Generic:Conn_01x04", "DEBUG UART0", "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
     500, 300, 0, _1="VCC_3V3", _2="DBG_TX", _3="DBG_RX", _4="GND")
part("D6", "Device:LED", "GREEN status", LED, 380, 350, 0, _1="LED_K", _2="LED_STAT")       # 1=K 2=A
res("R36", "1k", 395, 350, "LED_K", "GND")
part("D7", "Device:LED", "RED 3V3", LED, 420, 350, 0, _1="LED_PWR_K", _2="VCC_3V3")
res("R37", "1k", 435, 350, "LED_PWR_K", "GND")
for i, net in enumerate(("VBAT", "VCC_5V", "VCC_3V3", "VM_SW", "IMOT_L", "IMOT_R", "WDT_OK", "GND")):
    part(f"TP{i+1}", "Connector:TestPoint", net, TP, 380 + 12.7 * i, 385, 0, _1=net)
for i in range(4):
    part(f"H{i+1}", "Mechanical:MountingHole_Pad", "M3", "MountingHole:MountingHole_3.2mm_M3_Pad_Via",
         500 + 12.7 * i, 350, 0, _1="GND")

# ---------------------------------------------------------------- derived
NETS = defaultdict(list)
for ref, lib, val, fp, x, y, rot, pins in P:
    for pin, net in pins.items():
        NETS[net].append((ref, pin))

if __name__ == "__main__":
    print(f"{len(P)} parts, {len(NETS)} nets")
    single = [n for n, v in NETS.items() if len(v) == 1 and not n.startswith("NC_")]
    print("single-pin nets (check!):", single)
    for n in sorted(NETS):
        print(f"{n:16s} {len(NETS[n]):3d}  " + " ".join(f"{r}.{p}" for r, p in NETS[n]))
