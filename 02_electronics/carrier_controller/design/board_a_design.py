"""ATLAS Board A (carrier main controller) — design source of truth.

ESP32-S3-WROOM-1-N8 (no PSRAM, so IO35-37 are free) running the carrier firmware
(03_firmware/carrier_esp32s3): both docking FSMs, wheel control, latches, Board C, link to the
ROS 2 computer over native USB.

  Power      4S LiFePO4 (10-14.6 V) XT60 -> 15 A blade fuse -> AOD409 reverse-polarity -> TVS -> bulk
             5 V / 3 A TPS54302 (logic, servos, Board C 3V3 source) -> AP2112K 3.3 V (OR-ed with USB VBUS)
  Wheels     4 x DRV8871 (one per wheel, ILIM 22k = 2.9 A trip), left pair / right pair share IN1/IN2
             (skid steer: same command per side, GPIO budget); 20 mOhm + INA180A1 (0.4 V/A) per wheel;
             4 quadrature encoders on the 4 PCNT units.  Sizing: >= 0.64 N.m @ 95 rpm per wheel.
  Latches    2 x 12 V pull solenoid (spring fail-locked pin), AO3400A low side + SS34 flyback,
             driven from IO45/IO46 (strap pins that must be LOW at reset == latch stays locked).
  INTERLOCK  motor rail VM_SW (AOD409) is ON only if
                 MOT_EN (firmware) AND WDT_OK (TPS3823, 1.6 s) AND ESTOP_OK (NC mushroom)
                 AND BAY_SAFE_1 AND BAY_SAFE_2,      BAY_SAFE_n = NOT(SEATED_n AND LATCHED_N_n)
             i.e. the carrier physically cannot drive while an agent is seated but not latched —
             the Stateflow/Python invariant, now in gates. Sensor polarities are chosen so a broken
             wire reads "seated" / "not latched" (fail-safe).
  Board C    16-pin link: I2C, BAY_EN_1/2 (only asserted in DOCKED), dock UARTs, ID ADC, KALERT_N.
  Expander   TCA9534 (0x20): SEATED_1/2, LATCHED_N_1/2, KALERT_N_1/2 (inputs), LED, buzzer.
"""
from collections import defaultdict

R = "Resistor_SMD:R_0603_1608Metric"
R12 = "Resistor_SMD:R_1206_3216Metric"
C = "Capacitor_SMD:C_0603_1608Metric"
C8 = "Capacitor_SMD:C_0805_2012Metric"
C12 = "Capacitor_SMD:C_1206_3216Metric"
SOT23 = "Package_TO_SOT_SMD:SOT-23"
SOT5 = "Package_TO_SOT_SMD:SOT-23-5"
SOT6 = "Package_TO_SOT_SMD:SOT-23-6"
DPAK = "Package_TO_SOT_SMD:TO-252-2"
SMA = "Diode_SMD:D_SMA"
LED = "LED_SMD:LED_0603_1608Metric"
TP = "TestPoint:TestPoint_Pad_D1.5mm"
DRV_FP = "Package_SO:Texas_HTSOP-8-1EP_3.9x4.9mm_P1.27mm_EP2.95x4.9mm_Mask2.4x3.1mm_ThermalVias"

P = []
def part(ref, lib, val, fp, x, y, rot=0, **pins):
    P.append((ref, lib, val, fp, x, y, rot, {k.lstrip("_"): v for k, v in pins.items()}))
def res(ref, val, x, y, a, b, fp=R):
    part(ref, "Device:R", val, fp, x, y, 0, _1=a, _2=b)
def cap(ref, val, x, y, a, b, fp=C):
    part(ref, "Device:C", val, fp, x, y, 0, _1=a, _2=b)

# ============================== 1. power entry
part("J1", "Connector_Generic:Conn_01x02", "BATT 4S (+,-)", "Connector_AMASS:AMASS_XT60PW-M_1x02_P7.20mm_Horizontal",
     30, 40, 0, _1="VBAT_RAW", _2="GND")
part("F1", "Device:Fuse", "15A mini blade", "Fuse:Fuseholder_Blade_Mini_Keystone_3568", 55, 40, 0, _1="VBAT_RAW", _2="VBAT_F")
part("Q1", "Device:Q_PMOS_GDS", "AOD409 (rev-pol)", DPAK, 80, 40, 0, _1="GND", _2="VBAT_F", _3="VBAT")
part("D1", "Device:D_TVS", "SMBJ16CA", "Diode_SMD:D_SMB", 100, 40, 0, _1="VBAT", _2="GND")
part("C1", "Device:C_Polarized", "470uF 25V", "Capacitor_SMD:CP_Elec_10x10.5", 115, 40, 0, _1="VBAT", _2="GND")
res("R1", "100k", 130, 40, "VBAT", "VBAT_SENSE")
res("R2", "18k", 140, 40, "VBAT_SENSE", "GND")              # 14.6 V -> 2.23 V
cap("C2", "100nF", 150, 40, "VBAT_SENSE", "GND")
part("J2", "Connector_Generic:Conn_01x02", "SBC POWER OUT (VBAT,GND)", "Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal",
     170, 40, 0, _1="VBAT", _2="GND")
part("J3", "Connector_Generic:Conn_01x02", "BOARD C POWER (VBAT,GND)", "Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal",
     190, 40, 0, _1="VBAT", _2="GND")

# ============================== 2. 5 V / 3 A buck + 3.3 V LDO (USB OR-ing as Board B)
part("U1", "Regulator_Switching:TPS54302", "TPS54302", SOT6, 40, 90, 0,
     _1="GND", _2="SW_5V", _3="VBAT", _4="FB_5V", _5="EN_5V", _6="BST_5V")
cap("C3", "10uF 25V", 20, 90, "VBAT", "GND", C12)
cap("C4", "10uF 25V", 20, 105, "VBAT", "GND", C12)
cap("C5", "100nF", 30, 105, "VBAT", "GND")
res("R3", "100k", 50, 110, "VBAT", "EN_5V")
res("R4", "20k", 60, 110, "EN_5V", "GND")                   # UVLO ~ 8 V (bq EN 1.2 V threshold)
cap("C6", "100nF", 60, 75, "BST_5V", "SW_5V")
part("L1", "Device:L", "10uH 4A", "Inductor_SMD:L_Bourns_SRR1260", 75, 90, 0, _1="SW_5V", _2="VCC_5V")
res("R5", "75k 1%", 90, 90, "VCC_5V", "FB_5V")               # 0.596 * (1 + 75/10) = 5.07 V
res("R6", "10k 1%", 90, 105, "FB_5V", "GND")
cap("C7", "22uF 10V", 100, 90, "VCC_5V", "GND", C12)
cap("C8", "22uF 10V", 107, 90, "VCC_5V", "GND", C12)
part("D2", "Device:D_Schottky", "SS14", SMA, 130, 85, 0, _1="V_LDO_IN", _2="VCC_5V")
part("D3", "Device:D_Schottky", "SS14", SMA, 130, 100, 0, _1="V_LDO_IN", _2="VBUS")
part("U2", "Regulator_Linear:AP2112K-3.3", "AP2112K-3.3", SOT5, 150, 90, 0,
     _1="V_LDO_IN", _2="GND", _3="V_LDO_IN", _4="NC_LDO", _5="VCC_3V3")
cap("C9", "1uF", 140, 105, "V_LDO_IN", "GND")
cap("C10", "10uF", 165, 105, "VCC_3V3", "GND", C8)

# ============================== 3. USB-C to the ROS 2 computer (CDC) — same as Board B
part("J4", "Connector:USB_C_Receptacle_USB2.0_16P", "USB-C (ROS 2 link)",
     "Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal", 200, 90, 0,
     A1="GND", A12="GND", B1="GND", B12="GND", SH="GND", A4="VBUS", A9="VBUS", B4="VBUS", B9="VBUS",
     A5="CC1", B5="CC2", A6="USB_DP", B6="USB_DP", A7="USB_DN", B7="USB_DN", A8="NC_SBU1", B8="NC_SBU2")
res("R7", "5.1k", 225, 80, "CC1", "GND")
res("R8", "5.1k", 235, 80, "CC2", "GND")
part("U3", "Power_Protection:USBLC6-2SC6", "USBLC6-2SC6", SOT6, 250, 90, 0,
     _1="USB_DP", _2="GND", _3="USB_DN", _4="USB_DN", _5="VBUS", _6="USB_DP")

# ============================== 4. MCU
esp = dict(_1="GND", _40="GND", _41="GND", _2="VCC_3V3", _3="ESP_EN",
           _39="VBAT_SENSE",  # IO1  ADC1_CH0
           _38="IMOT_LF",     # IO2  ADC1_CH1
           _15="ID_1",        # IO3  ADC1_CH2
           _4="IMOT_LR", _5="IMOT_RF", _6="IMOT_RR",   # IO4-6
           _7="ID_2",         # IO7
           _12="I2C_SDA", _17="I2C_SCL",               # IO8, IO9
           _18="MOT_L_IN1", _19="MOT_L_IN2", _20="MOT_R_IN1", _21="MOT_R_IN2",   # IO10-13
           _22="ENC_LF_A", _8="ENC_LF_B", _9="ENC_LR_A", _10="ENC_LR_B",          # IO14-17
           _11="ENC_RF_A", _23="ENC_RF_B", _28="ENC_RR_A", _29="ENC_RR_B",       # IO18, IO21, IO35, IO36
           _30="MOT_EN", _31="WDI",                                              # IO37, IO38
           _32="DATA_TX_1", _33="DATA_RX_1", _34="DATA_TX_2", _35="DATA_RX_2",   # IO39-42
           _24="BAY_EN_1", _25="BAY_EN_2",                                       # IO47, IO48
           _26="LATCH_OPEN_1", _16="LATCH_OPEN_2",                               # IO45, IO46 (straps: low at reset)
           _37="ESTOP_OK", _36="EXP_INT",                                        # IO43, IO44
           _27="ESP_IO0", _13="USB_DN", _14="USB_DP")
part("U4", "RF_Module:ESP32-S3-WROOM-1", "ESP32-S3-WROOM-1-N8", "RF_Module:ESP32-S3-WROOM-1", 320, 110, 0, **esp)
cap("C11", "10uF", 280, 60, "VCC_3V3", "GND", C8)
cap("C12", "100nF", 290, 60, "VCC_3V3", "GND")
res("R9", "10k", 270, 80, "VCC_3V3", "ESP_EN")
cap("C13", "1uF", 270, 95, "ESP_EN", "GND")
part("SW1", "Switch:SW_Push", "RESET", "Button_Switch_SMD:SW_SPST_TL3342", 255, 95, 90, _1="ESP_EN", _2="GND")
res("R10", "10k", 270, 120, "VCC_3V3", "ESP_IO0")
part("SW2", "Switch:SW_Push", "BOOT", "Button_Switch_SMD:SW_SPST_TL3342", 255, 125, 90, _1="ESP_IO0", _2="GND")

# ============================== 5. safety interlock: watchdog, E-stop, bay gates -> motor rail
part("U5", "atlas_extra:TPS3823-xxDBV", "TPS3823-33DBV", SOT5, 40, 180, 0,
     _1="WDT_OK", _2="GND", _3="VCC_3V3", _4="WDI", _5="VCC_3V3")
cap("C14", "100nF", 25, 195, "VCC_3V3", "GND")
part("J5", "Connector_Generic:Conn_01x02", "E-STOP (NC)", "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical",
     40, 220, 0, _1="VCC_3V3", _2="ESTOP_OK")
res("R11", "10k", 55, 225, "ESTOP_OK", "GND")               # pressed or wire cut -> LOW
for n, x in ((1, 80), (2, 110)):
    part(f"U{5+n}", "74xGxx:74LVC1G00", "74LVC1G00", SOT5, x, 180, 0,
         _1=f"SEATED_{n}", _2=f"LATCHED_N_{n}", _3="GND", _4=f"BAY_SAFE_{n}", _5="VCC_3V3")
    cap(f"C{14+n}", "100nF", x, 195, "VCC_3V3", "GND")
part("U8", "74xGxx:74LVC1G11", "74LVC1G11", SOT6, 140, 180, 0,
     _1="MOT_EN", _2="GND", _3="WDT_OK", _4="EN_A", _5="VCC_3V3", _6="ESTOP_OK")
part("U9", "74xGxx:74LVC1G11", "74LVC1G11", SOT6, 170, 180, 0,
     _1="EN_A", _2="GND", _3="BAY_SAFE_1", _4="MOT_ENABLE", _5="VCC_3V3", _6="BAY_SAFE_2")
cap("C17", "100nF", 140, 195, "VCC_3V3", "GND")
cap("C18", "100nF", 170, 195, "VCC_3V3", "GND")
res("R12", "100k", 130, 210, "MOT_EN", "GND")
res("R13", "100k", 190, 210, "MOT_ENABLE", "GND")
part("Q2", "Device:Q_NMOS_GSD", "2N7002", SOT23, 205, 180, 0, _1="MOT_ENABLE", _2="GND", _3="VM_G")
res("R14", "100k", 220, 170, "VBAT", "VM_G")
part("Q3", "Device:Q_PMOS_GDS", "AOD409 (motor rail)", DPAK, 235, 180, 0, _1="VM_G", _2="VM_SW", _3="VBAT")
part("C19", "Device:C_Polarized", "220uF 25V", "Capacitor_SMD:CP_Elec_8x10", 255, 185, 0, _1="VM_SW", _2="GND")
part("D4", "Device:LED", "RED motors armed", LED, 270, 180, 0, _1="LED_ARM_K", _2="MOT_ENABLE")
res("R15", "1k", 280, 180, "LED_ARM_K", "GND")

# ============================== 6. bay sensors + latch solenoids
for n, y in ((1, 250), (2, 290)):
    part(f"J{5+n}", "Connector_Generic:Conn_01x03", f"BAY {n} SENSORS (GND,SEATED,LATCHED_N)",
         "Connector_JST:JST_PH_B3B-PH-K_1x03_P2.00mm_Vertical", 30, y, 0, _1="GND", _2=f"SEATED_{n}", _3=f"LATCHED_N_{n}")
    res(f"R{14+2*n}", "10k", 50, y, "VCC_3V3", f"SEATED_{n}")       # NC switch to GND opens when seated
    res(f"R{15+2*n}", "10k", 60, y, "VCC_3V3", f"LATCHED_N_{n}")    # NO switch to GND closes when locked
    cap(f"C{18+2*n}", "100nF", 70, y, f"SEATED_{n}", "GND")
    cap(f"C{19+2*n}", "100nF", 80, y, f"LATCHED_N_{n}", "GND")
    part(f"J{7+n}", "Connector_Generic:Conn_01x02", f"LATCH {n} SOLENOID (12V)",
         "Connector_JST:JST_VH_B2P-VH_1x02_P3.96mm_Vertical", 120, y, 0, _1="VBAT", _2=f"SOL_{n}")
    part(f"Q{3+n}", "Transistor_FET:AO3400A", "AO3400A", SOT23, 140, y, 0, _1=f"SOL_G_{n}", _2="GND", _3=f"SOL_{n}")
    part(f"D{4+n}", "Diode:SS34", "SS34 flyback", SMA, 160, y, 0, _1="VBAT", _2=f"SOL_{n}")
    res(f"R{19+2*n}", "100", 175, y, f"LATCH_OPEN_{n}", f"SOL_G_{n}")
    res(f"R{20+2*n}", "100k", 185, y, f"LATCH_OPEN_{n}", "GND")     # strap pins IO45/46 must be low at reset

# ============================== 7. I/O expander, status
part("U10", "Interface_Expansion:TCA9534", "TCA9534PWR", "Package_SO:TSSOP-16_4.4x5mm_P0.65mm", 230, 260, 0,
     _1="GND", _2="GND", _3="GND", _4="SEATED_1", _5="LATCHED_N_1", _6="SEATED_2", _7="LATCHED_N_2", _8="GND",
     _9="KALERT_N_1", _10="KALERT_N_2", _11="LED_STAT", _12="BUZZ", _13="EXP_INT", _14="I2C_SCL", _15="I2C_SDA",
     _16="VCC_3V3")
cap("C24", "100nF", 250, 250, "VCC_3V3", "GND")
res("R25", "10k", 250, 270, "VCC_3V3", "EXP_INT")
res("R26", "4.7k", 260, 230, "VCC_3V3", "I2C_SDA")
res("R27", "4.7k", 270, 230, "VCC_3V3", "I2C_SCL")
part("D7", "Device:LED", "GREEN status", LED, 280, 260, 0, _1="LED_STAT_K", _2="LED_STAT")
res("R28", "1k", 290, 260, "LED_STAT_K", "GND")
part("Q6", "Transistor_FET:AO3400A", "AO3400A", SOT23, 300, 280, 0, _1="BUZZ", _2="GND", _3="BUZZ_N")
part("BZ1", "Device:Buzzer", "5V magnetic buzzer", "Buzzer_Beeper:Buzzer_12x9.5RM7.6", 315, 270, 0, _1="VCC_5V", _2="BUZZ_N")
res("R29", "100k", 300, 295, "BUZZ", "GND")
part("J10", "Connector_Generic:Conn_01x04", "QWIIC I2C (IMU)", "Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal",
     330, 240, 0, _1="GND", _2="VCC_3V3", _3="I2C_SDA", _4="I2C_SCL")
part("J11", "Connector_Generic:Conn_02x08_Odd_Even", "TO BOARD C", "Connector_PinHeader_2.54mm:PinHeader_2x08_P2.54mm_Vertical",
     360, 260, 0, _1="VCC_3V3", _2="GND", _3="I2C_SDA", _4="I2C_SCL", _5="BAY_EN_1", _6="BAY_EN_2",
     _7="DATA_TX_1", _8="DATA_RX_1", _9="DATA_TX_2", _10="DATA_RX_2", _11="ID_1", _12="ID_2",
     _13="KALERT_N_1", _14="KALERT_N_2", _15="GND", _16="GND")
res("R30", "100k", 380, 240, "BAY_EN_1", "GND")
res("R31", "100k", 390, 240, "BAY_EN_2", "GND")

# ============================== 8. wheels: 4 x (shunt, INA180A1, DRV8871, motor + encoder connector)
WHEELS = [("LF", "MOT_L_IN1", "MOT_L_IN2"), ("LR", "MOT_L_IN1", "MOT_L_IN2"),
          ("RF", "MOT_R_IN1", "MOT_R_IN2"), ("RR", "MOT_R_IN1", "MOT_R_IN2")]
for i, (w, in1, in2) in enumerate(WHEELS):
    x0, k = 30 + 95 * i, 40 + 10 * i
    res(f"R{k}", "20m 1%", x0, 340, "VM_SW", f"VM_{w}", R12)
    part(f"U{11+i}", "Amplifier_Current:INA180A1", "INA180A1", SOT5, x0 + 20, 340, 0,
         _1=f"ISNS_{w}_RAW", _2="GND", _3="VM_SW", _4=f"VM_{w}", _5="VCC_3V3")
    res(f"R{k+1}", "1k", x0 + 40, 340, f"ISNS_{w}_RAW", f"IMOT_{w}")
    cap(f"C{30+4*i}", "100nF", x0 + 50, 340, f"IMOT_{w}", "GND")
    part(f"U{15+i}", "Driver_Motor:DRV8871DDA", "DRV8871DDA", DRV_FP, x0 + 20, 380, 0,
         _1="GND", _7="GND", _9="GND", _2=in2, _3=in1, _4=f"ILIM_{w}", _5=f"VM_{w}", _6=f"MOT_{w}_P", _8=f"MOT_{w}_N")
    res(f"R{k+2}", "22k (2.9A trip)", x0 + 45, 385, f"ILIM_{w}", "GND")
    part(f"C{31+4*i}", "Device:C_Polarized", "47uF 25V", "Capacitor_SMD:CP_Elec_6.3x7.7", x0, 380, 0, _1=f"VM_{w}", _2="GND")
    cap(f"C{32+4*i}", "100nF", x0 + 8, 395, f"VM_{w}", "GND")
    cap(f"C{33+4*i}", "100nF", x0 + 60, 405, f"ENC_{w}_A", "GND", C)   # encoder edge filter (hall, 10k)
    part(f"J{12+i}", "Connector_Generic:Conn_01x02", f"MOTOR {w}", "Connector_JST:JST_VH_B2P-VH_1x02_P3.96mm_Vertical",
         x0 + 70, 375, 0, _1=f"MOT_{w}_P", _2=f"MOT_{w}_N")
    part(f"J{16+i}", "Connector_Generic:Conn_01x04", f"ENCODER {w}", "Connector_JST:JST_PH_B4B-PH-K_1x04_P2.00mm_Vertical",
         x0 + 70, 395, 0, _1="VCC_3V3", _2="GND", _3=f"ENC_{w}_A", _4=f"ENC_{w}_B")
    res(f"R{k+3}", "10k", x0 + 50, 415, "VCC_3V3", f"ENC_{w}_A")
    res(f"R{k+4}", "10k", x0 + 60, 415, "VCC_3V3", f"ENC_{w}_B")

# ============================== 9. test points, mounting
for i, net in enumerate(("VBAT", "VCC_5V", "VCC_3V3", "VM_SW", "MOT_ENABLE", "WDT_OK", "ESTOP_OK", "GND")):
    part(f"TP{i+1}", "Connector:TestPoint", net, TP, 30 + 12.7 * i, 440, 0, _1=net)
for i in range(4):
    part(f"H{i+1}", "Mechanical:MountingHole_Pad", "M3", "MountingHole:MountingHole_3.2mm_M3_Pad_Via",
         150 + 12.7 * i, 440, 0, _1="GND")

# ---------------------------------------------------------------- derived
NETS = defaultdict(list)
for ref, lib, val, fp, x, y, rot, pins in P:
    for pin, net in pins.items():
        NETS[net].append((ref, pin))


def interlock(mot_en, wdt_ok, estop_ok, seated, latched_n):
    """Boolean model of U6-U9 for the truth-table test (seated/latched_n are per-bay tuples)."""
    bay_safe = [not (s and l) for s, l in zip(seated, latched_n)]
    return mot_en and wdt_ok and estop_ok and all(bay_safe)


if __name__ == "__main__":
    print(f"{len(P)} parts, {len(NETS)} nets")
    single = [n for n, v in NETS.items() if len(v) == 1 and not n.startswith("NC_")]
    print("single-pin nets (check!):", single)
