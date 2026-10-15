/* Board A rev A pin map. Source of truth: 02_electronics/carrier_controller/design/board_a_design.py */
#pragma once
#define PIN_VBAT_SENSE  1    /* ADC1_CH0, 100k/18k */
#define PIN_IMOT_LF     2    /* ADC1_CH1, INA180A1 x20 on 20 mOhm: 0.4 V per A */
#define PIN_ID_1        3    /* ADC1_CH2, Board C: 10k pull-up, agent 10k/22k */
#define PIN_IMOT_LR     4
#define PIN_IMOT_RF     5
#define PIN_IMOT_RR     6
#define PIN_ID_2        7
#define PIN_I2C_SDA     8
#define PIN_I2C_SCL     9
#define PIN_MOT_L_IN1  10    /* both left DRV8871s */
#define PIN_MOT_L_IN2  11
#define PIN_MOT_R_IN1  12    /* both right DRV8871s */
#define PIN_MOT_R_IN2  13
#define PIN_ENC_LF_A   14
#define PIN_ENC_LF_B   15
#define PIN_ENC_LR_A   16
#define PIN_ENC_LR_B   17
#define PIN_ENC_RF_A   18
#define PIN_ENC_RF_B   21
#define PIN_ENC_RR_A   35
#define PIN_ENC_RR_B   36
#define PIN_MOT_EN     37    /* AND-ed with WDT_OK, ESTOP_OK, BAY_SAFE_1/2 in hardware */
#define PIN_WDI        38
#define PIN_DATA_TX_1  39
#define PIN_DATA_RX_1  40
#define PIN_DATA_TX_2  41
#define PIN_DATA_RX_2  42
#define PIN_ESTOP_OK   43
#define PIN_EXP_INT    44
#define PIN_LATCH_1    45    /* strap: low at reset == latch locked */
#define PIN_LATCH_2    46
#define PIN_BAY_EN_1   47
#define PIN_BAY_EN_2   48
#define I2C_TCA9534    0x20
#define I2C_DAC_1      0x60
#define I2C_DAC_2      0x61
/* INA226 per bay: { shunt (I, Vpad), VCHG contact, GND contact } */
#define INA_BAY1       {0x40, 0x46, 0x44}
#define INA_BAY2       {0x45, 0x42, 0x43}
