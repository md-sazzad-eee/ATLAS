/* Board B rev A1 pin map. Source of truth: 02_electronics/agent_controller/design/board_b_design.py
 * (verified against the KiCad netlist by verify_netlist.py). Do not edit one without the other. */
#pragma once
#define PIN_I2C_SDA      1
#define PIN_I2C_SCL      2
#define PIN_PI_RX        3      /* ESP32 RX  <- Pi TX  (UART1) */
#define PIN_VBAT_SENSE   4      /* ADC1_CH3, 100k/22k divider  */
#define PIN_IMOT_L       5      /* ADC1_CH4, INA180A2 x50 on 20 mOhm: 1.0 V per A */
#define PIN_IMOT_R       6      /* ADC1_CH5 */
#define PIN_BATT_NTC     7      /* ADC1_CH6, 10k pull-up to 3V3, 10k NTC (B 3950) to GND */
#define PIN_DOCK_VSENSE  8      /* ADC1_CH7, 100k/22k divider on VCHG */
#define PIN_SENS_ADC     9      /* ADC1_CH8, research instrument analogue output */
#define PIN_SENS_GPIO   10
#define PIN_ENC_L_A     11
#define PIN_ENC_L_B     12
#define PIN_ENC_R_A     13
#define PIN_ENC_R_B     14
#define PIN_MOT_L_IN1   15
#define PIN_MOT_L_IN2   16
#define PIN_LED_STAT    17
#define PIN_PI_PWR_EN   18
#define PIN_MOT_EN      21      /* AND-ed with WDT_OK in hardware (74LVC1G08) */
#define PIN_WDI         38      /* TPS3823 watchdog input, 1.6 s timeout */
#define PIN_DOCK_TX     39      /* UART2, half-duplex DATA pad through 1k */
#define PIN_DOCK_RX     40
#define PIN_CHG_EN      41
#define PIN_MOT_R_IN2   42
#define PIN_MOT_R_IN1   47
#define PIN_PI_TX       48      /* ESP32 TX  -> Pi RX  (UART1) */

#define ADC_CH_VBAT      ADC_CHANNEL_3
#define ADC_CH_IMOT_L    ADC_CHANNEL_4
#define ADC_CH_IMOT_R    ADC_CHANNEL_5
#define ADC_CH_NTC       ADC_CHANNEL_6
#define ADC_CH_DOCK      ADC_CHANNEL_7
#define ADC_CH_SENS      ADC_CHANNEL_8

#define DIV_VBAT_NUM 122        /* (100k + 22k) / 22k */
#define DIV_VBAT_DEN 22
