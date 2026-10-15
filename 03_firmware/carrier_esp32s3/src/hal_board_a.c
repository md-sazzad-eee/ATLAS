/* Board A HAL: side PWM -> 2x2 DRV8871, 4 PCNT encoders, ADC1, I2C (TCA9534, 6x INA226, 2x MCP4725). */
#include "hal_board_a.h"
#include <math.h>
#include "board_a_pins.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

#define PWM_MAX 1023
static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t cali;
static pcnt_unit_handle_t enc[4];
static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t tca, dac[2], ina[2][3];
static bool wdi;
static const int mot_pins[4] = {PIN_MOT_L_IN1, PIN_MOT_L_IN2, PIN_MOT_R_IN1, PIN_MOT_R_IN2};

static i2c_master_dev_handle_t dev(uint8_t addr)
{
    i2c_device_config_t c = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = 400000};
    i2c_master_dev_handle_t h; ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &c, &h)); return h;
}
static esp_err_t wr16(i2c_master_dev_handle_t h, uint8_t reg, uint16_t v)
{
    uint8_t b[3] = {reg, (uint8_t)(v >> 8), (uint8_t)v}; return i2c_master_transmit(h, b, 3, 5);
}
static int32_t rd16s(i2c_master_dev_handle_t h, uint8_t reg)
{
    uint8_t b[2] = {0}; if (i2c_master_transmit_receive(h, &reg, 1, b, 2, 5) != ESP_OK) return 0;
    return (int16_t)((b[0] << 8) | b[1]);
}

static pcnt_unit_handle_t quad(int pa, int pb)
{
    pcnt_unit_config_t uc = {.high_limit = 30000, .low_limit = -30000, .flags.accum_count = 1};
    pcnt_unit_handle_t u; ESP_ERROR_CHECK(pcnt_new_unit(&uc, &u));
    pcnt_glitch_filter_config_t f = {.max_glitch_ns = 1000}; pcnt_unit_set_glitch_filter(u, &f);
    pcnt_chan_config_t ca = {.edge_gpio_num = pa, .level_gpio_num = pb}, cb = {.edge_gpio_num = pb, .level_gpio_num = pa};
    pcnt_channel_handle_t A, B; pcnt_new_channel(u, &ca, &A); pcnt_new_channel(u, &cb, &B);
    pcnt_channel_set_edge_action(A, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(A, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(B, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(B, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_add_watch_point(u, 30000); pcnt_unit_add_watch_point(u, -30000);
    pcnt_unit_enable(u); pcnt_unit_clear_count(u); pcnt_unit_start(u);
    return u;
}

static int mv(adc_channel_t ch)
{
    int raw = 0, out = 0;
    if (adc_oneshot_read(adc, ch, &raw) != ESP_OK) return 0;
    if (cali && adc_cali_raw_to_voltage(cali, raw, &out) == ESP_OK) return out;
    return raw * 3100 / 4095;
}

void board_a_init(void)
{
    gpio_config_t o = {.pin_bit_mask = (1ULL << PIN_MOT_EN) | (1ULL << PIN_WDI) | (1ULL << PIN_BAY_EN_1) | (1ULL << PIN_BAY_EN_2) |
                                       (1ULL << PIN_LATCH_1) | (1ULL << PIN_LATCH_2), .mode = GPIO_MODE_OUTPUT};
    gpio_config(&o);
    board_a_safe_outputs();
    gpio_config_t i = {.pin_bit_mask = (1ULL << PIN_ESTOP_OK) | (1ULL << PIN_EXP_INT), .mode = GPIO_MODE_INPUT};
    gpio_config(&i);
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT, .timer_num = 0, .freq_hz = 20000, .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&t);
    for (int k = 0; k < 4; k++) { ledc_channel_config_t c = {.gpio_num = mot_pins[k], .speed_mode = LEDC_LOW_SPEED_MODE, .channel = k, .timer_sel = 0}; ledc_channel_config(&c); }
    enc[W_LF] = quad(PIN_ENC_LF_A, PIN_ENC_LF_B); enc[W_LR] = quad(PIN_ENC_LR_A, PIN_ENC_LR_B);
    enc[W_RF] = quad(PIN_ENC_RF_A, PIN_ENC_RF_B); enc[W_RR] = quad(PIN_ENC_RR_A, PIN_ENC_RR_B);
    adc_oneshot_unit_init_cfg_t uc = {.unit_id = ADC_UNIT_1}; adc_oneshot_new_unit(&uc, &adc);
    adc_oneshot_chan_cfg_t cc = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    for (int ch = 0; ch <= 6; ch++) adc_oneshot_config_channel(adc, ch, &cc);
    adc_cali_curve_fitting_config_t cf = {.unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    if (adc_cali_create_scheme_curve_fitting(&cf, &cali) != ESP_OK) cali = NULL;
    i2c_master_bus_config_t bc = {.i2c_port = 0, .sda_io_num = PIN_I2C_SDA, .scl_io_num = PIN_I2C_SCL,
                                  .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7};
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &bus));
    tca = dev(I2C_TCA9534); dac[0] = dev(I2C_DAC_1); dac[1] = dev(I2C_DAC_2);
    const uint8_t a[2][3] = {INA_BAY1, INA_BAY2};
    for (int b = 0; b < 2; b++)
        for (int k = 0; k < 3; k++) {
            ina[b][k] = dev(a[b][k]);
            wr16(ina[b][k], 0x00, 0x4527);               /* avg 16, 1.1 ms conversions, shunt+bus continuous */
            if (k > 0) {                                  /* contact monitors: ALERT on shunt over-voltage */
                wr16(ina[b][k], 0x07, 32000);             /* 80 mV = 80 mOhm at 1 A (2.5 uV/LSB) */
                wr16(ina[b][k], 0x06, 0x8000);            /* SOL */
            }
        }
    uint8_t cfg[2] = {0x03, 0x3F};                        /* P0-P5 inputs, P6 LED, P7 buzzer outputs */
    i2c_master_transmit(tca, cfg, 2, 5);
}

void board_a_read(atlas_carrier_in_t *in)
{
    static const int sign[4] = {1, 1, -1, -1};            /* right side mounted mirrored */
    for (int k = 0; k < 4; k++) { int c; pcnt_unit_get_count(enc[k], &c); in->enc[k] = sign[k] * c; }
    in->vbat_mv = (uint16_t)(mv(ADC_CHANNEL_0) * 118 / 18);
    const adc_channel_t ich[4] = {ADC_CHANNEL_1, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5};
    for (int k = 0; k < 4; k++) in->imot_ma[k] = (uint16_t)(mv(ich[k]) * 5 / 2);   /* 0.4 V/A */
    in->bay[0].id_mv = (uint16_t)mv(ADC_CHANNEL_2); in->bay[1].id_mv = (uint16_t)mv(ADC_CHANNEL_6);
    in->estop_ok = gpio_get_level(PIN_ESTOP_OK);
    uint8_t reg = 0x00, p = 0xFF;
    i2c_master_transmit_receive(tca, &reg, 1, &p, 1, 5);
    for (int b = 0; b < 2; b++) {
        atlas_bay_in_t *bi = &in->bay[b];
        bi->seated = (p >> (2 * b)) & 1;                  /* NC switch opens when seated -> pulled high */
        bi->latched = !((p >> (2 * b + 1)) & 1);           /* LATCHED_N low when the pin is locked */
        bi->kalert = !((p >> (4 + b)) & 1);
        int32_t vsh = rd16s(ina[b][0], 0x01);             /* 2.5 uV/LSB on 20 mOhm -> 0.125 mA/LSB */
        bi->i_ma = (uint16_t)(vsh > 0 ? vsh / 8 : 0);
        bi->vpad_mv = (uint16_t)(rd16s(ina[b][0], 0x02) * 5 / 4);
        bi->vdrop_vchg_uv = rd16s(ina[b][1], 0x01) * 5 / 2;
        bi->vdrop_gnd_uv = rd16s(ina[b][2], 0x01) * 5 / 2;
    }
}

static void side(int a, int b, float d)
{
    if (d > 1) d = 1; else if (d < -1) d = -1;
    uint32_t on = (uint32_t)lroundf(fabsf(d) * PWM_MAX);
    uint32_t da = on == 0 ? 0 : (d > 0 ? PWM_MAX : PWM_MAX - on), db = on == 0 ? 0 : (d > 0 ? PWM_MAX - on : PWM_MAX);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, a, da); ledc_update_duty(LEDC_LOW_SPEED_MODE, a);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, b, db); ledc_update_duty(LEDC_LOW_SPEED_MODE, b);
}

void board_a_apply(const atlas_carrier_out_t *o, uint16_t vbat_mv)
{
    static uint16_t last_iset[2] = {0xFFFF, 0xFFFF};
    float vb = vbat_mv > 9000 ? vbat_mv / 1000.0f : 12.8f;
    gpio_set_level(PIN_MOT_EN, o->mot_en);
    side(0, 1, o->mot_en ? o->motor_l_v / vb : 0);
    side(2, 3, o->mot_en ? -o->motor_r_v / vb : 0);
    for (int b = 0; b < 2; b++) {
        if (o->iset_ma[b] != last_iset[b]) {               /* DAC -> 2:1 divider -> ISET (1 V/A) */
            uint32_t code = (uint32_t)o->iset_ma[b] * 2 * 4095 / 3300; if (code > 4095) code = 4095;
            uint8_t w[2] = {(uint8_t)(code >> 8), (uint8_t)code};    /* fast-write mode */
            if (i2c_master_transmit(dac[b], w, 2, 5) == ESP_OK) last_iset[b] = o->iset_ma[b];
        }
    }
    gpio_set_level(PIN_BAY_EN_1, o->bay_en[0]); gpio_set_level(PIN_BAY_EN_2, o->bay_en[1]);
    gpio_set_level(PIN_LATCH_1, o->latch_open[0]); gpio_set_level(PIN_LATCH_2, o->latch_open[1]);
    uint8_t out[2] = {0x01, (uint8_t)((o->led ? 0x40 : 0) | (o->buzzer ? 0x80 : 0))};
    i2c_master_transmit(tca, out, 2, 5);
}

void board_a_kick_watchdog(void) { wdi = !wdi; gpio_set_level(PIN_WDI, wdi); }

void board_a_safe_outputs(void)
{
    gpio_set_level(PIN_MOT_EN, 0); gpio_set_level(PIN_BAY_EN_1, 0); gpio_set_level(PIN_BAY_EN_2, 0);
    gpio_set_level(PIN_LATCH_1, 0); gpio_set_level(PIN_LATCH_2, 0);
}
