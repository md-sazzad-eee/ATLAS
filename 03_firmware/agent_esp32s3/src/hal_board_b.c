/* Board B HAL: LEDC PWM -> DRV8871, PCNT x4 quadrature, ADC1 one-shot (calibrated), GPIO. */
#include "hal_board_b.h"
#include <math.h>
#include "board_b_pins.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "hal";
static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t cali;
static pcnt_unit_handle_t enc[2];
static bool wdi_level;

#define PWM_HZ   20000
#define PWM_RES  LEDC_TIMER_10_BIT
#define PWM_MAX  1023

static const int mot_pins[4] = {PIN_MOT_L_IN1, PIN_MOT_L_IN2, PIN_MOT_R_IN1, PIN_MOT_R_IN2};

static void pwm_init(void)
{
    ledc_timer_config_t t = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = PWM_RES,
                             .timer_num = LEDC_TIMER_0, .freq_hz = PWM_HZ, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t c = {.gpio_num = mot_pins[i], .speed_mode = LEDC_LOW_SPEED_MODE, .channel = i,
                                   .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0};
        ESP_ERROR_CHECK(ledc_channel_config(&c));
    }
}

static void set_duty(int ch, uint32_t d) { ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, d); ledc_update_duty(LEDC_LOW_SPEED_MODE, ch); }

/* DRV8871: forward = IN1 high, IN2 PWM-inverted (drive/brake slow decay: better low-speed linearity).
   duty fraction d in [-1, 1]. d == 0 -> both low (coast). */
static void motor(int m, float d)
{
    int a = 2 * m, b = 2 * m + 1;
    if (d > 1) d = 1; else if (d < -1) d = -1;
    uint32_t on = (uint32_t)lroundf(fabsf(d) * PWM_MAX);
    if (on == 0) { set_duty(a, 0); set_duty(b, 0); }
    else if (d > 0) { set_duty(a, PWM_MAX); set_duty(b, PWM_MAX - on); }
    else { set_duty(a, PWM_MAX - on); set_duty(b, PWM_MAX); }
}

static pcnt_unit_handle_t quad(int pa, int pb)
{
    pcnt_unit_config_t uc = {.high_limit = 30000, .low_limit = -30000, .flags.accum_count = 1};
    pcnt_unit_handle_t u; ESP_ERROR_CHECK(pcnt_new_unit(&uc, &u));
    pcnt_glitch_filter_config_t f = {.max_glitch_ns = 1000}; ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(u, &f));
    pcnt_chan_config_t ca = {.edge_gpio_num = pa, .level_gpio_num = pb}, cb = {.edge_gpio_num = pb, .level_gpio_num = pa};
    pcnt_channel_handle_t A, B; ESP_ERROR_CHECK(pcnt_new_channel(u, &ca, &A)); ESP_ERROR_CHECK(pcnt_new_channel(u, &cb, &B));
    pcnt_channel_set_edge_action(A, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(A, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(B, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(B, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_unit_add_watch_point(u, 30000); pcnt_unit_add_watch_point(u, -30000);   /* needed for accum_count */
    ESP_ERROR_CHECK(pcnt_unit_enable(u)); ESP_ERROR_CHECK(pcnt_unit_clear_count(u)); ESP_ERROR_CHECK(pcnt_unit_start(u));
    return u;
}

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t uc = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&uc, &adc));
    adc_oneshot_chan_cfg_t cc = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    const adc_channel_t chs[] = {ADC_CH_VBAT, ADC_CH_IMOT_L, ADC_CH_IMOT_R, ADC_CH_NTC, ADC_CH_DOCK, ADC_CH_SENS};
    for (unsigned i = 0; i < sizeof chs / sizeof chs[0]; i++) ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, chs[i], &cc));
    adc_cali_curve_fitting_config_t cf = {.unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    if (adc_cali_create_scheme_curve_fitting(&cf, &cali) != ESP_OK) { cali = NULL; ESP_LOGW(TAG, "no ADC eFuse calibration"); }
}

static int mv(adc_channel_t ch)
{
    int raw = 0, out = 0;
    if (adc_oneshot_read(adc, ch, &raw) != ESP_OK) return 0;
    if (cali && adc_cali_raw_to_voltage(cali, raw, &out) == ESP_OK) return out;
    return raw * 3100 / 4095;
}

/* 10k NTC (B = 3950, R25 = 10k) to GND with a 10k pull-up to 3.3 V. Returns 0.1 degC. */
static int16_t ntc_dc(int v_mv)
{
    if (v_mv <= 10 || v_mv >= 3290) return v_mv <= 10 ? 1500 : -400;   /* short -> "hot", open -> "cold": both inhibit charging */
    float r = 10000.0f * v_mv / (3300.0f - v_mv);
    float t = 1.0f / (1.0f / 298.15f + logf(r / 10000.0f) / 3950.0f) - 273.15f;
    return (int16_t)lroundf(t * 10);
}

void board_b_init(void)
{
    gpio_config_t o = {.pin_bit_mask = (1ULL << PIN_MOT_EN) | (1ULL << PIN_CHG_EN) | (1ULL << PIN_PI_PWR_EN) |
                                       (1ULL << PIN_LED_STAT) | (1ULL << PIN_WDI),
                       .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&o));
    board_b_safe_outputs();
    pwm_init();
    enc[0] = quad(PIN_ENC_L_A, PIN_ENC_L_B);
    enc[1] = quad(PIN_ENC_R_A, PIN_ENC_R_B);
    adc_init();
}

void board_b_read(atlas_agent_in_t *in)
{
    int c;
    pcnt_unit_get_count(enc[0], &c); in->enc_l = c;
    pcnt_unit_get_count(enc[1], &c); in->enc_r = -c;          /* right motor is mirrored */
    in->vbat_mv   = (uint16_t)(mv(ADC_CH_VBAT) * DIV_VBAT_NUM / DIV_VBAT_DEN);
    in->imot_l_ma = (uint16_t)mv(ADC_CH_IMOT_L);              /* 1 mV == 1 mA */
    in->imot_r_ma = (uint16_t)mv(ADC_CH_IMOT_R);
    in->batt_temp_dc = ntc_dc(mv(ADC_CH_NTC));
    in->dock_mv   = (uint16_t)(mv(ADC_CH_DOCK) * DIV_VBAT_NUM / DIV_VBAT_DEN);
    in->sens_mv   = (uint16_t)mv(ADC_CH_SENS);
}

void board_b_apply(const atlas_agent_out_t *out, uint16_t vbat_mv)
{
    float vb = vbat_mv > 5000 ? vbat_mv / 1000.0f : 7.4f;
    gpio_set_level(PIN_MOT_EN, out->mot_en);
    motor(0, out->mot_en ? out->motor_l_v / vb : 0);
    motor(1, out->mot_en ? -out->motor_r_v / vb : 0);           /* mirrored mounting */
    gpio_set_level(PIN_CHG_EN, out->chg_en);
    gpio_set_level(PIN_PI_PWR_EN, out->pi_pwr_en);
    gpio_set_level(PIN_LED_STAT, out->led);
}

void board_b_kick_watchdog(void) { wdi_level = !wdi_level; gpio_set_level(PIN_WDI, wdi_level); }

void board_b_safe_outputs(void)
{
    gpio_set_level(PIN_MOT_EN, 0); gpio_set_level(PIN_CHG_EN, 0);
    for (int i = 0; i < 4; i++) gpio_set_level(mot_pins[i], 0);
}
