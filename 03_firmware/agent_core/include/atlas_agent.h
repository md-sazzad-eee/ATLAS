/* ATLAS-S agent application core (Board B, ESP32-S3).
 *
 * Hardware-independent: everything the agent DECIDES lives here and is unit-tested on the host
 * against a DC-motor + differential-drive plant (test/test_agent.c). The ESP32 port
 * (03_firmware/agent_esp32s3) only reads sensors into atlas_agent_in_t, calls atlas_agent_tick()
 * every 10 ms and applies atlas_agent_out_t to the pins.
 *
 * Responsibilities
 *   - wheel-speed PI loops (gains from build_speed_loop.m) + odometry from quadrature encoders
 *   - go-to-goal navigation for TASK_CMD, timed DRIVE_CMD for docking manoeuvres (carrier FSM drives)
 *   - task execution: measure / calibration_check / recalibrate (SENS_ADC), inspect (Pi vision tag),
 *     return_to_dock
 *   - safety: carrier heartbeat loss -> stop + motor power off; stall / over-current -> FAULT;
 *     battery under-voltage; watchdog kicked only while the loop is healthy
 *   - charge enable only with dock voltage present, pack temperature in range, pack below cut-off
 *   - Pi Zero power sequencing (power on demand, SHUTDOWN message, hard off after grace period)
 */
#ifndef ATLAS_AGENT_H
#define ATLAS_AGENT_H
#include <stdbool.h>
#include <stdint.h>
#include "atlas_pi.h"
#include "atlas_proto.h"

#define AGENT_DT_MS 10u

enum { HEALTH_READY = 0, HEALTH_BUSY = 1, HEALTH_DEGRADED = 2, HEALTH_FAULT = 3, HEALTH_OFFLINE = 4 };
enum { RES_SUCCESS = 0, RES_INCONCLUSIVE = 1, RES_MEAS_INVALID = 2, RES_FAILED = 3, RES_ABORTED = 4 };
enum { KIND_INSPECT = 0, KIND_MEASURE = 1, KIND_CAL_CHECK = 2, KIND_RECAL = 3, KIND_RETURN = 4 };
typedef enum { MODE_IDLE, MODE_GOTO, MODE_WORK, MODE_DRIVE, MODE_SAFE_STOP, MODE_FAULT } agent_mode_t;
enum { LINK_CARRIER = 0, LINK_PI = 1 };
enum { FAULT_NONE = 0, FAULT_STALL_L = 1, FAULT_STALL_R = 2, FAULT_UNDERVOLT = 4, FAULT_OVERTEMP = 8 };

typedef struct {
    /* geometry / drive (atlas_params.yaml agent.*) */
    float wheel_r_m, track_m, counts_per_wheel_rev, vmax_mps, wmax_radps;
    float kp, ki;                 /* V per rad/s, V per rad (build_speed_loop.m)        */
    /* navigation */
    float arrive_tol_m, heading_tol_rad, k_heading, k_dist;
    /* safety */
    uint32_t hb_timeout_ms;       /* carrier heartbeat                                  */
    uint16_t stall_ma; uint32_t stall_ms;
    uint16_t vbat_low_mv, vbat_cutoff_mv;
    /* charging */
    uint16_t dock_present_mv, chg_stop_mv, chg_restart_mv;
    int16_t chg_tmin_dc, chg_tmax_dc;      /* pack temperature window, 0.1 degC     */
    /* tasks */
    uint32_t measure_ms, inspect_tag_timeout_ms, pi_boot_ms, pi_shutdown_grace_ms;
    float sens_gain, sens_offset, cal_ref_value;   /* value = gain*mV/1000 + offset   */
    bool has_pi;                  /* Scout: Pi Zero 2 W camera module fitted            */
} atlas_agent_cfg_t;

typedef struct {
    uint32_t now_ms;
    int32_t enc_l, enc_r;         /* cumulative quadrature counts, +ve = forward         */
    uint16_t vbat_mv, imot_l_ma, imot_r_ma, dock_mv, sens_mv;
    int16_t batt_temp_dc;         /* 0.1 degC from the NTC                              */
} atlas_agent_in_t;

typedef struct {
    float motor_l_v, motor_r_v;   /* signed volts; port converts to PWM with vbat       */
    bool mot_en, chg_en, pi_pwr_en, wdi, led;
} atlas_agent_out_t;

typedef void (*atlas_agent_send_fn)(int link, uint8_t type, const void *payload, size_t n, void *ctx);

typedef struct {
    atlas_agent_cfg_t cfg;
    atlas_agent_out_t out;
    agent_mode_t mode, mode_before_stop;
    atlas_pi_t pi_l, pi_r;
    float x, y, yaw, wl_meas, wr_meas, wl_ref, wr_ref;
    int32_t enc_l_prev, enc_r_prev; bool enc_init;
    uint32_t now, last_hb, last_tel, t_mode;
    uint32_t stall_l_since, stall_r_since;
    uint8_t faults, seq_carrier, seq_pi;
    bool charging, docked, hb_seen;
    /* current task */
    bool task_active; atlas_task_cmd_t task; uint32_t task_t0;
    double sens_acc; uint32_t sens_n;
    bool tag_seen; atlas_vision_tag_t tag;
    uint16_t cal_epoch; uint16_t last_result_id; uint8_t last_result_status;
    /* drive command (docking) */
    float drv_v, drv_w; uint32_t drv_until, brake_until;
    /* Pi power */
    bool pi_wanted; uint32_t pi_on_since, pi_off_at; bool pi_shutdown_sent;
    atlas_agent_send_fn send; void *send_ctx;
    uint32_t tick_count, frames_rx, frames_bad;
} atlas_agent_t;

void atlas_agent_default_cfg(atlas_agent_cfg_t *c, bool has_pi);
void atlas_agent_init(atlas_agent_t *a, const atlas_agent_cfg_t *c, atlas_agent_send_fn send, void *ctx);
/* call every AGENT_DT_MS */
const atlas_agent_out_t *atlas_agent_tick(atlas_agent_t *a, const atlas_agent_in_t *in);
/* deliver one decoded frame from a link */
void atlas_agent_on_frame(atlas_agent_t *a, int link, uint8_t type, const uint8_t *payload, int n);
const char *atlas_agent_mode_name(agent_mode_t m);
#endif
