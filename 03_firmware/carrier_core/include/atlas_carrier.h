/* ATLAS-M carrier application core (Board A, ESP32-S3). Hardware-independent and host-tested
 * (test/test_carrier.c); the ESP32 port only fills atlas_carrier_in_t and applies atlas_carrier_out_t.
 *
 *  - 2 docking FSMs (03_firmware/common/atlas_dock_fsm, parity-tested against Stateflow)
 *  - move permit: every bay is (DOCKED and latched) or (IDLE and empty)  [hardware gate on Board A
 *    additionally removes motor power if any bay is seated-but-unlatched]
 *  - per-bay charge controller for Board C: contact check at 0.5 A (4-wire contact resistance),
 *    then 1.0 A CC; trips on contact resistance > 80 mOhm, ends at < 50 mA taper; only in DOCKED
 *  - skid-steer side speed loops (shared PWM per side, mean of 2 encoders), cmd_vel dead-man,
 *    front/rear slip detection
 *  - frames to/from the ROS 2 computer (USB CDC): HEARTBEAT, DRIVE_CMD (cmd_vel), DOCK_CMD in;
 *    CARRIER_STATUS, BAY_STATUS out
 */
#ifndef ATLAS_CARRIER_H
#define ATLAS_CARRIER_H
#include <stdbool.h>
#include <stdint.h>
#include "atlas_dock_fsm.h"
#include "atlas_pi.h"
#include "atlas_proto.h"

#define CARRIER_DT_MS 10u
enum { W_LF = 0, W_LR = 1, W_RF = 2, W_RR = 3 };
typedef enum { CHG_OFF, CHG_CHECK, CHG_CC, CHG_DONE, CHG_FAULT } chg_state_t;

typedef struct {
    float wheel_r_m, track_m, counts_per_wheel_rev, vmax_mps, wmax_radps, kp, ki, vlimit;
    uint32_t cmd_ttl_max_ms, sbc_timeout_ms;
    /* charging */
    uint16_t i_check_ma, i_charge_ma, i_term_ma, rc_max_mohm;
    uint32_t check_settle_ms, check_window_ms, term_hold_ms, rc_trip_ms;
    /* agent identity by ID resistor (Board C pull-up 10k to 3V3) */
    uint16_t id_mv[3], id_tol_mv;          /* [1] scout 10k -> 1650 mV, [2] research 22k -> 2269 mV */
    uint8_t dock_retry_limit;
} atlas_carrier_cfg_t;

typedef struct {
    bool seated, latched;                  /* latched = LATCHED_N low (pin fully locked)          */
    uint16_t id_mv;
    bool kalert;                           /* INA226 alert (hardware already cut BAY_EN)         */
    uint16_t i_ma, vpad_mv;                /* INA226 #1                                          */
    int32_t vdrop_vchg_uv, vdrop_gnd_uv;   /* INA226 #2/#3 shunt-voltage = I * R_contact         */
} atlas_bay_in_t;

typedef struct {
    uint32_t now_ms;
    int32_t enc[4];
    uint16_t imot_ma[4], vbat_mv;
    bool estop_ok;
    atlas_bay_in_t bay[2];
} atlas_carrier_in_t;

typedef struct {
    float motor_l_v, motor_r_v;
    bool mot_en, move_permit, wdi, buzzer, led;
    bool bay_en[2], latch_open[2];
    uint16_t iset_ma[2];
    int8_t agent_drive[2];                 /* forwarded to the agent (via ROS / Wi-Fi) as DRIVE_CMD */
} atlas_carrier_out_t;

typedef struct {
    atlas_dock_fsm_t fsm;
    chg_state_t chg; uint32_t chg_t0, low_since, rc_bad_since;
    double acc_i, acc_v, acc_g; uint32_t acc_n;
    uint16_t rc_v_mohm, rc_g_mohm;
    uint8_t expected_agent;
    bool cmd_dock, cmd_undock, cmd_reset, agent_comm_ok;
    uint32_t unseated_since, cmd_t;
} atlas_bay_t;

typedef void (*atlas_carrier_send_fn)(uint8_t type, const void *payload, size_t n, void *ctx);

typedef struct {
    atlas_carrier_cfg_t cfg;
    atlas_carrier_out_t out;
    atlas_bay_t bay[2];
    atlas_pi_t pi_l, pi_r;
    int32_t enc_prev[4]; bool enc_init;
    float wl, wr, w_wheel[4], x, y, yaw;
    float v_cmd, w_cmd; uint32_t cmd_until;
    uint32_t now, last_sbc, last_status, still_since, brake_until;
    bool sbc_seen, soft_estop, slip_l, slip_r;
    uint32_t slip_l_since, slip_r_since;
    atlas_carrier_send_fn send; void *send_ctx;
} atlas_carrier_t;

void atlas_carrier_default_cfg(atlas_carrier_cfg_t *c);
void atlas_carrier_init(atlas_carrier_t *c, const atlas_carrier_cfg_t *cfg, atlas_carrier_send_fn send, void *ctx);
const atlas_carrier_out_t *atlas_carrier_tick(atlas_carrier_t *c, const atlas_carrier_in_t *in);
void atlas_carrier_on_frame(atlas_carrier_t *c, uint8_t type, const uint8_t *payload, int n);
const char *atlas_chg_state_name(chg_state_t s);
#endif
