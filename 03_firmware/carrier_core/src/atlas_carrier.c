/* ATLAS-M carrier application core. See atlas_carrier.h. */
#include "atlas_carrier.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define HB_SOFT_ESTOP 0x01u
#define HB_AGENT1_OK  0x02u
#define HB_AGENT2_OK  0x04u
#define BRAKE_MS      300u
#define STATUS_MS     200u

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static bool elapsed(uint32_t now, uint32_t since, uint32_t ms) { return (uint32_t)(now - since) >= ms; }

const char *atlas_chg_state_name(chg_state_t s)
{
    static const char *n[] = {"OFF", "CHECK", "CC", "DONE", "FAULT"};
    return (unsigned)s < 5 ? n[s] : "?";
}

void atlas_carrier_default_cfg(atlas_carrier_cfg_t *c)
{
    memset(c, 0, sizeof *c);
    c->wheel_r_m = 0.050f; c->track_m = 0.400f;              /* atlas_params.yaml carrier.*          */
    c->counts_per_wheel_rev = 11.0f * 4.0f * 50.0f;         /* ASSUMED 11 CPR hall, x4, 1:50 gear   */
    c->vmax_mps = 0.40f; c->wmax_radps = 1.2f;
    c->kp = 0.9f; c->ki = 11.5f; c->vlimit = 12.0f;         /* tuned on the assumed motor (test)    */
    c->cmd_ttl_max_ms = 500; c->sbc_timeout_ms = 500;
    c->i_check_ma = 500; c->i_charge_ma = 1000; c->i_term_ma = 50; c->rc_max_mohm = 80;
    c->check_settle_ms = 1500; c->check_window_ms = 500; c->term_hold_ms = 10000; c->rc_trip_ms = 200;
    c->id_mv[0] = 0; c->id_mv[1] = 1650; c->id_mv[2] = 2269; c->id_tol_mv = 150;
    c->dock_retry_limit = 3;
}

void atlas_carrier_init(atlas_carrier_t *c, const atlas_carrier_cfg_t *cfg, atlas_carrier_send_fn send, void *ctx)
{
    memset(c, 0, sizeof *c);
    c->cfg = *cfg; c->send = send; c->send_ctx = ctx;
    float dt = CARRIER_DT_MS / 1000.0f;
    atlas_pi_init(&c->pi_l, cfg->kp, cfg->ki, dt, -cfg->vlimit, cfg->vlimit);
    atlas_pi_init(&c->pi_r, cfg->kp, cfg->ki, dt, -cfg->vlimit, cfg->vlimit);
    for (int b = 0; b < 2; b++) atlas_dock_init(&c->bay[b].fsm, cfg->dock_retry_limit);
}

void atlas_carrier_on_frame(atlas_carrier_t *c, uint8_t type, const uint8_t *p, int n)
{
    switch (type) {
    case MSG_HEARTBEAT: {
        if (n < (int)sizeof(atlas_heartbeat_t)) return;
        atlas_heartbeat_t hb; memcpy(&hb, p, sizeof hb);
        c->last_sbc = c->now; c->sbc_seen = true;
        c->soft_estop = hb.state & HB_SOFT_ESTOP;
        c->bay[0].agent_comm_ok = hb.state & HB_AGENT1_OK;
        c->bay[1].agent_comm_ok = hb.state & HB_AGENT2_OK;
        break;
    }
    case MSG_DRIVE_CMD: {
        if (n != 6) return;
        int16_t v, w; uint16_t ttl; memcpy(&v, p, 2); memcpy(&w, p + 2, 2); memcpy(&ttl, p + 4, 2);
        c->v_cmd = clampf(v / 1000.0f, -c->cfg.vmax_mps, c->cfg.vmax_mps);
        c->w_cmd = clampf(w / 1000.0f, -c->cfg.wmax_radps, c->cfg.wmax_radps);
        c->cmd_until = c->now + (ttl > c->cfg.cmd_ttl_max_ms ? c->cfg.cmd_ttl_max_ms : ttl);
        break;
    }
    case MSG_DOCK_CMD: {
        if (n != 3 || p[0] > 1) return;
        atlas_bay_t *b = &c->bay[p[0]];
        if (p[1] == 0) { b->cmd_dock = true; b->cmd_t = c->now; b->expected_agent = p[2] <= 2 ? p[2] : 0; }
        else if (p[1] == 1) b->cmd_undock = true;
        else if (p[1] == 2) b->cmd_reset = true;
        break;
    }
    default: break;
    }
}

static bool id_ok(const atlas_carrier_t *c, const atlas_bay_t *b, uint16_t mv)
{
    const atlas_carrier_cfg_t *k = &c->cfg;
    for (int a = 1; a <= 2; a++)
        if ((b->expected_agent == 0 || b->expected_agent == a) && abs((int)mv - (int)k->id_mv[a]) <= k->id_tol_mv) return true;
    return false;
}

/* ---- Board C charge controller (one bay) ---- */
static void charge(atlas_carrier_t *c, int i, const atlas_bay_in_t *in, bool *en, uint16_t *iset)
{
    atlas_bay_t *b = &c->bay[i];
    const atlas_carrier_cfg_t *k = &c->cfg;
    bool docked = b->fsm.out.state == DOCK_DOCKED;
    if (!docked) { b->chg = CHG_OFF; *en = false; *iset = 0; return; }
    if (in->kalert && (b->chg == CHG_CHECK || b->chg == CHG_CC)) b->chg = CHG_FAULT;   /* hardware tripped */
    switch (b->chg) {
    case CHG_OFF:
        b->chg = CHG_CHECK; b->chg_t0 = c->now; b->acc_i = b->acc_v = b->acc_g = 0; b->acc_n = 0;
        /* fall through */
    case CHG_CHECK:
        *iset = k->i_check_ma; *en = true;
        if (elapsed(c->now, b->chg_t0, k->check_settle_ms)) {
            b->acc_i += in->i_ma; b->acc_v += in->vdrop_vchg_uv; b->acc_g += in->vdrop_gnd_uv; b->acc_n++;
        }
        if (elapsed(c->now, b->chg_t0, k->check_settle_ms + k->check_window_ms)) {
            double ia = b->acc_i / b->acc_n;
            if (ia < 100.0) { b->chg = CHG_DONE; break; }             /* pack full or agent switch open */
            b->rc_v_mohm = (uint16_t)fmin(65535.0, b->acc_v / b->acc_n / ia);   /* uV / mA = mOhm */
            b->rc_g_mohm = (uint16_t)fmin(65535.0, b->acc_g / b->acc_n / ia);
            if (b->rc_v_mohm > k->rc_max_mohm || b->rc_g_mohm > k->rc_max_mohm) b->chg = CHG_FAULT;
            else { b->chg = CHG_CC; b->low_since = c->now; b->rc_bad_since = c->now; }
        }
        break;
    case CHG_CC:
        *iset = k->i_charge_ma; *en = true;
        if (in->i_ma > 300) {
            b->rc_v_mohm = (uint16_t)fmin(65535.0, (double)in->vdrop_vchg_uv / in->i_ma);
            b->rc_g_mohm = (uint16_t)fmin(65535.0, (double)in->vdrop_gnd_uv / in->i_ma);
            if (b->rc_v_mohm <= k->rc_max_mohm && b->rc_g_mohm <= k->rc_max_mohm) b->rc_bad_since = c->now;
            else if (elapsed(c->now, b->rc_bad_since, k->rc_trip_ms)) { b->chg = CHG_FAULT; break; }
        } else b->rc_bad_since = c->now;
        if (in->i_ma >= k->i_term_ma) b->low_since = c->now;
        else if (elapsed(c->now, b->low_since, k->term_hold_ms)) b->chg = CHG_DONE;
        break;
    default: break;
    }
    if (b->chg == CHG_DONE || b->chg == CHG_FAULT) { *en = false; *iset = 0; }
}

const atlas_carrier_out_t *atlas_carrier_tick(atlas_carrier_t *c, const atlas_carrier_in_t *in)
{
    const atlas_carrier_cfg_t *k = &c->cfg;
    atlas_carrier_out_t *o = &c->out;
    const float dt = CARRIER_DT_MS / 1000.0f, kq = 2.0f * (float)M_PI / k->counts_per_wheel_rev;
    c->now = in->now_ms;
    bool sbc_ok = c->sbc_seen && !elapsed(c->now, c->last_sbc, k->sbc_timeout_ms + 1);
    bool estop = !in->estop_ok || c->soft_estop;

    /* wheels, odometry, slip */
    if (!c->enc_init) { memcpy(c->enc_prev, in->enc, sizeof c->enc_prev); c->enc_init = true; }
    for (int i = 0; i < 4; i++) { c->w_wheel[i] = (float)(in->enc[i] - c->enc_prev[i]) * kq / dt; c->enc_prev[i] = in->enc[i]; }
    c->wl = 0.5f * (c->w_wheel[W_LF] + c->w_wheel[W_LR]);
    c->wr = 0.5f * (c->w_wheel[W_RF] + c->w_wheel[W_RR]);
    float v = k->wheel_r_m * (c->wl + c->wr) / 2.0f, w = k->wheel_r_m * (c->wr - c->wl) / k->track_m;
    c->x += v * dt * cosf(c->yaw); c->y += v * dt * sinf(c->yaw); c->yaw += w * dt;
    if (fabsf(c->w_wheel[W_LF] - c->w_wheel[W_LR]) < 1.5f) c->slip_l_since = c->now;
    if (fabsf(c->w_wheel[W_RF] - c->w_wheel[W_RR]) < 1.5f) c->slip_r_since = c->now;
    c->slip_l = elapsed(c->now, c->slip_l_since, 300); c->slip_r = elapsed(c->now, c->slip_r_since, 300);
    if (fabsf(v) > 0.01f || fabsf(w) > 0.02f) c->still_since = c->now;
    bool stationary = elapsed(c->now, c->still_since, 200);

    /* bays: docking FSM + charging */
    bool permit = true;
    for (int i = 0; i < 2; i++) {
        atlas_bay_t *b = &c->bay[i];
        const atlas_bay_in_t *bi = &in->bay[i];
        if (bi->seated) b->unseated_since = c->now;
        atlas_dock_in_t u = {
            .cmd_dock = b->cmd_dock, .cmd_undock = b->cmd_undock, .seated = bi->seated, .latch_locked = bi->latched,
            .id_ok = id_ok(c, b, bi->id_mv), .comm_ok = b->agent_comm_ok, .carrier_stationary = stationary,
            .agent_clear = elapsed(c->now, b->unseated_since, 500), .estop = estop, .reset = b->cmd_reset};
        atlas_dock_state_t before = b->fsm.out.state;
        atlas_dock_step(&b->fsm, &u);
        /* commands are consumed once acted on (or immediately if they cannot apply in this state) */
        bool waiting = (before == DOCK_INIT || before == DOCK_IDLE) && b->fsm.out.state == DOCK_IDLE;
        if (!waiting || elapsed(c->now, b->cmd_t, 2000)) b->cmd_dock = false;   /* may wait <= 2 s for 'stationary' */
        b->cmd_undock = false; b->cmd_reset = false;
        atlas_dock_state_t s = b->fsm.out.state;
        bool bay_ok = (s == DOCK_DOCKED && b->fsm.out.move_permit) || (s == DOCK_IDLE && !bi->seated);
        permit = permit && bay_ok;
        o->latch_open[i] = b->fsm.out.latch_open_cmd;
        o->agent_drive[i] = b->fsm.out.drive_cmd;
        charge(c, i, bi, &o->bay_en[i], &o->iset_ma[i]);
    }
    o->move_permit = permit && !estop && sbc_ok;

    /* skid-steer drive with dead-man and permit */
    bool cmd_live = (int32_t)(c->now - c->cmd_until) < 0 && sbc_ok;
    float vc = 0, wc = 0;
    if (cmd_live && o->move_permit) { vc = c->v_cmd; wc = c->w_cmd; }
    bool moving = vc != 0.0f || wc != 0.0f;
    if (moving) c->brake_until = c->now + BRAKE_MS;
    bool braking = !moving && (int32_t)(c->now - c->brake_until) < 0 && o->move_permit;
    if (moving || braking) {
        float wl_ref = (vc - wc * k->track_m / 2.0f) / k->wheel_r_m, wr_ref = (vc + wc * k->track_m / 2.0f) / k->wheel_r_m;
        o->motor_l_v = atlas_pi_step(&c->pi_l, wl_ref, c->wl);
        o->motor_r_v = atlas_pi_step(&c->pi_r, wr_ref, c->wr);
        o->mot_en = true;
    } else {
        o->motor_l_v = o->motor_r_v = 0; o->mot_en = false;
        atlas_pi_reset(&c->pi_l); atlas_pi_reset(&c->pi_r);
    }

    o->wdi = !o->wdi;
    bool alarm = c->bay[0].fsm.out.state == DOCK_FAULT || c->bay[1].fsm.out.state == DOCK_FAULT ||
                 c->bay[0].chg == CHG_FAULT || c->bay[1].chg == CHG_FAULT;
    o->buzzer = alarm && ((c->now / 250) & 1);
    o->led = sbc_ok ? true : ((c->now / 500) & 1);

    if (elapsed(c->now, c->last_status, STATUS_MS) && c->send) {
        c->last_status = c->now;
        uint16_t imax = 0; for (int i = 0; i < 4; i++) if (in->imot_ma[i] > imax) imax = in->imot_ma[i];
        atlas_carrier_status_t st = {c->now, in->vbat_mv,
            (uint8_t)((o->move_permit ? 1 : 0) | (estop ? 2 : 0) | (o->mot_en ? 4 : 0) | (!cmd_live ? 8 : 0) | (c->slip_l ? 16 : 0) | (c->slip_r ? 32 : 0)),
            (int16_t)lroundf(c->wl * k->wheel_r_m * 1000), (int16_t)lroundf(c->wr * k->wheel_r_m * 1000), imax,
            (int32_t)lroundf(c->x * 1000), (int32_t)lroundf(c->y * 1000), (int16_t)lroundf(c->yaw * 1000)};
        c->send(MSG_CARRIER_STATUS, &st, sizeof st, c->send_ctx);
        for (int i = 0; i < 2; i++) {
            atlas_bay_t *b = &c->bay[i]; const atlas_bay_in_t *bi = &in->bay[i];
            atlas_bay_status_t bs = {(uint8_t)i, (int8_t)b->fsm.out.state,
                (uint8_t)((bi->seated ? 1 : 0) | (bi->latched ? 2 : 0) | (id_ok(c, b, bi->id_mv) ? 4 : 0) |
                          (b->chg == CHG_CC || b->chg == CHG_CHECK ? 8 : 0) | (b->chg == CHG_FAULT ? 16 : 0) |
                          (b->fsm.out.move_permit ? 32 : 0) | (bi->kalert ? 64 : 0)),
                b->expected_agent, b->fsm.out.drive_cmd, b->fsm.retries, (uint8_t)b->chg,
                bi->vpad_mv, bi->i_ma, b->rc_v_mohm, b->rc_g_mohm};
            c->send(MSG_BAY_STATUS, &bs, sizeof bs, c->send_ctx);
        }
    }
    return o;
}
