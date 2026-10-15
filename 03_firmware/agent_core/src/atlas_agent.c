/* ATLAS-S agent application core. See atlas_agent.h for the contract. */
#include "atlas_agent.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define HB_ESTOP        0x01u   /* carrier heartbeat state bits */
#define HB_CLEAR_FAULTS 0x80u
#define KIND_CANCEL     0xFFu
#define TEL_PERIOD_MS   500u

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float wrap_pi(float a) { while (a > (float)M_PI) a -= 2.0f * (float)M_PI; while (a < -(float)M_PI) a += 2.0f * (float)M_PI; return a; }
static bool elapsed(uint32_t now, uint32_t since, uint32_t ms) { return (uint32_t)(now - since) >= ms; }

const char *atlas_agent_mode_name(agent_mode_t m)
{
    static const char *n[] = {"IDLE", "GOTO", "WORK", "DRIVE", "SAFE_STOP", "FAULT"};
    return (unsigned)m < 6 ? n[m] : "?";
}

void atlas_agent_default_cfg(atlas_agent_cfg_t *c, bool has_pi)
{
    memset(c, 0, sizeof *c);
    c->wheel_r_m = 0.0325f; c->track_m = 0.100f;               /* atlas_params.yaml agent.*          */
    c->counts_per_wheel_rev = 12.0f * 4.0f * 150.0f;           /* 12 CPR x4 quadrature x 1:150 gear */
    c->vmax_mps = 0.20f; c->wmax_radps = 2.0f;
    c->kp = 0.476f; c->ki = 15.993f;                           /* build_speed_loop.m (pidtune)       */
    c->arrive_tol_m = 0.03f; c->heading_tol_rad = 0.35f; c->k_heading = 2.5f; c->k_dist = 1.0f;
    c->hb_timeout_ms = 2000; c->stall_ma = 1100; c->stall_ms = 300;
    c->vbat_low_mv = 6400; c->vbat_cutoff_mv = 6000;
    c->dock_present_mv = 7000; c->chg_stop_mv = 8100; c->chg_restart_mv = 7900;
    c->chg_tmin_dc = 0; c->chg_tmax_dc = 450;
    c->measure_ms = 3000; c->inspect_tag_timeout_ms = 10000; c->pi_boot_ms = 20000; c->pi_shutdown_grace_ms = 15000;
    c->sens_gain = 1.0f; c->sens_offset = 0.0f; c->cal_ref_value = 1.0f;
    c->has_pi = has_pi;
}

#define BRAKE_MS 300u
static bool is_moving(agent_mode_t m) { return m == MODE_GOTO || m == MODE_DRIVE; }
static void set_mode(atlas_agent_t *a, agent_mode_t m)
{
    if (a->mode == m) return;
    /* leaving a driving mode: hold zero speed under closed-loop control for BRAKE_MS before the motor
       rail is switched off (except on FAULT, where power is cut at once) */
    if (is_moving(a->mode) && !is_moving(m) && m != MODE_FAULT) a->brake_until = a->now + BRAKE_MS;
    if (m == MODE_FAULT) a->brake_until = a->now;
    bool keep_pi = is_moving(a->mode) && is_moving(m);
    a->mode = m; a->t_mode = a->now;
    if (!keep_pi && !is_moving(m)) return;            /* braking keeps integrator state */
    atlas_pi_reset(&a->pi_l); atlas_pi_reset(&a->pi_r);
}

static void send_result(atlas_agent_t *a, uint8_t status, float value, float unc)
{
    atlas_task_result_t r = {a->task.task_id, status, value, unc, a->cal_epoch};
    a->last_result_id = a->task.task_id; a->last_result_status = status;
    a->task_active = false; a->pi_wanted = false;
    if (a->send) a->send(LINK_CARRIER, MSG_TASK_RESULT, &r, sizeof r, a->send_ctx);
}

void atlas_agent_init(atlas_agent_t *a, const atlas_agent_cfg_t *c, atlas_agent_send_fn send, void *ctx)
{
    memset(a, 0, sizeof *a);
    a->cfg = *c; a->send = send; a->send_ctx = ctx;
    float vlim = 7.4f;                                         /* nominal pack; port rescales by vbat */
    atlas_pi_init(&a->pi_l, c->kp, c->ki, AGENT_DT_MS / 1000.0f, -vlim, vlim);
    atlas_pi_init(&a->pi_r, c->kp, c->ki, AGENT_DT_MS / 1000.0f, -vlim, vlim);
    a->mode = MODE_IDLE; a->last_result_id = 0xFFFF;
}

/* ---------------------------------------------------------------- incoming frames */
void atlas_agent_on_frame(atlas_agent_t *a, int link, uint8_t type, const uint8_t *p, int n)
{
    a->frames_rx++;
    if (link == LINK_PI) {
        if (type == MSG_VISION_TAG && n == (int)sizeof(atlas_vision_tag_t) && a->task_active &&
            a->task.kind == KIND_INSPECT && a->mode == MODE_WORK) {
            memcpy(&a->tag, p, sizeof a->tag); a->tag_seen = true;
        }
        return;
    }
    switch (type) {
    case MSG_HEARTBEAT:
        if (n < (int)sizeof(atlas_heartbeat_t)) break;
        a->last_hb = a->now; a->hb_seen = true;
        {
            atlas_heartbeat_t hb; memcpy(&hb, p, sizeof hb);
            if (hb.state & HB_ESTOP) {
                if (a->mode == MODE_GOTO || a->mode == MODE_DRIVE) { a->mode_before_stop = a->mode; set_mode(a, MODE_SAFE_STOP); }
                a->hb_seen = false;                           /* stay stopped until a normal heartbeat */
            }
            if ((hb.state & HB_CLEAR_FAULTS) && a->mode == MODE_FAULT) { a->faults = 0; set_mode(a, MODE_IDLE); }
        }
        break;
    case MSG_TASK_CMD: {
        if (n != (int)sizeof(atlas_task_cmd_t)) { a->frames_bad++; break; }
        atlas_task_cmd_t t; memcpy(&t, p, sizeof t);
        if (t.kind == KIND_CANCEL) {
            if (a->task_active) { send_result(a, RES_ABORTED, 0, 0); set_mode(a, MODE_IDLE); }
            break;
        }
        if (a->task_active && t.task_id == a->task.task_id) break;           /* retransmission */
        if (!a->task_active && t.task_id == a->last_result_id) {            /* result lost: resend */
            atlas_task_result_t r = {t.task_id, a->last_result_status, 0, 0, a->cal_epoch};
            if (a->send) a->send(LINK_CARRIER, MSG_TASK_RESULT, &r, sizeof r, a->send_ctx);
            break;
        }
        if (a->task_active) send_result(a, RES_ABORTED, 0, 0);              /* pre-empted */
        a->task = t;
        if (a->mode == MODE_FAULT || t.kind > KIND_RETURN || (t.kind == KIND_INSPECT && !a->cfg.has_pi)) {
            send_result(a, RES_FAILED, 0, 0); break;
        }
        a->task_active = true; a->task_t0 = a->now; a->sens_acc = 0; a->sens_n = 0; a->tag_seen = false;
        if (t.kind == KIND_INSPECT) a->pi_wanted = true;                    /* boot the Pi while driving */
        set_mode(a, MODE_GOTO);
        break;
    }
    case MSG_SET_POSE: {                              /* carrier tells the agent where it was deployed */
        if (n != 10) { a->frames_bad++; break; }
        int32_t x, y; int16_t yaw; memcpy(&x, p, 4); memcpy(&y, p + 4, 4); memcpy(&yaw, p + 8, 2);
        a->x = x / 1000.0f; a->y = y / 1000.0f; a->yaw = yaw / 1000.0f;
        break;
    }
    case MSG_DRIVE_CMD: {
        if (n != 6 || a->task_active || a->mode == MODE_FAULT) break;
        int16_t v, w; uint16_t ttl; memcpy(&v, p, 2); memcpy(&w, p + 2, 2); memcpy(&ttl, p + 4, 2);
        a->drv_v = clampf(v / 1000.0f, -a->cfg.vmax_mps, a->cfg.vmax_mps);
        a->drv_w = clampf(w / 1000.0f, -a->cfg.wmax_radps, a->cfg.wmax_radps);
        a->drv_until = a->now + (ttl > 1000 ? 1000 : ttl);                  /* bounded dead-man time */
        if (a->mode != MODE_SAFE_STOP) set_mode(a, MODE_DRIVE);
        break;
    }
    default: break;
    }
}

/* ---------------------------------------------------------------- periodic */
static void odometry(atlas_agent_t *a, const atlas_agent_in_t *in)
{
    const float k = 2.0f * (float)M_PI / a->cfg.counts_per_wheel_rev, dt = AGENT_DT_MS / 1000.0f;
    if (!a->enc_init) { a->enc_l_prev = in->enc_l; a->enc_r_prev = in->enc_r; a->enc_init = true; }
    float dl = (float)(in->enc_l - a->enc_l_prev) * k, dr = (float)(in->enc_r - a->enc_r_prev) * k;
    a->enc_l_prev = in->enc_l; a->enc_r_prev = in->enc_r;
    a->wl_meas = dl / dt; a->wr_meas = dr / dt;
    float ds = a->cfg.wheel_r_m * (dl + dr) / 2.0f, dth = a->cfg.wheel_r_m * (dr - dl) / a->cfg.track_m;
    a->x += ds * cosf(a->yaw + dth / 2.0f); a->y += ds * sinf(a->yaw + dth / 2.0f);
    a->yaw = wrap_pi(a->yaw + dth);
}

static void charging(atlas_agent_t *a, const atlas_agent_in_t *in)
{
    const atlas_agent_cfg_t *c = &a->cfg;
    a->docked = in->dock_mv >= c->dock_present_mv;
    bool temp_ok = in->batt_temp_dc >= c->chg_tmin_dc && in->batt_temp_dc <= c->chg_tmax_dc;
    if (!a->docked || !temp_ok || in->vbat_mv >= c->chg_stop_mv) a->charging = false;
    else if (in->vbat_mv <= c->chg_restart_mv) a->charging = true;          /* hysteresis */
    if (!temp_ok) a->faults |= FAULT_OVERTEMP; else a->faults &= (uint8_t)~FAULT_OVERTEMP;
}

static void work(atlas_agent_t *a, const atlas_agent_in_t *in)
{
    const atlas_agent_cfg_t *c = &a->cfg;
    if (a->task.kind == KIND_INSPECT) {
        uint32_t deadline = c->pi_boot_ms + c->inspect_tag_timeout_ms;
        if (a->tag_seen) send_result(a, RES_SUCCESS, (float)a->tag.tag_id, 1.0f - a->tag.quality / 255.0f);
        else if (elapsed(a->now, a->pi_on_since, deadline) && a->out.pi_pwr_en) send_result(a, RES_INCONCLUSIVE, 0, 1);
        if (!a->task_active) set_mode(a, MODE_IDLE);
        return;
    }
    a->sens_acc += in->sens_mv; a->sens_n++;
    if (!elapsed(a->now, a->t_mode, c->measure_ms)) return;
    float mean_mv = (float)(a->sens_acc / a->sens_n);
    if (mean_mv < 5.0f || mean_mv > 3250.0f) { send_result(a, RES_MEAS_INVALID, 0, 0); set_mode(a, MODE_IDLE); return; }
    float value = c->sens_gain * mean_mv / 1000.0f + a->cfg.sens_offset;
    if (a->task.kind == KIND_RECAL) {                     /* zero against the reference sample */
        a->cfg.sens_offset += c->cal_ref_value - value; value = c->cal_ref_value;
        a->cal_epoch++;
    }
    send_result(a, RES_SUCCESS, value, 0.01f * fabsf(value) + 0.002f);
    set_mode(a, MODE_IDLE);
}

static void navigate(atlas_agent_t *a, float *v, float *w)
{
    const atlas_agent_cfg_t *c = &a->cfg;
    float dx = a->task.x_mm / 1000.0f - a->x, dy = a->task.y_mm / 1000.0f - a->y, d = sqrtf(dx * dx + dy * dy);
    if (d < c->arrive_tol_m) {
        *v = *w = 0;
        if (a->task.kind == KIND_RETURN) { send_result(a, RES_SUCCESS, 0, 0); set_mode(a, MODE_IDLE); }
        else set_mode(a, MODE_WORK);
        return;
    }
    float err = wrap_pi(atan2f(dy, dx) - a->yaw);
    *w = clampf(c->k_heading * err, -c->wmax_radps, c->wmax_radps);
    *v = fabsf(err) > c->heading_tol_rad ? 0.0f : clampf(c->k_dist * d, 0.03f, c->vmax_mps) * cosf(err);
}

const atlas_agent_out_t *atlas_agent_tick(atlas_agent_t *a, const atlas_agent_in_t *in)
{
    const atlas_agent_cfg_t *c = &a->cfg;
    a->now = in->now_ms; a->tick_count++;
    bool comm_ok = a->hb_seen && !elapsed(a->now, a->last_hb, c->hb_timeout_ms + 1);

    odometry(a, in);
    charging(a, in);

    /* battery protection */
    if (in->vbat_mv < c->vbat_cutoff_mv && in->vbat_mv > 1000) a->faults |= FAULT_UNDERVOLT;

    /* comm loss -> safe stop (resume when the carrier is back) */
    if (!comm_ok && (a->mode == MODE_GOTO || a->mode == MODE_DRIVE)) { a->mode_before_stop = a->mode; set_mode(a, MODE_SAFE_STOP); }
    if (comm_ok && a->mode == MODE_SAFE_STOP) set_mode(a, a->mode_before_stop == MODE_GOTO && a->task_active ? MODE_GOTO : MODE_IDLE);

    if ((a->faults & (FAULT_STALL_L | FAULT_STALL_R | FAULT_UNDERVOLT)) && a->mode != MODE_FAULT) {
        if (a->task_active) send_result(a, RES_FAILED, 0, 0);
        set_mode(a, MODE_FAULT);
    }

    float v = 0, w = 0;
    switch (a->mode) {
    case MODE_GOTO:  navigate(a, &v, &w); break;
    case MODE_WORK:  work(a, in); break;
    case MODE_DRIVE:
        if ((int32_t)(a->now - a->drv_until) >= 0) set_mode(a, MODE_IDLE);
        else { v = a->drv_v; w = a->drv_w; }
        break;
    default: break;
    }

    bool moving = is_moving(a->mode);
    bool braking = !moving && (int32_t)(a->now - a->brake_until) < 0;
    a->wl_ref = (v - w * c->track_m / 2.0f) / c->wheel_r_m;
    a->wr_ref = (v + w * c->track_m / 2.0f) / c->wheel_r_m;
    if (moving || braking) {
        a->out.motor_l_v = atlas_pi_step(&a->pi_l, a->wl_ref, a->wl_meas);
        a->out.motor_r_v = atlas_pi_step(&a->pi_r, a->wr_ref, a->wr_meas);
    } else {
        a->out.motor_l_v = a->out.motor_r_v = 0; atlas_pi_reset(&a->pi_l); atlas_pi_reset(&a->pi_r);
    }
    a->out.mot_en = moving || braking;

    /* stall: over-current, or full effort with the wheel not turning */
    if (moving) {
        bool sl = in->imot_l_ma > c->stall_ma || (fabsf(a->out.motor_l_v) > 5.0f && fabsf(a->wl_meas) < 0.3f);
        bool sr = in->imot_r_ma > c->stall_ma || (fabsf(a->out.motor_r_v) > 5.0f && fabsf(a->wr_meas) < 0.3f);
        if (!sl) a->stall_l_since = a->now; else if (elapsed(a->now, a->stall_l_since, c->stall_ms)) a->faults |= FAULT_STALL_L;
        if (!sr) a->stall_r_since = a->now; else if (elapsed(a->now, a->stall_r_since, c->stall_ms)) a->faults |= FAULT_STALL_R;
    } else { a->stall_l_since = a->stall_r_since = a->now; }

    /* Pi power sequencing */
    if (c->has_pi) {
        if (a->pi_wanted) {
            if (!a->out.pi_pwr_en) a->pi_on_since = a->now;
            a->out.pi_pwr_en = true; a->pi_shutdown_sent = false;
        } else if (a->out.pi_pwr_en) {
            if (!a->pi_shutdown_sent) {
                if (a->send) a->send(LINK_PI, MSG_SHUTDOWN, NULL, 0, a->send_ctx);
                a->pi_shutdown_sent = true; a->pi_off_at = a->now + c->pi_shutdown_grace_ms;
            } else if ((int32_t)(a->now - a->pi_off_at) >= 0) a->out.pi_pwr_en = false;
        }
    }

    a->out.chg_en = a->charging;                      /* allowed in FAULT too: a flat pack must charge */
    a->out.wdi = !a->out.wdi;                         /* hardware watchdog: kicked every healthy tick */
    a->out.led = a->mode == MODE_FAULT ? ((a->now / 100) & 1) : (moving || a->mode == MODE_WORK);

    if (elapsed(a->now, a->last_tel, TEL_PERIOD_MS)) {
        a->last_tel = a->now;
        atlas_telemetry_t t;
        memset(&t, 0, sizeof t);
        t.t_ms = a->now; t.vbat_mv = in->vbat_mv;
        float soc = (in->vbat_mv - 6000.0f) / (8400.0f - 6000.0f) * 100.0f;   /* coarse OCV estimate */
        t.soc_pct = (uint8_t)clampf(soc, 0, 100);
        t.health = a->mode == MODE_FAULT ? HEALTH_FAULT
                 : (a->task_active || a->mode == MODE_SAFE_STOP || a->mode == MODE_DRIVE) ? HEALTH_BUSY
                 : in->vbat_mv < c->vbat_low_mv ? HEALTH_DEGRADED : HEALTH_READY;
        t.wl_mrad_s = (int16_t)(a->wl_meas * 1000); t.wr_mrad_s = (int16_t)(a->wr_meas * 1000);
        t.il_ma = in->imot_l_ma; t.ir_ma = in->imot_r_ma;
        t.x_mm = (int32_t)lroundf(a->x * 1000); t.y_mm = (int32_t)lroundf(a->y * 1000);
        t.yaw_mrad = (int16_t)lroundf(a->yaw * 1000);
        t.flags = (uint8_t)((a->out.chg_en ? 1 : 0) | (a->docked ? 2 : 0) | (a->out.mot_en ? 4 : 0) | (a->out.pi_pwr_en ? 8 : 0));
        if (a->send) a->send(LINK_CARRIER, MSG_TELEMETRY, &t, sizeof t, a->send_ctx);
    }
    return &a->out;
}
