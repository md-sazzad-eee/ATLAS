/* Host tests for the agent core against a physical plant:
 * two DC gearmotors (same assumed parameters as build_speed_loop.m), differential-drive kinematics,
 * quadrature encoders, a measurement sensor and a fake Pi. Frames go through the real COBS/CRC codec. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "atlas_agent.h"

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---------------- plant ---------------- */
typedef struct {
    double wm[2], th[2], x, y, yaw, t;     /* motor speed, wheel angle, pose */
    double load[2];                         /* extra wheel load torque N.m   */
    double i[2];
    double vbat; double sens_mv; int dock_mv; int temp_dc;
} plant_t;
static const double R = 6.0, Kt = 4.5e-3, Ke = 4.5e-3, Jm = 1.0e-7, B = 1.0e-7, N = 150, ETA = 0.7;
static const double r = 0.0325, TRACK = 0.100, MASS = 1.2;

static void plant_step(plant_t *p, double vl, double vr, bool en, double dt)
{
    const double J = Jm + (MASS / 2 * r * r) / (N * N * ETA), Troll = MASS * 9.81 * 0.03 / 2 * r;
    double V[2] = {en ? vl : 0, en ? vr : 0};
    for (int k = 0; k < 2; k++) {
        double vb = V[k] > p->vbat ? p->vbat : (V[k] < -p->vbat ? -p->vbat : V[k]);
        double i = en ? (vb - Ke * p->wm[k]) / R : 0;          /* coast when disabled */
        double wwheel = p->wm[k] / N;
        double Tl = (Troll + p->load[k]) * tanh(wwheel / 0.05);
        if (p->load[k] > 1.0) { p->wm[k] = 0; p->i[k] = en ? vb / R : 0; continue; }   /* wheel jammed */
        p->wm[k] += dt * (Kt * i - B * p->wm[k] - Tl / (N * ETA)) / J;
        p->i[k] = i;
        p->th[k] += dt * p->wm[k] / N;
    }
    double wl = p->wm[0] / N, wr = p->wm[1] / N;
    double v = r * (wl + wr) / 2, w = r * (wr - wl) / TRACK;
    p->x += dt * v * cos(p->yaw); p->y += dt * v * sin(p->yaw); p->yaw += dt * w; p->t += dt;
}

/* ---------------- harness ---------------- */
typedef struct { uint8_t type; uint8_t buf[ATLAS_MAX_PAYLOAD]; int n; int link; } msg_t;
static msg_t outbox[4096]; static int nout;
static atlas_rx_t rx_carrier, rx_pi;
static int cur_link;
static void on_rx(uint8_t type, uint8_t seq, const uint8_t *pl, int n, void *ctx)
{
    (void)seq; (void)ctx;
    if (nout < 4096) { outbox[nout].type = type; memcpy(outbox[nout].buf, pl, (size_t)n); outbox[nout].n = n; outbox[nout].link = cur_link; nout++; }
}
static void agent_send(int link, uint8_t type, const void *pl, size_t n, void *ctx)
{
    (void)ctx; static uint8_t seq; uint8_t f[ATLAS_MAX_FRAME + 4];
    size_t len = atlas_frame_build(type, seq++, pl, n, f);   /* through the real codec */
    cur_link = link; atlas_rx_feed(link == LINK_PI ? &rx_pi : &rx_carrier, f, len);
}
static void to_agent(atlas_agent_t *a, int link, uint8_t type, const void *pl, size_t n)
{
    uint8_t f[ATLAS_MAX_FRAME + 4], t, s, out[ATLAS_MAX_PAYLOAD];
    size_t len = atlas_frame_build(type, 0, pl, n, f);
    int m = atlas_frame_parse(f, len - 1, &t, &s, out);
    atlas_agent_on_frame(a, link, t, out, m);
}
static msg_t *last_of(uint8_t type) { for (int i = nout - 1; i >= 0; i--) if (outbox[i].type == type) return &outbox[i]; return NULL; }

typedef struct { atlas_agent_t a; plant_t p; uint32_t ms; bool hb; double max_v; } sim_t;

static void sim_init(sim_t *s, bool has_pi)
{
    memset(s, 0, sizeof *s); nout = 0;
    atlas_rx_init(&rx_carrier, on_rx, NULL); atlas_rx_init(&rx_pi, on_rx, NULL);
    atlas_agent_cfg_t c; atlas_agent_default_cfg(&c, has_pi);
    c.sens_gain = 1.0f; c.sens_offset = 0.0f; c.cal_ref_value = 1.0f;
    atlas_agent_init(&s->a, &c, agent_send, NULL);
    s->p.vbat = 7.6; s->p.sens_mv = 930; s->p.temp_dc = 250; s->hb = true;
}

static void sim_run(sim_t *s, double seconds, void (*hook)(sim_t *))
{
    int steps = (int)(seconds * 1000 / AGENT_DT_MS);
    for (int k = 0; k < steps; k++) {
        if (s->hb && s->ms % 100 == 0) { atlas_heartbeat_t hb = {s->ms, 0}; to_agent(&s->a, LINK_CARRIER, MSG_HEARTBEAT, &hb, sizeof hb); }
        if (hook) hook(s);
        const double cpr = 12 * 4 * 150;
        atlas_agent_in_t in = {
            .now_ms = s->ms,
            .enc_l = (int32_t)floor(s->p.th[0] / (2 * M_PI) * cpr), .enc_r = (int32_t)floor(s->p.th[1] / (2 * M_PI) * cpr),
            .vbat_mv = (uint16_t)(s->p.vbat * 1000), .imot_l_ma = (uint16_t)(fabs(s->p.i[0]) * 1000),
            .imot_r_ma = (uint16_t)(fabs(s->p.i[1]) * 1000), .dock_mv = (uint16_t)s->p.dock_mv,
            .sens_mv = (uint16_t)s->p.sens_mv, .batt_temp_dc = (int16_t)s->p.temp_dc};
        const atlas_agent_out_t *o = atlas_agent_tick(&s->a, &in);
        for (int j = 0; j < 10; j++) plant_step(&s->p, o->motor_l_v, o->motor_r_v, o->mot_en, 0.001);
        double v = fabs(r * (s->p.wm[0] + s->p.wm[1]) / (2 * N)); if (v > s->max_v) s->max_v = v;
        s->ms += AGENT_DT_MS;
    }
}

static void send_task(sim_t *s, uint16_t id, uint8_t kind, double x, double y)
{
    atlas_task_cmd_t t = {id, kind, (int32_t)lround(x * 1000), (int32_t)lround(y * 1000), 120};
    to_agent(&s->a, LINK_CARRIER, MSG_TASK_CMD, &t, sizeof t);
}
static bool result(uint16_t id, atlas_task_result_t *r)
{
    for (int i = nout - 1; i >= 0; i--)
        if (outbox[i].type == MSG_TASK_RESULT) { memcpy(r, outbox[i].buf, sizeof *r); if (r->task_id == id) return true; }
    return false;
}

/* ---------------- tests ---------------- */
static void test_goto_and_measure(void)
{
    sim_t s; sim_init(&s, false);
    send_task(&s, 1, KIND_MEASURE, 1.0, 0.6);
    atlas_task_result_t res; double t_arr = -1;
    for (int k = 0; k < 400 && !result(1, &res); k++) { sim_run(&s, 0.1, NULL); if (t_arr < 0 && s.a.mode == MODE_WORK) t_arr = s.ms / 1000.0; }
    double err = hypot(s.p.x - 1.0, s.p.y - 0.6), odo = hypot(s.a.x - s.p.x, s.a.y - s.p.y);
    printf("  goto (1.0,0.6): arrived %.1f s, true-pose error %.1f mm, odometry error %.1f mm, vmax %.3f m/s\n",
           t_arr, err * 1000, odo * 1000, s.max_v);
    CHECK(result(1, &res), "no result");
    CHECK(res.status == RES_SUCCESS && fabsf(res.value - 0.930f) < 0.002f, "status %d value %f", res.status, res.value);
    CHECK(err < 0.04, "position error %.3f", err);
    CHECK(odo < 0.01, "odometry drift %.3f", odo);
    CHECK(s.max_v <= 0.205, "speed limit exceeded %.3f", s.max_v);
    CHECK(t_arr > 0 && t_arr < 12, "too slow %.1f", t_arr);
    CHECK(!s.a.out.mot_en && s.a.mode == MODE_IDLE, "motors still enabled after task");
    msg_t *tel = last_of(MSG_TELEMETRY); CHECK(tel && tel->n == (int)sizeof(atlas_telemetry_t), "telemetry");
}

static void test_recalibration(void)
{
    sim_t s; sim_init(&s, false);
    s.p.sens_mv = 1150;                            /* drifted sensor reads 1.15 on a 1.00 reference */
    send_task(&s, 2, KIND_CAL_CHECK, 0.2, 0); sim_run(&s, 10, NULL);
    atlas_task_result_t r; CHECK(result(2, &r) && fabsf(r.value - 1.15f) < 0.003f, "cal check %f", r.value);
    send_task(&s, 3, KIND_RECAL, 0.2, 0); sim_run(&s, 6, NULL);
    CHECK(result(3, &r) && r.cal_epoch == 1, "recal epoch %d", r.cal_epoch);
    send_task(&s, 4, KIND_CAL_CHECK, 0.2, 0); sim_run(&s, 6, NULL);
    CHECK(result(4, &r) && fabsf(r.value - 1.0f) < 0.003f && r.cal_epoch == 1, "post-recal value %f epoch %d", r.value, r.cal_epoch);
    printf("  recalibration: drifted 1.150 -> recal -> %.3f (epoch %d)\n", r.value, r.cal_epoch);
}

static void drop_hb(sim_t *s) { s->hb = !(s->ms >= 2000 && s->ms < 6000); }
static void test_comm_loss_safe_stop(void)
{
    sim_t s; sim_init(&s, false);
    send_task(&s, 5, KIND_MEASURE, 2.0, 0);
    double x_at_loss = 0, x_stop = 0; uint32_t t_stop = 0;
    for (int k = 0; k < 120; k++) {
        sim_run(&s, 0.1, drop_hb);
        if (s.ms == 2000) x_at_loss = s.p.x;
        if (!t_stop && s.a.mode == MODE_SAFE_STOP) { t_stop = s.ms; }
        if (s.ms == 5900) x_stop = s.p.x;
    }
    atlas_task_result_t r;
    printf("  comm loss at 2.0 s: SAFE_STOP at %.2f s (2 s timeout), %.0f mm travelled from loss to standstill\n",
           t_stop / 1000.0, (x_stop - x_at_loss) * 1000); (void)r;
    CHECK(t_stop >= 3900 && t_stop <= 4200, "safe stop at %u ms", t_stop);
    
    sim_init(&s, false); send_task(&s, 5, KIND_MEASURE, 2.0, 0);
    for (int k = 0; k < 55; k++) sim_run(&s, 0.1, drop_hb);
    CHECK(s.a.mode == MODE_SAFE_STOP && !s.a.out.mot_en && fabs(s.p.wm[0]) < 1, "not stopped during loss: mode %s", atlas_agent_mode_name(s.a.mode));
    for (int k = 0; k < 150 && !result(5, &r); k++) sim_run(&s, 0.1, drop_hb);
    CHECK(result(5, &r) && r.status == RES_SUCCESS, "did not resume after comm returned");
}

static void block_left(sim_t *s) { if (s->ms >= 1500) s->p.load[0] = 5.0; }
static void test_stall_fault(void)
{
    sim_t s; sim_init(&s, false);
    send_task(&s, 6, KIND_MEASURE, 2.0, 0);
    sim_run(&s, 4, block_left);
    atlas_task_result_t r;
    CHECK(s.a.mode == MODE_FAULT && (s.a.faults & FAULT_STALL_L), "no stall fault (mode %s)", atlas_agent_mode_name(s.a.mode));
    CHECK(result(6, &r) && r.status == RES_FAILED, "task not failed");
    CHECK(!s.a.out.mot_en, "motors not disabled");
    send_task(&s, 7, KIND_MEASURE, 0, 0); sim_run(&s, 0.1, block_left);
    CHECK(result(7, &r) && r.status == RES_FAILED, "task accepted in FAULT");
    s.p.load[0] = 0; atlas_heartbeat_t hb = {0, 0x80}; to_agent(&s.a, LINK_CARRIER, MSG_HEARTBEAT, &hb, sizeof hb);
    CHECK(s.a.mode == MODE_IDLE, "fault not cleared");
    printf("  stall: left wheel blocked at 1.5 s -> FAULT, task FAILED, motor power off, cleared by carrier\n");
}

static void test_charging(void)
{
    sim_t s; sim_init(&s, false);
    s.p.vbat = 7.5; sim_run(&s, 0.2, NULL); CHECK(!s.a.out.chg_en, "charging off-dock");
    s.p.dock_mv = 8300; sim_run(&s, 0.2, NULL); CHECK(s.a.out.chg_en && s.a.docked, "not charging on dock");
    s.p.temp_dc = 470; sim_run(&s, 0.2, NULL); CHECK(!s.a.out.chg_en, "charging when hot");
    s.p.temp_dc = 250; sim_run(&s, 0.2, NULL); CHECK(s.a.out.chg_en, "no restart after cool");
    s.p.vbat = 8.15; sim_run(&s, 0.2, NULL); CHECK(!s.a.out.chg_en, "no cut-off at full");
    s.p.vbat = 8.0; sim_run(&s, 0.2, NULL); CHECK(!s.a.out.chg_en, "hysteresis missing");
    s.p.vbat = 7.85; sim_run(&s, 0.2, NULL); CHECK(s.a.out.chg_en, "no restart below 7.9 V");
    s.p.temp_dc = -20; sim_run(&s, 0.2, NULL); CHECK(!s.a.out.chg_en, "charging below 0 C");
    msg_t *tel = last_of(MSG_TELEMETRY); atlas_telemetry_t t; memcpy(&t, tel->buf, sizeof t);
    CHECK(t.flags & 0x02, "docked flag");
    printf("  charging: dock presence, 0-45 C window, 8.10 V stop / 7.90 V restart all enforced\n");
}

static void test_drive_cmd_deadman(void)
{
    sim_t s; sim_init(&s, false);
    uint8_t d[6]; int16_t v = 50, w = 0; uint16_t ttl = 300; memcpy(d, &v, 2); memcpy(d + 2, &w, 2); memcpy(d + 4, &ttl, 2);
    to_agent(&s.a, LINK_CARRIER, MSG_DRIVE_CMD, d, 6);
    sim_run(&s, 0.25, NULL); CHECK(s.a.mode == MODE_DRIVE && s.a.out.mot_en, "not driving");
    sim_run(&s, 0.2, NULL); CHECK(s.a.mode == MODE_IDLE, "drive cmd not expired");
    sim_run(&s, 0.2, NULL); CHECK(!s.a.out.mot_en, "motor rail not cut after braking");
    double x1 = s.p.x; sim_run(&s, 0.5, NULL);
    printf("  DRIVE_CMD 50 mm/s, ttl 300 ms: moved %.1f mm then stopped (dead-man)\n", x1 * 1000);
    CHECK(x1 > 0.005 && x1 < 0.03 && fabs(s.p.x - x1) < 0.003, "x %.4f", x1);
}

static void pi_reply(sim_t *s)
{
    if (s->a.out.pi_pwr_en && s->ms == s->a.pi_on_since + 20000) {   /* Pi booted, sees tag 7 */
        atlas_vision_tag_t t = {7, 300, 0, 500, 0, 230};
        to_agent(&s->a, LINK_PI, MSG_VISION_TAG, &t, sizeof t);
    }
}
static void test_inspect_with_pi(void)
{
    sim_t s; sim_init(&s, true);
    send_task(&s, 8, KIND_INSPECT, 0.5, 0);
    sim_run(&s, 1, pi_reply); CHECK(s.a.out.pi_pwr_en, "Pi not powered for inspect");
    sim_run(&s, 25, pi_reply);
    atlas_task_result_t r; CHECK(result(8, &r) && r.status == RES_SUCCESS && (int)r.value == 7, "inspect result");
    sim_run(&s, 5, NULL); CHECK(last_of(MSG_SHUTDOWN) && s.a.out.pi_pwr_en, "no graceful shutdown msg / cut too early");
    sim_run(&s, 12, NULL); CHECK(!s.a.out.pi_pwr_en, "Pi not powered off after grace");
    sim_init(&s, true); send_task(&s, 9, KIND_INSPECT, 0.5, 0); sim_run(&s, 35, NULL);
    CHECK(result(9, &r) && r.status == RES_INCONCLUSIVE, "no-tag inspect should be INCONCLUSIVE");
    sim_init(&s, false); send_task(&s, 10, KIND_INSPECT, 0.5, 0);
    CHECK(result(10, &r) && r.status == RES_FAILED, "inspect without camera must FAIL");
    printf("  inspect: Pi powered on demand, tag 7 reported, SHUTDOWN then power cut after 15 s grace\n");
}

static void test_idempotent_and_cancel(void)
{
    sim_t s; sim_init(&s, false);
    send_task(&s, 11, KIND_RETURN, 0.3, 0); sim_run(&s, 5, NULL);
    int before = 0; for (int i = 0; i < nout; i++) before += outbox[i].type == MSG_TASK_RESULT;
    send_task(&s, 11, KIND_RETURN, 0.3, 0); sim_run(&s, 0.1, NULL);
    int after = 0; for (int i = 0; i < nout; i++) after += outbox[i].type == MSG_TASK_RESULT;
    CHECK(after == before + 1 && s.a.mode == MODE_IDLE, "retransmitted task re-executed instead of result resent");
    send_task(&s, 12, KIND_MEASURE, 3, 0); sim_run(&s, 1, NULL);
    atlas_task_cmd_t c = {12, 0xFF, 0, 0, 0}; to_agent(&s.a, LINK_CARRIER, MSG_TASK_CMD, &c, sizeof c);
    atlas_task_result_t r; CHECK(result(12, &r) && r.status == RES_ABORTED && s.a.mode == MODE_IDLE, "cancel");
    uint8_t pose[10]; int32_t x = 10000, y = 1000; int16_t yaw = 1571;
    memcpy(pose, &x, 4); memcpy(pose + 4, &y, 4); memcpy(pose + 8, &yaw, 2);
    to_agent(&s.a, LINK_CARRIER, MSG_SET_POSE, pose, 10);
    CHECK(fabsf(s.a.x - 10.0f) < 1e-3 && fabsf(s.a.yaw - 1.571f) < 1e-3, "set pose");
}

static void test_undervoltage(void)
{
    sim_t s; sim_init(&s, false);
    send_task(&s, 13, KIND_MEASURE, 2, 0); sim_run(&s, 1, NULL);
    s.p.vbat = 5.9; sim_run(&s, 0.1, NULL);
    atlas_task_result_t r;
    CHECK(s.a.mode == MODE_FAULT && !s.a.out.mot_en && result(13, &r) && r.status == RES_FAILED, "undervoltage");
}

int main(void)
{
    printf("[agent core vs DC-motor/diff-drive plant]\n");
    test_goto_and_measure();
    test_recalibration();
    test_comm_loss_safe_stop();
    test_stall_fault();
    test_charging();
    test_drive_cmd_deadman();
    test_inspect_with_pi();
    test_idempotent_and_cancel();
    test_undervoltage();
    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
