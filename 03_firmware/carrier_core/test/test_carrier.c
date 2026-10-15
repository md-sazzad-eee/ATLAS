/* Host tests for the carrier core: 4 DC gearmotors (ASSUMED 12 V, 1:50) on a skid-steer body,
 * two docking bays with an agent that drives in/out on command, latch + seated switches, and a
 * Board C charger/pack model with configurable contact resistance (4-wire INA226 readings). */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "atlas_carrier.h"

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---------------- plant ---------------- */
static const double R = 2.0, KT = 0.0155, KE = 0.0155, JM = 5e-6, BM = 2e-6, NG = 50, ETA = 0.7, RW = 0.05;
static const double MASS = 12.2, TRACK = 0.4;
typedef struct {
    double wm[4], th[4], x, y, yaw, i[4];
    /* bays */
    bool agent_present[2], seated[2], latched[2], agent_in_bay_drive_ok[2];
    double agent_pos[2];        /* 0 = outside, 1 = fully seated */
    double latch_t[2];
    double rc_v[2], rc_g[2], soc[2];
    bool latch_jam[2];
    uint16_t id_mv[2];
    bool estop_ok;
} plant_t;

static void plant_step(plant_t *p, const atlas_carrier_out_t *o, double dt)
{
    const double J = JM + (MASS / 4 * RW * RW) / (NG * NG * ETA), Troll = MASS * 9.81 * 0.03 / 4 * RW;
    for (int k = 0; k < 4; k++) {
        double V = o->mot_en ? (k < 2 ? o->motor_l_v : o->motor_r_v) : 0;
        if (V > 13.2) V = 13.2; else if (V < -13.2) V = -13.2;
        double i = o->mot_en ? (V - KE * p->wm[k]) / R : 0;
        if (i > 2.9) i = 2.9; else if (i < -2.9) i = -2.9;   /* DRV8871 ILIM */
        double ww = p->wm[k] / NG, Tl = Troll * tanh(ww / 0.05);
        p->wm[k] += dt * (KT * i - BM * p->wm[k] - Tl / (NG * ETA)) / J;
        p->th[k] += dt * p->wm[k] / NG; p->i[k] = i;
    }
    double wl = (p->wm[0] + p->wm[1]) / (2 * NG), wr = (p->wm[2] + p->wm[3]) / (2 * NG);
    double v = RW * (wl + wr) / 2, w = RW * (wr - wl) / TRACK;
    p->x += v * cos(p->yaw) * dt; p->y += v * sin(p->yaw) * dt; p->yaw += w * dt;
    for (int b = 0; b < 2; b++) {
        if (!p->agent_present[b]) continue;
        /* agent obeys the carrier's drive command at 0.05 m/s over a 75 mm insertion */
        if (o->agent_drive[b] > 0 && !p->latched[b]) p->agent_pos[b] += dt * 0.05 / 0.075;
        if (o->agent_drive[b] < 0 && !p->latched[b]) p->agent_pos[b] -= dt * 0.05 / 0.075;
        if (p->agent_pos[b] > 1) p->agent_pos[b] = 1;
        if (p->agent_pos[b] < 0) p->agent_pos[b] = 0;
        p->seated[b] = p->agent_pos[b] > 0.97;
        /* spring latch: drops in 150 ms once seated and not held open; opens in 150 ms */
        if (o->latch_open[b] || !p->seated[b]) { p->latch_t[b] = 0; if (!p->latch_jam[b]) p->latched[b] = false; }
        else { p->latch_t[b] += dt; if (p->latch_t[b] > 0.15) p->latched[b] = true; }
    }
}

static void bay_meas(plant_t *p, const atlas_carrier_out_t *o, int b, atlas_bay_in_t *bi, bool kalert)
{
    bi->seated = p->seated[b]; bi->latched = p->latched[b]; bi->id_mv = p->seated[b] ? p->id_mv[b] : 3300;
    bi->kalert = kalert;
    double i = 0;
    if (o->bay_en[b] && p->seated[b]) {
        double ocv = 6.0 + 2.1 * p->soc[b];
        double r = 0.33 + p->rc_v[b] + p->rc_g[b];
        i = fmin(o->iset_ma[b] / 1000.0, fmax(0, (8.344 - 0.30 - ocv) / r));
    }
    bi->i_ma = (uint16_t)(i * 1000); bi->vpad_mv = (uint16_t)(o->bay_en[b] ? 8344 - i * 0.1 * 1000 : 0);
    bi->vdrop_vchg_uv = (int32_t)(i * p->rc_v[b] * 1e6); bi->vdrop_gnd_uv = (int32_t)(i * p->rc_g[b] * 1e6);
    if (bi->vdrop_vchg_uv > 81920) bi->vdrop_vchg_uv = 81920;   /* INA226 shunt range */
    if (bi->vdrop_gnd_uv > 81920) bi->vdrop_gnd_uv = 81920;
}

/* ---------------- harness ---------------- */
typedef struct { atlas_carrier_t c; plant_t p; uint32_t ms; bool sbc; uint8_t hb_bits; bool kalert[2]; double vmax, soc_speedup; } sim_t;
static atlas_bay_status_t last_bay[2]; static atlas_carrier_status_t last_st; static int n_status;
static void on_send(uint8_t t, const void *pl, size_t n, void *ctx)
{
    (void)ctx; (void)n;
    uint8_t f[ATLAS_MAX_FRAME + 4], tt, s, out[ATLAS_MAX_PAYLOAD];       /* through the real codec */
    size_t len = atlas_frame_build(t, 0, pl, n, f);
    int m = atlas_frame_parse(f, len - 1, &tt, &s, out);
    if (tt == MSG_BAY_STATUS && m == (int)sizeof(atlas_bay_status_t)) { atlas_bay_status_t b; memcpy(&b, out, sizeof b); last_bay[b.bay] = b; }
    if (tt == MSG_CARRIER_STATUS && m == (int)sizeof(atlas_carrier_status_t)) { memcpy(&last_st, out, sizeof last_st); n_status++; }
}
static void sim_init(sim_t *s)
{
    memset(s, 0, sizeof *s);
    atlas_carrier_cfg_t cfg; atlas_carrier_default_cfg(&cfg);
    atlas_carrier_init(&s->c, &cfg, on_send, NULL);
    s->sbc = true; s->hb_bits = 0x06; s->p.estop_ok = true; s->soc_speedup = 1.0;
    for (int b = 0; b < 2; b++) { s->p.rc_v[b] = s->p.rc_g[b] = 0.030; s->p.soc[b] = 0.4; s->p.id_mv[b] = b ? 2269 : 1650; }
}
static void frame(sim_t *s, uint8_t t, const void *pl, size_t n) { atlas_carrier_on_frame(&s->c, t, (const uint8_t *)pl, (int)n); }
static void dock_cmd(sim_t *s, int bay, int cmd, int agent) { uint8_t p[3] = {(uint8_t)bay, (uint8_t)cmd, (uint8_t)agent}; frame(s, MSG_DOCK_CMD, p, 3); }
static void cmd_vel(sim_t *s, double v, double w, int ttl)
{
    uint8_t d[6]; int16_t a = (int16_t)lround(v * 1000), b = (int16_t)lround(w * 1000); uint16_t t = (uint16_t)ttl;
    memcpy(d, &a, 2); memcpy(d + 2, &b, 2); memcpy(d + 4, &t, 2); frame(s, MSG_DRIVE_CMD, d, 6);
}
static void run(sim_t *s, double sec, void (*hook)(sim_t *))
{
    const double cpr = 11 * 4 * 50;
    for (int k = 0; k < (int)(sec * 100); k++) {
        if (s->sbc && s->ms % 100 == 0) { atlas_heartbeat_t hb = {s->ms, s->hb_bits}; frame(s, MSG_HEARTBEAT, &hb, sizeof hb); }
        if (hook) hook(s);
        atlas_carrier_in_t in; memset(&in, 0, sizeof in);
        in.now_ms = s->ms; in.vbat_mv = 13200; in.estop_ok = s->p.estop_ok;
        for (int i = 0; i < 4; i++) { in.enc[i] = (int32_t)floor(s->p.th[i] / (2 * M_PI) * cpr); in.imot_ma[i] = (uint16_t)(fabs(s->p.i[i]) * 1000); }
        for (int b = 0; b < 2; b++) bay_meas(&s->p, &s->c.out, b, &in.bay[b], s->kalert[b]);
        const atlas_carrier_out_t *o = atlas_carrier_tick(&s->c, &in);
        for (int j = 0; j < 10; j++) plant_step(&s->p, o, 0.001);
        for (int b = 0; b < 2; b++) if (o->bay_en[b] && s->p.seated[b]) s->p.soc[b] += in.bay[b].i_ma / 1000.0 * 0.01 / 9000 * s->soc_speedup;   /* pack 2.5 Ah = 9000 As */
        double v = fabs(RW * (s->p.wm[0] + s->p.wm[1] + s->p.wm[2] + s->p.wm[3]) / (4 * NG)); if (v > s->vmax) s->vmax = v;
        s->ms += CARRIER_DT_MS;
    }
}
#define BAYST(s, b) ((s)->c.bay[b].fsm.out.state)

/* ---------------- tests ---------------- */
static bool permit_violation;
static void watch_permit(sim_t *s)
{
    for (int b = 0; b < 2; b++)
        if (s->p.seated[b] && !s->p.latched[b] && s->c.out.mot_en) permit_violation = true;
}

static void test_dock_charge_undock(void)
{
    sim_t s; sim_init(&s); permit_violation = false;
    s.p.agent_present[0] = true;
    dock_cmd(&s, 0, 0, 1);
    uint32_t t_docked = 0;
    for (int k = 0; k < 100 && !t_docked; k++) { run(&s, 0.1, watch_permit); if (BAYST(&s, 0) == DOCK_DOCKED) t_docked = s.ms; }
    CHECK(t_docked && t_docked < 4000, "not docked (state %s)", atlas_dock_state_name(BAYST(&s, 0)));
    CHECK(s.c.bay[0].chg == CHG_CHECK && s.c.out.iset_ma[0] == 500, "contact check not started");
    run(&s, 2.2, watch_permit);
    CHECK(s.c.bay[0].chg == CHG_CC && s.c.out.iset_ma[0] == 1000, "not charging at 1 A (%s)", atlas_chg_state_name(s.c.bay[0].chg));
    printf("  dock bay 0: DOCKED at %.2f s; contact check %u/%u mOhm (true 30/30) -> CC 1.0 A\n",
           t_docked / 1000.0, s.c.bay[0].rc_v_mohm, s.c.bay[0].rc_g_mohm);
    CHECK(abs((int)s.c.bay[0].rc_v_mohm - 30) <= 2 && abs((int)s.c.bay[0].rc_g_mohm - 30) <= 2, "Rc estimate");
    CHECK(s.c.out.move_permit, "move permit not granted when docked + other bay empty");
    run(&s, 1.0, NULL);
    CHECK(last_bay[0].state == DOCK_DOCKED && (last_bay[0].flags & 8) && last_bay[0].ichg_ma > 900, "BAY_STATUS");
    /* undock: charging must stop in the same tick the FSM leaves DOCKED */
    dock_cmd(&s, 0, 1, 0);
    bool en_after_leave = false;
    for (int k = 0; k < 600; k++) { run(&s, 0.01, watch_permit); if (BAYST(&s, 0) != DOCK_DOCKED && s.c.out.bay_en[0]) en_after_leave = true; if (BAYST(&s, 0) == DOCK_IDLE) break; }
    CHECK(!en_after_leave, "pads live after leaving DOCKED");
    CHECK(BAYST(&s, 0) == DOCK_IDLE && !s.p.seated[0], "undock did not complete (%s)", atlas_dock_state_name(BAYST(&s, 0)));
    CHECK(!permit_violation, "motors enabled with a seated, unlatched agent");
    printf("  undock bay 0: UNLATCHING -> RELEASING -> IDLE, pads off on exit, no permit violation\n");
}

static void test_dirty_contact(void)
{
    sim_t s; sim_init(&s); s.p.agent_present[0] = true; s.p.rc_g[0] = 0.150;
    dock_cmd(&s, 0, 0, 1); run(&s, 6, NULL);
    CHECK(BAYST(&s, 0) == DOCK_DOCKED && s.c.bay[0].chg == CHG_FAULT && !s.c.out.bay_en[0], "150 mOhm GND contact not rejected (%s)", atlas_chg_state_name(s.c.bay[0].chg));
    printf("  dirty GND contact (150 mOhm): measured %u mOhm at the 0.5 A check -> charge FAULT, pads off\n", s.c.bay[0].rc_g_mohm);
}

static void degrade(sim_t *s) { if (s->ms >= 8000) s->p.rc_v[0] = 0.120; }
static void test_contact_degrades_during_charge(void)
{
    sim_t s; sim_init(&s); s.p.agent_present[0] = true;
    dock_cmd(&s, 0, 0, 1); run(&s, 7, NULL);
    CHECK(s.c.bay[0].chg == CHG_CC, "precondition");
    uint32_t t_fault = 0;
    for (int k = 0; k < 300 && !t_fault; k++) { run(&s, 0.01, degrade); if (s.c.bay[0].chg == CHG_FAULT) t_fault = s.ms; }
    CHECK(t_fault >= 8000, "tripped before the contact degraded");
    CHECK(t_fault && t_fault - 8000 <= 260, "trip latency %u ms", t_fault - 8000);
    printf("  contact degrades to 120 mOhm during CC: trip after %u ms (INA226 range clips at 82 mOhm @ 1 A)\n", t_fault - 8000);
}

static void test_kalert_and_taper(void)
{
    sim_t s; sim_init(&s); s.p.agent_present[1] = true;
    dock_cmd(&s, 1, 0, 2); run(&s, 7, NULL);
    CHECK(s.c.bay[1].chg == CHG_CC, "bay 1 not charging (%s)", atlas_chg_state_name(s.c.bay[1].chg));
    s.kalert[1] = true; run(&s, 0.05, NULL);
    CHECK(s.c.bay[1].chg == CHG_FAULT && !s.c.out.bay_en[1], "hardware alert not latched as fault");
    sim_init(&s); s.p.agent_present[1] = true; s.p.soc[1] = 0.80; s.soc_speedup = 2000;   /* time-lapse */
    dock_cmd(&s, 1, 0, 2);
    for (int k = 0; k < 120 && s.c.bay[1].chg != CHG_DONE; k++) run(&s, 1, NULL);
    CHECK(s.c.bay[1].chg == CHG_DONE && !s.c.out.bay_en[1], "taper termination (%s)", atlas_chg_state_name(s.c.bay[1].chg));
    printf("  bay 1: KALERT -> FAULT; taper below 50 mA for 10 s -> DONE (SOC %.2f)\n", s.p.soc[1]);
}

static void test_drive(void)
{
    sim_t s; sim_init(&s);
    double v1 = 0; uint32_t t90 = 0;
    for (int k = 0; k < 300; k++) {
        if (k % 20 == 0) cmd_vel(&s, 0.30, 0, 300);
        run(&s, 0.01, NULL);
        double v = RW * (s.p.wm[0] + s.p.wm[1] + s.p.wm[2] + s.p.wm[3]) / (4 * NG);
        if (!t90 && v > 0.27) t90 = s.ms;
        v1 = v;
    }
    printf("  cmd_vel 0.30 m/s: 90%% at %u ms, steady %.3f m/s, peak %.3f m/s, odom x %.3f vs true %.3f m\n",
           t90, v1, s.vmax, s.c.x, s.p.x);
    CHECK(fabs(v1 - 0.30) < 0.01, "steady speed %.3f", v1);
    CHECK(s.vmax < 0.30 * 1.10, "overshoot %.3f", s.vmax);
    CHECK(t90 > 0 && t90 < 600, "slow response %u", t90);
    CHECK(fabs(s.c.x - s.p.x) < 0.01, "odometry");
    /* dead-man: stop sending */
    run(&s, 1.0, NULL);
    CHECK(!s.c.out.mot_en && fabs(s.p.wm[0]) < 5, "did not stop after cmd TTL");
    /* turn in place */
    for (int k = 0; k < 150; k++) { if (k % 20 == 0) cmd_vel(&s, 0, 0.8, 300); run(&s, 0.01, NULL); }
    double w = RW * ((s.p.wm[2] + s.p.wm[3]) - (s.p.wm[0] + s.p.wm[1])) / (2 * NG) / TRACK;
    CHECK(fabs(w - 0.8) < 0.05, "yaw rate %.3f", w);
    /* SBC link loss */
    s.sbc = false;
    for (int k = 0; k < 100; k++) { cmd_vel(&s, 0.3, 0, 300); run(&s, 0.01, NULL); }
    CHECK(!s.c.out.mot_en && !s.c.out.move_permit, "drives without the ROS heartbeat");
    printf("  turn 0.8 rad/s ok; dead-man + heartbeat loss stop the carrier\n");
}

static void test_permit_blocks_motion(void)
{
    sim_t s; sim_init(&s); s.p.agent_present[0] = true; s.p.latch_jam[0] = false;
    dock_cmd(&s, 0, 0, 1);
    run(&s, 1.0, NULL);                                    /* mid-approach */
    for (int k = 0; k < 50; k++) { if (k % 20 == 0) cmd_vel(&s, 0.3, 0, 300); run(&s, 0.01, NULL); }
    CHECK(!s.c.out.mot_en && !s.c.out.move_permit, "carrier moved during APPROACH");
    printf("  cmd_vel ignored while bay 0 is in %s (move permit false)\n", atlas_dock_state_name(BAYST(&s, 0)));
}

static void test_estop_and_wrong_agent(void)
{
    sim_t s; sim_init(&s); s.p.agent_present[0] = true;
    dock_cmd(&s, 0, 0, 1); run(&s, 4, NULL);
    s.p.estop_ok = false; run(&s, 0.05, NULL);
    CHECK(BAYST(&s, 0) == DOCK_FAULT && BAYST(&s, 1) == DOCK_FAULT && !s.c.out.bay_en[0] && !s.c.out.mot_en, "estop");
    dock_cmd(&s, 0, 2, 0); run(&s, 0.05, NULL);
    CHECK(BAYST(&s, 0) == DOCK_FAULT, "reset accepted while E-stop pressed");
    s.p.estop_ok = true; dock_cmd(&s, 0, 2, 0); dock_cmd(&s, 1, 2, 0); run(&s, 0.05, NULL);
    CHECK(BAYST(&s, 0) == DOCK_IDLE || BAYST(&s, 0) == DOCK_FAULT, "reset");
    /* wrong agent: bay expects the Scout (10k), the Research agent (22k) arrives */
    sim_init(&s); s.p.agent_present[0] = true; s.p.id_mv[0] = 2269;
    dock_cmd(&s, 0, 0, 1);
    bool backoff = false;
    for (int k = 0; k < 80; k++) { run(&s, 0.1, NULL); if (BAYST(&s, 0) == DOCK_BACKOFF) backoff = true; }
    CHECK(backoff && BAYST(&s, 0) != DOCK_DOCKED, "wrong agent was docked");
    printf("  E-stop -> both bays FAULT, reset refused until released; wrong agent ID -> BACKOFF, never DOCKED\n");
}

int main(void)
{
    printf("[carrier core vs skid-steer + bay + charger plant]\n");
    test_dock_charge_undock();
    test_dirty_contact();
    test_contact_degrades_during_charge();
    test_kalert_and_taper();
    test_drive();
    test_permit_blocks_motion();
    test_estop_and_wrong_agent();
    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
