/* Software-in-the-loop carrier: the real carrier_core + a simple plant (wheels, two bays with agents
 * that obey the dock drive command, latch, charger), exposed as a byte stream identical to Board A's
 * USB link. Built as a shared library and driven by atlas_bridge/fake_carrier.py so the ROS 2
 * carrier_bridge can be tested against the actual firmware logic.
 *   cc -shared -fPIC -O2 -I../include -I../../common/include carrier_sim.c ../src/atlas_carrier.c ../../common/src/atlas_dock_fsm.c \
 *      ../../common/src/atlas_pi.c ../../common/src/atlas_proto.c -lm -o libatlas_carrier_sim.so */
#include <math.h>
#include <string.h>
#include "atlas_carrier.h"

static atlas_carrier_t C;
static atlas_rx_t RX;
static uint8_t txbuf[8192]; static int txn; static uint8_t seq;
static double th[4], wm[4], agent_pos[2], latch_t[2], soc[2];
static bool present[2], seated[2], latched[2];
static uint32_t now;

static void send(uint8_t t, const void *p, size_t n, void *ctx)
{
    (void)ctx; uint8_t f[ATLAS_MAX_FRAME + 4];
    size_t len = atlas_frame_build(t, seq++, p, n, f);
    if (len && txn + (int)len < (int)sizeof txbuf) { memcpy(txbuf + txn, f, len); txn += (int)len; }
}
static void on_rx(uint8_t t, uint8_t s, const uint8_t *p, int n, void *ctx) { (void)s; (void)ctx; atlas_carrier_on_frame(&C, t, p, n); }

void cs_init(int a0, int a1)
{
    atlas_carrier_cfg_t k; atlas_carrier_default_cfg(&k);
    atlas_carrier_init(&C, &k, send, NULL); atlas_rx_init(&RX, on_rx, NULL);
    memset(th, 0, sizeof th); memset(wm, 0, sizeof wm); memset(agent_pos, 0, sizeof agent_pos);
    present[0] = a0; present[1] = a1; seated[0] = seated[1] = latched[0] = latched[1] = false;
    soc[0] = soc[1] = 0.5; now = 0; txn = 0;
}
void cs_rx(const uint8_t *b, int n) { atlas_rx_feed(&RX, b, (size_t)n); }
int cs_tx(uint8_t *b, int max) { int n = txn < max ? txn : max; memcpy(b, txbuf, (size_t)n); memmove(txbuf, txbuf + n, (size_t)(txn - n)); txn -= n; return n; }

void cs_step_ms(int ms)
{
    const double cpr = 11 * 4 * 50, KE = 0.0155, R = 2.0, KT = 0.0155, J = 9.4e-6, N = 50, ETA = 0.7;
    for (int t = 0; t < ms; t += CARRIER_DT_MS) {
        atlas_carrier_in_t in; memset(&in, 0, sizeof in);
        in.now_ms = now; in.vbat_mv = 13100; in.estop_ok = true;
        for (int i = 0; i < 4; i++) in.enc[i] = (int32_t)floor(th[i] / (2 * M_PI) * cpr);
        const atlas_carrier_out_t *o = &C.out;
        for (int b = 0; b < 2; b++) {
            atlas_bay_in_t *bi = &in.bay[b];
            bi->seated = seated[b]; bi->latched = latched[b]; bi->id_mv = seated[b] ? (b ? 2269 : 1650) : 3300;
            double i = (o->bay_en[b] && seated[b]) ? fmin(o->iset_ma[b] / 1000.0, fmax(0, (8.04 - (6.0 + 2.1 * soc[b])) / 0.39)) : 0;
            bi->i_ma = (uint16_t)(i * 1000); bi->vpad_mv = o->bay_en[b] ? 8300 : 0;
            bi->vdrop_vchg_uv = (int32_t)(i * 0.030e6); bi->vdrop_gnd_uv = (int32_t)(i * 0.030e6);
        }
        o = atlas_carrier_tick(&C, &in);
        for (int s = 0; s < 10; s++) {                     /* 1 ms plant steps */
            for (int i = 0; i < 4; i++) {
                double V = o->mot_en ? (i < 2 ? o->motor_l_v : o->motor_r_v) : 0;
                double cur = o->mot_en ? (V - KE * wm[i]) / R : 0;
                wm[i] += 0.001 * (KT * cur - 2e-6 * wm[i] - 0.004 * tanh(wm[i] / N / 0.05) / (N * ETA)) / J;
                th[i] += 0.001 * wm[i] / N;
            }
            for (int b = 0; b < 2; b++) {
                if (!present[b]) continue;
                if (o->agent_drive[b] && !latched[b]) agent_pos[b] += 0.001 * o->agent_drive[b] * 0.05 / 0.075;
                agent_pos[b] = fmin(1, fmax(0, agent_pos[b]));
                seated[b] = agent_pos[b] > 0.97;
                if (o->latch_open[b] || !seated[b]) { latch_t[b] = 0; latched[b] = false; }
                else if ((latch_t[b] += 0.001) > 0.15) latched[b] = true;
                if (o->bay_en[b] && seated[b]) soc[b] += 0.001 * in.bay[b].i_ma / 1000.0 / 9000;
            }
        }
        now += CARRIER_DT_MS;
    }
}
int cs_state(int bay) { return C.bay[bay].fsm.out.state; }
