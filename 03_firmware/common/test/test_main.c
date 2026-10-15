/* Host unit tests for the ATLAS firmware core. Build: make -C 03_firmware/common test */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "atlas_dock_fsm.h"
#include "atlas_pi.h"
#include "atlas_proto.h"

static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- 1. docking FSM vs Stateflow traces (sample for sample) ---- */
static void test_dock_parity(const char *dir)
{
    for (int k = 1; k <= 8; k++) {
        char path[512]; snprintf(path, sizeof path, "%s/scenario_%d.csv", dir, k);
        FILE *fp = fopen(path, "r");
        CHECK(fp != NULL, "missing %s", path); if (!fp) continue;
        char line[512]; if (!fgets(line, sizeof line, fp)) { fclose(fp); continue; }
        atlas_dock_fsm_t f; atlas_dock_init(&f, 3);
        int n = 0, bad = 0;
        while (fgets(line, sizeof line, fp)) {
            double v[15]; int c = 0; char *tok = strtok(line, ",");
            while (tok && c < 15) { v[c++] = atof(tok); tok = strtok(NULL, ","); }
            if (c < 15) continue;
            atlas_dock_in_t in = { v[1] > .5, v[2] > .5, v[3] > .5, v[4] > .5, v[5] > .5, v[6] > .5,
                                   v[7] > .5, v[8] > .5, v[9] > .5, v[10] > .5 };
            const atlas_dock_out_t *o = atlas_dock_step(&f, &in);
            if ((int)o->state != (int)v[11] || o->drive_cmd != (int)v[12] || o->move_permit != (v[14] > .5)) {
                if (!bad) printf("  scenario %d first mismatch t=%.2f: C=%s Stateflow=%d\n", k, v[0], atlas_dock_state_name(o->state), (int)v[11]);
                bad++;
            }
            n++;
        }
        fclose(fp);
        CHECK(bad == 0, "scenario %d: %d/%d samples differ", k, bad, n);
        printf("  dock scenario %d: %d samples, %d mismatches\n", k, n, bad);
    }
}

/* ---- 2. PI loop on the same DC-motor model as build_speed_loop.m ---- */
static void test_pi_closed_loop(void)
{
    const double R = 6.0, L = 1.5e-3, Kt = 4.5e-3, Ke = 4.5e-3, Jm = 1e-7, b = 1e-7, N = 150, eta = 0.7;
    const double r = 0.0325, m = 1.2, Vb = 7.4, Ts = 0.01, cpr = 12 * 4 * 150;
    const double J = Jm + (m / 2 * r * r) / (N * N * eta);
    const double w_ref = 0.20 / r, T_ramp = 1.2 * 9.81 * (sin(12 * M_PI / 180) + 0.03 * cos(12 * M_PI / 180)) / 2 * r;
    atlas_pi_t pi; atlas_pi_init(&pi, 0.476f, 15.993f, (float)Ts, (float)-Vb, (float)Vb);
    double i = 0, wm = 0, th = 0, th_prev_q = 0, V = 0, peak = 0, dt = 1e-5, t = 0, w_at_09 = 0, w_end = 0, dip_min = 1e9;
    int steps = (int)(Ts / dt);
    for (int k = 0; k < 200; k++, t += Ts) {
        double thq = floor(th / (2 * M_PI / cpr)) * (2 * M_PI / cpr);
        double w_meas = (thq - th_prev_q) / Ts; th_prev_q = thq;
        double ref = (t >= 0.1 - 1e-9) ? w_ref : 0.0;
        V = atlas_pi_step(&pi, (float)ref, (float)w_meas);
        double TL = (t >= 1.0 - 1e-9) ? T_ramp : 0.0;
        for (int s = 0; s < steps; s++) {
            double di = (V - R * i - Ke * wm) / L, dw = (Kt * i - b * wm - TL / (N * eta)) / J;
            i += di * dt; wm += dw * dt; th += wm / N * dt;
        }
        double ww = wm / N;
        if (t < 1.0 && ww > peak) peak = ww;
        if (fabs(t - 0.9) < 1e-6) w_at_09 = ww;
        if (t > 1.0 && ww < dip_min) dip_min = ww;
        w_end = ww;
    }
    double os = 100 * (peak - w_ref) / w_ref;
    printf("  PI loop: overshoot %.1f %%, w(0.9s) %.3f, w(end) %.3f rad/s (ref %.3f), dip %.1f %%\n",
           os, w_at_09, w_end, w_ref, 100 * (w_ref - dip_min) / w_ref);
    CHECK(os > 0 && os < 10, "overshoot %.1f%% outside (0,10) - Simulink gave 4.8%%", os);
    CHECK(fabs(w_end - w_ref) / w_ref < 0.02, "steady-state error on ramp > 2%%");
    CHECK(100 * (w_ref - dip_min) / w_ref < 12, "load dip too large");
    /* anti-windup: saturate for 2 s, then the output must leave saturation within 0.2 s of ref drop */
    atlas_pi_init(&pi, 0.476f, 15.993f, 0.01f, -7.4f, 7.4f);
    for (int k = 0; k < 200; k++) atlas_pi_step(&pi, 100.0f, 0.0f);
    int k_out = -1;
    for (int k = 0; k < 100; k++) { float u = atlas_pi_step(&pi, 0.0f, 0.0f); if (u < 7.39f) { k_out = k; break; } }
    CHECK(k_out >= 0 && k_out < 20, "integrator wound up (left saturation after %d steps)", k_out);
}

/* ---- 3. protocol: round trips, corruption, streaming, vectors for the Python twin ---- */
static void test_proto(const char *vec_out)
{
    uint8_t frame[ATLAS_MAX_FRAME + 1], pl[ATLAS_MAX_PAYLOAD], type, seq;
    srand(1234);
    for (int t = 0; t < 2000; t++) {
        uint8_t buf[ATLAS_MAX_PAYLOAD]; size_t n = (size_t)(rand() % (ATLAS_MAX_PAYLOAD + 1));
        for (size_t i = 0; i < n; i++) buf[i] = (rand() % 4 == 0) ? 0 : (uint8_t)rand();
        size_t m = atlas_frame_build(0x11, (uint8_t)t, buf, n, frame);
        CHECK(m > 0 && frame[m - 1] == 0, "build failed");
        for (size_t i = 0; i + 1 < m; i++) if (frame[i] == 0) { CHECK(0, "zero inside COBS frame"); break; }
        int k = atlas_frame_parse(frame, m - 1, &type, &seq, pl);
        CHECK(k == (int)n && type == 0x11 && seq == (uint8_t)t && memcmp(pl, buf, n) == 0, "round trip %d", t);
        if (m > 3) { frame[1] ^= 0x5A; if (frame[1] == 0) frame[1] = 1;
                     CHECK(atlas_frame_parse(frame, m - 1, &type, &seq, pl) < 0 || memcmp(pl, buf, n) != 0, "corruption undetected"); }
    }
    /* streaming with garbage in between */
    struct { int got; } ctx = {0};
    atlas_rx_t rx;
    void cb(uint8_t ty, uint8_t sq, const uint8_t *p, int n, void *c) { (void)ty; (void)sq; (void)p; (void)n; ((int *)c)[0]++; }
    atlas_rx_init(&rx, cb, &ctx.got);
    atlas_telemetry_t tel = { 123456, 7900, 88, 0, 6150, -6150, 420, 380, 1500, -250, 785, 0x05 };
    size_t m = atlas_frame_build(MSG_TELEMETRY, 9, &tel, sizeof tel, frame);
    uint8_t junk[] = { 0x13, 0x37, 0x00, 0xFF, 0x00 };   /* line noise; resync on delimiter */
    atlas_rx_feed(&rx, junk, sizeof junk); atlas_rx_feed(&rx, frame, m); atlas_rx_feed(&rx, frame, 5); atlas_rx_feed(&rx, frame + 5, m - 5);
    CHECK(ctx.got == 2 && rx.errors == 2, "stream: got %d frames, %u errors", ctx.got, (unsigned)rx.errors);
    /* vectors */
    FILE *fp = fopen(vec_out, "w");
    if (fp) {
        fprintf(fp, "{\n \"telemetry_frame\": \"");
        for (size_t i = 0; i < m; i++) fprintf(fp, "%02x", frame[i]);
        fprintf(fp, "\",\n \"telemetry_size\": %zu,\n \"crc16_123456789\": %u\n}\n", sizeof tel, atlas_crc16((const uint8_t *)"123456789", 9));
        fclose(fp);
    }
    CHECK(atlas_crc16((const uint8_t *)"123456789", 9) == 0x29B1, "CRC-16/CCITT-FALSE check value");
}

int main(int argc, char **argv)
{
    const char *traces = argc > 1 ? argv[1] : "../../05_simulation/python/tests/fixtures/docking_traces";
    const char *vec = argc > 2 ? argv[2] : "test/proto_vectors.json";
    printf("[dock FSM parity vs Stateflow]\n"); test_dock_parity(traces);
    printf("[PI speed loop]\n"); test_pi_closed_loop();
    printf("[protocol]\n"); test_proto(vec);
    printf("\n%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
