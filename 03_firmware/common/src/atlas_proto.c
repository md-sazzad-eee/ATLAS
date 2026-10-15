#include "atlas_proto.h"
#include <string.h>

uint16_t atlas_crc16(const uint8_t *d, size_t n)       /* CRC-16/CCITT-FALSE */
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t atlas_cobs_encode(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t r = 0, w = 1, code_i = 0; uint8_t code = 1;
    while (r < n) {
        if (in[r] == 0) { out[code_i] = code; code = 1; code_i = w++; r++; }
        else { out[w++] = in[r++]; if (++code == 0xFF) { out[code_i] = code; code = 1; code_i = w++; } }
    }
    out[code_i] = code;
    return w;
}

size_t atlas_cobs_decode(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t r = 0, w = 0;
    while (r < n) {
        uint8_t code = in[r++];
        if (code == 0) return 0;
        for (uint8_t i = 1; i < code; i++) {
            if (r >= n || in[r] == 0) return 0;
            out[w++] = in[r++];
        }
        if (code != 0xFF && r < n) out[w++] = 0;
    }
    return w;
}

size_t atlas_frame_build(uint8_t type, uint8_t seq, const void *payload, size_t n, uint8_t *out)
{
    uint8_t raw[ATLAS_MAX_PAYLOAD + 4];
    if (n > ATLAS_MAX_PAYLOAD) return 0;
    raw[0] = type; raw[1] = seq;
    if (n) memcpy(raw + 2, payload, n);
    uint16_t crc = atlas_crc16(raw, n + 2);
    raw[n + 2] = (uint8_t)(crc & 0xFF); raw[n + 3] = (uint8_t)(crc >> 8);
    size_t m = atlas_cobs_encode(raw, n + 4, out);
    out[m] = 0;
    return m + 1;
}

int atlas_frame_parse(const uint8_t *frame, size_t n, uint8_t *type, uint8_t *seq, uint8_t *payload)
{
    uint8_t raw[ATLAS_MAX_FRAME];
    if (n == 0 || n > ATLAS_MAX_FRAME) return -1;
    size_t m = atlas_cobs_decode(frame, n, raw);
    if (m < 4) return -1;
    uint16_t crc = (uint16_t)(raw[m - 2] | (raw[m - 1] << 8));
    if (crc != atlas_crc16(raw, m - 2)) return -1;
    *type = raw[0]; *seq = raw[1];
    memcpy(payload, raw + 2, m - 4);
    return (int)(m - 4);
}

void atlas_rx_init(atlas_rx_t *r, atlas_rx_cb cb, void *ctx) { r->len = 0; r->errors = 0; r->cb = cb; r->ctx = ctx; }

void atlas_rx_feed(atlas_rx_t *r, const uint8_t *data, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (data[i] == 0) {
            if (r->len) {
                uint8_t type, seq, pl[ATLAS_MAX_PAYLOAD];
                int k = atlas_frame_parse(r->buf, r->len, &type, &seq, pl);
                if (k >= 0 && r->cb) r->cb(type, seq, pl, k, r->ctx); else if (k < 0) r->errors++;
            }
            r->len = 0;
        } else if (r->len < sizeof r->buf) {
            r->buf[r->len++] = data[i];
        } else {
            r->len = 0; r->errors++;      /* overrun: drop until next delimiter */
        }
    }
}
