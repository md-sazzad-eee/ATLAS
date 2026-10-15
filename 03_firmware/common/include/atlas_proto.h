/* ATLAS wire protocol: frame = COBS( type | seq | payload | CRC16-CCITT-FALSE(le) ) + 0x00.
 * Used on the dock DATA line, the ESP32<->Pi UART and (as UDP payload) ESP32<->carrier Wi-Fi.
 * Python twin: 04_ros2_ws/src/atlas_bridge/atlas_bridge/proto.py (cross-tested). */
#ifndef ATLAS_PROTO_H
#define ATLAS_PROTO_H
#include <stddef.h>
#include <stdint.h>

#define ATLAS_MAX_PAYLOAD 96
#define ATLAS_MAX_FRAME   (ATLAS_MAX_PAYLOAD + 4 + 2 + 2)

enum atlas_msg_type {
    MSG_HEARTBEAT = 0x01,   /* carrier/agent: uint32 t_ms, uint8 state                         */
    MSG_TELEMETRY = 0x02,   /* agent->carrier: atlas_telemetry_t                               */
    MSG_TASK_CMD  = 0x10,   /* carrier->agent: atlas_task_cmd_t                                */
    MSG_TASK_RESULT = 0x11, /* agent->carrier: atlas_task_result_t                             */
    MSG_DRIVE_CMD = 0x20,   /* carrier->agent: int16 v_mm_s, int16 w_mrad_s, uint16 ttl_ms     */
    MSG_SET_POSE  = 0x21,   /* carrier->agent: int32 x_mm, int32 y_mm, int16 yaw_mrad (world)  */
    MSG_DOCK_CMD  = 0x50,   /* ROS->carrier: uint8 bay, uint8 cmd (0 dock, 1 undock, 2 reset), uint8 expected agent (0 any,1 S1,2 S2) */
    MSG_BAY_STATUS = 0x51,  /* carrier->ROS: atlas_bay_status_t                                 */
    MSG_CARRIER_STATUS = 0x52, /* carrier->ROS: atlas_carrier_status_t                          */
    MSG_DOCK_RELAY = 0x53,  /* carrier<->ROS: uint8 bay, uint8 inner type, inner payload (agent frames over the dock DATA pad) */
    MSG_VISION_TAG = 0x30,  /* Pi->ESP32: atlas_vision_tag_t                                   */
    MSG_SHUTDOWN  = 0x40,   /* ESP32->Pi                                                       */
    MSG_ACK       = 0x7F
};

#pragma pack(push, 1)
typedef struct { uint32_t t_ms; uint8_t state; } atlas_heartbeat_t;   /* carrier state: b0 ESTOP, b7 CLEAR_FAULTS */
typedef struct {
    uint32_t t_ms; uint16_t vbat_mv; uint8_t soc_pct; uint8_t health;   /* atlas Health enum index */
    int16_t wl_mrad_s, wr_mrad_s; uint16_t il_ma, ir_ma; int32_t x_mm, y_mm; int16_t yaw_mrad;
    uint8_t flags;            /* b0 charging, b1 docked(VCHG present), b2 motor_enabled, b3 pi_on */
} atlas_telemetry_t;
typedef struct { uint16_t task_id; uint8_t kind; int32_t x_mm, y_mm; uint16_t timeout_s; } atlas_task_cmd_t;
typedef struct { uint16_t task_id; uint8_t status; float value; float uncertainty; uint16_t cal_epoch; } atlas_task_result_t;
typedef struct {
    uint8_t bay; int8_t state; uint8_t flags;   /* b0 seated b1 latched b2 id_ok b3 charging b4 charge_fault b5 move_permit b6 kalert */
    uint8_t agent; int8_t drive_cmd; uint8_t retries; uint8_t charge_state;
    uint16_t vpad_mv, ichg_ma, rc_vchg_mohm, rc_gnd_mohm;
} atlas_bay_status_t;
typedef struct {
    uint32_t t_ms; uint16_t vbat_mv; uint8_t flags;   /* b0 move_permit b1 estop b2 motors_enabled b3 cmd_timeout b4 slip_l b5 slip_r */
    int16_t vl_mm_s, vr_mm_s; uint16_t imot_max_ma; int32_t x_mm, y_mm; int16_t yaw_mrad;
} atlas_carrier_status_t;
typedef struct { uint16_t tag_id; int16_t x_mm, y_mm, z_mm; int16_t yaw_mrad; uint8_t quality; } atlas_vision_tag_t;
#pragma pack(pop)
/* wire sizes are mirrored by struct formats in atlas_bridge/proto.py */
_Static_assert(sizeof(atlas_telemetry_t) == 27, "telemetry");
_Static_assert(sizeof(atlas_bay_status_t) == 15, "bay status");
_Static_assert(sizeof(atlas_carrier_status_t) == 23, "carrier status");

uint16_t atlas_crc16(const uint8_t *d, size_t n);
size_t   atlas_cobs_encode(const uint8_t *in, size_t n, uint8_t *out);       /* no trailing 0 */
size_t   atlas_cobs_decode(const uint8_t *in, size_t n, uint8_t *out);       /* 0 = error */
/* build a complete frame incl. trailing 0x00; returns length or 0 */
size_t   atlas_frame_build(uint8_t type, uint8_t seq, const void *payload, size_t n, uint8_t *out);
/* parse one frame (without the 0x00); returns payload length, or -1 on CRC/COBS error */
int      atlas_frame_parse(const uint8_t *frame, size_t n, uint8_t *type, uint8_t *seq, uint8_t *payload);

/* streaming receiver: feed bytes, get callbacks per valid frame */
typedef void (*atlas_rx_cb)(uint8_t type, uint8_t seq, const uint8_t *payload, int n, void *ctx);
typedef struct { uint8_t buf[ATLAS_MAX_FRAME]; size_t len; uint32_t errors; atlas_rx_cb cb; void *ctx; } atlas_rx_t;
void atlas_rx_init(atlas_rx_t *r, atlas_rx_cb cb, void *ctx);
void atlas_rx_feed(atlas_rx_t *r, const uint8_t *data, size_t n);
#endif
