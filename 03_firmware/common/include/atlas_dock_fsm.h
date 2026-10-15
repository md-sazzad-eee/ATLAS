/* ATLAS docking state machine — C port of 05_simulation/matlab/atlas_docking_fsm.slx
 * (Stateflow reference) and 05_simulation/python/atlas/docking.py (Python twin).
 * Call atlas_dock_step() every ATLAS_DOCK_DT_MS milliseconds. No dynamic memory, no floats
 * required on the MCU path. Runs on the CARRIER MCU (Board A): it owns latch + move_permit.
 */
#ifndef ATLAS_DOCK_FSM_H
#define ATLAS_DOCK_FSM_H
#include <stdbool.h>
#include <stdint.h>

#define ATLAS_DOCK_DT_MS 10u

typedef enum {
    DOCK_INIT = -1, DOCK_IDLE = 0, DOCK_APPROACH = 1, DOCK_LATCHING = 2, DOCK_VERIFY = 3,
    DOCK_DOCKED = 4, DOCK_UNLATCHING = 5, DOCK_RELEASING = 6, DOCK_BACKOFF = 7, DOCK_FAULT = 9
} atlas_dock_state_t;

typedef struct {
    bool cmd_dock, cmd_undock, seated, latch_locked, id_ok, comm_ok,
         carrier_stationary, agent_clear, estop, reset;
} atlas_dock_in_t;

typedef struct {
    atlas_dock_state_t state;
    int8_t drive_cmd;        /* +1 slow in, 0 stop, -1 back out (sent to the agent) */
    bool latch_open_cmd;     /* true = retract pin; latch is spring fail-locked     */
    bool move_permit;        /* carrier may drive with this bay occupied            */
} atlas_dock_out_t;

typedef struct {
    atlas_dock_out_t out;
    uint32_t t_ms;           /* time since start                                    */
    uint32_t t_entry_ms;     /* time the current state was entered                  */
    uint8_t retries;
    uint8_t retry_limit;
    /* timeouts (ms) — defaults match the Stateflow chart */
    uint32_t to_approach, to_latch, to_verify, to_backoff_clear, to_backoff_max,
             to_unlatch, to_release;
} atlas_dock_fsm_t;

void atlas_dock_init(atlas_dock_fsm_t *f, uint8_t retry_limit);
const atlas_dock_out_t *atlas_dock_step(atlas_dock_fsm_t *f, const atlas_dock_in_t *in);
const char *atlas_dock_state_name(atlas_dock_state_t s);

#endif
