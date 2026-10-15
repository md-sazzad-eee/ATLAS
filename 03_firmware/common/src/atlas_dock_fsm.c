#include "atlas_dock_fsm.h"

void atlas_dock_init(atlas_dock_fsm_t *f, uint8_t retry_limit)
{
    f->out.state = DOCK_INIT; f->out.drive_cmd = 0; f->out.latch_open_cmd = false; f->out.move_permit = false;
    f->t_ms = 0; f->t_entry_ms = 0; f->retries = 0; f->retry_limit = retry_limit;
    f->to_approach = 15000; f->to_latch = 2000; f->to_verify = 3000; f->to_backoff_clear = 3000;
    f->to_backoff_max = 10000; f->to_unlatch = 2000; f->to_release = 10000;
}

static bool after(const atlas_dock_fsm_t *f, uint32_t ms) { return (f->t_ms - f->t_entry_ms) >= ms; }

static void enter(atlas_dock_fsm_t *f, atlas_dock_state_t s)
{
    atlas_dock_out_t *o = &f->out;
    o->state = s; f->t_entry_ms = f->t_ms;
    switch (s) {
    case DOCK_IDLE:       o->drive_cmd = 0; o->latch_open_cmd = false; o->move_permit = false; break;
    case DOCK_APPROACH:   o->drive_cmd = 1; o->latch_open_cmd = true;  o->move_permit = false; break;
    case DOCK_LATCHING:   o->drive_cmd = 0; o->latch_open_cmd = false; break;
    case DOCK_VERIFY:     o->drive_cmd = 0; break;
    case DOCK_DOCKED:     o->drive_cmd = 0; o->latch_open_cmd = false; break;
    case DOCK_UNLATCHING: o->latch_open_cmd = true; o->move_permit = false; break;
    case DOCK_RELEASING:  o->drive_cmd = -1; break;
    case DOCK_BACKOFF:    o->latch_open_cmd = true; o->drive_cmd = 0; f->retries++; break;
    case DOCK_FAULT:      o->drive_cmd = 0; o->latch_open_cmd = false; o->move_permit = false; break;
    default: break;
    }
}

static void go(atlas_dock_fsm_t *f, atlas_dock_state_t s)
{
    if (f->out.state == DOCK_DOCKED) f->out.move_permit = false;   /* exit action */
    enter(f, s);
}

const atlas_dock_out_t *atlas_dock_step(atlas_dock_fsm_t *f, const atlas_dock_in_t *u)
{
    atlas_dock_state_t s = f->out.state;
    if (s == DOCK_INIT) {
        enter(f, DOCK_IDLE);
    } else if (s != DOCK_FAULT && u->estop) {        /* parent transition has priority */
        go(f, DOCK_FAULT);
    } else switch (s) {
    case DOCK_FAULT:
        if (u->reset && !u->estop) go(f, DOCK_IDLE);
        break;
    case DOCK_IDLE:
        if (u->cmd_dock && u->carrier_stationary) { f->retries = 0; go(f, DOCK_APPROACH); }
        break;
    case DOCK_APPROACH:
        if (u->seated) go(f, DOCK_LATCHING);
        else if (after(f, f->to_approach)) go(f, DOCK_BACKOFF);
        break;
    case DOCK_LATCHING:
        if (u->seated && u->latch_locked) go(f, DOCK_VERIFY);
        else if (!u->seated || after(f, f->to_latch)) go(f, DOCK_BACKOFF);
        break;
    case DOCK_VERIFY:
        if (u->seated && u->latch_locked && u->id_ok && u->comm_ok) go(f, DOCK_DOCKED);
        else if (after(f, f->to_verify)) go(f, DOCK_BACKOFF);
        break;
    case DOCK_DOCKED:
        if (u->cmd_undock && u->carrier_stationary) go(f, DOCK_UNLATCHING);
        else if (!u->seated || !u->latch_locked) go(f, DOCK_FAULT);
        else f->out.move_permit = u->seated && u->latch_locked;
        break;
    case DOCK_UNLATCHING:
        if (!u->latch_locked) go(f, DOCK_RELEASING);
        else if (after(f, f->to_unlatch)) go(f, DOCK_FAULT);
        break;
    case DOCK_RELEASING:
        if (!u->seated && u->agent_clear) go(f, DOCK_IDLE);
        else if (after(f, f->to_release)) go(f, DOCK_FAULT);
        break;
    case DOCK_BACKOFF:
        if (!u->seated && after(f, f->to_backoff_clear) && f->retries >= f->retry_limit) go(f, DOCK_FAULT);
        else if (!u->seated && after(f, f->to_backoff_clear) && f->retries < f->retry_limit) go(f, DOCK_APPROACH);
        else if (after(f, f->to_backoff_max)) go(f, DOCK_FAULT);
        else f->out.drive_cmd = u->latch_locked ? 0 : -1;
        break;
    default: break;
    }
    f->t_ms += ATLAS_DOCK_DT_MS;
    return &f->out;
}

const char *atlas_dock_state_name(atlas_dock_state_t s)
{
    switch (s) {
    case DOCK_IDLE: return "IDLE"; case DOCK_APPROACH: return "APPROACH"; case DOCK_LATCHING: return "LATCHING";
    case DOCK_VERIFY: return "VERIFY"; case DOCK_DOCKED: return "DOCKED"; case DOCK_UNLATCHING: return "UNLATCHING";
    case DOCK_RELEASING: return "RELEASING"; case DOCK_BACKOFF: return "BACKOFF"; case DOCK_FAULT: return "FAULT";
    default: return "INIT";
    }
}
