/* Board B hardware access for the agent core: sensors in, actuators out. */
#pragma once
#include "atlas_agent.h"
void board_b_init(void);
void board_b_read(atlas_agent_in_t *in);            /* fills everything except now_ms */
void board_b_apply(const atlas_agent_out_t *out, uint16_t vbat_mv);
void board_b_kick_watchdog(void);                   /* toggles WDI */
void board_b_safe_outputs(void);                    /* motors off, charge off — used on any doubt */
