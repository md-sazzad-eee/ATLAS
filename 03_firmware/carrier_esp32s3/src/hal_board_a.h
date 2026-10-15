#pragma once
#include "atlas_carrier.h"
void board_a_init(void);
void board_a_read(atlas_carrier_in_t *in);
void board_a_apply(const atlas_carrier_out_t *o, uint16_t vbat_mv);
void board_a_kick_watchdog(void);
void board_a_safe_outputs(void);
