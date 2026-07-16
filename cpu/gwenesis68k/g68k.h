/*
 * EMU_G68K bridge: gwenesis const-table Musashi 68K core <-> picodrive.
 *
 * The core (m68kcpu.c and friends, this directory) is copied from gwenesis
 * (Genesis Plus GX lineage, see the header in m68kcpu.c / m68k.h). It has a
 * single global context `m68k`, runs to an ABSOLUTE master-cycle target
 * (7 x 68k cycles, uint32 up-counter m68k.cycles / m68k.cycle_end), and does
 * every memory access through m68k.memory_map[256] (64 KB pages, {base,
 * read8, read16, write8, write16}).
 *
 * This header is what picodrive-side code (pico_int.h, sek.c, memory.c,
 * pico_cmn.c) includes; g68k_bus.c implements the map bridge.
 *
 * Constraints:
 *  - single context => no Sega CD sub-68k. pico_int.h stubs the S68k macros
 *    so full (non-GNW) builds still compile, but MCD must not be run.
 *  - m68k.cycles is rebased to 0 at every timeslice (SekExecM68k), so the
 *    uint32 up-counter can never overflow and m68k_run() can never see an
 *    "already ahead" target.
 */
#ifndef G68K_BRIDGE_H
#define G68K_BRIDGE_H

#include "m68k.h"

/* m68k.h drags in macros.h, whose INLINE definition must not leak into
 * picodrive translation units (ym2612.c writes `static INLINE`); the core's
 * own .c files include m68k.h directly, not through this header, so they
 * keep it. Same dance pico_int.h does for EMU_M68K. */
#undef INLINE

/* mirrors STOP_LEVEL_* from m68kcpu.h (not exported by m68k.h) */
#define G68K_STOP_LEVEL_STOP 1
#define G68K_STOP_LEVEL_HALT 2

/* picodrive poll-detection marker; set to 1 by DBcc handlers in the core
 * (fame's NOT_POLLING equivalent), cleared by pico's poll detectors. */
extern int g68k_not_polling;

/* dummy backing for the unsupported Sega CD sub-68k macros (pico_int.h):
 * [0] fake cycles-left lvalue, [1] fake not_polling lvalue */
extern int g68k_s68k_stub[2];

/* one-time init: point every memory_map page at safe defaults (generic
 * picodrive dispatchers + a harmless fetch base) */
void g68k_bus_init(void);

/* decode picodrive's m68k_read8/read16/write8/write16 page tables into
 * m68k.memory_map[] for the given 68k address range. Called from every
 * cpu68k_map_* registration in pico/memory.c so runtime remaps (32X banking,
 * DRAM swap, ...) stay in sync. */
void g68k_map_sync_range(unsigned int start_addr, unsigned int end_addr);
void g68k_map_sync_all(void);

#endif /* G68K_BRIDGE_H */
