/*
 * verify_ym2612_const.c - proves the generated const tables are
 * byte-identical to the arrays init_tables() fills at runtime.
 *
 * Includes the GENERATED pico/sound/ym2612_const_tables.h (the const
 * arrays that ship in the M7 build), re-runs the exact runtime fill
 * loops into fresh arrays, and memcmp's the two.  Exit non-zero if any
 * table differs.
 *
 * Build+run on the host (after generating the header):
 *   cc -O2 -o verify_ym2612_const verify_ym2612_const.c -lm && ./verify_ym2612_const
 */
#include <stdio.h>
#include <string.h>
#include "ym2612_tables_fill.h"

/* the generated header uses these names/macros (as in ym2612.c) */
typedef uint16_t UINT16;
typedef int32_t  INT32;

#include "../pico/sound/ym2612_const_tables.h"

/* fresh runtime-filled copies */
static YM_UINT16 rt_sin_tab[256];
static YM_UINT16 rt_tl_tab2[13*TL_RES_LEN];
static YM_UINT16 rt_tl_tab[TL_TAB_LEN];
static YM_INT32  rt_lfo_pm_table[128*8*32];

static int check(const char *name, const void *cst, const void *rt, size_t bytes)
{
	int rc = memcmp(cst, rt, bytes);
	printf("  %-14s  %7zu bytes  memcmp=%d  %s\n",
		name, bytes, rc, rc == 0 ? "IDENTICAL" : "*** MISMATCH ***");
	return rc;
}

int main(void)
{
	int bad = 0;

	ym2612_fill_const_tables(rt_sin_tab, rt_tl_tab2, rt_tl_tab, rt_lfo_pm_table);

	printf("YM2612 const-table parity (generated const vs runtime fill):\n");
	bad |= !!check("ym_sin_tab",   ym_sin_tab,   rt_sin_tab,   sizeof rt_sin_tab);
	bad |= !!check("ym_tl_tab2",   ym_tl_tab2,   rt_tl_tab2,   sizeof rt_tl_tab2);
	bad |= !!check("ym_tl_tab",    ym_tl_tab,    rt_tl_tab,    sizeof rt_tl_tab);
	bad |= !!check("lfo_pm_table", lfo_pm_table, rt_lfo_pm_table, sizeof rt_lfo_pm_table);

	printf("\n%s\n", bad ? "RESULT: MISMATCH - do NOT ship" : "RESULT: all four byte-identical");
	return bad;
}
