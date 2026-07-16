/*
 * gen_ym2612_const.c - host codegen for pico/sound/ym2612_const_tables.h
 *
 * Runs the exact table-fill loops from init_tables() (see
 * ym2612_tables_fill.h) and emits the four resulting tables as C `const`
 * array initializers, so the M7 firmware (GNW_32X_CORE) can XIP them from
 * .rodata instead of generating them into ~351 KB of writable RAM.
 *
 * Build+run on the host:
 *   cc -O2 -o gen_ym2612_const gen_ym2612_const.c -lm
 *   ./gen_ym2612_const > ../pico/sound/ym2612_const_tables.h
 *
 * NOT compiled into the firmware.
 */
#include <stdio.h>
#include <stdlib.h>
#include "ym2612_tables_fill.h"

static YM_UINT16 ym_sin_tab[256];
static YM_UINT16 ym_tl_tab2[13*TL_RES_LEN];
static YM_UINT16 ym_tl_tab[TL_TAB_LEN];
static YM_INT32  lfo_pm_table[128*8*32];

static void dump_u16(FILE *f, const char *decl, const YM_UINT16 *a, int n)
{
	int i;
	fprintf(f, "%s = {\n", decl);
	for (i = 0; i < n; i++)
	{
		fprintf(f, "%u,", (unsigned)a[i]);
		if ((i & 15) == 15) fputc('\n', f);
	}
	if (n & 15) fputc('\n', f);
	fprintf(f, "};\n\n");
}

static void dump_s32(FILE *f, const char *decl, const YM_INT32 *a, int n)
{
	int i;
	fprintf(f, "%s = {\n", decl);
	for (i = 0; i < n; i++)
	{
		fprintf(f, "%d,", (int)a[i]);
		if ((i & 15) == 15) fputc('\n', f);
	}
	if (n & 15) fputc('\n', f);
	fprintf(f, "};\n\n");
}

int main(void)
{
	FILE *f = stdout;

	ym2612_fill_const_tables(ym_sin_tab, ym_tl_tab2, ym_tl_tab, lfo_pm_table);

	fprintf(f,
		"/*\n"
		" * ym2612_const_tables.h - GENERATED, do not edit by hand.\n"
		" *\n"
		" * Compile-time-constant YM2612 synthesis tables for the\n"
		" * GNW_32X_CORE (Cortex-M7) build.  Regenerate with:\n"
		" *   tools/gen_ym2612_const.c  (see that file for the command)\n"
		" *\n"
		" * These are byte-identical to the arrays init_tables() fills at\n"
		" * runtime in the upstream (non-GNW_32X_CORE) build; proven by\n"
		" * tools/verify_ym2612_const.c (memcmp == 0 for all four).\n"
		" *\n"
		" * Placing them in .rodata lets them XIP from flash and reclaims\n"
		" * ~351 KB of writable RAM (bss) on the device.\n"
		" */\n"
		"#ifndef YM2612_CONST_TABLES_H\n"
		"#define YM2612_CONST_TABLES_H\n\n"
		"/* declarations mirror pico/sound/ym2612.c, now const */\n\n");

	dump_u16(f, "const UINT16 ym_tl_tab[TL_TAB_LEN]",  ym_tl_tab,  TL_TAB_LEN);
	dump_u16(f, "const UINT16 ym_tl_tab2[13*TL_RES_LEN]", ym_tl_tab2, 13*TL_RES_LEN);
	dump_u16(f, "static const UINT16 ym_sin_tab[256]", ym_sin_tab, 256);
	dump_s32(f, "static const INT32 lfo_pm_table[128*8*32]", lfo_pm_table, 128*8*32);

	fprintf(f, "#endif /* YM2612_CONST_TABLES_H */\n");
	return 0;
}
