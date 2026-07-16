#include "../sh2.h"

#ifdef DRC_CMP
#include "../compiler.h"
#define BUSY_LOOP_HACKS 0
#else
#define BUSY_LOOP_HACKS 1
#endif

// MAME types
#ifndef INT8
typedef s8  INT8;
typedef s16 INT16;
typedef s32 INT32;
typedef u32 UINT32;
typedef u16 UINT16;
typedef u8  UINT8;
#endif

#ifdef DRC_SH2

// this nasty conversion is needed for drc-expecting memhandlers
#define MAKE_READFUNC(name, cname) \
static __inline unsigned int name(SH2 *sh2, unsigned int a) \
{ \
	unsigned int ret; \
	sh2->sr |= (sh2->icount << 12) | (sh2->no_polling); \
	ret = cname(a, sh2); \
	sh2->icount = (signed int)sh2->sr >> 12; \
	sh2->no_polling = (sh2->sr & SH2_NO_POLLING); \
	sh2->sr &= 0x3f3; \
	return ret; \
}

#define MAKE_WRITEFUNC(name, cname) \
static __inline void name(SH2 *sh2, unsigned int a, unsigned int d) \
{ \
	sh2->sr |= (sh2->icount << 12) | (sh2->no_polling); \
	cname(a, d, sh2); \
	sh2->icount = (signed int)sh2->sr >> 12; \
	sh2->no_polling = (sh2->sr & SH2_NO_POLLING); \
	sh2->sr &= 0x3f3; \
}

MAKE_READFUNC(RB, p32x_sh2_read8)
MAKE_READFUNC(RW, p32x_sh2_read16)
MAKE_READFUNC(RL, p32x_sh2_read32)
MAKE_WRITEFUNC(WB, p32x_sh2_write8)
MAKE_WRITEFUNC(WW, p32x_sh2_write16)
MAKE_WRITEFUNC(WL, p32x_sh2_write32)

#else

#define RB(sh2, a) p32x_sh2_read8(a, sh2)
#define RW(sh2, a) p32x_sh2_read16(a, sh2)
#define RL(sh2, a) p32x_sh2_read32(a, sh2)
#define WB(sh2, a, d) p32x_sh2_write8(a, d, sh2)
#define WW(sh2, a, d) p32x_sh2_write16(a, d, sh2)
#define WL(sh2, a, d) p32x_sh2_write32(a, d, sh2)

#endif

// some stuff from sh2comn.h
#define T	0x00000001
#define S	0x00000002
#define I	0x000000f0
#define Q	0x00000100
#define M	0x00000200

#define AM	0xc7ffffff

#define FLAGS	(M|Q|I|S|T)

#define Rn	((opcode>>8)&15)
#define Rm	((opcode>>4)&15)

#define sh2_state SH2

extern void lprintf(const char *fmt, ...);
#define logerror lprintf

#ifdef SH2_STATS
static SH2 sh2_stats;
static unsigned int op_refs[0x10000];
# define LRN  1
# define LRM  2
# define LRNM (LRN|LRM)
# define rlog(rnm) {   \
  int op = opcode;     \
  if ((rnm) & LRN) {   \
    op &= ~0x0f00;     \
    sh2_stats.r[Rn]++; \
  }                    \
  if ((rnm) & LRM) {   \
    op &= ~0x00f0;     \
    sh2_stats.r[Rm]++; \
  }                    \
  op_refs[op]++;       \
}
# define rlog1(x) sh2_stats.r[x]++
# define rlog2(x1,x2) sh2_stats.r[x1]++; sh2_stats.r[x2]++
#else
# define rlog(x)
# define rlog1(...)
# define rlog2(...)
#endif

#include "sh2.c"

/* RIG_SH2_COUNT: executed-instruction counter for the QEMU M7 feasibility rig
 * (tools/m7_qemu_rig). Never defined in device or libretro builds. */
#ifdef RIG_SH2_COUNT
unsigned long long g_sh2_insns;
#define RIG_SH2_TICK() (g_sh2_insns++)
#else
#define RIG_SH2_TICK() ((void)0)
#endif

#ifndef DRC_CMP

/* GNW_SH2_FASTLOOPS: cycle-exact fast-forward of the two loop shapes that
 * dominate 32X SH-2 time (sweep: DOOM 56% / Kolibri 62% idle in BRA-self
 * spins; VR 50% / Metal Head 11% in DT/BF countdown delays).
 *
 * Both levers reproduce the interpreter's EXACT cycle flow (the WS idle-skip
 * lesson: state-exact != cycle-exact).  Per-instruction costs in THIS
 * interpreter (dispatch charges 1, handlers add extra):
 *     NOP = 1,  DT = 1,  BF taken = 3,  BF not-taken = 1,  BRA = 2.
 * Note the BUSY_LOOP_HACKS blocks in mame/sh2.c are inert here: they compare
 * against the opcode at sh2->ppc, which in this dispatch loop is the
 * executing instruction's OWN address, so the pattern never matches.
 *
 * The skip is capped so sh2->icount stays >= 1 (never crosses the current
 * timeslice) and the last iteration is always interpreted for real.  Within
 * a slice the skipped instructions perform no data access, so no poll
 * detection, no sh2_end_run, no test_irq source exists mid-skip (irqs are
 * posted from memhandlers or between slices only; we bail if test_irq is
 * already pending).  At every slice boundary the architectural state and
 * icount are therefore bit-identical to plain interpretation.
 *
 * Off (#undef) compiles byte-identical to upstream; gnw_sh2_fastloops is a
 * runtime kill-switch for on-device A/B. */
#if defined(GNW_32X_CORE) && !defined(DRC_SH2)
#define GNW_SH2_FASTLOOPS 1
#endif

#ifdef GNW_SH2_FASTLOOPS

#ifndef GNW_SH2_FASTLOOPS_DEFAULT
#define GNW_SH2_FASTLOOPS_DEFAULT 1
#endif
int gnw_sh2_fastloops = GNW_SH2_FASTLOOPS_DEFAULT;

/* per-core negative cache, direct-mapped by PC: insn addresses where
 * detection already failed (backward BF whose body is not NOPs+DT — e.g.
 * comm/VDP poll loops — or BRA-self whose delay slot is not a NOP).  The
 * probe is inlined at the dispatch site so a hot REAL loop (a taken
 * backward BF every iteration) rejects in a few instructions with no call
 * and no re-scan; without that, Kolibri's polls paid +4-5% host for zero
 * skips.  Stale or evicted entries only cost missed skips, never
 * correctness. */
#define GNW_DL_REJ_SLOTS 8	/* 32 bytes per core */
static unsigned int gnw_dl_reject[2][GNW_DL_REJ_SLOTS];
#define GNW_DL_REJ_SLOT(sh2) \
	(&gnw_dl_reject[(sh2)->is_slave & 1][((sh2)->ppc >> 1) & (GNW_DL_REJ_SLOTS - 1)])

/* Fast-forward hook, entered only for a directly-fetched (non-delay-slot)
 * BF with negative displacement or BRA-to-self, with no irq test pending.
 * At entry: ppc = insn address, pc = ppc + 2, delay = 0, icount >= 1. */
static void gnw_sh2_fastloop(SH2 *sh2, UINT32 opcode)
{
	if (opcode == 0xaffe)	/* BRA $ */
	{
		/* idle spin `BRA $; NOP` — burns 3 cycles/iteration (BRA 2 +
		 * delay-slot NOP 1) forever; park by consuming every complete
		 * iteration that fits in the slice, then interpret the last
		 * one for real (exact exit state: pc/ppc/ea/delay/icount). */
		int m;
		if ((UINT32)(UINT16)RW(sh2, sh2->pc) != 0x0009) {
			*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
			return;
		}
		m = (sh2->icount - 1) / 3;
		if (m > 0)
			sh2->icount -= 3 * m;
		return;
	}

	/* countdown delay loop: backward BF whose body is NOPs + exactly one
	 * DT Rn.  One iteration = body (len * 1 cycle) + BF taken (3). */
	{
		int disp = (int)(opcode & 0xff) - 0x100;	/* [-128,-1] */
		unsigned int target = sh2->ppc + disp * 2 + 4;	/* = BF's taken pc */
		unsigned int len = (unsigned int)(-disp) - 2;	/* body insns */
		int dt_reg = -1;
		unsigned int a, k, v;
		int iter_cost, kmax;

		if (sh2->sr & T)	/* final pass: BF won't be taken */
			return;
		if (len - 1 > 3)	/* body of 1..4 insns only */
			return;

		for (a = target, k = 0; k < len; a += 2, k++) {
			UINT32 bop = (UINT32)(UINT16)RW(sh2, a);
			if (bop == 0x0009)			/* NOP */
				continue;
			if ((bop & 0xf0ff) == 0x4010 && dt_reg < 0) {
				dt_reg = (bop >> 8) & 0xf;	/* DT Rn */
				continue;
			}
			dt_reg = -1;
			break;
		}
		if (dt_reg < 0) {
			*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
			return;
		}

		/* T==0 after the body's DT means r[dt_reg] != 0 here */
		v = sh2->r[dt_reg];
		if (v < 2)
			return;
		iter_cost = (int)len + 3;
		kmax = (sh2->icount - 1) / iter_cost;	/* keep icount >= 1 */
		if (kmax > (int)(v - 1))		/* leave the final */
			kmax = (int)(v - 1);		/* iteration real   */
		if (kmax < 1)
			return;
		sh2->r[dt_reg] = v - (unsigned int)kmax;
		sh2->icount -= kmax * iter_cost;
		/* fall through: the pending BF executes for real (still
		 * taken: r[dt_reg] >= 1, T still 0), then the interpreter
		 * finishes the loop or the slice exactly as the unskipped
		 * flow would. */
	}
}

#endif /* GNW_SH2_FASTLOOPS */

int sh2_execute_interpreter(SH2 *sh2, int cycles)
{
	UINT32 opcode;
#ifdef GNW_SH2_FASTLOOPS
	UINT32 gnw_direct;
#endif

	sh2->icount = cycles;

	if (sh2->icount <= 0)
		goto out;

	do
	{
		if (sh2->delay)
		{
			sh2->ppc = sh2->delay;
			opcode = (UINT32)(UINT16)RW(sh2, sh2->delay);

			// TODO: more branch types
			if ((opcode >> 13) == 5) { // BRA/BSR
				sh2->r[15] -= 4;
				WL(sh2, sh2->r[15], sh2->sr);
				sh2->r[15] -= 4;
				WL(sh2, sh2->r[15], sh2->pc);
				sh2->pc = RL(sh2, sh2->vbr + 6 * 4);
				sh2->icount -= 5;
				opcode = 9; // NOP
			}

			sh2->pc -= 2;
#ifdef GNW_SH2_FASTLOOPS
			gnw_direct = 0;
#endif
		}
		else
		{
			sh2->ppc = sh2->pc;
			opcode = (UINT32)(UINT16)RW(sh2, sh2->pc);
#ifdef GNW_SH2_FASTLOOPS
			gnw_direct = 1;
#endif
		}

		sh2->delay = 0;
		sh2->pc += 2;
		RIG_SH2_TICK();

#ifdef GNW_SH2_FASTLOOPS
		/* cheap opcode pre-filter first, then the negative-cache probe
		 * inline (see gnw_dl_reject above), so ordinary hot loops
		 * reject in a few instructions without calling the helper */
		if (((opcode & 0xff80) == 0x8b80 || opcode == 0xaffe)
		    && gnw_direct && *GNW_DL_REJ_SLOT(sh2) != sh2->ppc
		    && !sh2->test_irq && gnw_sh2_fastloops)
			gnw_sh2_fastloop(sh2, opcode);
#endif

		switch (opcode & ( 15 << 12))
		{
		case  0<<12: op0000(sh2, opcode); break;
		case  1<<12: op0001(sh2, opcode); break;
		case  2<<12: op0010(sh2, opcode); break;
		case  3<<12: op0011(sh2, opcode); break;
		case  4<<12: op0100(sh2, opcode); break;
		case  5<<12: op0101(sh2, opcode); break;
		case  6<<12: op0110(sh2, opcode); break;
		case  7<<12: op0111(sh2, opcode); break;
		case  8<<12: op1000(sh2, opcode); break;
		case  9<<12: op1001(sh2, opcode); break;
		case 10<<12: op1010(sh2, opcode); break;
		case 11<<12: op1011(sh2, opcode); break;
		case 12<<12: op1100(sh2, opcode); break;
		case 13<<12: op1101(sh2, opcode); break;
		case 14<<12: op1110(sh2, opcode); break;
		default: op1111(sh2, opcode); break;
		}

		sh2->icount--;

		if (sh2->test_irq && !sh2->delay)
		{
			int level = sh2->pending_level;
			if (level > ((sh2->sr >> 4) & 0x0f))
			{
				int vector = sh2->irq_callback(sh2, level);
				sh2_do_irq(sh2, level, vector);
			}
			sh2->test_irq = 0;
		}
	}
	while (sh2->icount > 0 || sh2->delay);	/* can't interrupt before delay */

out:
	return sh2->icount;
}

#else // if DRC_CMP

int sh2_execute_interpreter(SH2 *sh2, int cycles)
{
	static unsigned int base_pc_[2] = { 0, 0 };
	static unsigned int end_pc_[2] = { 0, 0 };
	static unsigned char op_flags_[2][BLOCK_INSN_LIMIT];
	unsigned int *base_pc = &base_pc_[sh2->is_slave];
	unsigned int *end_pc = &end_pc_[sh2->is_slave];
	unsigned char *op_flags = op_flags_[sh2->is_slave];
	unsigned int pc_expect;
	UINT32 opcode;

	sh2->icount = sh2->cycles_timeslice = cycles;

	if (sh2->pending_level > ((sh2->sr >> 4) & 0x0f))
	{
		int level = sh2->pending_level;
		int vector = sh2->irq_callback(sh2, level);
		sh2_do_irq(sh2, level, vector);
	}
	pc_expect = sh2->pc;

	if (sh2->icount <= 0)
		goto out;

	do
	{
		if (!sh2->delay) {
			if (sh2->pc < *base_pc || sh2->pc >= *end_pc) {
				*base_pc = sh2->pc;
				scan_block(*base_pc, sh2->is_slave,
					op_flags, end_pc, NULL, NULL);
			}
			if ((op_flags[(sh2->pc - *base_pc) / 2]
				& OF_BTARGET) || sh2->pc == *base_pc
				|| pc_expect != sh2->pc) // branched
			{
				pc_expect = sh2->pc;
				if (sh2->icount <= 0)
					break;
			}

			do_sh2_trace(sh2, sh2->icount);
		}
		pc_expect += 2;

		if (sh2->delay)
		{
			sh2->ppc = sh2->delay;
			opcode = (UINT32)(UINT16)RW(sh2, sh2->delay);
			sh2->pc -= 2;
		}
		else
		{
			sh2->ppc = sh2->pc;
			opcode = (UINT32)(UINT16)RW(sh2, sh2->pc);
		}

		sh2->delay = 0;
		sh2->pc += 2;
		RIG_SH2_TICK();

		switch (opcode & ( 15 << 12))
		{
		case  0<<12: op0000(sh2, opcode); break;
		case  1<<12: op0001(sh2, opcode); break;
		case  2<<12: op0010(sh2, opcode); break;
		case  3<<12: op0011(sh2, opcode); break;
		case  4<<12: op0100(sh2, opcode); break;
		case  5<<12: op0101(sh2, opcode); break;
		case  6<<12: op0110(sh2, opcode); break;
		case  7<<12: op0111(sh2, opcode); break;
		case  8<<12: op1000(sh2, opcode); break;
		case  9<<12: op1001(sh2, opcode); break;
		case 10<<12: op1010(sh2, opcode); break;
		case 11<<12: op1011(sh2, opcode); break;
		case 12<<12: op1100(sh2, opcode); break;
		case 13<<12: op1101(sh2, opcode); break;
		case 14<<12: op1110(sh2, opcode); break;
		default: op1111(sh2, opcode); break;
		}

		sh2->icount--;

		if (sh2->test_irq && !sh2->delay && sh2->pending_level > ((sh2->sr >> 4) & 0x0f))
		{
			int level = sh2->pending_level;
			int vector = sh2->irq_callback(sh2, level);
			sh2_do_irq(sh2, level, vector);
			sh2->test_irq = 0;
		}

	}
	while (1);

out:
	return sh2->icount;
}

#endif // DRC_CMP

#ifdef SH2_STATS
#include <stdio.h>
#include <string.h>
#include "sh2dasm.h"

void sh2_dump_stats(void)
{
	static const char *rnames[] = {
		"R0", "R1", "R2",  "R3",  "R4",  "R5",  "R6",  "R7",
		"R8", "R9", "R10", "R11", "R12", "R13", "R14", "SP",
		"PC", "", "PR", "SR", "GBR", "VBR", "MACH", "MACL"
	};
	long long total;
	char buff[64];
	int u, i;

	// dump reg usage
	total = 0;
	for (i = 0; i < 24; i++)
		total += sh2_stats.r[i];

	for (i = 0; i < 24; i++) {
		if (i == 16 || i == 17 || i == 19)
			continue;
		printf("r %6.3f%% %-4s %9d\n", (double)sh2_stats.r[i] * 100.0 / total,
			rnames[i], sh2_stats.r[i]);
	}

	memset(&sh2_stats, 0, sizeof(sh2_stats));

	// dump ops
	printf("\n");
	total = 0;
	for (i = 0; i < 0x10000; i++)
		total += op_refs[i];

	for (u = 0; u < 16; u++) {
		int max = 0, op = 0;
		for (i = 0; i < 0x10000; i++) {
			if (op_refs[i] > max) {
				max = op_refs[i];
				op = i;
			}
		}
		DasmSH2(buff, 0, op);
		printf("i %6.3f%% %9d %s\n", (double)op_refs[op] * 100.0 / total,
			op_refs[op], buff);
		op_refs[op] = 0;
	}
	memset(op_refs, 0, sizeof(op_refs));
}
#endif

