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

/* GNW: inline SDRAM fast path for opcode fetch.  32X code lives in SDRAM, so
 * this avoids the cross-TU p32x_sh2_read16 call (LTO is disabled) on every
 * fetched instruction.  Identical addressing to the read16_map SDRAM entry
 * (p_sdram + (a & 0x3fffe)); the 0xdf mask ignores the cache-through bit
 * 0x20000000, matching read16_map indices 0x06/0x26. */
/* This used to expand to plain RW() -- i.e. exactly the cross-TU call the
 * comment says it avoids -- so the described fast path was documented but
 * never implemented. p32x_sh2_read16() does already short-circuit SDRAM
 * internally, so what the call actually cost was the call itself: bl +
 * prologue/epilogue across a TU boundary with LTO off, on EVERY fetched
 * guest instruction. Device measurement puts the SH-2 interpreter at ~100
 * cycles per guest instruction with msh2 at 59.5% of the frame, so this is
 * a small, bounded win -- not a fix for the dominant cost, which is D-cache
 * pressure from a 256 KB SDRAM working set against a 16 KB D-cache.
 *
 * The condition and the load below are copied verbatim from
 * p32x_sh2_read16()'s own SDRAM branch (pico/32x/memory.c), so a fetch
 * resolves to the identical byte either way; anything outside SDRAM still
 * goes through the full call. */
extern unsigned int gnw_sh2_rom_fetch_mask;

/* Cart ROM gets the same treatment (0726).  The SDRAM-only fast path above
 * was written for "32X code lives in SDRAM", which is true of the 32X's own
 * boot copy but not of the games: the device pcwall probe puts Doom's msh2 at
 * sdram 0.0% / cart-ROM 100%, so every one of its ~173 k fetches per frame was
 * still paying the cross-TU call.  gnw_sh2_rom_fetch_mask (pico/32x/memory.c)
 * is the CS1 read16 map entry's own mask, or 0 when that entry is a handler
 * (SSF2 banking) -- so the fast path resolves to the identical byte the map
 * would, and disables itself when the map is not plain memory.
 *
 * Both region tests fold the cache-through mirror with one AND: 0x26->0x06 and
 * 0x22->0x02 under 0xdf000000, and no other CS aliases onto them. */
#define GNW_FETCH_SD(sh2, addr)                                              \
  ((((addr) & 0xdf000000) == 0x06000000)                                     \
     ? (UINT32)*(UINT16 *)((UINT8 *)(sh2)->p_sdram + ((addr) & 0x3fffe))     \
   : ((((addr) & 0xdf000000) == 0x02000000) && gnw_sh2_rom_fetch_mask)       \
     ? (UINT32)*(UINT16 *)((UINT8 *)(sh2)->p_rom                             \
                           + ((addr) & gnw_sh2_rom_fetch_mask))              \
     : (UINT32)(UINT16)RW(sh2, addr))

#ifdef MD32X_DEVICE_PROFILE
/* Guest DATA-access cost probe -- companion to the opcode-fetch probe below.
 *
 * The fetch probe answered its question (10.3-10.4 cycles, ~10-12% of a core's
 * wall, stable across two very different scenes -- so external-flash latency is
 * NOT what makes a guest instruction cost ~86 device cycles).  What is left is
 * data accesses and the interpreter's own decode/execute.  This brackets the
 * data side the same way, so the two per-event costs plus the instruction count
 * account for the wall without a single scene-matched A/B -- which matters,
 * because cyc/insn itself moved 101.6 -> 85.7 between two device dumps purely
 * because the busier scene had better locality and longer slices.
 *
 * Deliberately a call, not an inline expansion: RB/RW/RL/WB/WW/WL appear at
 * ~50 sites inside an 8 KB interpreter that lives in ITCM with 4 KB of head
 * room, and inlining a bracket at each of them would not fit.  The cost of
 * that extra call inflates the PROFILE build's cyc/insn (it is not in the
 * release build), but it sits OUTSIDE the bracket, so the number this reports
 * -- what one guest load or store really costs -- is unaffected.
 *
 * Prime period, and a different prime from the fetch probe's, so neither
 * aliases a tight guest loop nor samples in lockstep with the other. */
#define GNW_DA_PERIOD 29
extern int gnw_pcwall_armed;
unsigned int gnw_da_cyc[2], gnw_da_n[2];    /* reads,  per core */
unsigned int gnw_daw_cyc[2], gnw_daw_n[2];  /* writes, per core */
static int gnw_da_cnt[2], gnw_daw_cnt[2];

static UINT32 __attribute__((noinline)) gnw_probe_rd(SH2 *sh2, UINT32 a, int k)
{
	int c = sh2->is_slave & 1;
	unsigned int t0;
	UINT32 v;

	if (--gnw_da_cnt[c] > 0)
		return k == 0 ? p32x_sh2_read8(a, sh2)
		     : k == 1 ? p32x_sh2_read16(a, sh2)
		              : p32x_sh2_read32(a, sh2);
	t0 = *(volatile unsigned int *)0xE0001004;
	v = k == 0 ? p32x_sh2_read8(a, sh2)
	  : k == 1 ? p32x_sh2_read16(a, sh2)
	           : p32x_sh2_read32(a, sh2);
	gnw_da_cyc[c] += *(volatile unsigned int *)0xE0001004 - t0;
	gnw_da_n[c]++;
	gnw_da_cnt[c] = gnw_pcwall_armed ? GNW_DA_PERIOD : (1 << 30);
	return v;
}

static void __attribute__((noinline)) gnw_probe_wr(SH2 *sh2, UINT32 a, UINT32 d, int k)
{
	int c = sh2->is_slave & 1;
	unsigned int t0;

	if (--gnw_daw_cnt[c] > 0) {
		if (k == 0)      p32x_sh2_write8(a, d, sh2);
		else if (k == 1) p32x_sh2_write16(a, d, sh2);
		else             p32x_sh2_write32(a, d, sh2);
		return;
	}
	t0 = *(volatile unsigned int *)0xE0001004;
	if (k == 0)      p32x_sh2_write8(a, d, sh2);
	else if (k == 1) p32x_sh2_write16(a, d, sh2);
	else             p32x_sh2_write32(a, d, sh2);
	gnw_daw_cyc[c] += *(volatile unsigned int *)0xE0001004 - t0;
	gnw_daw_n[c]++;
	gnw_daw_cnt[c] = gnw_pcwall_armed ? GNW_DA_PERIOD : (1 << 30);
}

/* GNW_FETCH_SD's fallback arm uses RW(), so a fetch that lands outside SDRAM
 * and ROM would book itself as a data read. Both cores measure 0.0% outside
 * those two regions, so this cannot bias anything in practice. */
#undef RB
#undef RW
#undef RL
#undef WB
#undef WW
#undef WL
#define RB(sh2, a) gnw_probe_rd(sh2, a, 0)
#define RW(sh2, a) gnw_probe_rd(sh2, a, 1)
#define RL(sh2, a) gnw_probe_rd(sh2, a, 2)
#define WB(sh2, a, d) gnw_probe_wr(sh2, a, d, 0)
#define WW(sh2, a, d) gnw_probe_wr(sh2, a, d, 1)
#define WL(sh2, a, d) gnw_probe_wr(sh2, a, d, 2)
#endif /* MD32X_DEVICE_PROFILE */

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

/* MD32X_DEVICE_PROFILE: per-core guest instruction counters, device-only.
 * The msh2/ssh2 DWT buckets (main_md32x.c) already give real cycles spent
 * per core; dividing by these gives cycles-per-guest-instruction, the number
 * that tells whether a core's cost is dispatch overhead (ratio close to the
 * other core's) or memory-stall-bound (ratio far higher — e.g. XIP/cache
 * misses fetching game code straight out of external flash, see cart.c's
 * GNW_32X_CORE zero-copy binding). Never defined in the release build. */
#ifdef MD32X_DEVICE_PROFILE
unsigned long long gnw_sh2_insn_count[2];	/* [0]=master [1]=slave */
#define GNW_SH2_INSN_TICK(sh2) (gnw_sh2_insn_count[(sh2)->is_slave & 1]++)

/* Sampled guest-PC wall attribution (device DWT cycles, per core).
 *
 * Answers the question the QEMU histogram cannot (instructions != device
 * cycles): which guest-PC REGION owns the msh2/ssh2 wall — the ROM render
 * loops the histogram flagged, other ROM code, or SDRAM code. Every
 * GNW_PCWALL_PERIOD dispatched instructions the probe reads DWT_CYCCNT and
 * attributes the delta since the previous sample (of the same core, within
 * the same interpreter slice — the stamp resets at slice entry, so time in
 * the other core / 68K / VDP never leaks in) to a bucket chosen by the
 * current ppc:
 *   - a page histogram over a ROM window whose base and page size are
 *     defines, so each pass can widen to re-find the hot set or narrow onto
 *     it (see the aiming history below; mass landing in rom_hi is the
 *     signal to slide the window),
 *   - whole-region sums for ROM-above-window, SDRAM, and everything else.
 * The porting layer (md32x_profile.c) freezes gnw_pcwall_armed and prints
 * shares in the one-shot /32x_dwt.txt dump; the disarmed per-insn cost
 * collapses to a counter decrement that never reaches zero. uint32 bucket
 * sums are safe: armed only from init to the ~64-frame dump (< 1 G cycles
 * total, far below wrap).
 *
 * MEMORY PLACEMENT: the bucket tables live in a caller-provided AHB block
 * (see gnw_sh2_pcwall_arm), NOT in this file's BSS — the MD32X overlay BSS
 * was already within ~300 B of __RAM_EMU_END__ when MD32X_DEVICE_PROFILE
 * first met the merged testbed tree, and 536 B of static tables tipped it
 * over (link-time "MD32X BSS overflow"). Only the per-insn-hot countdown /
 * stamp words stay here (~44 B). The tables are touched every 32 insns
 * from the cold sample path, where an AHB access is irrelevant. */
#define GNW_PCWALL_PERIOD    32
/* Aiming history (Doom, device):
 * pass 1 (SHIFT=10, BASE=0): rom<64K 0.0%, rom_hi 94.9%/99.9% — the
 *   QEMU-flagged first-64K loops are cold on device.
 * pass 2 (SHIFT=16, BASE=0, whole 4 MB): msh2 = 77.6% page 0x030000 +
 *   17.3% page 0x040000; ssh2 = 84.6% page 0x040000 + 15.3% 0x030000.
 *   The hot set is two adjacent 64 KB pages, split by core.
 * pass 3 (2 KB x 64 over 0x030000..0x04ffff): title-scene only — the
 *   boot-anchored window profiled the logo. Discarded once the porting
 *   layer learned to open the window after 1200 warmup frames.
 * pass 4 (same geometry, gameplay window): msh2 = 37.7% page 0x04d800 +
 *   34.5% 0x049000; ssh2 = 70.5% page 0x04e800. Disassembly: 0x049000 is
 *   real render math; 0x04d800 mixes crt0 (COMM "68UP"/"S_OK" boot
 *   handshake polls + 256K ROM->SDRAM copy), the IRQ prologue, a SLEEP
 *   idle and the VInt ISR; 0x04e800 holds a TAS spinlock. Page resolution
 *   cannot split poll/memcpy/ISR/SLEEP costs.
 * pass 5: 128 B pages x 64 = 0x04d000..0x04efff — instruction-run
 *   resolution. Result: msh2 35.0% in the SINGLE page 0x0204df00, the
 *   master's frame-wait spin (see gnw_sh2_fastloop's BT/S case); ssh2
 *   63.6% in 0x0204e800, a TAS spinlock.
 * pass 6 (same geometry, with the BT/S fold shipped): 0x0204df00 fell to
 *   0.2% — the fold fires — and msh2's wall moved out from under the
 *   window: rom_win 11.0%, rom_hi 88.9%, sdram 0.0%.  Frame wall -18.7%,
 *   msh2 -27.3% against pass 5 on the same scene.
 * pass 7 (current): 16 KB pages x 64 = ROM 0x000000..0x0fffff.  Re-widened
 *   to re-find the post-fold hot set (pass 2 put all of Doom's mass in the
 *   0x030000/0x040000 64 KB pages, so 1 MB covers it with rom_hi as the
 *   escape hatch), at 4x the resolution of the pass-2 map. */
#define GNW_PCWALL_PAGE_SHIFT 14                  /* 16=64K, 14=16K, 7=128B */
#define GNW_PCWALL_WIN_BASE  0x00000000u          /* offset into ROM */
#define GNW_PCWALL_NBUCK     64
#define GNW_PCWALL_WIN_SIZE  ((unsigned int)GNW_PCWALL_NBUCK << GNW_PCWALL_PAGE_SHIFT)
enum { GNW_PCWALL_ROM_HI = 0, GNW_PCWALL_SDRAM, GNW_PCWALL_OTHER,
       GNW_PCWALL_NREGION };
/* Caller-provided block word count: [core0 hist][core1 hist][core0 regions]
 * [core1 regions] — md32x_profile.c allocates exactly this many uint32. */
#define GNW_PCWALL_BLOCK_WORDS (2 * GNW_PCWALL_NBUCK + 2 * GNW_PCWALL_NREGION)
int gnw_pcwall_armed;                             /* porting layer clears  */
const unsigned int gnw_pcwall_win_base = GNW_PCWALL_WIN_BASE; /* for the dump */
const unsigned int gnw_pcwall_nbuck = GNW_PCWALL_NBUCK;       /* for the dump */
const unsigned int gnw_pcwall_page_shift = GNW_PCWALL_PAGE_SHIFT; /* for the dump */
const unsigned int gnw_pcwall_block_words = GNW_PCWALL_BLOCK_WORDS; /* alloc size */
unsigned int *gnw_pcwall_hist_p[2];               /* cycles, ROM window    */
unsigned int *gnw_pcwall_region_p[2];             /* cycles, coarse        */
unsigned int gnw_pcwall_samples[2];
static int gnw_pcwall_cnt[2];
static unsigned int gnw_pcwall_last[2];

/* Opcode-fetch cost probe, and the interpreter slice counter.
 *
 * pcwall says WHERE the wall is; the memory-dispatch ledger said it is not in
 * the write path (1.1% of msh2).  What is left in the 101-134 cycles per
 * dispatched guest instruction is fetch + dispatch + op body, and only the
 * fetch can be a memory stall: the cart ROM is XIP out of external flash
 * (diag "rom cached: addr=0x93210000") behind a 16 KB D-cache that the 256 KB
 * SDRAM working set is also thrashing.  This brackets one fetch in every
 * GNW_FETCH_PERIOD with DWT_CYCCNT so the dump can say what a fetch really
 * costs.  Two readings decide the next lever: a few cycles means the fetch is
 * cached and the cost is interpreter dispatch (specialise the hot ops); tens
 * of cycles means it is flash latency (cache ROM in RAM, and the RAM budget
 * question has to be reopened).
 *
 * The period is deliberately PRIME: a power of two aliases against tight
 * guest loops (a 4-instruction loop sampled every 32 would sample the same
 * instruction for ever) and against the 16-halfword D-cache line, so the
 * sampled fetch would never be the line-filling one.
 *
 * Bias: only the non-delay-slot fetch site is instrumented (one site instead
 * of two, for overlay text -- the MD32X overlay has ~100 B of margin), but
 * every loop iteration fetches exactly one opcode, so total fetches equals
 * the dispatched-instruction count either way.  The CYCCNT pair itself costs
 * ~10 cycles; gnw_fetch_ovh_x8 is 8 back-to-back empty pairs measured at arm
 * time, for the dump to subtract.
 *
 * The slice counter is the control: picodrive syncs the SH-2s every STEP_N
 * (976) 68K cycles, ~131 slices/frame, but p32x_sh2_poll_detect can force
 * early syncs.  If slices/frame ran into the thousands, per-slice overhead —
 * not per-instruction cost — would be the disease, and insns/slice names it. */
#define GNW_FETCH_PERIOD 31
#define GNW_DWTC (*(volatile unsigned int *)0xE0001004)
unsigned int gnw_fetch_cyc[2];      /* summed bracketed deltas, per core */
unsigned int gnw_fetch_n[2];        /* bracketed fetches, per core       */
unsigned int gnw_fetch_ovh_x8;      /* 8 empty CYCCNT pairs, calibration */
unsigned int gnw_sh2_slices[2];     /* sh2_execute_interpreter entries   */
static int gnw_fetch_cnt[2];

#define GNW_FETCH(sh2, addr, dst) do {                                       \
	int c_ = (sh2)->is_slave & 1;                                        \
	if (--gnw_fetch_cnt[c_] > 0) {                                       \
		(dst) = GNW_FETCH_SD(sh2, addr);                             \
	} else {                                                             \
		unsigned int t0_ = GNW_DWTC;                                 \
		(dst) = GNW_FETCH_SD(sh2, addr);                             \
		gnw_fetch_cyc[c_] += GNW_DWTC - t0_;                         \
		gnw_fetch_n[c_]++;                                           \
		gnw_fetch_cnt[c_] = gnw_pcwall_armed ? GNW_FETCH_PERIOD      \
		                                     : (1 << 30);            \
	}                                                                    \
} while (0)
#define GNW_SLICE_TICK(sh2) (gnw_sh2_slices[(sh2)->is_slave & 1]++)

static void __attribute__((noinline)) gnw_pcwall_sample(SH2 *sh2)
{
	unsigned int now = *(volatile unsigned int *)0xE0001004; /* DWT_CYCCNT */
	int core = sh2->is_slave & 1;

	if (!gnw_pcwall_armed) {
		gnw_pcwall_cnt[core] = 1 << 30;   /* disarmed: never resample */
		return;
	}
	gnw_pcwall_cnt[core] = GNW_PCWALL_PERIOD;

	unsigned int d = now - gnw_pcwall_last[core];
	gnw_pcwall_last[core] = now;
	gnw_pcwall_samples[core]++;

	unsigned int a = sh2->ppc & 0x1fffffff;   /* fold cache-through mirror */
	if (a - 0x02000000u < 0x400000u) {        /* 32X ROM, 4 MB */
		unsigned int off = a - 0x02000000u - GNW_PCWALL_WIN_BASE;
		if (off < GNW_PCWALL_WIN_SIZE)
			gnw_pcwall_hist_p[core][off >> GNW_PCWALL_PAGE_SHIFT] += d;
		else
			gnw_pcwall_region_p[core][GNW_PCWALL_ROM_HI] += d;
	} else if (a - 0x06000000u < 0x40000u) {  /* SDRAM, 256 KB */
		gnw_pcwall_region_p[core][GNW_PCWALL_SDRAM] += d;
	} else {
		gnw_pcwall_region_p[core][GNW_PCWALL_OTHER] += d;
	}
}

/* Porting-layer entry point: arm (or re-arm) the probe. `block` is a zeroed
 * uint32[gnw_pcwall_block_words] the caller owns (AHB — see MEMORY
 * PLACEMENT above); NULL leaves the probe disarmed. Must reset the
 * countdowns — a tick that fired while disarmed parks its counter at 1<<30,
 * and flipping gnw_pcwall_armed alone would leave that core asleep. */
void gnw_sh2_pcwall_arm(unsigned int *block)
{
	unsigned int now = *(volatile unsigned int *)0xE0001004;

	if (block == NULL)
		return;
	gnw_pcwall_hist_p[0]   = block;
	gnw_pcwall_hist_p[1]   = block + GNW_PCWALL_NBUCK;
	gnw_pcwall_region_p[0] = block + 2 * GNW_PCWALL_NBUCK;
	gnw_pcwall_region_p[1] = block + 2 * GNW_PCWALL_NBUCK + GNW_PCWALL_NREGION;
	gnw_pcwall_last[0] = gnw_pcwall_last[1] = now;
	gnw_pcwall_cnt[0] = gnw_pcwall_cnt[1] = GNW_PCWALL_PERIOD;

	/* fetch probe: calibrate the CYCCNT-pair self-cost with 8 empty pairs,
	 * then open its countdown on the same instant as everything else */
	{
		unsigned int i, s = 0;
		for (i = 0; i < 8; i++) {
			unsigned int a = GNW_DWTC;
			unsigned int b = GNW_DWTC;
			s += b - a;
		}
		gnw_fetch_ovh_x8 = s;
	}
	gnw_fetch_cyc[0] = gnw_fetch_cyc[1] = 0;
	gnw_fetch_n[0] = gnw_fetch_n[1] = 0;
	gnw_fetch_cnt[0] = gnw_fetch_cnt[1] = GNW_FETCH_PERIOD;
	gnw_sh2_slices[0] = gnw_sh2_slices[1] = 0;
	gnw_da_cyc[0] = gnw_da_cyc[1] = gnw_da_n[0] = gnw_da_n[1] = 0;
	gnw_daw_cyc[0] = gnw_daw_cyc[1] = gnw_daw_n[0] = gnw_daw_n[1] = 0;
	gnw_da_cnt[0] = gnw_da_cnt[1] = GNW_DA_PERIOD;
	gnw_daw_cnt[0] = gnw_daw_cnt[1] = GNW_DA_PERIOD;

	gnw_pcwall_armed = 1;
}

/* Slice entry: re-stamp so the delta of the first in-slice sample cannot
 * span the other core's slice / 68K / VDP time. */
#define GNW_PCWALL_ENTER(sh2) \
	(gnw_pcwall_last[(sh2)->is_slave & 1] = *(volatile unsigned int *)0xE0001004)
#define GNW_PCWALL_TICK(sh2) \
	((--gnw_pcwall_cnt[(sh2)->is_slave & 1] <= 0) ? gnw_pcwall_sample(sh2) : (void)0)

#else
#define GNW_SH2_INSN_TICK(sh2) ((void)0)
#define GNW_PCWALL_ENTER(sh2) ((void)0)
#define GNW_PCWALL_TICK(sh2) ((void)0)
#define GNW_FETCH(sh2, addr, dst) ((dst) = GNW_FETCH_SD(sh2, addr))
#define GNW_SLICE_TICK(sh2) ((void)0)
#endif

/* RIG_SH2_PC_HIST: SH-2 guest-PC histogram for the QEMU M7 feasibility rig.
 * Two sparse open-addressed tables (master/slave), keyed by ppc, counting
 * direct vs delay-slot executions. Reveals which guest loops eat the msh2/
 * ssh2 phase — fastloop-off shows loops fastloop already kills, fastloop-on
 * shows the residual hot set. Never compiled in device/libretro builds.
 * rig_32x.c reads the table (non-static) and prints the top-N report. */
#ifdef RIG_SH2_PC_HIST
#define RIG_PC_HIST_SLOTS 8192
struct rig_pc_slot {
	unsigned int pc;
	unsigned int occupied;
	unsigned short opcode;
	unsigned long long dir;
	unsigned long long dly;
};
struct rig_pc_slot rig_pchist[2][RIG_PC_HIST_SLOTS];

static void rig_pchist_tick(SH2 *sh2, int is_delay, unsigned short opcode)
{
	unsigned core = sh2->is_slave & 1;
	unsigned int pc = sh2->ppc;
	unsigned start = (pc >> 1) & (RIG_PC_HIST_SLOTS - 1);
	for (unsigned i = 0; i < RIG_PC_HIST_SLOTS; i++) {
		unsigned s = (start + i) & (RIG_PC_HIST_SLOTS - 1);
		struct rig_pc_slot *e = &rig_pchist[core][s];
		if (!e->occupied) {
			e->occupied = 1; e->pc = pc; e->opcode = opcode;
			e->dir = is_delay ? 0 : 1; e->dly = is_delay ? 1 : 0;
			return;
		}
		if (e->pc == pc) {
			if (is_delay) e->dly++; else e->dir++;
			return;
		}
	}
	/* table full — extremely unlikely with 8192 slots; sample silently */
}
#define RIG_PC_HIST_TICK(sh2, is_delay, op) rig_pchist_tick(sh2, is_delay, op)
#else
#define RIG_PC_HIST_TICK(sh2, is_delay, op) ((void)0)
#endif

/* RIG_POLL_PEEK: diagnostic for the QEMU M7 rig. On the first visit to each
 * backward-branch site (BF/BFS/BT/BTS with negative disp8), snapshot the full
 * register file + gbr so the rig can resolve each spin loop's poll address
 * and classify its memory region. Never compiled in device/libretro builds. */
#ifdef RIG_POLL_PEEK
struct rig_peek_entry {
	unsigned int pc;
	unsigned short op;
	int core;
	unsigned int r[16];
	unsigned int gbr, vbr, sr;
};
struct rig_peek_entry rig_peek_log[128];
unsigned int rig_peek_seen[256];
int rig_peek_n = 0;

void rig_poll_peek(SH2 *sh2, UINT32 opcode)
{
	unsigned int pc = sh2->ppc;
	unsigned slot = (pc >> 1) & 255;
	if (rig_peek_seen[slot] == pc) return;
	if (rig_peek_n >= 128) return;
	rig_peek_seen[slot] = pc;
	struct rig_peek_entry *e = &rig_peek_log[rig_peek_n++];
	e->pc = pc; e->op = (unsigned short)opcode; e->core = sh2->is_slave & 1;
	for (int i = 0; i < 16; i++) e->r[i] = sh2->r[i];
	e->gbr = sh2->gbr; e->vbr = sh2->vbr; e->sr = sh2->sr;
}

static inline void rig_poll_peek_check(SH2 *sh2, UINT32 opcode)
{
	unsigned btop = opcode & 0xff00;
	if (btop != 0x8b00 && btop != 0x8f00
	    && btop != 0x8900 && btop != 0x8d00) return;
	int disp8 = (int)(signed char)(opcode & 0xff);
	if (disp8 >= 0) return;
	rig_poll_peek(sh2, opcode);
}
#define RIG_POLL_PEEK_HOOK(sh2, op) rig_poll_peek_check(sh2, op)
#else
#define RIG_POLL_PEEK_HOOK(sh2, op) ((void)0)
#endif

/* RIG_SDRAM_POLL_DIAG: counters + samples for the SDRAM poll case of
 * gnw_sh2_fastloop. Off => byte-identical (no code emitted). */
#ifdef RIG_SDRAM_POLL_DIAG
struct rig_spd_sample { unsigned int pc, bop1, bop2, pa; };
#define RIG_SPD_LOG_N 64
struct rig_spd_sample rig_spd_log[RIG_SPD_LOG_N];
volatile unsigned int rig_spd_tries, rig_spd_hits;
volatile unsigned int rig_spd_bad_bop, rig_spd_bad_addr;
volatile unsigned int rig_spd_addr_06, rig_spd_addr_00, rig_spd_addr_02;
volatile unsigned int rig_spd_addr_22, rig_spd_addr_40, rig_spd_addr_other;
volatile unsigned int rig_spd_log_n = 0;
static inline void rig_spd_sample(unsigned int pc, unsigned int bop1,
		unsigned int bop2, unsigned int pa) {
	unsigned int i = rig_spd_log_n;
	if (i < RIG_SPD_LOG_N) {
		rig_spd_log[i].pc = pc;
		rig_spd_log[i].bop1 = bop1;
		rig_spd_log[i].bop2 = bop2;
		rig_spd_log[i].pa = pa;
		rig_spd_log_n = i + 1;
	}
}
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

/* BRA-self idle-skip using scheduler SLEEP state. Default OFF — only
 * safe for verified ROMs. Set to 1 by main_md32x.c via CRC whitelist. */
#ifndef GNW_SH2_IDLE_SKIP_DEFAULT
#define GNW_SH2_IDLE_SKIP_DEFAULT 0
#endif
int gnw_sh2_idle_skip = GNW_SH2_IDLE_SKIP_DEFAULT;

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
		if (gnw_sh2_idle_skip) {
			/* Whitelisted ROM: set SLEEP instead of burning icount.
			 * The scheduler skips sleeping SH-2s and advances their
			 * m68krcycles_done to target — same cycle accounting as
			 * burning the slice, but zero host cost.
			 * Wake: sh2_internal_irq clears SLEEP on timer/VBlank/hint;
			 * p32x_sh2_poll_event clears SLEEP at VBlank (IDLE_STATES). */
			sh2->state |= SH2_STATE_SLEEP;
			sh2->icount = 0;
			return;
		}
		m = (sh2->icount - 1) / 3;
		if (m > 0)
			sh2->icount -= 3 * m;
		return;
	}

	/* SDRAM poll loop: backward BT/BF whose body is exactly
	 *   MOV.W @Rm,Rn      (0x6nm1)   or
	 *   MOV.W @(disp4,Rm),R0 (0x85dm)
	 * followed by TST Rn,Rm (0x2nm8) with TST dest == MOV.W dest.
	 * The SH-2 spins on a shared SDRAM slot (cache-through bit 0x20000000
	 * stripped) waiting for the other core to write it.  Dominant
	 * ssh2/msh2 hot path on Kolibri 32X (~86% / 12% of guest insns) and
	 * the same shape recurs on Metal Head / Tempo.
	 *
	 * Each iteration re-reads the slot (real side effect — the other
	 * core's write must be visible) and recomputes T.  iter_cost = 5
	 * (MOV.W 1 + TST 1 + BT/BF taken 3).  On exit (polled bit changed)
	 * T is set so the real BT/BF falls through; R[dest] holds the last
	 * read.  If the slice ends still looping, T is left at the loop
	 * value so the real BT/BF branches back and the scheduler runs the
	 * producer on the next slice.  No RPOLL/SLEEP state — deadlock-free. */
	if ((opcode & 0xff00) == 0x8900 || (opcode & 0xff00) == 0x8b00)	/* BT / BF */
	{
		int disp8 = (int)(signed char)(opcode & 0xff);
		int is_bt = (opcode & 0xff00) == 0x8900;
#ifdef RIG_SDRAM_POLL_DIAG
		rig_spd_tries++;
#endif
		if (disp8 < 0)
		{
			unsigned int target = sh2->ppc + disp8 * 2 + 4;
			if (target + 4 == sh2->ppc
			    && (target & 0xc6000000) == 0x06000000)	/* body 2 insns, in SDRAM (RW() on a sysreg/comm target would fire poll_detect and corrupt the guest's poll state) */
			{
				UINT32 bop1 = (UINT32)(UINT16)RW(sh2, target);
				UINT32 bop2 = (UINT32)(UINT16)RW(sh2, target + 2);
				int dest_reg = -1, base_reg = -1, mask_reg;
				unsigned int pa;
				int self_test;
				UINT32 mask;

				if ((bop1 & 0xf00f) == 0x6001) {		/* MOV.W @Rm,Rn */
					dest_reg = (bop1 >> 8) & 0xf;
					base_reg = (bop1 >> 4) & 0xf;
					pa = sh2->r[base_reg];
				} else if ((bop1 & 0xff00) == 0x8500) {	/* MOV.W @(disp4,Rm),R0 */
					/* encoding 1000 0101 dddd mmmm: bits 7-4 = disp, bits 3-0 = Rm */
					dest_reg = 0;
					base_reg = bop1 & 0xf;
					pa = sh2->r[base_reg] + ((unsigned int)((bop1 >> 4) & 0xf)) * 2;
				}
#ifdef RIG_SDRAM_POLL_DIAG
				if (dest_reg < 0
				    || (bop2 & 0xf00f) != 0x2008
				    || ((bop2 >> 8) & 0xf) != (unsigned)dest_reg) {
					rig_spd_bad_bop++;
					rig_spd_sample(sh2->ppc, bop1, bop2, 0);
				}
#endif
				if (dest_reg >= 0
				    && (bop2 & 0xf00f) == 0x2008		/* TST Rm,Rn */
				    && ((bop2 >> 8) & 0xf) == (unsigned)dest_reg) {
					mask_reg = (bop2 >> 4) & 0xf;
					self_test = (mask_reg == dest_reg);
					pa &= ~0x20000000;			/* strip cache-through */
#ifdef RIG_SDRAM_POLL_DIAG
					if ((pa & 0xff000000) != 0x06000000) {
						rig_spd_bad_addr++;
						rig_spd_sample(sh2->ppc, bop1, bop2, pa);
						if ((pa & 0xff000000) == 0x06000000) rig_spd_addr_06++;
						else if ((pa & 0xff000000) == 0x00000000 || (pa & 0xff000000) == 0x20000000) rig_spd_addr_00++;
						else if ((pa & 0xff000000) == 0x02000000 || (pa & 0xff000000) == 0x22000000) rig_spd_addr_02++;
						else if ((pa & 0xff000000) == 0x22000000) rig_spd_addr_22++;
						else if ((pa & 0xff000000) == 0x40000000) rig_spd_addr_40++;
						else rig_spd_addr_other++;
					}
#endif
					if ((pa & 0xff000000) == 0x06000000) {	/* SDRAM */
						mask = self_test ? 0xffff : sh2->r[mask_reg];
#ifdef RIG_SDRAM_POLL_DIAG
						rig_spd_hits++;
						rig_spd_sample(sh2->ppc, bop1, bop2, pa);
#endif
						/* BT taken (T==1) loops; BF taken (T==0) loops.
						 * TST: T = ((val & mask) == 0). */
						int want_t_loop = is_bt ? 1 : 0;
						while (sh2->icount >= 5) {
							unsigned int val = (UINT32)(UINT16)RW(sh2, pa);
							int t = ((val & mask) == 0) ? 1 : 0;
							sh2->r[dest_reg] = val;	/* MOV.W dest */
							if (t != want_t_loop) {		/* exit cond met */
								if (t) sh2->t_flag = T;
								else   sh2->t_flag = 0;
								return;			/* BT/BF falls through */
							}
							sh2->icount -= 5;
						}
						/* slice exhausted still in loop: leave T at the
						 * loop value so the real BT/BF branches back */
						if (want_t_loop) sh2->t_flag = T;
						else            sh2->t_flag = 0;
						return;
					}
				}
			}
		}
		/* not an SDRAM poll we recognise: fall through to the
		 * countdown cases below (no reject caching here) */
	}

	/* BTS SDRAM poll loop (Doom 32X frame-wait): backward BT/S whose
	 * 2-insn body is
	 *     CMP/EQ Rm,Rn        (0x3nm0)   T = (masked == ref)
	 *     MOV.L  @Rb,Rd       (0x6db2)   reload the polled slot
	 * and whose delay slot is
	 *     AND    Rk,Rd        (0x2dk9)   mask the reload
	 * polling an SDRAM long the VInt ISR increments.  Doom's master parks
	 * here after finishing each frame's work (0x0204df30, ROM+0x4df30:
	 * `CMP/EQ R0,R2; MOV.L @R1,R0; BT/S -2; AND R3,R0` on 0x06001170) —
	 * the device pcwall probe measured this ONE 128-byte page at 35% of
	 * msh2 wall / ~27% of PicoFrame during gameplay (pass 5, 0726).
	 *
	 * Fold model (mirrors THIS interpreter, incl. mame's BT/S executing
	 * the delay slot on BOTH paths): one continuing iteration =
	 * BTS taken(2) + slot AND(1) + head CMP(1) + MOV.L reload(1) = 5.
	 * Each folded iteration re-reads the slot via RL() (real side effect —
	 * the ISR's/other core's write must be visible) and recomputes T.  On
	 * exit T=0 is left so the real BT/S is dispatched untaken: mame then
	 * runs the slot AND for real and falls through — final register state,
	 * T, pc and icount are bit-identical to plain interpretation.  If the
	 * slice ends still looping, T holds the loop value and the real BT/S
	 * re-branches on the next slice after IRQs run.  No SLEEP/RPOLL state,
	 * deadlock-free; base/mask/ref registers are required distinct from
	 * the destination so the loop cannot mutate its own operands. */
	if ((opcode & 0xff00) == 0x8d00)	/* BT/S */
	{
		int disp8 = (int)(signed char)(opcode & 0xff);
		unsigned int target = sh2->ppc + disp8 * 2 + 4;

		if (disp8 >= 0 || target + 4 != sh2->ppc) {	/* not a 2-insn backward loop */
			*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
			return;
		}
		/* opcode fetches must stay off sysreg/comm space (poll_detect) */
		if ((target & 0xdf000000) != 0x02000000
		    && (target & 0xdf000000) != 0x06000000) {
			*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
			return;
		}
		{
			UINT32 head = (UINT32)(UINT16)RW(sh2, target);
			UINT32 load = (UINT32)(UINT16)RW(sh2, target + 2);
			UINT32 slot = (UINT32)(UINT16)RW(sh2, sh2->ppc + 2);
			int dest, base, mask_reg, ref, cmp_n, cmp_m;
			unsigned int pa;

			if ((head & 0xf00f) != 0x3000		/* CMP/EQ Rm,Rn  */
			    || (load & 0xf00f) != 0x6002	/* MOV.L @Rm,Rn  */
			    || (slot & 0xf00f) != 0x2009) {	/* AND Rm,Rn     */
				*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
				return;
			}
			dest = (load >> 8) & 0xf;
			base = (load >> 4) & 0xf;
			mask_reg = (slot >> 4) & 0xf;
			cmp_n = (head >> 8) & 0xf;
			cmp_m = (head >> 4) & 0xf;
			if (((slot >> 8) & 0xf) != (unsigned)dest) {	/* AND dest != reload dest */
				*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
				return;
			}
			if (cmp_n == dest)      ref = cmp_m;
			else if (cmp_m == dest) ref = cmp_n;
			else                    ref = -1;
			if (ref < 0 || base == dest || mask_reg == dest || ref == dest) {
				*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
				return;
			}
			pa = sh2->r[base] & ~0x20000000;	/* strip cache-through */
			if ((pa & 0xff000000) != 0x06000000 || (pa & 3) != 0) {
				/* polled slot must be an aligned SDRAM long; a comm/
				 * sysreg target must keep its poll_detect behaviour */
				*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
				return;
			}
			if (sh2->t_flag == 0)	/* this BT/S falls through: nothing to fold */
				return;
			{
				UINT32 mask_v = sh2->r[mask_reg];
				UINT32 ref_v  = sh2->r[ref];
				while (sh2->icount >= 5) {
					UINT32 masked = sh2->r[dest] & mask_v;	/* delay slot */
					int t = (masked == ref_v);		/* head CMP/EQ */
					sh2->r[dest] = (UINT32)RL(sh2, pa);	/* reload */
					sh2->t_flag = t ? T : 0;
					sh2->icount -= 5;
					if (!t)
						return;	/* real BT/S dispatches untaken */
				}
				return;	/* slice exhausted: real BT/S re-branches */
			}
		}
	}

	/* BFS countdown loop: backward BFS (delay-slot branch) whose body is
	 * a single TST Rn,Rn (sets T = (Rn==0)) and whose delay slot is
	 * ADD #-1,Rn.  Loop: do { Rn--; } while (Rn != 0); on loop exit
	 * Rn = 0xFFFFFFFF (the final delay-slot decrement runs after T sets).
	 * Dominant msh2 hot path on DOOM 32X (~62% of guest master insns),
	 * invisible to the DT-only BF case below because BFS (0x8Fxx) and a
	 * TST+ADD#-1 body are both outside its filters. */
	if ((opcode & 0xff00) == 0x8f00)
	{
		int disp8 = (int)(signed char)(opcode & 0xff);	/* [-128,127] */
		unsigned int target, bfs_at, dslot_at;
		UINT32 bop, dop;
		int rn;
		unsigned int v;
		int iter_cost, kmax;

		if (disp8 >= 0) {			/* forward: not a loop */
			*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
			return;
		}
		target = sh2->ppc + disp8 * 2 + 4;	/* BFS taken pc */
		bfs_at = sh2->ppc;
		if (target + 2 != bfs_at) {		/* exactly 1 body insn */
			*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
			return;
		}
		bop = (UINT32)(UINT16)RW(sh2, target);	/* TST Rn,Rn */
		if ((bop & 0xf00f) != 0x2008
		    || ((bop >> 8) & 0xf) != ((bop >> 4) & 0xf)) {
			*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
			return;
		}
		rn = (bop >> 8) & 0xf;
		dslot_at = bfs_at + 2;
		dop = (UINT32)(UINT16)RW(sh2, dslot_at);
#ifdef RIG_SDRAM_POLL_DIAG
		rig_spd_sample(sh2->ppc, bop, dop, sh2->gbr);
		rig_spd_tries++;
#endif
		if ((dop & 0xf0ff) == 0x70ff && ((dop >> 8) & 0xf) == rn) {
			/* ADD #-1,Rn: countdown (Doom).  iter = TST(1) + BFS
			 * taken(3) + ADD delay(1) = 5 host-icount; tuned against
			 * fb checksum, keep bit-exact to plain interp. */
			if (sh2->t_flag)			/* T==1: BFS not taken */
				return;
			v = sh2->r[rn];
			if (v < 2)				/* last iteration real */
				return;
			iter_cost = 5;
			kmax = (sh2->icount - 1) / iter_cost;
			if (kmax > (int)(v - 1))
				kmax = (int)(v - 1);
			if (kmax < 1)
				return;
			sh2->r[rn] = v - (unsigned int)kmax;
			sh2->icount -= kmax * iter_cost;
			return;
		}
		/* MOV.W @(disp8,GBR),R0 (0xC5xx) in the delay slot: SDRAM poll loop
		 * (Metal Head ~59% msh2).  Loop shape: TST R0,R0 (body, 1 insn) +
		 * BFS back + MOV.W @(disp8,GBR),R0 (delay slot poll read).  Each
		 * iteration: R0 = MEM[GBR + disp8*2]; TST sets T=(R0==0); BFS
		 * taken while T==0.  Exit when the slot reads 0.  body TST
		 * self-test forces rn==0, matching the MOV.W dest R0.  Every
		 * iteration performs the real SDRAM read (side-effect safe,
		 * deadlock-free: icount exhaustion => BFS taken => next timeslice
		 * runs the producer).  iter_cost = TST1 + BFS3 + MOV.W1 = 5. */
		if ((dop & 0xff00) == 0xc500 && rn == 0) {
			unsigned int disp_gbr = dop & 0xff;
			unsigned int poll_addr = (sh2->gbr + disp_gbr * 2) & ~0x20000000u;
			if ((poll_addr & 0xc6000000) == 0x06000000) {
				if (sh2->t_flag)		/* T==1: BFS not taken */
					return;
				while (sh2->icount >= 5) {
					unsigned int val = (unsigned int)(UINT16)RW(sh2, poll_addr);
					sh2->r[0] = val;
					sh2->icount -= 5;
					if (val == 0) {
						sh2->t_flag = T;	/* T=1: BFS exits */
						return;		/* BFS not taken */
					}
					/* T stays 0: BFS taken (loop) */
				}
				sh2->t_flag = 0;			/* slice done: BFS taken */
				return;
			}
		}
		*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
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

		if (sh2->t_flag)	/* final pass: BF won't be taken */
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
#ifdef RIG_SH2_PC_HIST
	int rig_is_delay = 0;
#endif

	sh2->icount = cycles;

	if (sh2->icount <= 0)
		goto out;

	GNW_PCWALL_ENTER(sh2);
	GNW_SLICE_TICK(sh2);

	const void *gnw_dt[16] = {
		&&gnw_op0,  &&gnw_op1,  &&gnw_op2,  &&gnw_op3,
		&&gnw_op4,  &&gnw_op5,  &&gnw_op6,  &&gnw_op7,
		&&gnw_op8,  &&gnw_op9,  &&gnw_opA,  &&gnw_opB,
		&&gnw_opC,  &&gnw_opD,  &&gnw_opE,  &&gnw_opF
	};

	do
	{
		if (sh2->delay)
		{
			sh2->ppc = sh2->delay;
			opcode = GNW_FETCH_SD(sh2, sh2->delay);

			// TODO: more branch types
		if ((opcode >> 13) == 5) { // BRA/BSR
			sh2->sr = (sh2->sr & ~T) | sh2->t_flag;	/* reconcile lazy T */
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
#ifdef RIG_SH2_PC_HIST
			rig_is_delay = 1;
#endif
		}
		else
		{
			sh2->ppc = sh2->pc;
			GNW_FETCH(sh2, sh2->pc, opcode);
#ifdef GNW_SH2_FASTLOOPS
			gnw_direct = 1;
#endif
#ifdef RIG_SH2_PC_HIST
			rig_is_delay = 0;
#endif
		}

		sh2->delay = 0;
		sh2->pc += 2;
		RIG_SH2_TICK();
		GNW_SH2_INSN_TICK(sh2);
		GNW_PCWALL_TICK(sh2);
		RIG_PC_HIST_TICK(sh2, rig_is_delay, (unsigned short)opcode);
		RIG_POLL_PEEK_HOOK(sh2, opcode);

#ifdef GNW_SH2_FASTLOOPS
		/* cheap opcode pre-filter first, then the negative-cache probe
		 * inline (see gnw_dl_reject above), so ordinary hot loops
		 * reject in a few instructions without calling the helper.
		 *
		 * The four candidates are BT/BF/BT/S/BF/S with a negative
		 * displacement -- 0x8980, 0x8b80, 0x8d80, 0x8f80 under mask
		 * 0xff80.  Those are exactly the 0x8n80 opcodes whose n has
		 * both bit3 and bit0 set (n = 9, B, D, F), so one masked
		 * compare against 0xf980 covers all four with no false
		 * positives -- four compare-and-branches per DISPATCHED
		 * INSTRUCTION saved, on a path that runs ~173 k times a frame. */
		if ((((opcode & 0xf980) == 0x8980) || opcode == 0xaffe)
		    && gnw_direct && *GNW_DL_REJ_SLOT(sh2) != sh2->ppc
		    && !sh2->test_irq && gnw_sh2_fastloops)
			gnw_sh2_fastloop(sh2, opcode);
#endif

		goto *gnw_dt[opcode >> 12];
		gnw_op0:  op0000(sh2, opcode); goto gnw_next;
		gnw_op1:  op0001(sh2, opcode); goto gnw_next;
		gnw_op2:  op0010(sh2, opcode); goto gnw_next;
		gnw_op3:  op0011(sh2, opcode); goto gnw_next;
		gnw_op4:  op0100(sh2, opcode); goto gnw_next;
		gnw_op5:  op0101(sh2, opcode); goto gnw_next;
		gnw_op6:  op0110(sh2, opcode); goto gnw_next;
		gnw_op7:  op0111(sh2, opcode); goto gnw_next;
		gnw_op8:  op1000(sh2, opcode); goto gnw_next;
		gnw_op9:  op1001(sh2, opcode); goto gnw_next;
		gnw_opA:  op1010(sh2, opcode); goto gnw_next;
		gnw_opB:  op1011(sh2, opcode); goto gnw_next;
		gnw_opC:  op1100(sh2, opcode); goto gnw_next;
		gnw_opD:  op1101(sh2, opcode); goto gnw_next;
		gnw_opE:  op1110(sh2, opcode); goto gnw_next;
		gnw_opF:  op1111(sh2, opcode); goto gnw_next;
		gnw_next:

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
			opcode = GNW_FETCH_SD(sh2, sh2->delay);
			sh2->pc -= 2;
		}
		else
		{
			sh2->ppc = sh2->pc;
			opcode = GNW_FETCH_SD(sh2, sh2->pc);
		}

		sh2->delay = 0;
		sh2->pc += 2;
		RIG_SH2_TICK();

		RIG_POLL_PEEK_HOOK(sh2, opcode);
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

