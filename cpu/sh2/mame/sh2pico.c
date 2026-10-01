#include "../sh2.h"

#ifdef DRC_CMP
#include "../compiler.h"
#define BUSY_LOOP_HACKS 0
#elif defined(GNW_32X_CORE) && !defined(GNW_KEEP_BUSY_LOOP_HACKS)
/* BUSY_LOOP_HACKS is DEAD CODE in this dispatch loop, and it is not free.
 *
 * Both blocks (mame/sh2.c BRA and DT) do
 *     next_opcode = RW(sh2, sh2->ppc & AM)
 * and compare it against the opcode they expect to FOLLOW them -- 0x0009 (NOP)
 * after BRA, 0x8bfd (BF $-2) after DT. But sh2->ppc in this loop is the
 * EXECUTING instruction's own address, not the next one, so what comes back is
 * the BRA/DT opcode itself. 0xaffe != 0x0009 and 0x4n10 != 0x8bfd: neither
 * comparison can ever be true. sh2pico.c's own fastloop comment already says
 * so ("inert here").
 *
 * Inert, but it still issues a guest 16-bit read on EVERY dispatched DT and
 * EVERY dispatched BRA -- and DT is the counter of the countdown loops this
 * core spends its time in. Compiling it out removes a whole guest read from
 * two hot op bodies and changes nothing a hash can see; the read has no side
 * effects here (guest PCs are cart ROM or SDRAM, both of which take RW's
 * inline fast path, so no handler, no poll detect, no icount charge).
 *
 * Only for GNW_32X_CORE: upstream builds keep the upstream behaviour.
 * -DGNW_KEEP_BUSY_LOOP_HACKS restores it for an A/B. */
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

/* Inline data-read fast paths, the same move the opcode fetch got and for the
 * same reason: every guest load was a cross-TU call (LTO is off) into a
 * function whose first act is to re-derive the region. The bus census says
 * 87,086 guest data accesses per frame against 165,262 dispatched
 * instructions -- more than half of all instructions pay this -- and it also
 * says which region each class lands in, which fixes the test order here:
 *
 *   r8   cart-ROM 88%   SDRAM  2%      -> ROM first
 *   r16  cart-ROM 93%   SDRAM  5%      -> ROM first
 *   r32  SDRAM    82%   cart-ROM 14%   -> SDRAM first
 *
 * Each arm reproduces byte for byte what p32x_sh2_read8/16/32 would return for
 * that region, signedness included -- the ROM arm sign-extends like the map
 * branch, the SDRAM arm zero-extends like its own -- so this cannot change a
 * value even where a caller does not immediately cast. Anything else still
 * takes the call.
 *
 * The old objection to inlining here was ITCM: RB/RW/RL/WB/WW/WL appear at ~50
 * sites in an 8 KB interpreter with 4 KB of head room. That objection was
 * about inlining a DWT probe BRACKET at each site. A fast path is a few
 * instructions, and the measured text growth is well inside the head room.
 *
 * GNW_NO_INLINE_DATA_READ restores the plain calls for A/B. */
#ifdef GNW_NO_INLINE_DATA_READ
#define RB(sh2, a) p32x_sh2_read8(a, sh2)
#define RW(sh2, a) p32x_sh2_read16(a, sh2)
#define RL(sh2, a) p32x_sh2_read32(a, sh2)
#else
#define GNW_MEM_BE2(a) ((a) ^ 1)
#ifdef GNW_INLINE_READ_L_ONLY
#define RB(sh2, a) p32x_sh2_read8(a, sh2)
#define RW(sh2, a) p32x_sh2_read16(a, sh2)
#else
#define RB(sh2, a) ({ UINT32 a_ = (a); UINT32 h_ = a_ & 0xdf000000;          \
  h_ == gnw_fw_rom.region                                                    \
    ? (UINT32)*(INT8 *)(gnw_fw_rom.base + GNW_MEM_BE2(a_ & gnw_fw_rom.mask)) \
  : h_ == 0x06000000                                                         \
    ? (UINT32)((UINT8 *)(sh2)->p_sdram)[GNW_MEM_BE2(a_ & 0x3ffff)]           \
    : p32x_sh2_read8(a_, sh2); })
#define RW(sh2, a) ({ UINT32 a_ = (a); UINT32 h_ = a_ & 0xdf000000;          \
  h_ == gnw_fw_rom.region                                                    \
    ? (UINT32)*(INT16 *)(gnw_fw_rom.base + (a_ & gnw_fw_rom.mask))           \
  : h_ == 0x06000000                                                         \
    ? (UINT32)*(UINT16 *)((UINT8 *)(sh2)->p_sdram + (a_ & 0x3fffe))          \
    : p32x_sh2_read16(a_, sh2); })
#endif
#define RL(sh2, a) ({ UINT32 a_ = (a); UINT32 h_ = a_ & 0xdf000000;          \
  h_ == 0x06000000                                                           \
    ? CPU_BE2(*(UINT32 *)((UINT8 *)(sh2)->p_sdram + (a_ & 0x3fffc)))         \
  : h_ == gnw_fw_rom.region                                                  \
    ? CPU_BE2(*(UINT32 *)(gnw_fw_rom.base + (a_ & gnw_fw_rom.mask)))         \
    : p32x_sh2_read32(a_, sh2); })
#endif
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
struct gnw_fetch_win { unsigned int region, mask; unsigned char *base; };
extern struct gnw_fetch_win gnw_fw_rom;

/* GNW_FETCH_OLD_WINDOW restores the pre-window form for A/B. */
#ifdef GNW_FETCH_OLD_WINDOW
#define GNW_FETCH_SD(sh2, addr)                                              \
  ((((addr) & 0xdf000000) == 0x06000000)                                     \
     ? (UINT32)*(UINT16 *)((UINT8 *)(sh2)->p_sdram + ((addr) & 0x3fffe))     \
   : ((((addr) & 0xdf000000) == 0x02000000) && gnw_sh2_rom_fetch_mask)       \
     ? (UINT32)*(UINT16 *)((UINT8 *)(sh2)->p_rom                             \
                           + ((addr) & gnw_sh2_rom_fetch_mask))              \
     : (UINT32)(UINT16)RW(sh2, addr))
#else
/* __builtin_expect because gcc otherwise lays SDRAM out as the fall-through
 * and branches the ROM arm away, which costs a taken branch plus a branch back
 * on the path that runs ~165 k times a frame -- eating most of what the window
 * saves. The device pcwall probe measured Doom's msh2 at cart-ROM 100%. */
#define GNW_FETCH_SD(sh2, addr)                                              \
  (__builtin_expect(((addr) & 0xdf000000) == gnw_fw_rom.region, 1)           \
     ? (UINT32)*(UINT16 *)(gnw_fw_rom.base + ((addr) & gnw_fw_rom.mask))     \
   : (((addr) & 0xdf000000) == 0x06000000)                                   \
     ? (UINT32)*(UINT16 *)((UINT8 *)(sh2)->p_sdram + ((addr) & 0x3fffe))     \
     : (UINT32)(UINT16)RW(sh2, addr))
#endif

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
/* Region buckets (2026-08-19): the flat average mixes slow cart-ROM (XIP
 * flash) reads with fast on-MCU SDRAM reads, underpricing any cache-the-ROM
 * lever by ~2x. 0=SDRAM(0x06/0x26) 1=ROM(0x02/0x22) 2=DRAM(0x04/0x24)
 * 3=other; mirrors fold via a & 0xdf000000. */
unsigned int gnw_da_cyc_b[4][2], gnw_da_n_b[4][2];
/* Write buckets omitted: writes are ~14k/frame (minor) and the overlay BSS
 * has no room for a second 64 B pair (2026-08-19 overflow round). */
static int gnw_da_region(UINT32 a)
{
	UINT32 h = a & 0xdf000000u;
	return h == 0x06000000u ? 0 : h == 0x02000000u ? 1 : h == 0x04000000u ? 2 : 3;
}
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
	{
		unsigned int d = *(volatile unsigned int *)0xE0001004 - t0;
		int b = gnw_da_region(a);
		gnw_da_cyc[c] += d;
		gnw_da_n[c]++;
		gnw_da_cyc_b[b][c] += d;
		gnw_da_n_b[b][c]++;
	}
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
/* 2026-08-20: re-aimed. At 16 KB over ROM 0..1 MB the device profile put
 * 60.9% of msh2 in ONE page, 0x02048000, with 0x02038000 at 19.8% and
 * 0x0204c000 at 9.8% -- every hot page inside 0x02030000..0x02050000. A
 * window that answers "which 16 KB" when the answer is already known is a
 * wasted run, so: 4 KB pages over the 256 KB that contains all of them. */
#ifndef GNW_PCWALL_PAGE_SHIFT
#define GNW_PCWALL_PAGE_SHIFT 12                  /* 16=64K, 14=16K, 12=4K, 7=128B */
#endif
#ifndef GNW_PCWALL_WIN_BASE
#define GNW_PCWALL_WIN_BASE  0x00030000u          /* offset into ROM */
#endif
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

/* Hoisted-window fetch (2026-08-20).
 *
 * GNW_FETCH_SD reads three fields of the global gnw_fw_rom on EVERY fetched
 * guest instruction: `ldr region` plus an `ldrd mask, base`. gcc cannot keep
 * them in registers across the dispatch loop because the op bodies call the
 * memory handlers, which are extern and may write anything.
 *
 * They do not change inside a slice, and that is a property of the code, not
 * an assumption: gnw_fw_rom is written only by bank_switch_rom_sh2(), whose
 * two callers are PicoMemSetup32x() and p32x_update_banks(); the values it
 * derives from -- Pico.rom, gnw_rom_map_mask and carthw_ssf2_active -- are all
 * fixed at cart load (carthw_ssf2_startup / carthw_ssf2_unload). Neither
 * caller can run from inside sh2_execute_interpreter.
 *
 * So hoist them once per slice (~570 dispatched instructions) instead of once
 * per instruction. -DGNW_FW_NO_HOIST restores the per-instruction loads.
 *
 * IF YOU ADD A RUNTIME WRITER OF gnw_fw_rom, this breaks silently: a slice in
 * flight keeps fetching through the old base. End the run there, or turn the
 * hoist off. */
#define GNW_FETCH_SD_W(sh2, addr, R_, M_, B_)                                \
  (__builtin_expect(((addr) & 0xdf000000) == (R_), 1)                        \
     ? (UINT32)*(UINT16 *)((B_) + ((addr) & (M_)))                           \
   : (((addr) & 0xdf000000) == 0x06000000)                                   \
     ? (UINT32)*(UINT16 *)((UINT8 *)(sh2)->p_sdram + ((addr) & 0x3fffe))     \
     : (UINT32)(UINT16)RW(sh2, addr))

#if defined(GNW_FW_NO_HOIST) || defined(GNW_FETCH_OLD_WINDOW) \
    || defined(MD32X_DEVICE_PROFILE)
/* The profile build brackets the fetch with a DWT pair and must keep the exact
 * shape it measures; the old-window arm has no window to hoist. */
#define GNW_FW_HOIST_DECL()   ((void)0)
#define GNW_FETCH_H(sh2, addr, dst)  GNW_FETCH(sh2, addr, dst)
#define GNW_FETCH_SD_H(sh2, addr)    GNW_FETCH_SD(sh2, addr)
#else
#define GNW_FW_HOIST_DECL()                                                  \
	const unsigned int gnw_fw_r_ = gnw_fw_rom.region;                    \
	const unsigned int gnw_fw_m_ = gnw_fw_rom.mask;                      \
	unsigned char * const gnw_fw_b_ = gnw_fw_rom.base
#define GNW_FETCH_H(sh2, addr, dst)                                          \
	((dst) = GNW_FETCH_SD_W(sh2, addr, gnw_fw_r_, gnw_fw_m_, gnw_fw_b_))
#define GNW_FETCH_SD_H(sh2, addr)                                            \
	GNW_FETCH_SD_W(sh2, addr, gnw_fw_r_, gnw_fw_m_, gnw_fw_b_)
#endif

/* RIG_SH2_PC_HIST: SH-2 guest-PC histogram for the QEMU M7 feasibility rig.
 * Two sparse open-addressed tables (master/slave), keyed by ppc, counting
 * direct vs delay-slot executions. Reveals which guest loops eat the msh2/
 * ssh2 phase — fastloop-off shows loops fastloop already kills, fastloop-on
 * shows the residual hot set. Never compiled in device/libretro builds.
 * rig_32x.c reads the table (non-static) and prints the top-N report. */
#ifdef RIG_SH2_PC_HIST
/* Sized for the attract loop (188 unique PCs) and then used, unchanged, on a
 * resumed gameplay savestate that has 8,329 -- more than the table holds. A
 * saturated linear-probe table turns every miss into a full 8192-slot scan,
 * per guest instruction: the rig's host cost per frame went from ~15M to as
 * much as 3.74G, a 460x "spread" that looked like an emulator mystery and was
 * entirely this instrument. Overridable now (-DRIG_PC_HIST_SLOTS=32768), the
 * probe is bounded, and saturation is REPORTED rather than sampled silently. */
#ifndef RIG_PC_HIST_SLOTS
#define RIG_PC_HIST_SLOTS 8192
#endif
#ifndef RIG_PC_HIST_MAXPROBE
#define RIG_PC_HIST_MAXPROBE 64
#endif
unsigned long long rig_pchist_dropped[2];
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
	unsigned limit = RIG_PC_HIST_MAXPROBE < RIG_PC_HIST_SLOTS
	               ? RIG_PC_HIST_MAXPROBE : RIG_PC_HIST_SLOTS;
	for (unsigned i = 0; i < limit; i++) {
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
	/* Probe window exhausted: this instruction is NOT counted. Say so --
	 * a silent drop turns an under-count into a confident-looking percentage.
	 * rig_32x.c prints this next to the report; if it is nonzero, raise
	 * RIG_PC_HIST_SLOTS and re-run before believing any share. */
	rig_pchist_dropped[core]++;
}
#define RIG_PC_HIST_TICK(sh2, is_delay, op) rig_pchist_tick(sh2, is_delay, op)
#else
#define RIG_PC_HIST_TICK(sh2, is_delay, op) ((void)0)
#endif

/* RIG_ABLATE_LOOPS: price the texture inner loops by REMOVING their work, not
 * by reading their share off a profile -- this repo has been wrong twice about
 * a lever whose profile share looked large and whose ablation delta did not
 * (memory stalls do not appear in an instruction count, and an instruction
 * count is all a guest-PC histogram is).
 *
 * Doom's two hot loops are counted down by DT on a register: 0x02049084 with
 * R12, 0x02049284 with R6, together ~37% of all guest SH-2 instructions in the
 * resumed gameplay window. Clamping the counter to 1 at loop entry collapses
 * each to a single iteration: the loop still runs, still exits through its own
 * BFS, and every register it touches is left in a shape its caller accepts --
 * so control flow downstream is unchanged and only the pixels are missing.
 * The screen is wrong on purpose. The number is the ceiling on any HLE of it.
 *
 *   EXTRA_DEF="-DRIG_ABLATE_LOOPS"                 both loops
 *   EXTRA_DEF="-DRIG_ABLATE_LOOPS -DRIG_ABL_PC2=0" loop 1 only
 * Addresses are this build of retail Doom; override with -DRIG_ABL_PC1/PC2. */
#ifdef RIG_ABLATE_LOOPS
#ifndef RIG_ABL_PC1
#define RIG_ABL_PC1 0x02049084
#endif
#ifndef RIG_ABL_REG1
#define RIG_ABL_REG1 12
#endif
#ifndef RIG_ABL_PC2
#define RIG_ABL_PC2 0x02049284
#endif
#ifndef RIG_ABL_REG2
#define RIG_ABL_REG2 6
#endif
unsigned long long rig_abl_hits[2], rig_abl_iters_skipped[2];
static void rig_ablate_tick(SH2 *sh2)
{
	unsigned int pc = sh2->ppc;
	if (RIG_ABL_PC1 && pc == (unsigned)RIG_ABL_PC1 && sh2->r[RIG_ABL_REG1] > 1) {
		rig_abl_hits[0]++;
		rig_abl_iters_skipped[0] += sh2->r[RIG_ABL_REG1] - 1;
		sh2->r[RIG_ABL_REG1] = 1;
	} else if (RIG_ABL_PC2 && pc == (unsigned)RIG_ABL_PC2 && sh2->r[RIG_ABL_REG2] > 1) {
		rig_abl_hits[1]++;
		rig_abl_iters_skipped[1] += sh2->r[RIG_ABL_REG2] - 1;
		sh2->r[RIG_ABL_REG2] = 1;
	}
}
#define RIG_ABLATE_TICK(sh2) rig_ablate_tick(sh2)
#else
#define RIG_ABLATE_TICK(sh2) ((void)0)
#endif

/* RIG_LOOP_REGS: dump the register file at the texture loops' entry, first N
 * visits. The question it answers is narrow and decisive: WHICH REGION do the
 * loop's loads and stores address? p32x_sh2_read8/16/32 have a fast path for
 * SDRAM (0x06/0x26) and nothing else, so a loop reading its texels out of cart
 * ROM (0x02/0x22) pays the full map lookup on every pixel -- the same shape as
 * the opcode-fetch path, which measured "sdram 0.0% / cart-ROM 100%" and got
 * gnw_sh2_rom_fetch_mask for exactly that reason. If the data side has the
 * same miss, it is the same one-line-class fix on 68% of the frame. */
#ifdef RIG_LOOP_REGS
#ifndef RIG_LOOP_REGS_N
#define RIG_LOOP_REGS_N 6
#endif
extern int printf(const char *, ...);
static int rig_lr_n[2];
static void rig_loop_regs_tick(SH2 *sh2)
{
	unsigned int pc = sh2->ppc;
	int which = pc == 0x02049084u ? 0 : pc == 0x02049284u ? 1 : -1;
	if (which < 0 || rig_lr_n[which] >= RIG_LOOP_REGS_N)
		return;
	rig_lr_n[which]++;
	printf("[lr] loop%d pc=%08x r0=%08x r1=%08x r2=%08x r3=%08x r4=%08x "
	       "r5=%08x r6=%08x r7=%08x r8=%08x r9=%08x r10=%08x r12=%08x\n",
	       which + 1, pc,
	       (unsigned)sh2->r[0], (unsigned)sh2->r[1], (unsigned)sh2->r[2],
	       (unsigned)sh2->r[3], (unsigned)sh2->r[4], (unsigned)sh2->r[5],
	       (unsigned)sh2->r[6], (unsigned)sh2->r[7], (unsigned)sh2->r[8],
	       (unsigned)sh2->r[9], (unsigned)sh2->r[10], (unsigned)sh2->r[12]);
}
#define RIG_LOOP_REGS_TICK(sh2) rig_loop_regs_tick(sh2)
#else
#define RIG_LOOP_REGS_TICK(sh2) ((void)0)
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
/* Ablation switch. The pre-filter below costs ~7 host instructions on EVERY
 * guest instruction (three movw of its constants, two compares, two branches)
 * to catch loops it can skip. This tree's own rule is that a test which adds
 * work to skip work tends to lose on this chip, so the tax has to be priced
 * against the saving rather than assumed: build with -DGNW_SH2_NO_FASTLOOPS to
 * measure without it. */
#ifndef GNW_SH2_NO_FASTLOOPS
#define GNW_SH2_FASTLOOPS 1
#endif
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

#ifndef GNW_NO_TEXCOL_HLE
		/* Texture-column loop (R_DrawColumn): seven body instructions,
		 * ~38% of all guest SH-2 instructions in Doom 32X gameplay, and
		 * the largest single lever left inside the core. Ablating it
		 * entirely (clamping the counter to 1) is worth -15.4% of frame
		 * cost, so a native execution that still does the real memory
		 * work should land just under that.
		 *
		 *   MOV.B  @Rsrc,Rt        texel, sign-extended
		 *   ADDC   Rfs,Rfrac       fixed-point fraction, carry out
		 *   ADDC   Ris,Rsrc        source pointer + carry
		 *   SHLL   Rt              index *= 2  (writes T, dead: DT follows)
		 *   MOV.W  @(R0,Rt),Rt     colormap lookup
		 *   DT     Rcnt            count--, T = (count == 0)
		 *   MOV.W  Rt,@Rdst        pixel store
		 *   BFS    back
		 *   ADD    Rstride,Rdst    delay slot: next scanline
		 *
		 * Matched structurally -- opcode forms plus register agreement,
		 * never an address -- so it fits any build of this renderer
		 * rather than one cart. The register numbers are read out of the
		 * matched opcodes, not assumed.
		 *
		 * Entry state is: the body of the current iteration has run, T is
		 * DT's result, and the delay slot has NOT. Each folded iteration
		 * therefore replays [delay slot of the previous] + [one body],
		 * which leaves exactly that same shape behind -- so the
		 * interpreter resumes on the real BFS with nothing to fix up.
		 *
		 * Every access goes through RB/RW/WW, the interpreter's own
		 * macros, so the bytes and the side effects are identical; the
		 * framebuffer and audio hashes are the gate. GNW_NO_TEXCOL_HLE
		 * disables it for A/B. */
		if (target + 14 == bfs_at) {
			UINT32 b0 = (UINT32)(UINT16)RW(sh2, target);
			UINT32 b1 = (UINT32)(UINT16)RW(sh2, target + 2);
			UINT32 b2 = (UINT32)(UINT16)RW(sh2, target + 4);
			UINT32 b3 = (UINT32)(UINT16)RW(sh2, target + 6);
			UINT32 b4 = (UINT32)(UINT16)RW(sh2, target + 8);
			UINT32 b5 = (UINT32)(UINT16)RW(sh2, target + 10);
			UINT32 b6 = (UINT32)(UINT16)RW(sh2, target + 12);
			UINT32 ds = (UINT32)(UINT16)RW(sh2, bfs_at + 2);
			int rt   = (b0 >> 8) & 0xf, rsrc = (b0 >> 4) & 0xf;
			int rfrac= (b1 >> 8) & 0xf, rfs  = (b1 >> 4) & 0xf;
			int ris  = (b2 >> 4) & 0xf;
			int rcnt = (b5 >> 8) & 0xf;
			int rdst = (b6 >> 8) & 0xf, rstr = (ds >> 4) & 0xf;

			if ((b0 & 0xf00f) == 0x6000		/* MOV.B @Rsrc,Rt  */
			 && (b1 & 0xf00f) == 0x300e		/* ADDC Rfs,Rfrac  */
			 && (b2 & 0xf00f) == 0x300e		/* ADDC Ris,Rsrc   */
			 && ((b2 >> 8) & 0xf) == rsrc
			 && (b3 & 0xf0ff) == 0x4000		/* SHLL Rt         */
			 && ((b3 >> 8) & 0xf) == rt
			 && (b4 & 0xf00f) == 0x000d		/* MOV.W @(R0,Rt),Rt */
			 && ((b4 >> 8) & 0xf) == rt && ((b4 >> 4) & 0xf) == rt
			 && (b5 & 0xf0ff) == 0x4010		/* DT Rcnt         */
			 && (b6 & 0xf00f) == 0x2001		/* MOV.W Rt,@Rdst  */
			 && ((b6 >> 4) & 0xf) == rt
			 && (ds & 0xf00f) == 0x300c		/* ADD Rstride,Rdst */
			 && ((ds >> 8) & 0xf) == rdst) {
				/* 7 body insns + BFS taken + delay slot, on the
				 * same convention the single-insn case above uses
				 * (1 body insn there costs 5). */
				const int iter_cost = 11;

				if (sh2->t_flag)	/* BFS not taken: loop is ending */
					return;
				/* Call the interpreter's OWN op functions, in
				 * order, rather than reimplementing them. sh2.c
				 * is included above this point, so they are in
				 * scope -- and that is the difference between
				 * "should be identical" and "is". Reimplementing
				 * would have to reproduce ADDC's exact carry
				 * form, SHLL writing T, DT also setting
				 * no_polling, and every op's sh2->ea update, any
				 * one of which is a silent divergence. */
				while (sh2->icount >= iter_cost && sh2->r[rcnt] >= 2) {
					ADD(sh2, rstr, rdst);	/* delay slot   */
					MOVBL(sh2, rsrc, rt);	/* texel        */
					ADDC(sh2, rfs, rfrac);	/* frac + carry */
					ADDC(sh2, ris, rsrc);	/* src  + carry */
					SHLL(sh2, rt);		/* index *= 2   */
					MOVWL0(sh2, rt, rt);	/* colormap     */
					DT(sh2, rcnt);		/* count--, T   */
					MOVWS(sh2, rt, rdst);	/* pixel store  */

					sh2->icount -= iter_cost;
					if (sh2->t_flag)	/* loop ends here */
						return;
				}
				return;
			}
			*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
			return;
		}

		/* Span loop (R_DrawSpan): eleven body instructions, the second
		 * of Doom's two renderer inner loops and ~17.9% of guest master
		 * instructions on its own. Two fixed-point coordinates are
		 * combined into a texture offset with SWAP.W + masks + OR, the
		 * texel goes through the colormap, and the pixel is stored with a
		 * PRE-DECREMENT -- the span is written right to left.
		 *
		 *   SWAP.W Rq,Racc          ADD    Rs1,Rq
		 *   AND    Rk1,Racc         ADD    Rs2,Rp
		 *   AND    Rk2,Rtmp         SHLL   Racc
		 *   OR     Rtmp,Racc        MOV.W  @(R0,Rc),Racc
		 *   MOV.B  @(R0,Rb),Racc    DT     Rcnt
		 *                           MOV.W  Racc,@-Rdst
		 *   BFS back / SWAP.W Rp,Rtmp in the delay slot
		 *
		 * Written out straight rather than through a general folder.
		 * A general version was built and measured first: it identified
		 * both loops correctly and still came out SLOWER than this one
		 * loop alone (12,797,823 against 12,560,257), because a per-op
		 * dispatch inside the fold costs more than it saves. Zero
		 * dispatch is the whole point. */
		if (target + 22 == bfs_at) {
			UINT32 b0 = (UINT32)(UINT16)RW(sh2, target);
			UINT32 b1 = (UINT32)(UINT16)RW(sh2, target + 2);
			UINT32 b2 = (UINT32)(UINT16)RW(sh2, target + 4);
			UINT32 b3 = (UINT32)(UINT16)RW(sh2, target + 6);
			UINT32 b4 = (UINT32)(UINT16)RW(sh2, target + 8);
			UINT32 b5 = (UINT32)(UINT16)RW(sh2, target + 10);
			UINT32 b6 = (UINT32)(UINT16)RW(sh2, target + 12);
			UINT32 b7 = (UINT32)(UINT16)RW(sh2, target + 14);
			UINT32 b8 = (UINT32)(UINT16)RW(sh2, target + 16);
			UINT32 b9 = (UINT32)(UINT16)RW(sh2, target + 18);
			UINT32 b10 = (UINT32)(UINT16)RW(sh2, target + 20);
			UINT32 ds = (UINT32)(UINT16)RW(sh2, bfs_at + 2);
			int acc = (b0 >> 8) & 0xf, q   = (b0 >> 4) & 0xf;
			int k1  = (b1 >> 4) & 0xf;
			int tmp = (b2 >> 8) & 0xf, k2  = (b2 >> 4) & 0xf;
			int b_  = (b4 >> 4) & 0xf;
			int s1  = (b5 >> 4) & 0xf;
			int p   = (b6 >> 8) & 0xf, s2  = (b6 >> 4) & 0xf;
			int c   = (b8 >> 4) & 0xf;
			int cnt = (b9 >> 8) & 0xf;
			int dst = (b10 >> 8) & 0xf;

			if ((b0 & 0xf00f) == 0x6009			/* SWAP.W Rq,Racc   */
			 && (b1 & 0xf00f) == 0x2009			/* AND Rk1,Racc     */
			 && ((b1 >> 8) & 0xf) == acc
			 && (b2 & 0xf00f) == 0x2009			/* AND Rk2,Rtmp     */
			 && (b3 & 0xf00f) == 0x200b			/* OR Rtmp,Racc     */
			 && ((b3 >> 8) & 0xf) == acc && ((b3 >> 4) & 0xf) == tmp
			 && (b4 & 0xf00f) == 0x000c			/* MOV.B @(R0,Rb),Racc */
			 && ((b4 >> 8) & 0xf) == acc
			 && (b5 & 0xf00f) == 0x300c			/* ADD Rs1,Rq       */
			 && ((b5 >> 8) & 0xf) == q
			 && (b6 & 0xf00f) == 0x300c			/* ADD Rs2,Rp       */
			 && (b7 & 0xf0ff) == 0x4000			/* SHLL Racc        */
			 && ((b7 >> 8) & 0xf) == acc
			 && (b8 & 0xf00f) == 0x000d			/* MOV.W @(R0,Rc),Racc */
			 && ((b8 >> 8) & 0xf) == acc
			 && (b9 & 0xf0ff) == 0x4010			/* DT Rcnt          */
			 && (b10 & 0xf00f) == 0x2005			/* MOV.W Racc,@-Rdst */
			 && ((b10 >> 4) & 0xf) == acc
			 && (ds & 0xf00f) == 0x6009			/* SWAP.W Rp,Rtmp   */
			 && ((ds >> 8) & 0xf) == tmp && ((ds >> 4) & 0xf) == p) {
				const int iter_cost = 15;	/* 11 body + BFS + delay */

				if (sh2->t_flag)
					return;
				while (sh2->icount >= iter_cost && sh2->r[cnt] >= 2) {
					SWAPW(sh2, p, tmp);	/* delay slot */
					SWAPW(sh2, q, acc);
					AND(sh2, k1, acc);
					AND(sh2, k2, tmp);
					OR(sh2, tmp, acc);
					MOVBL0(sh2, b_, acc);
					ADD(sh2, s1, q);
					ADD(sh2, s2, p);
					SHLL(sh2, acc);
					MOVWL0(sh2, c, acc);
					DT(sh2, cnt);
					MOVWM(sh2, acc, dst);

					sh2->icount -= iter_cost;
					if (sh2->t_flag)
						return;
				}
				return;
			}
			*GNW_DL_REJ_SLOT(sh2) = sh2->ppc;
			return;
		}
#endif
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

/* The fastloop gate, moved OFF the per-instruction path (2026-08-20).
 *
 * It used to run ahead of the dispatch switch: `bic r0, nibble, #2; cmp r0, #8;
 * bne` on EVERY dispatched guest instruction, to find candidates that by
 * construction live only in the 0x8xxx and 0xaxxx nibbles -- the same two
 * values the switch below is already about to branch on. Asking the question
 * twice is what cost the three instructions. Inside the case labels the top
 * nibble is known, so the test that remains is the part the nibble does not
 * already answer:
 *
 *   case 0x8: (opcode & 0xf980) == 0x8980  ->  (opcode & 0x0980) == 0x0980
 *   case 0xA: opcode == 0xaffe             ->  unchanged, one compare
 *
 * Semantics are identical: the gate still runs BEFORE the operation executes,
 * and gnw_sh2_fastloop still leaves the final iteration to be interpreted for
 * real by the op body that follows it. Build -DGNW_FASTLOOP_GATE_IN_LOOP to
 * restore the old shape for an A/B. */
#if defined(GNW_SH2_FASTLOOPS) && !defined(GNW_FASTLOOP_GATE_IN_LOOP)
#define GNW_FASTLOOP_GATE_8(sh2, opcode, direct) do {                        \
	if (((opcode) & 0x0980) == 0x0980 && (direct)                        \
	    && *GNW_DL_REJ_SLOT(sh2) != (sh2)->ppc                           \
	    && !(sh2)->test_irq && gnw_sh2_fastloops)                        \
		gnw_sh2_fastloop((sh2), (opcode));                           \
} while (0)
#define GNW_FASTLOOP_GATE_A(sh2, opcode, direct) do {                        \
	if ((opcode) == 0xaffe && (direct)                                   \
	    && *GNW_DL_REJ_SLOT(sh2) != (sh2)->ppc                           \
	    && !(sh2)->test_irq && gnw_sh2_fastloops)                        \
		gnw_sh2_fastloop((sh2), (opcode));                           \
} while (0)
#else
#define GNW_FASTLOOP_GATE_8(sh2, opcode, direct) do { } while (0)
#define GNW_FASTLOOP_GATE_A(sh2, opcode, direct) do { } while (0)
#endif

/* RIG_OPHIST: which SH-2 opcode groups the frame is actually made of.
 *
 * Dispatch-shape questions -- is a flatter table worth it, which op body is
 * worth hand-tuning -- need the distribution, not a guess. The first answer it
 * gave closed a lever rather than opening one: on Doom gameplay msh2 spends
 * 12.08% of its dispatched instructions in the 0x00xx group, of which 64.7% is
 * NOP (7.8% of everything) and 28.0% is RTS -- and short-circuiting NOP ahead
 * of op0000's second-level switch was worth 0.10%, i.e. nothing. gcc's jump
 * table for that switch is already cheap.
 *
 * Rig-only; off => no code emitted. */
#ifdef RIG_OPHIST
unsigned int rig_ophist[2][256];
unsigned int rig_ophist_lo0[2][256];   /* low byte of the 0x00xx group */
#define RIG_OPHIST_TICK(sh2, op) do {                                        \
	int c_ = (sh2)->is_slave & 1;                                        \
	rig_ophist[c_][((op) >> 8) & 0xff]++;                                \
	if ((((op) >> 8) & 0xff) == 0) rig_ophist_lo0[c_][(op) & 0xff]++;    \
} while (0)
#else
#define RIG_OPHIST_TICK(sh2, op) ((void)0)
#endif

/* RIG_OPCOST / RIG_SH2_SKELETON (2026-08-24 rebuild): the picodrive side of
 * the opcode-cost census whose report block (rig_32x.c, -DRIG_OPCOST) was
 * committed in ad236375 while these hooks were not -- the numbers quoted in
 * that message came from a local patch that is gone. Rig-only, off => no code.
 *
 * RIG_OPCOST samples 1 dispatched instruction in 64 and times the span from
 * the top of the dispatch loop to the loop tail -- fetch, PC update, delay
 * handling, dispatch, handler, icount--, IRQ poll. CMSDK tick quantisation
 * cancels in the mean over ~1e5 samples per group; the empty-span calibration
 * every 4096 samples prices the two timer reads themselves.
 *
 * RIG_SH2_SKELETON empties the sixteen handler bodies and keeps everything
 * else, so the whole-loop cost per dispatched instruction becomes the fixed
 * cost the census is hunting. The per-case counters are volatile so gcc
 * cannot collapse the jump table back into nothing; their load+store is the
 * one known bias (~3 host insns), noted in the report. */
#ifdef RIG_OPCOST
extern uint32_t rig_timer_now(void);
unsigned long long rig_opcost[2][16];
unsigned long long rig_opcnt[2][16];
unsigned long long rig_opnow_cost, rig_opnow_n;
static uint32_t rig_op_t0_;
static unsigned rig_op_ctr, rig_op_armed_, rig_op_smp_;
#define RIG_OPCOST_T0(sh2) do {                                              \
	rig_op_armed_ = 0;                                                   \
	if (!((rig_op_ctr++) & 63)) {                                        \
		rig_op_t0_ = rig_timer_now();                                \
		rig_op_armed_ = 1;                                           \
	}                                                                    \
} while (0)
#define RIG_OPCOST_T1(sh2, op) do {                                          \
	if (rig_op_armed_) {                                                 \
		int c_ = (sh2)->is_slave & 1;                                \
		rig_opcost[c_][((op) >> 12) & 0xf] +=                          \
			(unsigned)rig_timer_now() - rig_op_t0_;               \
		rig_opcnt[c_][((op) >> 12) & 0xf]++;                          \
		if (!(rig_op_smp_++ & 63)) {                                   \
			uint32_t a_ = rig_timer_now(), b_ = rig_timer_now();  \
			rig_opnow_cost += b_ - a_;                            \
			rig_opnow_n++;                                        \
		}                                                            \
	}                                                                    \
} while (0)
#else
#define RIG_OPCOST_T0(sh2) ((void)0)
#define RIG_OPCOST_T1(sh2, op) ((void)0)
#endif

#ifdef RIG_GBR_CENSUS
/* 0xC5 = MOV.W @(d,GBR),R0 is 83.84%% of the msh2 0xC group (OPHIST,
 * 2026-08-24) while the group costs 4.6 ticks against the 1.6-1.9
 * baseline.  GBR's value decides whether that is a lever (points into
 * ROM/SDRAM -> read fast path missed every time) or real work (points
 * into the 32X register file -> side-effecting I/O reads).  One bucket
 * per GBR top byte, split master/slave. */
unsigned long long rig_gbr_hist[2][256];
#endif

#ifdef RIG_SH2_SKELETON
static volatile unsigned rig_skel_hit[16];
#endif

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#ifndef GNW_SH2_FASTLOOPS
#error Native kernels require existing GNW fastloop direct-fetch tracking
#endif
#define PIXEL_NATIVE_TAIL 1
#define NATIVE_INACTIVE_FOLD 1
/* Experimental generic SDRAM flag/countdown fold. Offline proof only. */
static unsigned long long countdown_iterations;
static unsigned countdown_calls;
static unsigned countdown_fold(SH2 *s) {
  if(s->delay || s->test_irq || s->icount<15 || !s->p_sdram) return 0;
  uint32_t pc=s->pc;
  if((pc&0xdf000000u)!=0x06000000u || (pc&0x3ffffu)>0x3fff6u || (pc&1))return 0;
  const uint16_t *code=(const uint16_t *)((const uint8_t *)s->p_sdram+(pc&0x3fffeu));
  uint16_t load=code[0],dt=code[3];
  if((load&0xff00)!=0x8400 || code[1]!=0x2008 ||
     (code[2]&0xff80)!=0x8b00 || (dt&0xf0ff)!=0x4010 || code[4]!=0x8bfa)return 0;
  unsigned base=(load>>4)&15,counter=(dt>>8)&15;
  if(!base || !counter || counter==base)return 0;
  uint32_t a=s->r[base]+(load&15),n=s->r[counter];
  if((a&0xdf000000u)!=0x06000000u || n<2)return 0;
  if(((const uint8_t *)s->p_sdram)[(a&0x3ffffu)^1])return 0;
  unsigned k=(s->icount-1)/7;
  if(k>=n)k=n-1;
  if(!k)return 0;
  s->r[0]=0;s->r[counter]=n-k;s->t_flag=0;
  s->pc=pc;s->ppc=pc+8;s->ea=pc;
  s->no_polling=SH2_NO_POLLING;s->icount-=7*k;
  countdown_iterations+=k;countdown_calls++;
  return k;
}

/* Offline generic opcode-pattern native kernel; no title or PC binding. */
static unsigned pixel_calls, pixel_proofs, pixel_declines, pixel_groups;
static int pixel_proof_active;
static void pixel_store(SH2 *s, unsigned a, unsigned d) {
  unsigned short *p=(unsigned short *)s->p_dram+((a&0x1ffffu)>>1);
  if(a&0x20000u){unsigned v=*p;if(!(d&255))d|=v&255;if(!(d&0xff00))d|=v&0xff00;}
  *p=d;
}
/* Called after the first opcode fetch, before it executes. */
static unsigned pixel_native_impl(SH2 *s, unsigned pc) {
  if(pixel_proof_active || s->delay || s->test_irq || s->icount<8 || !s->p_sdram || !s->p_dram)return 0;
  if((pc&0xdf000000u)!=0x06000000u || (pc&0x3ffffu)>0x3fff0u || (pc&1))return 0;
  const unsigned short *p=(const unsigned short *)((const unsigned char *)s->p_sdram+(pc&0x3ffffu));
  unsigned n=0, r0=s->r[0],r6=s->r[6],r4=s->r[4],r8=s->r[8],r2=s->r[2],r1=0,t=s->t_flag,c=0;
  unsigned data[4], tail=0;
  while(n<4 && s->icount>=(int)(8*(n+1)) && (pc&0x3ffffu)+16*(n+1)<=0x40000u){
    const unsigned short *q=p+8*n;
    unsigned dt=0;
#ifdef PIXEL_NATIVE_TAIL
    dt=q[7]==0x4710;
#endif
    if(q[0]!=0x612c || q[1]!=0x4218 || q[2]!=0x212b || q[3]!=0x2015 || q[4]!=0x38de || q[5]!=0x6240 || q[6]!=0x34ee || (q[7]!=0x2615 && !dt))break;
    unsigned a=r0-2,b=r6-2;
    if((a&0xdf000000u)!=0x04000000u || (b&0xdf000000u)!=0x04000000u || ((a|b)&1) || (r4&0xdf000000u)!=0x06000000u)break;
    r1=(r2&255)|(r2<<8);data[n]=r1&65535;
    unsigned old=r8,tmp=old+s->r[13];r8=tmp+t;t=(old>tmp)||(tmp>r8);
    c=r4;r2=(unsigned)(int)(signed char)((unsigned char *)s->p_sdram)[(c&0x3ffffu)^1];
    old=c;tmp=old+s->r[14];r4=tmp+t;t=(old>tmp)||(tmp>r4);
    r0=a;if(!dt)r6=b;n++;
    if(dt){tail=1;break;}
  }
  if(!n)return 0;
#ifdef PIXEL_SHADOW_PROOF
  static int guards_checked;
  if(!guards_checked){
    guards_checked=1;
    for(unsigned i=0;i<12;i++){
      SH2 q=*s, saved;
      unsigned qpc=pc;
      switch(i){
      case 0:q.delay=pc;break;
      case 1:q.test_irq=1;break;
      case 2:q.icount=7;break;
      case 3:q.p_sdram=0;break;
      case 4:q.p_dram=0;break;
      case 5:q.r[0]=0x20004002;break;
      case 6:q.r[6]=0x20004002;break;
      case 7:q.r[4]=0x20004000;break;
      case 8:q.r[0]|=1;break;
      case 9:q.r[6]|=1;break;
      case 10:qpc=0x0603fff2;break;
      case 11:qpc=0x02000000;break;
      }
      saved=q;
      if(pixel_native(&q,qpc) || memcmp(&q,&saved,sizeof(q))){printf("PIXEL_GUARD_FAIL %u\n",i);exit(2);}
      pixel_declines++;
    }
  }
  SH2 before,reference;
  unsigned short *slots[8],values[8],expected[8];
  if(pixel_proofs<2048){
    for(unsigned i=0;i<n;i++){
      slots[2*i]=(unsigned short *)s->p_dram+(((s->r[0]-2*(i+1))&0x1ffffu)>>1);
      slots[2*i+1]=(unsigned short *)s->p_dram+(((s->r[6]-2*(i+1))&0x1ffffu)>>1);
    }
    for(unsigned i=0;i<2*n;i++)values[i]=*slots[i];
    before=*s;reference=*s;reference.pc=pc;reference.icount=8*n;
    pixel_proof_active=1;
    sh2_execute_interpreter(&reference,8*n);
    pixel_proof_active=0;
    for(unsigned i=0;i<2*n;i++)expected[i]=*slots[i];
    for(unsigned i=0;i<2*n;i++)*slots[i]=values[i];
    reference.icount=before.icount-8*n;
  }
#endif
  for(unsigned i=0;i<n;i++){
    pixel_store(s,s->r[0]-2*(i+1),data[i]);
    if(!tail || i+1<n)pixel_store(s,s->r[6]-2*(i+1),data[i]);
  }
  s->r[0]=r0;s->r[1]=r1;s->r[2]=r2;s->r[4]=r4;s->r[6]=r6;s->r[8]=r8;s->t_flag=t;
  if(tail){s->r[7]--;s->t_flag=s->r[7]==0;s->no_polling=SH2_NO_POLLING;}
  s->ea=c;s->pc=pc+16*n;s->ppc=s->pc-2;s->icount-=8*n;
#ifdef PIXEL_SHADOW_PROOF
  if(pixel_proofs<2048){
    unsigned bad=0;for(unsigned i=0;i<2*n;i++)bad|=*slots[i]!=expected[i];
    if(memcmp(s,&reference,sizeof(*s)) || bad){
      printf("PIXEL_PROOF_FAIL pc=%08x context=%d memory=%d\n",pc,memcmp(s,&reference,sizeof(*s)),bad);
      exit(2);
    }
    pixel_proofs++;
  }
#endif
  pixel_calls++;pixel_groups+=n;return 8*n;
}

/* Guarded native arithmetic/control patterns, independent of title and PC. */
#include <string.h>
#include <stdlib.h>
static unsigned control_calls[3],control_proofs[3],control_taken[3];
static int control_proof_active;
static int control_ram(unsigned a){return (a&0xdf000000u)==0x06000000u;}
static int control_memory(SH2 *s,unsigned a,unsigned width){
  if(control_ram(a))return s->p_sdram!=0;
  return (a&0xdf000000u)==gnw_fw_rom.region && gnw_fw_rom.base && gnw_sh2_rom_fetch_mask && (width!=4 || !(a&3));
}
static unsigned control_rd32(SH2 *s,unsigned a){
  const unsigned char *base;unsigned off;
  if(control_ram(a)){base=s->p_sdram;off=a&0x3fffcu;}
  else{base=gnw_fw_rom.base;off=a&gnw_fw_rom.mask;}
  unsigned v=*(const unsigned *)(base+off);return (v<<16)|(v>>16);
}
static unsigned control_rd8(SH2 *s,unsigned a){
  const unsigned char *base;unsigned off;
  if(control_ram(a)){base=s->p_sdram;off=a&0x3ffffu;}
  else{base=gnw_fw_rom.base;off=a&gnw_fw_rom.mask;}
  return (unsigned)(int)(signed char)base[off^1];
}
static unsigned control_native(SH2 *s,unsigned pc,unsigned kind){
  if(control_proof_active || s->delay || s->test_irq || !s->p_sdram || !control_ram(pc) || (pc&1))return 0;
  const unsigned short *p=(const unsigned short *)((const unsigned char *)s->p_sdram+(pc&0x3ffffu));
  unsigned cycles,branch=0,r0=s->r[0],r1=s->r[1],r2=s->r[2],r3=s->r[3],r8=s->r[8],r13=s->r[13],macl=s->macl,mach=s->mach;
  if(kind==0){
    if(s->icount<11 || (pc&0x3ffffu)>0x3ffecu)return 0;
    static const unsigned short ops[]={0x201f,0x4129,0x021a,0x201f,0x3c2c,0x021a,0x3b2c,0x4710,0x8fd7,0x7e20};
    for(unsigned i=0;i<10;i++)if(p[i]!=ops[i])return 0;
    r2=(unsigned)((int)(short)r0*(short)r1);r1>>=16;
    macl=(unsigned)((int)(short)r0*(short)r1);
    branch=s->r[7]!=1;cycles=10+branch;
  } else {
    if(s->icount<(kind==1?23:15) || (pc&0x3ffffu)>(kind==1?0x3ffd6u:0x3ffe4u))return 0;
    static const unsigned short ops[]={0x3d88,0x3d1d,0x3d8c,0x51e4,0x001a,0x3012,0x890d,0x52e1,0x680d,0x4029,0x302c,0x6304,0x4819,0x6004,0x3038,0x208f,0x51e0,0x001a,0x4019,0xa004,0x303c};
    unsigned skip=kind==1?0:7;
    for(unsigned i=0;i<21-skip;i++)if(p[i]!=ops[i+skip])return 0;
    if(kind==1){
      unsigned a=s->r[14]+16;
      if(!control_memory(s,a,4))return 0;
      r13-=r8;
      unsigned long long product=(unsigned long long)((long long)(int)r13*(int)r1);
      mach=(unsigned)(product>>32);macl=(unsigned)product;r13+=r8;
      r1=control_rd32(s,a);r0=macl;branch=r0>=r1;
    }
    if(branch)cycles=10;
    else {
      if(!control_memory(s,s->r[14]+4,4) || !control_memory(s,s->r[14],4))return 0;
      r2=control_rd32(s,s->r[14]+4);r8=r0&65535;r0=(r0>>16)+r2;
      if(!control_memory(s,r0,1) || !control_memory(s,r0+1,1))return 0;
      r3=control_rd8(s,r0);
      r0=control_rd8(s,r0+1);
      r8>>=8;r0-=r3;macl=(unsigned)((int)(short)r0*(short)r8);
      r1=control_rd32(s,s->r[14]);r0=(macl>>8)+r3;cycles=kind==1?23:15;
    }
  }
#ifdef CONTROL_SHADOW_PROOF
  SH2 reference;
  if(control_proofs[kind]<2048){
    reference=*s;reference.pc=pc;
    control_proof_active=1;sh2_execute_interpreter(&reference,cycles);control_proof_active=0;
    reference.icount=s->icount-cycles;
  }
#endif
  if(kind==0){
    s->r[1]=r1;s->r[12]+=r2;s->r[2]=macl;s->r[11]+=macl;s->macl=macl;
    s->r[7]--;s->t_flag=!branch;s->no_polling=SH2_NO_POLLING;
    s->r[14]+=32;s->pc=branch?pc-62:pc+20;s->ppc=pc+18;
    if(branch)s->ea=s->pc;
  }else{
    s->r[13]=r13;s->mach=mach;s->macl=macl;s->r[1]=r1;s->r[0]=r0;if(kind==1)s->t_flag=branch;
    if(branch){s->pc=pc+42;s->ppc=pc+12;}
    else{s->r[2]=r2;s->r[3]=r3;s->r[8]=r8;s->pc=pc+(kind==1?50:36);s->ppc=pc+(kind==1?40:26);}
    s->ea=s->pc;
  }
  s->icount-=cycles;
#ifdef CONTROL_SHADOW_PROOF
  if(control_proofs[kind]<2048){
    if(memcmp(s,&reference,sizeof(*s))){
      printf("CONTROL_FAIL kind=%u pc=%08x cycles=%u\n",kind,pc,cycles);
      const unsigned *a=(const unsigned *)s,*b=(const unsigned *)&reference;
      for(unsigned i=0;i<sizeof(*s)/4;i++)if(a[i]!=b[i])printf("DIFF word=%u actual=%08x reference=%08x\n",i,a[i],b[i]);
      exit(2);
    }
    control_proofs[kind]++;
  }
#endif
  control_calls[kind]++;control_taken[kind]+=branch;return cycles;
}
#ifdef CONTROL_SHADOW_PROOF
/* Synthetic inputs use the real core and bus; live game proofs run afterwards. */
void control_synthetic_proof(SH2 *base){
  static unsigned short ram[0x20000];
  static const unsigned short mul[]={0x201f,0x4129,0x021a,0x201f,0x3c2c,0x021a,0x3b2c,0x4710,0x8fd7,0x7e20};
  static const unsigned short coord[]={0x3d88,0x3d1d,0x3d8c,0x51e4,0x001a,0x3012,0x890d,0x52e1,0x680d,0x4029,0x302c,0x6304,0x4819,0x6004,0x3038,0x208f,0x51e0,0x001a,0x4019,0xa004,0x303c};
  static const unsigned edge[]={0,1,2,0x7fff,0x8000,0xffff,0x10000,0x7fffffff,0x80000000,0xffffffff};
  unsigned seed=0x32c04,checks=0,declines=0,branches[3][2]={{0}};
  for(unsigned i=0;i<0x20000;i++)ram[i]=(i*73u)^0x80ffu;
  for(unsigned kind=0;kind<3;kind++){
    memcpy(ram+0x100,kind==0?mul:kind==1?coord:coord+7,kind==0?sizeof(mul):kind==1?sizeof(coord):sizeof(coord)-14);
    for(unsigned i=0;i<4096;i++){
      SH2 a=*base,b,original;unsigned pc=0x06000200u;
      a.p_sdram=ram;a.delay=0;a.test_irq=0;a.pc=pc+2;a.ppc=pc;a.icount=i%32;
      for(unsigned j=0;j<16;j++){seed=seed*1664525u+1013904223u;a.r[j]=i<100?edge[(i+j)%10]:seed;}
      a.t_flag=i&1;a.r[14]=0x06001000u;a.r[7]=i%4==0?1:i%4==1?0:seed;
      unsigned *table=(unsigned *)(ram+0x800);
      table[0]=seed;table[1]=0x00000601; /* CPU_BE2 -> SDRAM 06010000 */
      table[4]=i&1?0xffffffffu:0;
      if(i%16>=1 && i%16<=6)a.icount=64;
      switch(i%16){
      case 1:a.test_irq=1;break;
      case 2:a.delay=pc;break;
      case 3:a.p_sdram=0;break;
      case 4:if(kind)a.r[14]=0x20004000;else pc|=1;break;
      case 5:pc=0x0603fff8u;break;
      case 6:pc=0x02000200u;break;
      }
      original=a;b=a;
      unsigned consumed=control_native(&a,pc,kind);
      if(!consumed){
        if(memcmp(&a,&original,sizeof(a))){printf("CONTROL_DECLINE_FAIL kind=%u case=%u\n",kind,i);exit(2);}
        declines++;continue;
      }
      b.pc=pc;control_proof_active=1;sh2_execute_interpreter(&b,consumed);control_proof_active=0;
      b.icount=original.icount-consumed;
      if(memcmp(&a,&b,sizeof(a))){printf("CONTROL_SYNTH_FAIL kind=%u case=%u\n",kind,i);exit(2);}
      branches[kind][a.t_flag!=0]++;checks++;
    }
  }
  printf("CONTROL_SYNTH checks=%u declines=%u mul_t0=%u mul_t1=%u coord_t0=%u coord_t1=%u interp_t0=%u interp_t1=%u\n",checks,declines,branches[0][0],branches[0][1],branches[1][0],branches[1][1],branches[2][0],branches[2][1]);
  memset(control_calls,0,sizeof(control_calls));memset(control_proofs,0,sizeof(control_proofs));memset(control_taken,0,sizeof(control_taken));
}
#endif

/* Native repeated SDRAM word lookup / framebuffer span. No fixed PC. */
#include <string.h>
#include <stdlib.h>
static unsigned lookup_calls,lookup_groups,lookup_proofs;
static int lookup_proof_active;
static unsigned lookup_native_impl(SH2 *s,unsigned pc){
  if(lookup_proof_active || s->delay || s->test_irq || s->icount<3 || !s->p_sdram || !s->p_dram || (pc&0xdf000000u)!=0x06000000u || (pc&1))return 0;
  unsigned n=0,r0=s->r[0],r1=s->r[1],r6=s->r[6],r8=s->r[8],t=s->t_flag,ea=s->ea;
  unsigned data[8],ops=0;
  const unsigned short *p=(const unsigned short *)((const unsigned char *)s->p_sdram+(pc&0x3ffffu));
  while(n<8){
    unsigned count=n?4:3;
    if(s->icount<(int)(ops+count) || (pc&0x3ffffu)+2*(ops+count)>0x40000u)break;
    const unsigned short *q=p+ops;
    if(n){if(q[0]!=0x38de)break;q++;}
    if(q[0]!=0x010d || q[1]!=0x30ee || q[2]!=0x2615)break;
    unsigned a=r0+r0,b=r6-2;
    if((a&0xdf000000u)!=0x06000000u || (b&0xdf000000u)!=0x04000000u || (b&1))break;
    unsigned old,tmp;
    if(n){old=r8;tmp=old+s->r[13];r8=tmp+t;t=(old>tmp)||(tmp>r8);}
    r1=(unsigned)(int)(short)*(const unsigned short *)((const unsigned char *)s->p_sdram+(a&0x3fffeu));
    old=r0;tmp=old+s->r[14];r0=tmp+t;t=(old>tmp)||(tmp>r0);
    data[n]=r1&65535;r6=b;ea=a;n++;ops+=count;
  }
  if(!n)return 0;
#ifdef LOOKUP_SHADOW_PROOF
  SH2 reference;unsigned short *slots[8],values[8],expected[8];
  if(lookup_proofs<2048){
    for(unsigned i=0;i<n;i++){slots[i]=(unsigned short *)s->p_dram+(((s->r[6]-2*(i+1))&0x1ffffu)>>1);values[i]=*slots[i];}
    reference=*s;reference.pc=pc;
    lookup_proof_active=1;sh2_execute_interpreter(&reference,ops);lookup_proof_active=0;
    reference.icount=s->icount-ops;
    for(unsigned i=0;i<n;i++)expected[i]=*slots[i];
    for(unsigned i=0;i<n;i++)*slots[i]=values[i];
  }
#endif
  for(unsigned i=0;i<n;i++){
    unsigned a=s->r[6]-2*(i+1),d=data[i];
    unsigned short *pd=(unsigned short *)s->p_dram+((a&0x1ffffu)>>1);
    if(a&0x20000u){unsigned old=*pd;if(!(d&255))d|=old&255;if(!(d&0xff00))d|=old&0xff00;}
    *pd=d;
  }
  s->r[0]=r0;s->r[1]=r1;s->r[6]=r6;s->r[8]=r8;s->t_flag=t;s->ea=ea;s->pc=pc+2*ops;s->ppc=s->pc-2;s->icount-=ops;
#ifdef LOOKUP_SHADOW_PROOF
  if(lookup_proofs<2048){
    unsigned bad=0;for(unsigned i=0;i<n;i++)bad|=*slots[i]!=expected[i];
    if(memcmp(s,&reference,sizeof(*s)) || bad){printf("LOOKUP_FAIL pc=%08x context=%d memory=%u\n",pc,memcmp(s,&reference,sizeof(*s)),bad);exit(2);}
    lookup_proofs++;
  }
#endif
  lookup_calls++;lookup_groups+=n;return ops;
}

/* Bounded native chains: parameter loads, computed jump, coordinate, loop tail. */
#include <string.h>
#include <stdlib.h>
static unsigned chain_calls,chain_parts,chain_proofs,inactive_calls,inactive_iterations;
static unsigned inactive_native(SH2 *s,unsigned pc,unsigned first_callback);
static int chain_proof_active;
static unsigned chain_entry(SH2 *s,unsigned pc){
  if(s->delay || s->test_irq || s->icount<5 || !s->p_sdram || !control_ram(pc) || (pc&1) || (pc&0x3ffffu)>0x3fff8u)return 0;
  const unsigned short *p=(const unsigned short *)((const unsigned char *)s->p_sdram+(pc&0x3ffffu));
  if(p[0]!=0x50e6 || p[1]!=0x51e3 || p[2]!=0x402b || p[3]!=0x58e2)return 0;
  unsigned a=s->r[14];
  if(!control_memory(s,a+24,4) || !control_memory(s,a+12,4) || !control_memory(s,a+8,4))return 0;
  unsigned callback=control_rd32(s,a+24);
#ifdef NATIVE_INACTIVE_FOLD
  unsigned folded=inactive_native(s,pc,callback);if(folded)return folded;
#endif
  s->r[0]=callback;s->r[1]=control_rd32(s,a+12);s->r[8]=control_rd32(s,a+8);
  s->pc=s->r[0];s->ppc=pc+6;s->ea=a+8;s->icount-=5;return 5;
}
static unsigned inactive_native(SH2 *s,unsigned pc,unsigned first_callback){
  if(s->icount<17)return 0;
  static const unsigned short idle[]={0xe100,0xe000,0x201f,0x4129,0x021a,0x201f,0x3c2c,0x021a,0x3b2c,0x4710,0x8fd7,0x7e20};
  unsigned n=0,used=0,a=s->r[14],counter=s->r[7],callback=0,last_a=0;
  while(n<48){
    unsigned cost=counter==1?17:18;
    if(s->icount<(int)(used+cost) || !control_memory(s,a+24,4) || !control_memory(s,a+12,4) || !control_memory(s,a+8,4))break;
    unsigned cb=n?control_rd32(s,a+24):first_callback;
    if(!control_ram(cb) || (cb&1) || (cb&0x3ffffu)>0x3ffe8u || cb-58!=pc)break;
    const unsigned short *q=(const unsigned short *)((const unsigned char *)s->p_sdram+(cb&0x3ffffu));
    if(cb!=callback){unsigned j;for(j=0;j<12;j++)if(q[j]!=idle[j])break;if(j!=12)break;}
    callback=cb;last_a=a;a+=32;counter--;used+=cost;n++;
    if(counter==0)break;
  }
  if(!n)return 0;
  s->r[0]=s->r[1]=s->r[2]=s->macl=0;s->r[8]=control_rd32(s,last_a+8);
  s->r[7]=counter;s->r[14]=a;s->t_flag=counter==0;s->no_polling=SH2_NO_POLLING;
  s->pc=counter?pc:callback+24;s->ppc=callback+22;s->ea=counter?pc:last_a+8;s->icount-=used;
  inactive_calls++;inactive_iterations+=n;return used;
}
/* Pattern match on guest code: halfword compare, out at the first mismatch.
 * newlib's memcmp is byte-wise and, on the device, lives in internal flash. */
static inline __attribute__((always_inline)) int hw_eq(const void *a,const unsigned short *b,unsigned n){
  const unsigned short *x=(const unsigned short *)a;
  for(unsigned i=0;i<n;i++)if(x[i]!=b[i])return 0;
  return 1;
}
static unsigned mix_native_impl(SH2 *s,unsigned pc);
static unsigned blitb_native_impl(SH2 *s,unsigned pc);
static unsigned blitc_native_impl(SH2 *s,unsigned pc);
static unsigned blita_native_impl(SH2 *s,unsigned pc);
static unsigned idle_native_impl(SH2 *s,unsigned pc);
static unsigned fill_native_impl(SH2 *s,unsigned pc);
static unsigned chain_native_impl(SH2 *s,unsigned pc);
static unsigned pixel_native_impl(SH2 *s,unsigned pc);
static unsigned lookup_native_impl(SH2 *s,unsigned pc);
static unsigned pcm_tail_native_impl(SH2 *s,unsigned pc);
static unsigned irq_native_impl(SH2 *s,unsigned pc);

#ifdef RIG_NATIVE_COST
#include <stdint.h>
extern uint32_t rig_timer_now(void);
unsigned long long native_cost[16],native_ncall[16];
#define NC_WRAP(i,n) static unsigned n(SH2 *s,unsigned pc){unsigned long long t0=rig_timer_now();unsigned r=n##_impl(s,pc);native_cost[i]+=rig_timer_now()-t0;native_ncall[i]++;return r;}
#else
#define NC_WRAP(i,n) static inline unsigned n(SH2 *s,unsigned pc){return n##_impl(s,pc);}
#endif
NC_WRAP(0,mix_native)
NC_WRAP(1,blitb_native)
NC_WRAP(2,blitc_native)
NC_WRAP(3,blita_native)
NC_WRAP(4,idle_native)
NC_WRAP(5,fill_native)
NC_WRAP(6,chain_native)
NC_WRAP(7,pixel_native)
NC_WRAP(8,lookup_native)
NC_WRAP(9,pcm_tail_native)
NC_WRAP(10,irq_native)
static unsigned chain_native_impl(SH2 *s,unsigned pc){
  if(chain_proof_active)return 0;
  { unsigned m=mix_native(s,pc); if(m)return m; }
#ifdef CHAIN_SHADOW_PROOF
  SH2 before,reference;
  if(chain_proofs<2048)before=*s;
#endif
  unsigned used=chain_entry(s,pc),parts=1;
  if(!used)return 0;
  while(parts<48 && s->icount>=5 && !s->delay && !s->test_irq && s->p_sdram && control_ram(s->pc) && !(s->pc&1)){
    unsigned oldpc=s->pc,oldppc=s->ppc;
    unsigned short op=*(unsigned short *)((unsigned char *)s->p_sdram+(oldpc&0x3ffffu));
    if(op!=0x50e6 && op!=0x3d88 && op!=0x201f && op!=0x52e1)break;
    s->ppc=oldpc;s->pc=oldpc+2;
    unsigned c;
    if(op==0x50e6)c=chain_entry(s,oldpc);
    else c=control_native(s,oldpc,op==0x3d88?1:op==0x52e1?2:0);
    if(!c){s->pc=oldpc;s->ppc=oldppc;break;}
    used+=c;parts++;
  }
#ifdef CHAIN_SHADOW_PROOF
  if(chain_proofs<2048){
    reference=before;reference.pc=pc;
    chain_proof_active=1;control_proof_active=1;
    sh2_execute_interpreter(&reference,used);
    chain_proof_active=0;control_proof_active=0;
    reference.icount=before.icount-used;
    if(memcmp(s,&reference,sizeof(*s))){
      printf("CHAIN_FAIL pc=%08x cycles=%u parts=%u\n",pc,used,parts);
      const unsigned *a=(const unsigned *)s,*b=(const unsigned *)&reference;
      for(unsigned i=0;i<sizeof(*s)/4;i++)if(a[i]!=b[i])printf("DIFF word=%u actual=%08x reference=%08x\n",i,a[i],b[i]);
      exit(2);
    }
    chain_proofs++;
  }
#endif
  chain_calls++;chain_parts+=parts;return used;
}
#ifdef CHAIN_SHADOW_PROOF
void chain_synthetic_proof(SH2 *base){
  static unsigned short ram[0x20000];
  static const unsigned short entry[]={0x50e6,0x51e3,0x402b,0x58e2};
  static const unsigned short idle[]={0xe100,0xe000,0x201f,0x4129,0x021a,0x201f,0x3c2c,0x021a,0x3b2c,0x4710,0x8fd7,0x7e20};
  static const unsigned counters[]={0,1,2,16,0xffffffffu};
  unsigned checks=0,declines=0,seed=0x32f01;
  memcpy(ram+0x100,entry,sizeof(entry));memcpy(ram+0x11d,idle,sizeof(idle));
  for(unsigned i=0;i<64;i++){
    unsigned *p=(unsigned *)(ram+0x800+16*i);
    p[2]=i*0x9876543u;p[3]=~i;p[6]=0x023a0600u;
  }
  for(unsigned i=0;i<1024;i++){
    SH2 a=*base,b,original;unsigned pc=0x06000200u;
    a.p_sdram=ram;a.pc=pc+2;a.ppc=pc;a.delay=a.test_irq=0;a.icount=i%513;
    for(unsigned j=0;j<16;j++){seed=seed*1664525u+1013904223u;a.r[j]=seed;}
    a.r[14]=0x06001000u;a.r[7]=counters[i%5];a.t_flag=i&1;
    switch(i%32){case 1:a.delay=pc;break;case 2:a.test_irq=1;break;case 3:a.p_sdram=0;break;case 4:a.r[14]=0x20004000u;break;case 5:pc|=1;break;}
    original=a;b=a;
    unsigned used=chain_native(&a,pc);
    if(!used){if(memcmp(&a,&original,sizeof(a))){printf("CHAIN_DECLINE_FAIL case=%u\n",i);exit(2);}declines++;continue;}
    b.pc=pc;chain_proof_active=1;control_proof_active=1;sh2_execute_interpreter(&b,used);chain_proof_active=0;control_proof_active=0;b.icount=original.icount-used;
    if(memcmp(&a,&b,sizeof(a))){printf("CHAIN_SYNTH_FAIL case=%u budget=%d cycles=%u\n",i,original.icount,used);exit(2);}
    checks++;
  }
  printf("CHAIN_SYNTH checks=%u declines=%u\n",checks,declines);
  chain_calls=chain_parts=chain_proofs=inactive_calls=inactive_iterations=0;
}
#endif

/* Pure arithmetic/sample limiting tail; leaves ring-buffer/MMIO writes alone. */
#include <string.h>
#include <stdlib.h>
static unsigned pcm_tail_calls,pcm_tail_proofs;
static int pcm_tail_proof_active;
static unsigned pcm_tail_word(SH2 *s,unsigned a){
  const unsigned char *base;unsigned off;
  if(control_ram(a)){base=s->p_sdram;off=a&0x3fffeu;}
  else{base=gnw_fw_rom.base;off=a&gnw_fw_rom.mask;}
  return (unsigned)(int)(short)*(const unsigned short *)(base+off);
}
static unsigned pcm_tail_native_impl(SH2 *s,unsigned pc){
  if(pcm_tail_proof_active || s->delay || s->test_irq || s->icount<25 || !s->p_sdram || !control_ram(pc) || (pc&1) || (pc&0x3ffffu)>0x3ffd6u)return 0;
  static const unsigned short ops[]={0xd21c,0x951c,0x3b2d,0x911e,0x0b0a,0x3b5c,0x3c2d,0x4b15,0x8901,0x31b3,0xeb01,0x8900,0x6b13,0x0c0a,0x3c5c,0x4c15,0x8901,0x31c3,0xec01,0x8900,0x6c13};
  const unsigned short *p=(const unsigned short *)((const unsigned char *)s->p_sdram+(pc&0x3ffffu));
  for(unsigned i=0;i<21;i++)if(p[i]!=ops[i])return 0;
  unsigned a2=((pc+4)&~3u)+0x70,a5=pc+0x3e,a1=pc+0x46;
  if(!control_memory(s,a2,4) || !control_memory(s,a5,2) || !control_memory(s,a1,2))return 0;
  unsigned r2=control_rd32(s,a2),r5=pcm_tail_word(s,a5),r1=pcm_tail_word(s,a1),ea=a1,cycles=10;
  unsigned long long left=(unsigned long long)((long long)(int)s->r[11]*(int)r2),right=(unsigned long long)((long long)(int)s->r[12]*(int)r2);
  unsigned r11=(unsigned)(left>>32)+r5,r12=(unsigned)(right>>32)+r5,t=(int)r11>0;
  if(t){cycles+=3;ea=pc+22;}else{cycles+=3;t=(int)r1>=(int)r11;r11=1;}
  if(t){cycles+=3;ea=pc+26;}else{cycles+=2;r11=r1;}
  cycles+=3;t=(int)r12>0;
  if(t){cycles+=3;ea=pc+38;}else{cycles+=3;t=(int)r1>=(int)r12;r12=1;}
  unsigned ppc;
  if(t){cycles+=3;ea=pc+42;ppc=pc+38;}else{cycles+=2;r12=r1;ppc=pc+40;}
#ifdef PCM_TAIL_SHADOW_PROOF
  SH2 reference;
  if(pcm_tail_proofs<2048){reference=*s;reference.pc=pc;pcm_tail_proof_active=1;sh2_execute_interpreter(&reference,cycles);pcm_tail_proof_active=0;reference.icount=s->icount-cycles;}
#endif
  s->r[1]=r1;s->r[2]=r2;s->r[5]=r5;s->r[11]=r11;s->r[12]=r12;s->macl=(unsigned)right;s->mach=(unsigned)(right>>32);s->t_flag=t;s->ea=ea;s->pc=pc+42;s->ppc=ppc;s->icount-=cycles;
#ifdef PCM_TAIL_SHADOW_PROOF
  if(pcm_tail_proofs<2048){if(memcmp(s,&reference,sizeof(*s))){printf("PCM_TAIL_FAIL pc=%08x cycles=%u\n",pc,cycles);exit(2);}pcm_tail_proofs++;}
#endif
  pcm_tail_calls++;return cycles;
}
#ifdef PCM_TAIL_SHADOW_PROOF
void pcm_tail_synthetic_proof(SH2 *base){
  static unsigned short ram[0x1000];
  static const unsigned short ops[]={0xd21c,0x951c,0x3b2d,0x911e,0x0b0a,0x3b5c,0x3c2d,0x4b15,0x8901,0x31b3,0xeb01,0x8900,0x6b13,0x0c0a,0x3c5c,0x4c15,0x8901,0x31c3,0xec01,0x8900,0x6c13};
  static const unsigned edge[]={0,1,2,0x7fff,0x8000,0xffff,0x10000,0x7fffffff,0x80000000,0xffffffff};
  unsigned seed=0x32b15,checks=0,declines=0;
  memcpy(ram+0x100,ops,sizeof(ops));
  for(unsigned i=0;i<4096;i++){
    SH2 a=*base,b,original;unsigned pc=0x06000200u;
    a.p_sdram=ram;a.pc=pc+2;a.ppc=pc;a.delay=a.test_irq=0;a.icount=i%32;a.t_flag=i&1;
    seed=seed*1664525u+1013904223u;a.r[11]=i<100?edge[i/10]:seed;
    seed=seed*1664525u+1013904223u;a.r[12]=i<100?edge[i%10]:seed;
    unsigned gain=i<100?edge[(i/10+i%10)%10]:seed;
    *(unsigned *)(ram+0x13a)=(gain<<16)|(gain>>16);
    ram[0x11f]=(unsigned short)(seed>>8);ram[0x123]=(unsigned short)seed;
    unsigned g=i%16;
    if(g>=1 && g<=5)a.icount=64;
    switch(g){case 1:a.delay=pc;break;case 2:a.test_irq=1;break;case 3:a.p_sdram=0;break;case 4:pc|=1;break;case 5:pc=0x02000200u;break;}
    original=a;b=a;unsigned used=pcm_tail_native(&a,pc);
    if(!used){if(memcmp(&a,&original,sizeof(a))){printf("PCM_TAIL_DECLINE_FAIL case=%u\n",i);exit(2);}declines++;continue;}
    b.pc=pc;pcm_tail_proof_active=1;sh2_execute_interpreter(&b,used);pcm_tail_proof_active=0;b.icount=original.icount-used;
    if(memcmp(&a,&b,sizeof(a))){printf("PCM_TAIL_SYNTH_FAIL case=%u cycles=%u\n",i,used);exit(2);}checks++;
  }
  printf("PCM_TAIL_SYNTH checks=%u declines=%u\n",checks,declines);
  pcm_tail_calls=pcm_tail_proofs=0;
}
#endif

/* Whole-sample PCM mixer (After Burner class sound driver, opcode-pattern
 * recognised, no title/PC whitelist): chain entry -> r7 voice callbacks
 * (active interpolator or retired zero) -> scale/limit tail -> ring store.
 * One native call per output sample instead of ~28 interpreter re-entries.
 * Everything is computed into locals first; any guard failure declines with
 * the guest state untouched. Cycle charges mirror the interpreter's per-op
 * costs (1/op, +1 BRA/BFS-taken/DMULS/MULS, +2 BT/BF-taken, JMP 2). */
static unsigned mix_calls,mix_proofs,mix_voice_total,mix_decl[8];
#ifdef RIG_SH2_COUNT
unsigned long long mix_slave_bucket[8],mix_budget_hist[8],mix_master_bucket[8],mix_pcbin[2][2048];
#endif
static int mix_proof_active;
static unsigned __attribute__((noinline)) mix_native_impl(SH2 *s,unsigned pc){
  if(mix_proof_active || s->delay || s->test_irq || !s->p_sdram || !control_ram(pc) || (pc&1))return 0;
  unsigned n=s->r[7];
  if(n==0 || n>48){mix_decl[0]++;return 0;}
#ifdef RIG_MIX_DIAG
  { extern unsigned long long mix_budget_hist[8]; int b=s->icount; mix_budget_hist[b<45?0:b<100?1:b<200?2:b<400?3:b<700?4:5]++; if(n==16)mix_budget_hist[6]++; }
#endif
  if(s->icount<=0){mix_decl[1]++;return 0;}
  if((pc&0x3ffffu)>0x3ffffu-0xa0)return 0;
  const unsigned short *p=(const unsigned short *)((const unsigned char *)s->p_sdram+(pc&0x3ffffu));
  static const unsigned short entry[]={0x50e6,0x51e3,0x402b,0x58e2};
  static const unsigned short active[]={0x3d88,0x3d1d,0x3d8c,0x51e4,0x001a,0x3012,0x890d,0x52e1,0x680d,0x4029,0x302c,0x6304,0x4819,0x6004,0x3038,0x208f,0x51e0,0x001a,0x4019,0xa004,0x303c};
  static const unsigned short retire[]={0xd01a,0x1e06,0xe100,0xe000};
  static const unsigned short tail[]={0x201f,0x4129,0x021a,0x201f,0x3c2c,0x021a,0x3b2c,0x4710,0x8fd7,0x7e20};
  static const unsigned short limit[]={0xd21c,0x951c,0x3b2d,0x911e,0x0b0a,0x3b5c,0x3c2d,0x4b15,0x8901,0x31b3,0xeb01,0x8900,0x6b13,0x0c0a,0x3c5c,0x4c15,0x8901,0x31c3,0xec01,0x8900,0x6c13};
  static const unsigned short ring[]={0xd80e,0xd906,0x6081,0x09b5,0x7002,0x09c5,0x7002,0x600c,0xa1d7,0x2801};
  if(!hw_eq(p,entry,sizeof entry/2)||!hw_eq(p+6,active,sizeof active/2)||!hw_eq(p+27,retire,sizeof retire/2)
     ||!hw_eq(p+31,tail,sizeof tail/2)||!hw_eq(p+41,limit,sizeof limit/2)||!hw_eq(p+62,ring,sizeof ring/2))return 0;
  unsigned cb_active=pc+12,cb_retired=pc+0x3a,tp=pc+0x52,rp=pc+0x7c;
  unsigned lit_retire=((pc+0x36+4)&~3u)+0x1a*4,lit8=((rp+4)&~3u)+0x0e*4,lit9=((rp+2+4)&~3u)+0x06*4;
  unsigned ta2=((tp+4)&~3u)+0x70,ta5=tp+0x3e,ta1=tp+0x46;
  if(!control_memory(s,lit_retire,4)||!control_memory(s,lit8,4)||!control_memory(s,lit9,4)
     ||!control_memory(s,ta2,4)||!control_memory(s,ta5,2)||!control_memory(s,ta1,2))return 0;
  if(control_rd32(s,lit_retire)!=cb_retired)return 0;
  /* Voices are folded while the budget holds a worst-case voice (39); the
   * tail needs a further 39 (28 + ring 11). A partial fold stops on the loop
   * boundary exactly as the interpreter's bf/s would. */
  unsigned a=s->r[14],r13=s->r[13],r11=s->r[11],r12=s->r[12],r3=s->r[3],mach=s->mach,cycles=0,k=0;
  unsigned r0=s->r[0],r1=s->r[1],r2=s->r[2],r8=s->r[8],macl=s->macl,last_ea=s->ea;
  unsigned long long retired=0;
  int budget=s->icount;
  /* Interpreter rule: an instruction runs whenever icount > 0 and may push it
   * negative; the overrun is settled by the next slice. Folding a whole voice
   * when its first instruction would run overruns by at most one voice (38). */
  for(;k<n && (int)cycles<budget;k++,a+=32){
    /* voice record: 32 aligned bytes of SDRAM, checked once, read directly */
    if((a&0xdf000003u)!=0x06000000u||(a&0x3ffffu)>0x40000u-32)break;
    const unsigned *vr=(const unsigned *)((const unsigned char *)s->p_sdram+(a&0x3ffffu));
#define MIX_RD(i) ((vr[i]<<16)|(vr[i]>>16))
    unsigned cb=MIX_RD(6);
    unsigned v1=MIX_RD(3),v8=MIX_RD(2),v0,vmacl;
    if(cb==cb_active){
      unsigned long long prod=(unsigned long long)((long long)(int)(r13-v8)*(int)v1);
      unsigned vmach=(unsigned)(prod>>32);vmacl=(unsigned)prod;
      unsigned e=MIX_RD(4);v0=vmacl;
      if(v0>=e){
        retired|=1ull<<k;v1=0;v0=0;mach=vmach;cycles+=5+10+4;last_ea=a+24;
      }else{
        unsigned v2=MIX_RD(1);v8=v0&65535;v0=(v0>>16)+v2;
        if(!control_memory(s,v0,1)||!control_memory(s,v0+1,1))break;
        r3=control_rd8(s,v0);v0=control_rd8(s,v0+1);v8>>=8;v0-=r3;
        vmacl=(unsigned)((int)(short)v0*(short)v8);v1=MIX_RD(0);v0=(vmacl>>8)+r3;mach=vmach;cycles+=5+23;last_ea=pc+0x3e;
      }
    }else if(cb==cb_retired){v1=0;v0=0;cycles+=5+2;last_ea=a+8;}
    else {mix_decl[2]++;break;}
    r2=(unsigned)((int)(short)v0*(short)v1);v1>>=16;macl=(unsigned)((int)(short)v0*(short)v1);
    r12+=r2;r11+=macl;r2=macl;r0=v0;r1=v1;r8=v8;cycles+=10+(k!=n-1);
#undef MIX_RD
  }
  if(!k){mix_decl[1]++;return 0;}
  unsigned full=(k==n && (int)cycles<budget);
  unsigned r5=0,r9=0,t=0,tr8=0;
  if(full){
    r2=control_rd32(s,ta2);r5=pcm_tail_word(s,ta5);r1=pcm_tail_word(s,ta1);
    unsigned long long left=(unsigned long long)((long long)(int)r11*(int)r2),right=(unsigned long long)((long long)(int)r12*(int)r2);
    r11=(unsigned)(left>>32)+r5;r12=(unsigned)(right>>32)+r5;t=(int)r11>0;cycles+=10;
    if(t)cycles+=3;else{cycles+=3;t=(int)r1>=(int)r11;r11=1;}
    if(t)cycles+=3;else{cycles+=2;r11=r1;}
    cycles+=3;t=(int)r12>0;
    if(t)cycles+=3;else{cycles+=3;t=(int)r1>=(int)r12;r12=1;}
    if(t)cycles+=3;else{cycles+=2;r12=r1;}
    macl=(unsigned)right;mach=(unsigned)(right>>32);
    tr8=control_rd32(s,lit8);r9=control_rd32(s,lit9);cycles+=11;
  }
#ifdef MIX_SHADOW_PROOF
  SH2 reference;unsigned short ring_was[2],ring_exp[2],*rs=0;unsigned cb_was[48],cb_exp[48];int proof=mix_proofs<2048;
  if(proof){
    unsigned ra=full?r9+(unsigned)(int)(short)RW(s,tr8):0;
    if(full && (!control_ram(ra)||!control_ram(ra+3)))proof=0;
    else{
      if(full){rs=(unsigned short *)((unsigned char *)s->p_sdram+(ra&0x3fffeu));ring_was[0]=rs[0];ring_was[1]=rs[1];}
      for(unsigned i=0;i<k;i++)cb_was[i]=control_rd32(s,s->r[14]+32*i+24);
      reference=*s;reference.pc=pc;
      mix_proof_active=1;chain_proof_active=1;control_proof_active=1;pcm_tail_proof_active=1;
      sh2_execute_interpreter(&reference,cycles);
      mix_proof_active=0;chain_proof_active=0;control_proof_active=0;pcm_tail_proof_active=0;
      reference.icount=s->icount-cycles;
      if(full){ring_exp[0]=rs[0];ring_exp[1]=rs[1];rs[0]=ring_was[0];rs[1]=ring_was[1];}
      for(unsigned i=0;i<k;i++){cb_exp[i]=control_rd32(s,s->r[14]+32*i+24);if(cb_exp[i]!=cb_was[i])WL(s,s->r[14]+32*i+24,cb_was[i]);}
    }
  }
#endif
  a=s->r[14];
  for(unsigned i=0;i<k;i++)if(retired>>i&1)WL(s,a+32*i+24,cb_retired);
  s->r[3]=r3;s->r[11]=r11;s->r[12]=r12;s->r[14]=a+32*k;s->r[7]=n-k;s->mach=mach;
  /* DT set this before any ring access; poll detection may clear it again. */
  s->no_polling=SH2_NO_POLLING;
  if(full){
    unsigned idx=(unsigned)(int)(short)RW(s,tr8);
    WW(s,r9+idx,r11&0xffffu);idx+=2;WW(s,r9+idx,r12&0xffffu);idx+=2;idx&=0xffu;
    WW(s,tr8,idx);
    s->r[0]=idx;s->r[1]=r1;s->r[2]=r2;s->r[5]=r5;s->r[8]=tr8;s->r[9]=r9;s->macl=macl;
    s->t_flag=t;s->ea=tr8;s->pc=rp+0x3c2;s->ppc=rp+18;
  }else{
    s->r[0]=r0;s->r[1]=r1;s->r[2]=r2;s->r[8]=r8;s->macl=macl;s->ppc=pc+0x50;
    /* all voices done but no budget for the tail: the dt fell to zero, bf/s not taken */
    if(k==n){s->t_flag=1;s->ea=last_ea;s->pc=tp;}
    else{s->t_flag=0;s->ea=pc;s->pc=pc;}
  }
  s->icount-=cycles;
#ifdef MIX_SHADOW_PROOF
  if(proof){
    unsigned bad=0;
    if(full && (rs[0]!=ring_exp[0]||rs[1]!=ring_exp[1]))bad|=1;
    for(unsigned i=0;i<k;i++)if(control_rd32(s,a+32*i+24)!=cb_exp[i])bad|=2;
    if(memcmp(s,&reference,sizeof *s)||bad){
      printf("MIX_FAIL pc=%08x cycles=%u n=%u k=%u full=%u bad=%u\n",pc,cycles,n,k,full,bad);
      const unsigned *x=(const unsigned *)s,*y=(const unsigned *)&reference;
      for(unsigned i=0;i<sizeof *s/4;i++)if(x[i]!=y[i])printf("DIFF word=%u native=%08x reference=%08x\n",i,x[i],y[i]);
      exit(2);
    }
    mix_proofs++;
  }
#endif
  mix_calls++;mix_voice_total+=k;if(!full)mix_decl[3]++;return cycles;
}
/* Scaled sprite line writer, variant C (byte source -> doubled word into two
 * framebuffer rows). Folds whole 8-pixel groups, the dt/bf-s loop back, the
 * per-line tail (next source row, next dest rows) and the head (Duff entry)
 * in one call, stopping only on interpreter-reconstructible boundaries.
 * Group budget follows the interpreter rule: a group runs when icount > 0. */
static unsigned blitc_calls,blitc_groups,blitc_lines,blitc_proofs,blitc_decl[4];
static int blitc_proof_active;
#define BLITC_CAP 32
static unsigned __attribute__((noinline,unused)) blitc_native_impl(SH2 *s,unsigned pc){
  if(blitc_proof_active || s->delay || s->test_irq || s->icount<=0 || !s->p_sdram || !s->p_dram)return 0;
  if((pc&0xdf000000u)!=0x06000000u || (pc&0x3ffffu)>0x3fe00u || (pc&0x3ffffu)<0x200u || (pc&1))return 0;
  const unsigned char *sd=(const unsigned char *)s->p_sdram;
  /* Find the loop end forward of pc: groups of 8, the last ending in dt r7 / bf/s. */
  unsigned end=0;
  for(unsigned g=0;g<8;g++){
    const unsigned short *q=(const unsigned short *)(sd+((pc+16*g)&0x3ffffu));
    if(q[0]!=0x612c||q[1]!=0x4218||q[2]!=0x212b||q[3]!=0x2015||q[4]!=0x38de||q[5]!=0x6240||q[6]!=0x34ee)return 0;
    if(q[7]==0x4710){end=pc+16*g;break;}
    if(q[7]!=0x2615)return 0;
  }
  if(!end)return 0;
  const unsigned short *le=(const unsigned short *)(sd+((end+16)&0x3ffffu));   /* bf/s, delay */
  if((le[0]&0xff00)!=0x8f00 || le[1]!=0x2615)return 0;
  unsigned body=end+16+4+2*(int)(signed char)(le[0]&0xff);
  if(body>pc || (pc-body)&15 || (pc-body)>=8*16)return 0;
  for(unsigned a=body;a<pc;a+=16){                        /* groups before pc must be groups too */
    const unsigned short *q=(const unsigned short *)(sd+(a&0x3ffffu));
    if(q[0]!=0x612c||q[7]!=0x2615)return 0;
  }
  unsigned tail=end+16+4;
  const unsigned short *tp=(const unsigned short *)(sd+(tail&0x3ffffu));
  static const unsigned short tailops[]={0x4b10,0x62f2,0x339c,0x960b,0x6039,0x220f,0x68e3,0x52f1,0x356c,0x041a,0x8fa7,0x34cc};
  if(!hw_eq(tp,tailops,sizeof tailops/2))return 0;
  unsigned lit6=tail+6+2+2+0x0b*2;                          /* mov.w @(disp,pc) at tail+6 */
  unsigned head=tail+20+4+2*(int)(signed char)(tp[10]&0xff);
  const unsigned short *hp=(const unsigned short *)(sd+(head&0x3ffffu));
  static const unsigned short headops[]={0x9005,0x4818,0x67a3,0x6653,0x305c,0x422b,0x6240};
  if(!hw_eq(hp,headops,sizeof headops/2))return 0;
  unsigned lit0=head+2+2+0x05*2;
  unsigned exitpc=tail+24;
  unsigned r15=s->r[15];
  if(!control_memory(s,r15,4)||!control_memory(s,r15+4,4)||!control_memory(s,lit6,2)||!control_memory(s,lit0,2))return 0;
  unsigned r0=s->r[0],r1=s->r[1],r2=s->r[2],r3=s->r[3],r4=s->r[4],r5=s->r[5],r6=s->r[6],r7=s->r[7],r8=s->r[8],r11=s->r[11];
  unsigned r9=s->r[9],r12=s->r[12],r13=s->r[13],r14=s->r[14],r10=s->r[10],macl=s->macl,t=s->t_flag,ea=s->ea,ppc=pc-2,cur=pc;
  unsigned cycles=0,nw=0,ng=0,nl=0,np=0,stop=0,dt=0;int budget=s->icount;
  unsigned waddr[2*BLITC_CAP],wval[2*BLITC_CAP];
  while(!stop && ng<BLITC_CAP){
    if((int)(cycles+(cur==end?11:8))>budget)break;   /* whole group (and its loop back) must fit */
    unsigned a=r0-2,b=r6-2;
    if((a&0xdf000000u)!=0x04000000u||(b&0xdf000000u)!=0x04000000u||((a|b)&1)||(r4&0xdf000000u)!=0x06000000u){blitc_decl[0]++;break;}
    r1=(r2&255)|(r2<<8);waddr[nw]=a;wval[nw++]=r1&65535;
    unsigned old=r8,tmp=old+r13;r8=tmp+t;t=(old>tmp)||(tmp>r8);
    unsigned c=r4;r2=(unsigned)(int)(signed char)sd[(c&0x3ffffu)^1];ea=c;
    old=c;tmp=old+r14;r4=tmp+t;t=(old>tmp)||(tmp>r4);
    r0=a;ng++;
    if(cur!=end){waddr[nw]=b;wval[nw++]=r1&65535;r6=b;cycles+=8;cur+=16;ppc=cur-2;continue;}
    /* last group: dt r7, bf/s body, delay mov.w r1,@-r6 */
    r7--;t=(r7==0);cycles+=8;dt=1;
    waddr[nw]=b;wval[nw++]=r1&65535;r6=b;
    if(r7){cycles+=2+1;cur=body;ea=body;ppc=end+16+2;np=1;continue;}
    cycles+=1+1;cur=tail;ppc=end+16+2;
    if((int)(cycles+13)>budget){stop=1;break;}
    /* tail */
    r11--;t=(r11==0);cycles+=1;
    r2=control_rd32(s,r15);ea=r15;r3+=r9;
    r6=(unsigned)(int)(short)*(const unsigned short *)(sd+(lit6&0x3fffeu));ea=lit6;
    r0=(r3>>16)|(r3<<16);macl=(unsigned)((int)(short)r0*(short)r2);r8=r14;
    r2=control_rd32(s,r15+4);ea=r15+4;r5+=r6;r4=macl;cycles+=9;nl++;
    if(!r11){cycles+=1+1;r4+=r12;cur=exitpc;ppc=tail+22;stop=2;break;}
    cycles+=2+1;r4+=r12;cur=head;ea=head;ppc=tail+22;
    if((r4&0xdf000000u)!=0x06000000u||(int)(cycles+8)>budget){stop=1;break;}
    /* head */
    r0=(unsigned)(int)(short)*(const unsigned short *)(sd+(lit0&0x3fffeu));ea=lit0;
    r8<<=8;r7=r10;r6=r5;r0+=r5;
    unsigned entry=r2;ea=entry;
    if(entry<body||entry>end||((entry-body)&15)){blitc_decl[1]++;stop=3;break;}
    c=r4;r2=(unsigned)(int)(signed char)sd[(c&0x3ffffu)^1];ea=c;
    cycles+=1+1+1+1+1+2+1;cur=entry;ppc=head+12;
  }
  if(stop==3){
    /* head partially applied is not reconstructible: undo by declining the whole call */
    blitc_decl[2]++;return 0;
  }
  if(!ng)return 0;
  (void)np;
#ifdef BLITC_SHADOW_PROOF
  SH2 reference;unsigned short values[2*BLITC_CAP],expected[2*BLITC_CAP];int proof=blitc_proofs<2048;
  if(proof){
    for(unsigned i=0;i<nw;i++)values[i]=*((unsigned short *)s->p_dram+((waddr[i]&0x1ffffu)>>1));
    reference=*s;reference.pc=pc;
    blitc_proof_active=1;pixel_proof_active=1;
    sh2_execute_interpreter(&reference,cycles);
    blitc_proof_active=0;pixel_proof_active=0;
    reference.icount=s->icount-cycles;
    for(unsigned i=0;i<nw;i++){unsigned short *pd=(unsigned short *)s->p_dram+((waddr[i]&0x1ffffu)>>1);expected[i]=*pd;}
    for(unsigned i=nw;i-->0;)*((unsigned short *)s->p_dram+((waddr[i]&0x1ffffu)>>1))=values[i];
  }
#endif
  for(unsigned i=0;i<nw;i++){
    unsigned a=waddr[i],d=wval[i];
    unsigned short *pd=(unsigned short *)s->p_dram+((a&0x1ffffu)>>1);
    if(a&0x20000u){unsigned o=*pd;if(!(d&255))d|=o&255;if(!(d&0xff00))d|=o&0xff00;}
    *pd=d;
  }
  s->r[0]=r0;s->r[1]=r1;s->r[2]=r2;s->r[3]=r3;s->r[4]=r4;s->r[5]=r5;s->r[6]=r6;s->r[7]=r7;s->r[8]=r8;s->r[11]=r11;
  s->macl=macl;s->t_flag=t;s->ea=ea;s->pc=cur;s->ppc=ppc;if(dt)s->no_polling=SH2_NO_POLLING;s->icount-=cycles;
#ifdef BLITC_SHADOW_PROOF
  if(proof){
    unsigned bad=0;
    for(unsigned i=0;i<nw;i++)if(*((unsigned short *)s->p_dram+((waddr[i]&0x1ffffu)>>1))!=expected[i])bad=1;
    if(memcmp(s,&reference,sizeof *s)||bad){
      printf("BLITC_FAIL pc=%08x cycles=%u groups=%u lines=%u stop=%u bad=%u\n",pc,cycles,ng,nl,stop,bad);
      const unsigned *x=(const unsigned *)s,*y=(const unsigned *)&reference;
      for(unsigned i=0;i<sizeof *s/4;i++)if(x[i]!=y[i])printf("DIFF word=%u native=%08x reference=%08x\n",i,x[i],y[i]);
      exit(2);
    }
    blitc_proofs++;
  }
#endif
  blitc_calls++;blitc_groups+=ng;blitc_lines+=nl;return cycles;
}
#ifdef RIG_SH2_COUNT
void blitc_report(void){printf("BLITC calls=%u groups=%u lines=%u proofs=%u decl_fb=%u decl_entry=%u decl_head=%u\n",blitc_calls,blitc_groups,blitc_lines,blitc_proofs,blitc_decl[0],blitc_decl[1],blitc_decl[2]);}
#endif
/* Scaled sprite line writer, variant A (16-bit lookup source -> one
 * framebuffer row). Entered on the mov.w @(r0,r0),r1 of a group, like
 * lookup_native; folds groups, the dt/bf-s loop back, the per-line tail and
 * the head (Duff entry via r4) in one call, stopping only on
 * interpreter-reconstructible boundaries. Group budget: interpreter rule. */
static unsigned blita_calls,blita_groups,blita_lines,blita_proofs,blita_decl[4];
static int blita_proof_active;
#define BLITA_CAP 32
static unsigned __attribute__((noinline,unused)) blita_native_impl(SH2 *s,unsigned pc){
  if(blita_proof_active || s->delay || s->test_irq || s->icount<=0 || !s->p_sdram || !s->p_dram)return 0;
  if((pc&0xdf000000u)!=0x06000000u || (pc&0x3ffffu)>0x3fe00u || (pc&0x3ffffu)<0x200u || (pc&1))return 0;
  const unsigned char *sd=(const unsigned char *)s->p_sdram;
  /* pc points at 010d (second op of a 4-op group). Group base = pc-2. */
  unsigned gp=pc-2,end=0;
  for(unsigned g=0;g<8;g++){
    const unsigned short *q=(const unsigned short *)(sd+((gp+8*g)&0x3ffffu));
    if(q[0]!=0x38de||q[1]!=0x010d||q[2]!=0x30ee)return 0;
    if(q[3]==0x4710){end=gp+8*g;break;}
    if(q[3]!=0x2615)return 0;
  }
  if(!end)return 0;
  const unsigned short *le=(const unsigned short *)(sd+((end+8)&0x3ffffu));   /* bf/s, delay */
  if((le[0]&0xff00)!=0x8f00 || le[1]!=0x2615)return 0;
  unsigned body=end+8+4+2*(int)(signed char)(le[0]&0xff);
  if(body>gp || (gp-body)&7 || (gp-body)>=8*8)return 0;
  for(unsigned a=body;a<gp;a+=8){
    const unsigned short *q=(const unsigned short *)(sd+(a&0x3ffffu));
    if(q[0]!=0x38de||q[3]!=0x2615)return 0;
  }
  unsigned tail=end+8+4;
  const unsigned short *tp=(const unsigned short *)(sd+(tail&0x3ffffu));
  static const unsigned short tailops[]={0x339c,0x6039,0x220f,0xe602,0x4618,0x001a,0x30cc,0x4001,0x4b10,0x8fca,0x356c};
  if(!hw_eq(tp,tailops,sizeof tailops/2))return 0;
  unsigned head=tail+18+4+2*(int)(signed char)(tp[9]&0xff);
  const unsigned short *hp=(const unsigned short *)(sd+(head&0x3ffffu));
  static const unsigned short headops[]={0x57f0,0x6653,0x58f1,0x307c,0x442b,0x67a3};
  if(!hw_eq(hp,headops,sizeof headops/2))return 0;
  unsigned exitpc=tail+22;
  unsigned r15=s->r[15];
  if(!control_memory(s,r15,4)||!control_memory(s,r15+4,4))return 0;
  unsigned r0=s->r[0],r1=s->r[1],r2=s->r[2],r3=s->r[3],r4=s->r[4],r5=s->r[5],r6=s->r[6],r7=s->r[7],r8=s->r[8],r11=s->r[11];
  unsigned r9=s->r[9],r12=s->r[12],r13=s->r[13],r14=s->r[14],r10=s->r[10],macl=s->macl,t=s->t_flag,ea=s->ea,ppc=pc-2,cur=gp;
  unsigned cycles=0,nw=0,ng=0,nl=0,stop=0,first=1,dt=0;int budget=s->icount;
  unsigned waddr[BLITA_CAP],wval[BLITA_CAP];
  while(!stop && ng<BLITA_CAP){
    if((int)(cycles+(first?3:4)+(cur==end?3:0))>budget)break;
    if(!first){unsigned old=r8,tmp=old+r13;r8=tmp+t;t=(old>tmp)||(tmp>r8);cycles+=1;}
    unsigned la=r0+r0,b=r6-2;
    if((la&0xdf000000u)!=0x06000000u||(la&1)||(b&0xdf000000u)!=0x04000000u||(b&1)){blita_decl[0]++;break;}
    r1=(unsigned)(int)(short)*(const unsigned short *)(sd+(la&0x3fffeu));ea=la;
    unsigned old=r0,tmp=old+r14;r0=tmp+t;t=(old>tmp)||(tmp>r0);
    cycles+=2;ng++;first=0;
    if(cur!=end){waddr[nw]=b;wval[nw++]=r1&65535;r6=b;cycles+=1;cur+=8;ppc=cur-2;continue;}
    /* last group: dt r7, bf/s body, delay mov.w r1,@-r6 */
    r7--;t=(r7==0);cycles+=1;dt=1;
    waddr[nw]=b;wval[nw++]=r1&65535;r6=b;
    if(r7){cycles+=2+1;cur=body;ea=body;ppc=end+8+2;continue;}
    cycles+=1+1;cur=tail;ppc=end+8+2;
    if((int)(cycles+12)>budget){stop=1;break;}
    /* tail */
    r3+=r9;r0=(r3>>16)|(r3<<16);macl=(unsigned)((int)(short)r0*(short)r2);
    r6=2;r6<<=8;r0=macl;r0+=r12;t=r0&1;r0>>=1;
    r11--;t=(r11==0);cycles+=9;nl++;
    if(!r11){cycles+=1+1;r5+=r6;cur=exitpc;ppc=tail+20;stop=2;break;}
    cycles+=2+1;r5+=r6;cur=head;ea=head;ppc=tail+20;
    if((int)(cycles+7)>budget){stop=1;break;}
    /* head */
    r7=control_rd32(s,r15);ea=r15;r6=r5;r8=control_rd32(s,r15+4);ea=r15+4;r0+=r7;
    unsigned entry=r4;ea=entry;
    if(entry<body||entry>end||((entry-body)&7)){blita_decl[1]++;stop=3;break;}
    r7=r10;cycles+=1+1+1+1+2+1;cur=entry;ppc=head+10;
  }
  if(stop==3){blita_decl[2]++;return 0;}
  if(!ng)return 0;
#ifdef BLITA_SHADOW_PROOF
  SH2 reference;unsigned short values[BLITA_CAP],expected[BLITA_CAP];int proof=blita_proofs<2048;
  if(proof){
    for(unsigned i=0;i<nw;i++)values[i]=*((unsigned short *)s->p_dram+((waddr[i]&0x1ffffu)>>1));
    reference=*s;reference.pc=pc;
    blita_proof_active=1;lookup_proof_active=1;
    sh2_execute_interpreter(&reference,cycles);
    blita_proof_active=0;lookup_proof_active=0;
    reference.icount=s->icount-cycles;
    for(unsigned i=0;i<nw;i++)expected[i]=*((unsigned short *)s->p_dram+((waddr[i]&0x1ffffu)>>1));
    for(unsigned i=nw;i-->0;)*((unsigned short *)s->p_dram+((waddr[i]&0x1ffffu)>>1))=values[i];
  }
#endif
  for(unsigned i=0;i<nw;i++){
    unsigned a=waddr[i],d=wval[i];
    unsigned short *pd=(unsigned short *)s->p_dram+((a&0x1ffffu)>>1);
    if(a&0x20000u){unsigned o=*pd;if(!(d&255))d|=o&255;if(!(d&0xff00))d|=o&0xff00;}
    *pd=d;
  }
  s->r[0]=r0;s->r[1]=r1;s->r[3]=r3;s->r[5]=r5;s->r[6]=r6;s->r[7]=r7;s->r[8]=r8;s->r[11]=r11;
  s->macl=macl;s->t_flag=t;s->ea=ea;s->pc=cur;s->ppc=ppc;if(dt)s->no_polling=SH2_NO_POLLING;s->icount-=cycles;
  (void)r2;(void)r4;
#ifdef BLITA_SHADOW_PROOF
  if(proof){
    unsigned bad=0;
    for(unsigned i=0;i<nw;i++)if(*((unsigned short *)s->p_dram+((waddr[i]&0x1ffffu)>>1))!=expected[i])bad=1;
    if(memcmp(s,&reference,sizeof *s)||bad){
      printf("BLITA_FAIL pc=%08x cycles=%u groups=%u lines=%u stop=%u bad=%u\n",pc,cycles,ng,nl,stop,bad);
      const unsigned *x=(const unsigned *)s,*y=(const unsigned *)&reference;
      for(unsigned i=0;i<sizeof *s/4;i++)if(x[i]!=y[i])printf("DIFF word=%u native=%08x reference=%08x\n",i,x[i],y[i]);
      exit(2);
    }
    blita_proofs++;
  }
#endif
  blita_calls++;blita_groups+=ng;blita_lines+=nl;return cycles;
}
#ifdef RIG_SH2_COUNT
void blita_report(void){printf("BLITA calls=%u groups=%u lines=%u proofs=%u decl_src=%u decl_entry=%u decl_head=%u\n",blita_calls,blita_groups,blita_lines,blita_proofs,blita_decl[0],blita_decl[1],blita_decl[2]);}
#endif
/* Scaled sprite line writer, variant B (16-bit lookup, byte-swapped word
 * into one framebuffer row; software-pipelined body of eight 5-op units in
 * two alternating orders, Duff entry on a unit). Folds units, the dt/bf-s
 * loop back, the per-line tail and the head; exact block-fit budget rule. */
static unsigned blitb_calls,blitb_units,blitb_lines,blitb_proofs,blitb_decl[8];
static int blitb_proof_active;
#define BLITB_CAP 40
static unsigned __attribute__((noinline)) blitb_native_impl(SH2 *s,unsigned pc){
  if(blitb_proof_active || s->delay || s->test_irq || s->icount<=0 || !s->p_sdram || !s->p_dram)return 0;
  if((pc&0xdf000000u)!=0x06000000u || (pc&0x3ffffu)>0x3fe00u || (pc&0x3ffffu)<0x200u || (pc&1))return 0;
  const unsigned char *sd=(const unsigned char *)s->p_sdram;
  static const unsigned short bodyops[]={
    0x6418,0x38de,0x010d,0x30ee,0x2645, 0x6418,0x2645,0x38de,0x010d,0x30ee,
    0x6418,0x38de,0x010d,0x30ee,0x2645, 0x6418,0x2645,0x38de,0x010d,0x30ee,
    0x6418,0x38de,0x010d,0x30ee,0x2645, 0x6418,0x2645,0x38de,0x010d,0x30ee,
    0x6418,0x38de,0x010d,0x30ee,0x2645, 0x6418,0x2645,0x38de,0x010d,0x30ee,
    0x4710,0x8fd5,0x6418};
  static const unsigned short tailops[]={0x339c,0x9609,0x6039,0x220f,0x4b10,0x64f2,0x356c,0x001a,0x8fc0,0x30cc};
  static const unsigned short headops[]={0x68e3,0x4818,0x4001,0x010d,0x38de,0x30ee,0x67a3,0x442b,0x6653};
  const unsigned short *q0=(const unsigned short *)(sd+(pc&0x3ffffu));
  if(q0[1]!=0x38de && q0[1]!=0x2645)return 0;
  /* the loop end marker (dt r7; bf/s) is at most eight units ahead */
  unsigned k=0,body=0;
  for(;k<8;k++){const unsigned short *q=q0+5*k;if(q[0]==0x4710&&q[1]==0x8fd5){body=pc+10*k-80;break;}}
  if(!body||!hw_eq(sd+(body&0x3ffffu),bodyops,sizeof bodyops/2)){blitb_decl[4]++;return 0;}
  unsigned loopend=body+80,tail=body+86,head=tail+16+4+2*(int)(signed char)(tailops[8]&0xff),exitpc=tail+20;
  if(!hw_eq(sd+(tail&0x3ffffu),tailops,sizeof tailops/2)||!hw_eq(sd+(head&0x3ffffu),headops,sizeof headops/2)){blitb_decl[5]++;return 0;}
  unsigned lit6=tail+2+4+0x09*2;
  unsigned r15=s->r[15];
  if(!control_memory(s,r15,4)||!control_memory(s,lit6,2)){blitb_decl[6]++;return 0;}
  unsigned r0=s->r[0],r1=s->r[1],r2=s->r[2],r3=s->r[3],r4=s->r[4],r5=s->r[5],r6=s->r[6],r7=s->r[7],r8=s->r[8],r11=s->r[11];
  unsigned r9=s->r[9],r12=s->r[12],r13=s->r[13],r14=s->r[14],r10=s->r[10],macl=s->macl,t=s->t_flag,ea=s->ea,ppc=pc-2,cur=pc;
  unsigned cycles=0,nu=0,nl=0,dt=0;int budget=s->icount;
  unsigned short *fb=(unsigned short *)s->p_dram;
  unsigned u=k;                        /* the marker is k units ahead: k units left, this one included */
  if(!u){blitb_decl[4]++;return 0;}
#ifdef BLITB_SHADOW_PROOF
  enum{PCAP=4096};static unsigned paddr[PCAP];static unsigned short pval[PCAP];unsigned np=0;
  int proof=blitb_proofs<2048;
  SH2 reference; if(proof){reference=*s;reference.pc=pc;}
#define BLITB_W(a,d) do{ if(proof){ if(np>=PCAP){proof=0;} else {paddr[np]=(a);pval[np]=fb[((a)&0x1ffffu)>>1];np++;} } }while(0)
#else
#define BLITB_W(a,d) do{}while(0)
#endif
  for(;;){
    unsigned last=(u==1);
    if((int)(cycles+(last?9:5))>budget)break;
    unsigned la=r0+r0,b=r6-2;
    if((la&0xdf000000u)!=0x06000000u||(b&0xdf000000u)!=0x04000000u||(b&1)){blitb_decl[0]++;break;}
    r4=(r1&0xffff0000u)|((r1&0xffu)<<8)|((r1>>8)&0xffu);
    { unsigned d=r4&65535; unsigned short *pd=fb+((b&0x1ffffu)>>1); BLITB_W(b,d);
      if(b&0x20000u){unsigned o=*pd;if(!(d&255))d|=o&255;if(!(d&0xff00))d|=o&0xff00;} *pd=(unsigned short)d; }
    r6=b;
    { unsigned long long x=(unsigned long long)r8+r13+t; r8=(unsigned)x;
      unsigned long long y=(unsigned long long)r0+r14+(unsigned)(x>>32); r0=(unsigned)y; t=(unsigned)(y>>32); }
    r1=(unsigned)(int)(short)*(const unsigned short *)(sd+(la&0x3fffeu));ea=la;
    cycles+=5;nu++;
    if(!last){cur+=10;ppc=cur-2;u--;continue;}
    r7--;t=(r7==0);dt=1;cycles+=1;
    r4=(r1&0xffff0000u)|((r1&0xffu)<<8)|((r1>>8)&0xffu);
    if(r7){cycles+=2+1;cur=body;ea=body;ppc=loopend+4;u=8;continue;}
    cycles+=1+1;cur=tail;ppc=loopend+4;
    if((int)(cycles+11)>budget)break;
    r3+=r9;
    r6=(unsigned)(int)(short)*(const unsigned short *)(sd+(lit6&0x3fffeu));ea=lit6;
    r0=(r3>>16)|(r3<<16);macl=(unsigned)((int)(short)r0*(short)r2);
    r11--;t=(r11==0);
    r4=control_rd32(s,r15);ea=r15;r5+=r6;r0=macl;cycles+=8;nl++;
    if(!r11){cycles+=1+1;r0+=r12;cur=exitpc;ppc=tail+18;break;}
    cycles+=2+1;r0+=r12;cur=head;ea=head;ppc=tail+18;
    if((int)(cycles+10)>budget)break;
    unsigned entry=r4;
    if(entry<body||entry>=loopend||((entry-body)%10)){blitb_decl[1]++;break;}
    { unsigned h0=r0>>1,hla=h0+h0; if((hla&0xdf000000u)!=0x06000000u){blitb_decl[2]++;break;} }
    r8=r14;r8<<=8;t=r0&1;r0>>=1;
    la=r0+r0;
    r1=(unsigned)(int)(short)*(const unsigned short *)(sd+(la&0x3fffeu));ea=la;
    { unsigned long long x=(unsigned long long)r8+r13+t; r8=(unsigned)x;
      unsigned long long y=(unsigned long long)r0+r14+(unsigned)(x>>32); r0=(unsigned)y; t=(unsigned)(y>>32); }
    r7=r10;ea=entry;r6=r5;cycles+=10;cur=entry;ppc=head+16;u=8-(entry-body)/10;
  }
  if(!nu&&!nl){blitb_decl[7]++;return 0;}
  s->r[0]=r0;s->r[1]=r1;s->r[3]=r3;s->r[4]=r4;s->r[5]=r5;s->r[6]=r6;s->r[7]=r7;s->r[8]=r8;s->r[11]=r11;
  s->macl=macl;s->t_flag=t;s->ea=ea;s->pc=cur;s->ppc=ppc;if(dt)s->no_polling=SH2_NO_POLLING;s->icount-=cycles;
  (void)r2;
#ifdef BLITB_SHADOW_PROOF
  if(proof){
    static unsigned short got[PCAP];
    for(unsigned i=0;i<np;i++)got[i]=fb[(paddr[i]&0x1ffffu)>>1];
    for(unsigned i=np;i-->0;)fb[(paddr[i]&0x1ffffu)>>1]=pval[i];
    blitb_proof_active=1;lookup_proof_active=1;
    sh2_execute_interpreter(&reference,cycles);
    blitb_proof_active=0;lookup_proof_active=0;
    reference.icount=s->icount;
    unsigned bad=0;
    for(unsigned i=0;i<np;i++)if(fb[(paddr[i]&0x1ffffu)>>1]!=got[i])bad=1;
    if(memcmp(s,&reference,sizeof *s)||bad){
      printf("BLITB_FAIL pc=%08x cycles=%u units=%u lines=%u bad=%u\n",pc,cycles,nu,nl,bad);
      const unsigned *x=(const unsigned *)s,*y=(const unsigned *)&reference;
      for(unsigned i=0;i<sizeof *s/4;i++)if(x[i]!=y[i])printf("DIFF word=%u native=%08x reference=%08x\n",i,x[i],y[i]);
      exit(2);
    }
    blitb_proofs++;
  }
#endif
#undef BLITB_W
  blitb_calls++;blitb_units+=nu;blitb_lines+=nl;return cycles;
}
#ifdef RIG_SH2_COUNT
void blitb_report(void){printf("BLITB calls=%u units=%u lines=%u proofs=%u decl_src=%u decl_entry=%u decl_head=%u entered=%u nobody=%u tailhead=%u mem=%u nounit=%u\n",blitb_calls,blitb_units,blitb_lines,blitb_proofs,blitb_decl[0],blitb_decl[1],blitb_decl[2],blitb_decl[3],blitb_decl[4],blitb_decl[5],blitb_decl[6],blitb_decl[7]);}
#endif
/* Slave main-loop idle pass: ring index compare (on-chip regs, read through
 * the bus so comm-poll detection and its end-of-run stay exact), command flag
 * test, loop back. One native pass per 13 interpreted instructions; stops
 * exactly where poll parking or the budget would stop the interpreter. */
static unsigned idle_calls,idle_passes,idle_proofs,idle_decl[4];
static int idle_proof_active;
static unsigned __attribute__((noinline)) idle_native_impl(SH2 *s,unsigned pc){
  if(idle_proof_active||s->delay||s->test_irq||s->icount<17||!s->p_sdram||!control_ram(pc)||(pc&1)||(pc&0x3ffffu)>0x3ff00u)return 0;
  const unsigned char *sd=(const unsigned char *)s->p_sdram;
  const unsigned short *p=(const unsigned short *)(sd+(pc&0x3ffffu));
  if(p[1]!=0x6181||p[2]!=0x8581||p[3]!=0x3100||p[4]!=0x8b02||(p[5]&0xf000)!=0xa000||p[6]!=0x0009)return 0;
  unsigned lit8=((pc+4)&~3u)+(p[0]&0xff)*4;
  int d12=p[5]&0xfff;if(d12&0x800)d12-=0x1000;
  unsigned tgt=pc+10+4+2*d12;
  if(!control_ram(tgt)||(tgt&0x3ffffu)>0x3ff00u)return 0;
  const unsigned short *q=(const unsigned short *)(sd+(tgt&0x3ffffu));
  if((q[0]&0xff00)!=0xd800||q[1]!=0x6081||q[2]!=0x2008||(q[3]&0xff00)!=0x8900)return 0;
  unsigned lit8b=((tgt+4)&~3u)+(q[0]&0xff)*4;
  unsigned top=tgt+6+4+2*(int)(signed char)(q[3]&0xff);
  if(!control_ram(top)||(top&0x3ffffu)>0x3ff00u)return 0;
  const unsigned short *w=(const unsigned short *)(sd+(top&0x3ffffu));
  int d12b=w[0]&0xfff;if(d12b&0x800)d12b-=0x1000;
  if((w[0]&0xf000)!=0xa000||w[1]!=0x0009||top+4+2*d12b!=pc)return 0;
  if(!control_memory(s,lit8,4)||!control_memory(s,lit8b,4))return 0;
  unsigned a8=control_rd32(s,lit8),f8=control_rd32(s,lit8b);
  if((a8&0xffffff00u)!=0xffffff00u||(a8&1)||!control_ram(f8)||(f8&1))return 0;
  /* peek without side effects: mixing pending or a command pending is not idle */
  const unsigned short *pr=(const unsigned short *)s->peri_regs;
  if(pr[MEM_BE2((a8&0x1fe)/2)]!=pr[MEM_BE2(((a8+2)&0x1fe)/2)]){idle_decl[0]++;return 0;}
  if(*(const unsigned short *)(sd+(f8&0x3fffeu))){idle_decl[1]++;return 0;}
#ifdef IDLE_SHADOW_PROOF
  SH2 reference;int proof=idle_proofs<2048;int ic0=s->icount;
  if(proof)reference=*s,reference.pc=pc;
#endif
  /* Op by op with the interpreter's own order (execute, then charge) and its
   * own stop rule (icount <= 0 after an op), so a comm-poll end_run inside a
   * bus read stops exactly where the interpreter would, delay slot included. */
  unsigned r8=s->r[8],r1=s->r[1],r0=s->r[0],t=s->t_flag,ea=s->ea,n=0,stop=0;
#define IDLE_OP(cost,after_pc,after_ppc,delay_at) do{ s->icount-=(cost); if(s->icount<=0){ s->pc=(after_pc); s->ppc=(after_ppc); s->delay=(delay_at); stop=1; } }while(0)
  while(!stop){
    r8=a8;ea=lit8;                                   IDLE_OP(1,pc+2,pc,0);            if(stop)break;
    r1=(unsigned)(int)(short)RW(s,a8);ea=a8;         IDLE_OP(1,pc+4,pc+2,0);          if(stop)break;
    r0=(unsigned)(int)(short)RW(s,a8+2);ea=a8+2;     IDLE_OP(1,pc+6,pc+4,0);          if(stop)break;
    t=(r0==r1);                                      IDLE_OP(1,pc+8,pc+6,0);          if(stop)break;
    if(!t){ea=pc+16;                                 IDLE_OP(3,pc+16,pc+8,0);         if(!stop){s->pc=pc+16;s->ppc=pc+8;s->delay=0;} stop=2;break;}
                                                     IDLE_OP(1,pc+10,pc+8,0);         if(stop)break;
    ea=tgt;                                          IDLE_OP(2,tgt,pc+10,pc+12);      if(stop)break;
                                                     IDLE_OP(1,tgt,pc+12,0);          if(stop)break;
    r8=f8;ea=lit8b;                                  IDLE_OP(1,tgt+2,tgt,0);          if(stop)break;
    r0=(unsigned)(int)(short)*(const unsigned short *)(sd+(f8&0x3fffeu));ea=f8; IDLE_OP(1,tgt+4,tgt+2,0); if(stop)break;
    t=(r0==0);                                       IDLE_OP(1,tgt+6,tgt+4,0);        if(stop)break;
    if(!t){                                          IDLE_OP(1,tgt+8,tgt+6,0);        if(!stop){s->pc=tgt+8;s->ppc=tgt+6;s->delay=0;} stop=3;break;}
    ea=top;                                          IDLE_OP(3,top,tgt+6,0);          if(stop)break;
    ea=pc;                                           IDLE_OP(2,pc,top,top+2);         if(stop)break;
                                                     IDLE_OP(1,pc,top+2,0);           n++; if(stop)break;
  }
#undef IDLE_OP
  if(!stop){s->pc=pc;s->ppc=top+2;s->delay=0;}
  s->r[8]=r8;s->r[1]=r1;s->r[0]=r0;s->t_flag=t;s->ea=ea;
#ifdef IDLE_SHADOW_PROOF
  if(proof){
    idle_proof_active=1;sh2_execute_interpreter(&reference,ic0);idle_proof_active=0;
    if(memcmp(s,&reference,sizeof *s)){
      printf("IDLE_FAIL pc=%08x ic0=%d passes=%u stop=%u\n",pc,ic0,n,stop);
      const unsigned *x=(const unsigned *)s,*y=(const unsigned *)&reference;
      for(unsigned i=0;i<sizeof *s/4;i++)if(x[i]!=y[i])printf("DIFF word=%u native=%08x reference=%08x\n",i,x[i],y[i]);
      exit(2);
    }
    idle_proofs++;
  }
#endif
  idle_calls++;idle_passes+=n;return 1;
}
/* Slave PWM interrupt: the generic IRQ entry (save r0/r1, vector from SR's
 * level through a table, jmp, save r8 in the slot) and the handler it lands
 * in (next long from a 256-byte ring to the PWM pair at GBR+0x34, ring index
 * += 4, lower the mask, toggle a flag byte, clear the PWM interrupt at
 * GBR+0x1C and read it back, restore, rte). ~1,000 times a frame in After
 * Burner, ~14% of all guest SH-2 instructions. Recognised by its code, never
 * by its address; every load and store goes through the bus macros the
 * interpreter uses, in its order, with ppc/pc/ea/icount kept as it keeps
 * them, and after every instruction the loop's own tail is applied: an IRQ
 * that would be taken (LDC SR and RTE set test_irq) or an exhausted slice
 * stops here and the interpreter's tail does the rest, exactly as it would
 * have. Declines before touching anything unless both blocks match. */
static unsigned irq_calls,irq_full,irq_proofs,irq_decl[2];
static int irq_proof_active;
static const unsigned short irq_entry_code[9]={0x2f06,0x0002,0x2f16,0x4009,0xd100,0xc93c,0x001e,0x402b,0x2f86};
static const unsigned short irq_body_code[22]={0xd100,0xd800,0x6011,0x088e,0x7004,0x600c,0x2101,0x6083,0xc20d,0x9000,
  0x400e,0xd100,0x8417,0xca02,0x8017,0xc10e,0xc50e,0x0009,0x68f6,0x61f6,0x60f6,0x002b};
static int irq_match(const unsigned short *p,const unsigned short *want,unsigned n,unsigned immmask){
  for(unsigned i=0;i<n;i++){unsigned m=(immmask>>i)&1?0xff00u:0xffffu;if((p[i]&m)!=want[i])return 0;}
  return 1;
}
static unsigned __attribute__((noinline)) irq_native_impl(SH2 *s,unsigned pc){
  if(irq_proof_active||s->delay||s->test_irq||s->icount<2||!s->p_sdram||!control_ram(pc)||(pc&3)||(pc&0x3ffffu)>0x3ff00u)return 0;
  const unsigned char *sd=(const unsigned char *)s->p_sdram;
  const unsigned short *e=(const unsigned short *)(sd+(pc&0x3ffffu));
  if(!irq_match(e,irq_entry_code,9,1u<<4)){irq_decl[0]++;return 0;}
  /* the vector the entry will pick, read without side effects */
  unsigned lit=((pc+8+4)&~3u)+(e[4]&0xff)*4;
  if(!control_ram(lit))return 0;
  unsigned tab=*(const unsigned *)(sd+(lit&0x3fffcu));tab=(tab>>16)|(tab<<16);
  unsigned sr=(s->sr&~T)|s->t_flag,slot=((sr>>2)&0x3cu)+tab;
  if(!control_ram(slot)||(slot&3))return 0;
  unsigned h=*(const unsigned *)(sd+(slot&0x3fffcu));h=(h>>16)|(h<<16);
  if(!control_ram(h)||(h&3)||(h&0x3ffffu)>0x3ff00u)return 0;
  const unsigned short *b=(const unsigned short *)(sd+(h&0x3ffffu));
  if(!irq_match(b,irq_body_code,22,(1u<<0)|(1u<<1)|(1u<<9)|(1u<<11))){irq_decl[1]++;return 0;}
  if(b[22]!=0x0009)return 0;
#ifdef IRQ_SHADOW_PROOF
  SH2 reference;int proof=irq_proofs<2048;int ic0=s->icount;
  if(proof)reference=*s;
#endif
  unsigned a;
  /* one instruction at address a: what the fetch does, then the op, then
   * the loop tail. TAIL returns to the interpreter's own tail. */
#define AT(addr) do{ a=(addr); s->ppc=a; s->pc=a+2; }while(0)
#define SLOT() do{ s->ppc=s->delay; s->delay=0; }while(0)
#define TAIL() do{ s->icount--; if(!s->delay){ if(s->test_irq){ if(s->pending_level>(int)((s->sr>>4)&15))goto out; s->test_irq=0; } if(s->icount<=0)goto out; } }while(0)
  AT(pc);      s->r[15]-=4; WL(s,s->r[15],s->r[0]);                                   TAIL();
  AT(pc+2);    s->r[0]=(s->sr&~T)|s->t_flag;                                          TAIL();
  AT(pc+4);    s->r[15]-=4; WL(s,s->r[15],s->r[1]);                                   TAIL();
  AT(pc+6);    s->r[0]>>=2;                                                           TAIL();
  AT(pc+8);    s->ea=((s->pc+2)&~3u)+(e[4]&0xff)*4; s->r[1]=RL(s,s->ea);              TAIL();
  AT(pc+10);   s->r[0]&=0x3c;                                                         TAIL();
  AT(pc+12);   s->ea=s->r[1]+s->r[0]; s->r[0]=RL(s,s->ea);                            TAIL();
  AT(pc+14);   s->delay=s->pc; s->pc=s->ea=s->r[0]; s->icount--;                      TAIL();
  if(s->pc!=h)goto out;   /* the bus gave another vector than the peek: let the interpreter go on */
  SLOT();      s->r[15]-=4; WL(s,s->r[15],s->r[8]);                                   TAIL();
  AT(h);       s->ea=((s->pc+2)&~3u)+(b[0]&0xff)*4; s->r[1]=RL(s,s->ea);              TAIL();
  AT(h+2);     s->ea=((s->pc+2)&~3u)+(b[1]&0xff)*4; s->r[8]=RL(s,s->ea);              TAIL();
  AT(h+4);     s->ea=s->r[1]; s->r[0]=(UINT32)(INT32)(INT16)RW(s,s->ea);              TAIL();
  AT(h+6);     s->ea=s->r[8]+s->r[0]; s->r[8]=RL(s,s->ea);                            TAIL();
  AT(h+8);     s->r[0]+=4;                                                            TAIL();
  AT(h+10);    s->r[0]&=0xff;                                                         TAIL();
  AT(h+12);    s->ea=s->r[1]; WW(s,s->ea,s->r[0]&0xffff);                             TAIL();
  AT(h+14);    s->r[0]=s->r[8];                                                       TAIL();
  AT(h+16);    s->ea=s->gbr+13*4; WL(s,s->ea,s->r[0]);                                TAIL();
  AT(h+18);    s->ea=s->pc+(b[9]&0xff)*2+2; s->r[0]=(UINT32)(INT32)(INT16)RW(s,s->ea); TAIL();
  AT(h+20);    s->sr=s->r[0]&FLAGS; s->t_flag=s->sr&T; s->test_irq=1;                 TAIL();
  AT(h+22);    s->ea=((s->pc+2)&~3u)+(b[11]&0xff)*4; s->r[1]=RL(s,s->ea);             TAIL();
  AT(h+24);    s->ea=s->r[1]+7; s->r[0]=(UINT32)(INT32)(INT16)(INT8)RB(s,s->ea);       TAIL();
  AT(h+26);    s->r[0]^=2;                                                            TAIL();
  AT(h+28);    s->ea=s->r[1]+7; WB(s,s->ea,s->r[0]&0xff);                             TAIL();
  AT(h+30);    s->ea=s->gbr+14*2; WW(s,s->ea,s->r[0]&0xffff);                         TAIL();
  AT(h+32);    s->ea=s->gbr+14*2; s->r[0]=(INT32)(INT16)RW(s,s->ea);                  TAIL();
  AT(h+34);                                                                           TAIL();
  AT(h+36);    s->r[8]=RL(s,s->r[15]); s->r[15]+=4;                                   TAIL();
  AT(h+38);    s->r[1]=RL(s,s->r[15]); s->r[15]+=4;                                   TAIL();
  AT(h+40);    s->r[0]=RL(s,s->r[15]); s->r[15]+=4;                                   TAIL();
  AT(h+42);    s->ea=s->r[15]; s->delay=s->pc; s->pc=RL(s,s->ea); s->r[15]+=4;
               s->ea=s->r[15]; s->sr=RL(s,s->ea)&FLAGS; s->t_flag=s->sr&T; s->r[15]+=4;
               s->icount-=3; s->test_irq=1;                                           TAIL();
  SLOT();                                                                             TAIL();
  irq_full++;
out:
#undef AT
#undef SLOT
#undef TAIL
#ifdef IRQ_SHADOW_PROOF
  /* The reference is the interpreter over the same cycles. Where the native
   * stopped with an interrupt about to be taken, the interpreter's tail would
   * take it inside that same call: those samples are not compared. */
  if(proof&&!(s->test_irq&&!s->delay&&s->pending_level>(int)((s->sr>>4)&15))){
    int used=ic0-s->icount;
    irq_proof_active=1;sh2_execute_interpreter(&reference,used);irq_proof_active=0;
    reference.icount=ic0-(used-reference.icount);
    if(memcmp(s,&reference,sizeof *s)){
      printf("IRQ_FAIL pc=%08x ic0=%d used=%d\n",pc,ic0,used);
      const unsigned *x=(const unsigned *)s,*y=(const unsigned *)&reference;
      for(unsigned i=0;i<sizeof *s/4;i++)if(x[i]!=y[i])printf("DIFF word=%u native=%08x reference=%08x\n",i,x[i],y[i]);
      exit(2);
    }
    irq_proofs++;
  }
#endif
  irq_calls++;return 1;
}
#ifdef RIG_SH2_COUNT
void irq_report(void){printf("IRQN calls=%u full=%u proofs=%u decl_entry=%u decl_body=%u\n",irq_calls,irq_full,irq_proofs,irq_decl[0],irq_decl[1]);}
#endif
#ifdef RIG_SH2_COUNT
void idle_report(void){printf("IDLE calls=%u passes=%u proofs=%u decl_mix=%u decl_cmd=%u\n",idle_calls,idle_passes,idle_proofs,idle_decl[0],idle_decl[1]);}
#endif
/* Unrolled column fill: per element load the next row pointer, clip-test the
 * current one, store one byte at (row + x), x++. Roles r3/r6 alternate.
 * Both paths cost 6; folded element by element with the block-fit rule,
 * stores go through the bus so overwrite-image semantics stay exact. */
static unsigned fill_calls,fill_elems,fill_proofs,fill_decl[4];
static int fill_proof_active;
#define FILL_CAP 64
static unsigned __attribute__((noinline)) fill_native_impl(SH2 *s,unsigned pc){
  if(fill_proof_active||s->delay||s->test_irq||s->icount<6||!s->p_sdram||!s->p_dram||!control_ram(pc)||(pc&1)||(pc&0x3ffffu)>0x3f000u)return 0;
  const unsigned char *sd=(const unsigned char *)s->p_sdram;
  unsigned r0=s->r[0],r1=s->r[1],r2=s->r[2],r3=s->r[3],r4=s->r[4],r5=s->r[5],r6=s->r[6],t=s->t_flag,ea=s->ea;
  unsigned cur=pc,cycles=0,n=0,nw=0;int budget=s->icount;
  unsigned waddr[FILL_CAP];unsigned char wval[FILL_CAP];
  while(n<FILL_CAP && (int)(cycles+6)<=budget){
    const unsigned short *q=(const unsigned short *)(sd+(cur&0x3ffffu));
    unsigned A,B;
    if(q[0]==0x6316){A=3;B=6;}else if(q[0]==0x6616){A=6;B=3;}else break;
    if(q[1]!=(0x3042|(B<<8))||q[2]!=0x8d01||q[3]!=(0x302c|(A<<8))||q[4]!=(0x0054|(B<<8))||q[5]!=0x7001)break;
    if(!control_memory(s,r1,4)){fill_decl[0]++;break;}
    unsigned rB=(B==3)?r3:r6,rA;
    unsigned st=rB+r0;
    if((st&0xdf000000u)!=0x04000000u){fill_decl[1]++;break;}
    rA=control_rd32(s,r1);ea=r1;r1+=4;
    t=(rB>=r4);
    if(t){ea=cur+4+4+2;rA+=r2;}
    else{rA+=r2;waddr[nw]=st;wval[nw++]=(unsigned char)r5;ea=st;}
    r0+=1;
    if(A==3)r3=rA;else r6=rA;
    cycles+=6;n++;cur+=12;
  }
  if(!n)return 0;
#ifdef FILL_SHADOW_PROOF
  SH2 reference;unsigned char values[FILL_CAP],expected[FILL_CAP];int proof=fill_proofs<2048;
  if(proof){
    for(unsigned i=0;i<nw;i++)values[i]=((unsigned char *)s->p_dram)[(waddr[i]&0x1ffffu)^1];
    reference=*s;reference.pc=pc;
    fill_proof_active=1;sh2_execute_interpreter(&reference,cycles);fill_proof_active=0;
    reference.icount=s->icount-cycles;
    for(unsigned i=0;i<nw;i++)expected[i]=((unsigned char *)s->p_dram)[(waddr[i]&0x1ffffu)^1];
    for(unsigned i=nw;i-->0;)((unsigned char *)s->p_dram)[(waddr[i]&0x1ffffu)^1]=values[i];
  }
#endif
  for(unsigned i=0;i<nw;i++)WB(s,waddr[i],wval[i]);
  s->r[0]=r0;s->r[1]=r1;s->r[3]=r3;s->r[6]=r6;s->t_flag=t;s->ea=ea;s->pc=cur;s->ppc=cur-2;s->icount-=cycles;
#ifdef FILL_SHADOW_PROOF
  if(proof){
    unsigned bad=0;
    for(unsigned i=0;i<nw;i++)if(((unsigned char *)s->p_dram)[(waddr[i]&0x1ffffu)^1]!=expected[i])bad=1;
    if(memcmp(s,&reference,sizeof *s)||bad){
      printf("FILL_FAIL pc=%08x cycles=%u elems=%u bad=%u\n",pc,cycles,n,bad);
      const unsigned *x=(const unsigned *)s,*y=(const unsigned *)&reference;
      for(unsigned i=0;i<sizeof *s/4;i++)if(x[i]!=y[i])printf("DIFF word=%u native=%08x reference=%08x\n",i,x[i],y[i]);
      exit(2);
    }
    fill_proofs++;
  }
#endif
  fill_calls++;fill_elems+=n;return cycles;
}
#ifdef RIG_SH2_COUNT
void fill_report(void){printf("FILL calls=%u elems=%u proofs=%u decl_tab=%u decl_dst=%u\n",fill_calls,fill_elems,fill_proofs,fill_decl[0],fill_decl[1]);}
#endif
#ifdef RIG_SH2_COUNT
void mix_report(void){printf("MIX calls=%u proofs=%u voices=%u decl_n=%u decl_budget=%u decl_cb=%u partial=%u\n",mix_calls,mix_proofs,mix_voice_total,mix_decl[0],mix_decl[1],mix_decl[2],mix_decl[3]);
  printf("MIX budget at entry: <45=%llu <100=%llu <200=%llu <400=%llu <700=%llu >=700=%llu (r7==16 entries=%llu)\n",mix_budget_hist[0],mix_budget_hist[1],mix_budget_hist[2],mix_budget_hist[3],mix_budget_hist[4],mix_budget_hist[5],mix_budget_hist[6]);
  printf("MIX master insns: blitA=%llu blitB=%llu blitC=%llu blitD=%llu 6b00_c000=%llu lo2000=%llu 2000_67c0=%llu hi=%llu\n",mix_master_bucket[0],mix_master_bucket[1],mix_master_bucket[2],mix_master_bucket[3],mix_master_bucket[4],mix_master_bucket[5],mix_master_bucket[6],mix_master_bucket[7]);
  for(int c=0;c<2;c++){unsigned long long tot=0;for(int i=0;i<2048;i++)tot+=mix_pcbin[c][i];printf("MIX pcbin core%d total=%llu top16:",c,tot);for(int k=0;k<16;k++){int b=-1;for(int i=0;i<2048;i++)if(mix_pcbin[c][i]&&(b<0||mix_pcbin[c][i]>mix_pcbin[c][b]))b=i;if(b<0)break;printf(" %05x=%llu",b<<7,mix_pcbin[c][b]);mix_pcbin[c][b]=0;}printf("\n");}
  printf("MIX slave insns: irq390=%llu mixer=%llu main7f4=%llu sub=%llu other=%llu\n",mix_slave_bucket[0],mix_slave_bucket[1],mix_slave_bucket[2],mix_slave_bucket[3],mix_slave_bucket[4]);}
#endif

#ifdef RIG_NATIVE_COST
extern unsigned long long rig_slices[2],rig_slice_cycles[2];
void native_cost_report(void){ {extern unsigned long long rig_runsh2[2],rig_runsh2_ticks[2]; printf("RUNSH2 master=%llu ticks=%llu slave=%llu ticks=%llu\n",rig_runsh2[0],rig_runsh2_ticks[0],rig_runsh2[1],rig_runsh2_ticks[1]);}printf("SLICES master=%llu slave=%llu cyc_master=%llu cyc_slave=%llu\n",rig_slices[0],rig_slices[1],rig_slice_cycles[0],rig_slice_cycles[1]);static const char *nm[]={"mix_native","blitb_native","blitc_native","blita_native","idle_native","fill_native","chain_native","pixel_native","lookup_native","pcm_tail_native"};for(int i=0;i<10;i++)printf("NCOST %s ticks=%llu calls=%llu\n",nm[i],native_cost[i],native_ncall[i]);}
#endif
#ifdef RIG_NATIVE_COST
unsigned long long rig_slices[2],rig_slice_ticks[2],rig_slice_cycles[2];
#endif
int sh2_execute_interpreter(SH2 *sh2, int cycles)
{
	UINT32 opcode;
#ifdef GNW_SH2_FASTLOOPS
	UINT32 gnw_direct;
#endif
#ifdef RIG_SH2_PC_HIST
	int rig_is_delay = 0;
#endif

	GNW_FW_HOIST_DECL();

	sh2->icount = cycles;

	if (sh2->icount <= 0)
		goto out;

	GNW_PCWALL_ENTER(sh2);
	GNW_SLICE_TICK(sh2);

	do
	{
		RIG_OPCOST_T0(sh2);
#ifdef RIG_MIX_DIAG
		{ extern unsigned long long mix_pcbin[2][2048]; mix_pcbin[sh2->is_slave&1][(sh2->pc&0x3ffff)>>7]++; }
		if (sh2->is_slave) { extern unsigned long long mix_slave_bucket[8],mix_budget_hist[8],mix_master_bucket[8],mix_pcbin[2][2048]; unsigned q=sh2->pc&0x3ffff; mix_slave_bucket[q<0x3c0&&q>=0x380?0:q<0x480&&q>=0x3c0?1:q>=0x7f4&&q<0x860?2:q<0x2000?3:4]++; }
		else { extern unsigned long long mix_master_bucket[8]; unsigned q=sh2->pc&0x3ffff; mix_master_bucket[q>=0x67c0&&q<0x6868?0:q>=0x6868&&q<0x6960?1:q>=0x6960&&q<0x6a4c?2:q>=0x6a4c&&q<0x6b00?3:q>=0x6b00&&q<0xc000?4:q<0x2000?5:q<0x67c0?6:7]++; }
#endif
		if (sh2->delay)
		{
			sh2->ppc = sh2->delay;
			opcode = GNW_FETCH_SD_H(sh2, sh2->delay);

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

			/* `sh2->pc -= 2` here and `sh2->pc += 2` after the
			 * if/else cancelled exactly; the round trip is gone and
			 * pc is now advanced only where it actually advances.
			 * Clearing delay moved in here too -- the else arm is
			 * reached only when delay is already 0, so that store
			 * was writing a zero over a zero on every ordinary
			 * instruction. */
			sh2->delay = 0;
#ifdef GNW_SH2_FASTLOOPS
			gnw_direct = 0;
#endif
#ifdef RIG_SH2_PC_HIST
			rig_is_delay = 1;
#endif
		}
		else
		{
			UINT32 cur_pc = sh2->pc;
			sh2->ppc = cur_pc;
			GNW_FETCH_H(sh2, cur_pc, opcode);
			sh2->pc = cur_pc + 2;
#ifdef GNW_SH2_FASTLOOPS
			gnw_direct = 1;
#endif
#ifdef RIG_SH2_PC_HIST
			rig_is_delay = 0;
#endif
		}

		RIG_SH2_TICK();
		RIG_OPHIST_TICK(sh2, opcode);
		GNW_SH2_INSN_TICK(sh2);
		GNW_PCWALL_TICK(sh2);
		RIG_PC_HIST_TICK(sh2, rig_is_delay, (unsigned short)opcode);
		RIG_ABLATE_TICK(sh2);
		RIG_LOOP_REGS_TICK(sh2);
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
		/* One more gate in front of the pre-filter, and a cheaper one.
		 * Both candidate families live in 0x8xxx (BT/BF/BT.S/BF.S) and
		 * 0xaxxx (BRA), and (op & 0xd000) == 0x8000 accepts exactly
		 * those two nibbles and nothing else -- 0x9/0xb/0xc-0xf and
		 * 0x0-0x7 all fail it. That is an AND with an encodable Thumb-2
		 * immediate, a compare and a branch: three instructions, against
		 * the eight gcc emits for the pair of masked compares below
		 * (it rebuilds both movw constants every iteration rather than
		 * hoisting them, register pressure being what it is in this
		 * loop). Instructions outside those two nibbles -- the large
		 * majority -- now pay three instead of eight.
		 *
		 * Strictly a superset test: everything the old condition
		 * accepted still reaches it, so behaviour is identical and the
		 * framebuffer and audio hashes must not move. */
		/* Gate on the SAME top nibble the dispatch below switches on.
		 * Both candidate families live in 0x8xxx and 0xaxxx, and the
		 * switch already needs opcode >> 12, so asking there costs one
		 * compare instead of a masked compare against a constant gcc
		 * was hoisting into a register of its own. Freeing that
		 * register matters more than the instruction: with the fetch
		 * window holding one and the gate constant another, gcc had
		 * started spilling the opcode to the stack and reloading it
		 * two instructions later, which is two more memory operations
		 * on every dispatched instruction. */
#ifdef GNW_FASTLOOP_GATE_IN_LOOP
		/* Ablation arm: the pre-2026-08-20 shape, where the nibble test
		 * ran on every dispatched instruction ahead of the switch. */
		if (((opcode >> 12) == 8 || (opcode >> 12) == 0xa)
		    && (((opcode & 0xf980) == 0x8980) || opcode == 0xaffe)
		    && gnw_direct && *GNW_DL_REJ_SLOT(sh2) != sh2->ppc
		    && !sh2->test_irq && gnw_sh2_fastloops)
			gnw_sh2_fastloop(sh2, opcode);
#endif
#endif

		/* Dispatch via a plain switch.  This used to be a computed goto
		 * through a 16-entry table copied onto the stack on every slice;
		 * the copy is slice-cheap but the per-instruction index path pays
		 * three sp arithmetics plus a stack load before the indirect
		 * branch.  GCC lowers this switch to a single rodata jump-table
		 * load, which the QEMU rig priced at -2.16% host instructions
		 * per frame on Doom gameplay (avg sh2 dispatched identical, fb
		 * checksums identical) -- measured 2026-08-18, ablation arm A3. */
		/* `& 0xf` and all sixteen labels present, so there is no value
		 * left for a default to catch: gcc drops the `cmp #14 / bhi`
		 * bounds check in front of the tbh. opcode comes from a ldrh
		 * (or the literal 9), so the mask is a hint, never a change. */
#ifdef RIG_SH2_SKELETON
		/* Ablation: dispatch and everything around it runs, the
		 * handler bodies do not. What remains per dispatched
		 * instruction IS the fixed cost being isolated. */
		switch ((opcode >> 12) & 0xf)
		{
		case 0x0: rig_skel_hit[0x0]++; break;
		case 0x1: rig_skel_hit[0x1]++; break;
		case 0x2: rig_skel_hit[0x2]++; break;
		case 0x3: rig_skel_hit[0x3]++; break;
		case 0x4: rig_skel_hit[0x4]++; break;
		case 0x5: rig_skel_hit[0x5]++; break;
		case 0x6: rig_skel_hit[0x6]++; break;
		case 0x7: rig_skel_hit[0x7]++; break;
		case 0x8: rig_skel_hit[0x8]++; break;
		case 0x9: rig_skel_hit[0x9]++; break;
		case 0xA: rig_skel_hit[0xA]++; break;
		case 0xB: rig_skel_hit[0xB]++; break;
		case 0xC: rig_skel_hit[0xC]++; break;
		case 0xD: rig_skel_hit[0xD]++; break;
		case 0xE: rig_skel_hit[0xE]++; break;
		case 0xF: rig_skel_hit[0xF]++; break;
		}
#else
		switch ((opcode >> 12) & 0xf)
		{
#ifdef GNW_BLIT_AC
		case 0x0: if(gnw_direct && opcode==0x010d && (blita_native(sh2,sh2->ppc) || lookup_native(sh2,sh2->ppc))) { RIG_OPCOST_T1(sh2,opcode);continue; } op0000(sh2,opcode);break;
#else
		case 0x0: if(gnw_direct && opcode==0x010d && lookup_native(sh2,sh2->ppc)) { RIG_OPCOST_T1(sh2,opcode);continue; } op0000(sh2,opcode);break;
#endif
		case 0x1: op0001(sh2, opcode); break;
		case 0x2: if(gnw_direct && opcode==0x201f && control_native(sh2,sh2->ppc,0)) { RIG_OPCOST_T1(sh2,opcode);continue; }
		          /* the native applies the loop tail itself after every instruction
		           * but the last; that one is this tail, so it must not be skipped */
		          if(gnw_direct && opcode==0x2f06 && irq_native(sh2,sh2->ppc)) goto gnw_irq_tail;
		          op0010(sh2,opcode);break;
		case 0x3: if(gnw_direct && opcode==0x3d88 && control_native(sh2,sh2->ppc,1)) { RIG_OPCOST_T1(sh2,opcode);continue; } op0011(sh2,opcode);break;
		case 0x4: op0100(sh2, opcode); break;
		case 0x5: if(gnw_direct && opcode==0x52e1 && control_native(sh2,sh2->ppc,2)) { RIG_OPCOST_T1(sh2,opcode);continue; } if(gnw_direct && opcode==0x50e6 && chain_native(sh2,sh2->ppc)) { RIG_OPCOST_T1(sh2,opcode);continue; } op0101(sh2,opcode);break;
#ifdef GNW_BLIT_AC
		case 0x6: if(gnw_direct && opcode==0x612c && (blitc_native(sh2,sh2->ppc) || pixel_native(sh2,sh2->ppc))) { RIG_OPCOST_T1(sh2,opcode);continue; }
#else
		case 0x6: if(gnw_direct && opcode==0x612c && pixel_native(sh2,sh2->ppc)) { RIG_OPCOST_T1(sh2,opcode);continue; }
#endif
		          if(gnw_direct && opcode==0x6418 && blitb_native(sh2,sh2->ppc)) { RIG_OPCOST_T1(sh2,opcode);continue; }
		          if(gnw_direct && (opcode==0x6316||opcode==0x6616) && fill_native(sh2,sh2->ppc)) { RIG_OPCOST_T1(sh2,opcode);continue; } op0110(sh2,opcode);break;
		case 0x7: op0111(sh2, opcode); break;
		case 0x8: GNW_FASTLOOP_GATE_8(sh2, opcode, gnw_direct);
		          op1000(sh2, opcode); break;
		case 0x9: op1001(sh2, opcode); break;
		case 0xA: GNW_FASTLOOP_GATE_A(sh2, opcode, gnw_direct);
		          op1010(sh2, opcode); break;
		case 0xB: op1011(sh2, opcode); break;
		case 0xC:
#ifdef RIG_GBR_CENSUS
			if ((opcode >> 8) == 0xc5)
				rig_gbr_hist[(sh2)->is_slave & 1][((sh2)->gbr >> 24) & 0xff]++;
#endif
			op1100(sh2, opcode); break;
		case 0xD: if(gnw_direct && opcode==0xd21c && pcm_tail_native(sh2,sh2->ppc)) { RIG_OPCOST_T1(sh2,opcode);continue; }
		          if(gnw_direct && opcode==0xd834 && idle_native(sh2,sh2->ppc)) { RIG_OPCOST_T1(sh2,opcode);continue; } op1101(sh2,opcode);break;
		case 0xE: op1110(sh2, opcode); break;
		case 0xF: op1111(sh2, opcode); break;
		}
#endif

		sh2->icount--;

gnw_irq_tail:
		/* The !delay guard is dead on every workload we can measure: an IRQ
		 * taken with a delay slot outstanding never happens in 900 attract
		 * frames (fb checksums byte-identical with the guard dropped, rig
		 * arm A5b vs A0, 2026-08-19), and dropping it removes one load+cmp
		 * from the per-instruction tail. The check that remains below
		 * (pending_level vs sr mask) still gates the actual IRQ service. */
		/* RESTORED 2026-08-19. The `&& !sh2->delay` guard was dropped on the
		 * strength of "never happens in 900 attract frames". It is
		 * load-bearing, and the mechanism is exact rather than statistical.
		 *
		 * sh2_do_irq pushes sh2->pc and overwrites it with the vector. It
		 * neither reads nor clears sh2->delay. So when a delayed branch has
		 * just executed -- delay = slot address, pc = branch target -- and the
		 * IRQ is serviced here, the next iteration still sees delay set,
		 * executes the delay-slot instruction, and then does its `pc -= 2`.
		 * That `-= 2` is written for a pc holding the branch target; it now
		 * holds the interrupt handler's entry, so the handler starts two bytes
		 * short.
		 *
		 * Rare, silent and state-corrupting, in exchange for one load and one
		 * compare per instruction. Attract frames failing to reproduce it is
		 * not evidence of safety: attract and gameplay are demonstrably
		 * different programs here -- 188 unique guest PCs against 8,586 -- and
		 * gameplay carries far more interrupts and far more branches.
		 *
		 * The loop's own exit condition states the invariant this restores:
		 * while (icount > 0 || sh2->delay) -- "can't interrupt before delay".
		 * The other interpreter in this file (sh2_execute_interpreter_trace)
		 * never lost it; the two forms disagreed until now. */
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
		if(gnw_direct && opcode==0x8bfa && sh2->pc+8==sh2->ppc)countdown_fold(sh2);
		RIG_OPCOST_T1(sh2, opcode);
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

