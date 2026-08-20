/* ======================================================================== */
/*                            MAIN 68K CORE                                 */
/* ======================================================================== */
/*
 * EMU_G68K backend for the GNW 32X (picodrive) port.
 *
 * Origin: copied from gwenesis (bzhxx/gwenesis, src/cpus/M68K/), which is
 * Eke-Eke's Genesis Plus GX fork of Karl Stenerud's Musashi 3.32 (see the
 * licensing header in m68k.h; Musashi non-commercial license + Genesis Plus
 * GX modifications). Copied rather than referenced so the picodrive fork
 * stays self-contained and the two cores can never alias symbols by accident.
 *
 * Why this core replaces FAME (EMU_F68K) here: FAME's JumpTable[0x10000]
 * lives in RAM (262,144 bytes) because it is filled at runtime; this fork's
 * jump/cycle tables are compile-time const (m68ki_instruction_jump_table.h /
 * m68ki_cycles.h) and land in .rodata, leaving only the ~5.5 KB context in
 * RAM - the difference is decisive inside the Game & Watch 724 KB RAM_EMU
 * budget.
 *
 * Local modifications (kept minimal, all marked "GNW:"):
 *  - m68k_run() inner loop compares against m68k.cycle_end instead of the
 *    call argument, so picodrive memhandlers can end a timeslice early
 *    (SekEndRun) or burn cycles (SekCyclesBurnRun) by adjusting
 *    cycle_end/cycles while the CPU is running.
 *  - DBcc opcode handlers set g68k_not_polling (see m68kops.h), mirroring
 *    FAME's NOT_POLLING marker that picodrive's 32X poll detection relies on.
 */

extern int vdp_68k_irq_ack(int int_level);

/* GNW: picodrive poll-detection marker (defined in g68k_bus.c),
 * set by DBcc handlers in m68kops.h - see fame's NOT_POLLING. */
extern int g68k_not_polling;

#define m68ki_cpu m68k
#define MUL (7)

/* ======================================================================== */
/* ================================ INCLUDES ============================== */
/* ======================================================================== */

#ifndef BUILD_TABLES
  #ifndef TABLES_FULL
    #include "m68ki_cycles.h"
  #else
    #include "m68ki_cycles_full.h"
  #endif
#endif

#include "m68kconf.h"
#include "m68kcpu.h"
#include "m68kops.h"
//#include "gwenesis_savestate.h"

/* ======================================================================== */
/* ================================= DATA ================================= */
/* ======================================================================== */

#ifdef BUILD_TABLES
static unsigned char m68ki_cycles[0x10000];
#endif

static int irq_latency;

m68ki_cpu_core m68k;


/* ======================================================================== */
/* =============================== CALLBACKS ============================== */
/* ======================================================================== */

/* Default callbacks used if the callback hasn't been set yet, or if the
 * callback is set to NULL
 */

#if M68K_EMULATE_INT_ACK == OPT_ON
/* Interrupt acknowledge */
static int default_int_ack_callback(int int_level)
{
  CPU_INT_LEVEL = 0;
  return M68K_INT_ACK_AUTOVECTOR;
}
#endif

#if M68K_EMULATE_RESET == OPT_ON
/* Called when a reset instruction is executed */
static void default_reset_instr_callback(void)
{
}
#endif

#if M68K_TAS_HAS_CALLBACK == OPT_ON
/* Called when a tas instruction is executed */
static int default_tas_instr_callback(void)
{
  return 1; // allow writeback
}
#endif

#if M68K_EMULATE_FC == OPT_ON
/* Called every time there's bus activity (read/write to/from memory */
static void default_set_fc_callback(unsigned int new_fc)
{
}
#endif


/* ======================================================================== */
/* ================================= API ================================== */
/* ======================================================================== */

/* Access the internals of the CPU */
unsigned int m68k_get_reg(m68k_register_t regnum)
{
  switch(regnum)
  {
    case M68K_REG_D0:  return m68ki_cpu.dar[0];
    case M68K_REG_D1:  return m68ki_cpu.dar[1];
    case M68K_REG_D2:  return m68ki_cpu.dar[2];
    case M68K_REG_D3:  return m68ki_cpu.dar[3];
    case M68K_REG_D4:  return m68ki_cpu.dar[4];
    case M68K_REG_D5:  return m68ki_cpu.dar[5];
    case M68K_REG_D6:  return m68ki_cpu.dar[6];
    case M68K_REG_D7:  return m68ki_cpu.dar[7];
    case M68K_REG_A0:  return m68ki_cpu.dar[8];
    case M68K_REG_A1:  return m68ki_cpu.dar[9];
    case M68K_REG_A2:  return m68ki_cpu.dar[10];
    case M68K_REG_A3:  return m68ki_cpu.dar[11];
    case M68K_REG_A4:  return m68ki_cpu.dar[12];
    case M68K_REG_A5:  return m68ki_cpu.dar[13];
    case M68K_REG_A6:  return m68ki_cpu.dar[14];
    case M68K_REG_A7:  return m68ki_cpu.dar[15];
    case M68K_REG_PC:  return MASK_OUT_ABOVE_32(m68ki_cpu.pc);
    case M68K_REG_SR:  return  m68ki_cpu.t1_flag        |
                  (m68ki_cpu.s_flag << 11)              |
                   m68ki_cpu.int_mask                   |
                  ((m68ki_cpu.x_flag & XFLAG_SET) >> 4) |
                  ((m68ki_cpu.n_flag & NFLAG_SET) >> 4) |
                  ((!m68ki_cpu.not_z_flag) << 2)        |
                  ((m68ki_cpu.v_flag & VFLAG_SET) >> 6) |
                  ((m68ki_cpu.c_flag & CFLAG_SET) >> 8);
    case M68K_REG_SP:  return m68ki_cpu.dar[15];
    case M68K_REG_USP:  return m68ki_cpu.s_flag ? m68ki_cpu.sp[0] : m68ki_cpu.dar[15];
    case M68K_REG_ISP:  return m68ki_cpu.s_flag ? m68ki_cpu.dar[15] : m68ki_cpu.sp[4];
#if M68K_EMULATE_PREFETCH
    case M68K_REG_PREF_ADDR:  return m68ki_cpu.pref_addr;
    case M68K_REG_PREF_DATA:  return m68ki_cpu.pref_data;
#endif
    case M68K_REG_IR:  return m68ki_cpu.ir;
    default:      return 0;
  }
}

void m68k_set_reg(m68k_register_t regnum, unsigned int value)
{
  switch(regnum)
  {
    case M68K_REG_D0:  REG_D[0] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_D1:  REG_D[1] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_D2:  REG_D[2] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_D3:  REG_D[3] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_D4:  REG_D[4] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_D5:  REG_D[5] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_D6:  REG_D[6] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_D7:  REG_D[7] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_A0:  REG_A[0] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_A1:  REG_A[1] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_A2:  REG_A[2] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_A3:  REG_A[3] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_A4:  REG_A[4] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_A5:  REG_A[5] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_A6:  REG_A[6] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_A7:  REG_A[7] = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_PC:  m68ki_jump(MASK_OUT_ABOVE_32(value)); return;
    case M68K_REG_SR:  m68ki_set_sr(value); return;
    case M68K_REG_SP:  REG_SP = MASK_OUT_ABOVE_32(value); return;
    case M68K_REG_USP:  if(FLAG_S)
                REG_USP = MASK_OUT_ABOVE_32(value);
              else
                REG_SP = MASK_OUT_ABOVE_32(value);
              return;
    case M68K_REG_ISP:  if(FLAG_S)
                REG_SP = MASK_OUT_ABOVE_32(value);
              else
                REG_ISP = MASK_OUT_ABOVE_32(value);
              return;
    case M68K_REG_IR:  REG_IR = MASK_OUT_ABOVE_16(value); return;
#if M68K_EMULATE_PREFETCH
    case M68K_REG_PREF_ADDR:  CPU_PREF_ADDR = MASK_OUT_ABOVE_32(value); return;
#endif
    default:      return;
  }
}

/* Set the callbacks */
#if M68K_EMULATE_INT_ACK == OPT_ON
void m68k_set_int_ack_callback(int  (*callback)(int int_level))
{
  CALLBACK_INT_ACK = callback ? callback : default_int_ack_callback;
}
#endif

#if M68K_EMULATE_RESET == OPT_ON
void m68k_set_reset_instr_callback(void  (*callback)(void))
{
  CALLBACK_RESET_INSTR = callback ? callback : default_reset_instr_callback;
}
#endif

#if M68K_TAS_HAS_CALLBACK == OPT_ON
void m68k_set_tas_instr_callback(int  (*callback)(void))
{
  CALLBACK_TAS_INSTR = callback ? callback : default_tas_instr_callback;
}
#endif

#if M68K_EMULATE_FC == OPT_ON
void m68k_set_fc_callback(void  (*callback)(unsigned int new_fc))
{
  CALLBACK_SET_FC = callback ? callback : default_set_fc_callback;
}
#endif

#ifdef LOGERROR

extern void error(char *format, ...);
extern uint16 v_counter;
#endif

/* ASG: rewrote so that the int_level is a mask of the IPL0/IPL1/IPL2 bits */
/* KS: Modified so that IPL* bits match with mask positions in the SR
 *     and cleaned out remenants of the interrupt controller.
 */
void m68k_update_irq(unsigned int mask)
{
  /* Update IRQ level */
  CPU_INT_LEVEL |= (mask << 8);
  
#ifdef LOGERROR
  error("[%d(%d)][%d(%d)] m68k IRQ Level = %d(0x%02x) (%x)\n", v_counter, m68k.cycles/3420, m68k.cycles, m68k.cycles%3420,CPU_INT_LEVEL>>8,FLAG_INT_MASK,m68k_get_reg(M68K_REG_PC));
#endif
}

void m68k_set_irq(unsigned int int_level)
{
  /* Set IRQ level */
  CPU_INT_LEVEL = int_level << 8;
  
#ifdef LOGERROR
  error("[%d(%d)][%d(%d)] m68k IRQ Level = %d(0x%02x) (%x)\n", v_counter, m68k.cycles/3420, m68k.cycles, m68k.cycles%3420,CPU_INT_LEVEL>>8,FLAG_INT_MASK,m68k_get_reg(M68K_REG_PC));
#endif
}

/* IRQ latency (Fatal Rewind, Sesame's Street Counting Cafe)*/
void m68k_set_irq_delay(unsigned int int_level)
{
  /* Prevent reentrance */
  if (!irq_latency)
  {
    /* This is always triggered from MOVE instructions (VDP CTRL port write) */
    /* We just make sure this is not a MOVE.L instruction as we could be in */
    /* the middle of its execution (first memory write).                   */
    if ((REG_IR & 0xF000) != 0x2000)
    {
      /* Finish executing current instruction */
      USE_CYCLES(CYC_INSTRUCTION[REG_IR]);

      /* One instruction delay before interrupt */
      irq_latency = 1;
      m68ki_trace_t1() /* auto-disable (see m68kcpu.h) */
      m68ki_use_data_space() /* auto-disable (see m68kcpu.h) */
      REG_IR = m68ki_read_imm_16();
      m68ki_instruction_jump_table[REG_IR]();
      m68ki_exception_if_trace() /* auto-disable (see m68kcpu.h) */
      irq_latency = 0;
    }

    /* Set IRQ level */
    CPU_INT_LEVEL = int_level << 8;
  }
  
#ifdef LOGERROR
  error("[%d(%d)][%d(%d)] m68k IRQ Level = %d(0x%02x) (%x)\n", v_counter, m68k.cycles/3420, m68k.cycles, m68k.cycles%3420,CPU_INT_LEVEL>>8,FLAG_INT_MASK,m68k_get_reg(M68K_REG_PC));
#endif

  /* Check interrupt mask to process IRQ  */
  m68ki_check_interrupts(); /* Level triggered (IRQ) */
}

#ifdef MD32X_DEVICE_PROFILE
/* 68K profile probe (device only). The 0726 cost model closed msh2 -- memory
 * is 25% of its wall, the rest is decode/execute, and both ITCM and DTCM
 * placement are spent. What was never looked at is the 32% of the frame
 * OUTSIDE msh2, of which the 68K is the largest slice at 12.9%.
 *
 * That number needs explaining before anything is built for it. On a 32X game
 * the 68K should be nearly idle -- the SH-2s run the game -- and picodrive
 * already has the machinery to skip it (m68k_poll_detect in pico/32x/memory.c
 * calls SekSetStop, after which m68k_run returns for the rest of the slice).
 * So there are two possibilities and they lead opposite ways:
 *
 *   stopped_cycles near the frame's 128 k -> the skip already works and the
 *     remaining cost is real 68K work; the axis is closed.
 *   stopped_cycles near zero -> Doom's 68K wait is NOT a 32X-register poll, so
 *     detection never fires, and the same fold that took 18.8% off the frame
 *     this morning applies here.
 *
 * Hence: guest cycles run vs skipped, dispatched instructions, and a PC
 * histogram (32 KB pages over the first 1 MB of ROM, where a Genesis-side
 * program lives) to say whether the time is one loop or spread out.
 * Tables are caller-provided AHB, like the SH-2 probe -- the MD32X overlay
 * BSS has 520 B and this file's BSS is inside it. */
#define GNW_M68K_NBUCK      32
#define GNW_M68K_PAGE_SHIFT 15                  /* 32 KB pages, 1 MB window */
/* Doom's Genesis side runs from the ROM MIRROR at 0x880000, not from 0x000000
 * (its park block is at 0x8808a8, its vblank spin at 0x8832a2 -- both read off
 * the device and confirmed by capstone). A window based at zero therefore
 * reported "outside window 100%" and named nothing, which is exactly what the
 * 2026-08-20 device profile did. Base it where the code is. */
#ifndef GNW_M68K_WIN_BASE
#define GNW_M68K_WIN_BASE   0x880000u
#endif
#define GNW_M68K_PERIOD     37                  /* prime; see the SH-2 probe */
const unsigned int gnw_m68k_win_base = GNW_M68K_WIN_BASE;
const unsigned int gnw_m68k_nbuck = GNW_M68K_NBUCK;
const unsigned int gnw_m68k_page_shift = GNW_M68K_PAGE_SHIFT;
const unsigned int gnw_m68k_block_words = GNW_M68K_NBUCK + 1;  /* +1 = other */
unsigned int *gnw_m68k_hist_p;                  /* [NBUCK] + [NBUCK]=other  */
unsigned long long gnw_m68k_insns;
unsigned int gnw_m68k_run_cyc;                  /* guest cycles executed     */
unsigned int gnw_m68k_stop_cyc;                 /* guest cycles skipped      */
unsigned int gnw_m68k_stop_hits;                /* m68k_run calls that skipped */
unsigned int gnw_m68k_samples;
/* Did the idle fold actually fire? The 2026-08-20 profile could not say: it
 * reported skip=0 and stop_calls=0, but stop_calls counts picodrive's
 * SekSetStop and the fold deliberately sets CPU_STOPPED directly (SekSetStop's
 * SekEndRun rebases the master clock, which is not what a spinning guest
 * does). This counts the fold itself. Profile builds only -- the shipping
 * build's budget for this whole feature is the eight-byte verdict cache. */
unsigned int gnw_m68k_idle_stops;
int gnw_m68k_armed;
static int gnw_m68k_cnt;
static unsigned int gnw_m68k_last;

void gnw_m68k_prof_arm(unsigned int *block)
{
	if (block == NULL)
		return;
	gnw_m68k_hist_p = block;
	gnw_m68k_insns = 0;
	gnw_m68k_run_cyc = gnw_m68k_stop_cyc = gnw_m68k_stop_hits = 0;
	gnw_m68k_samples = 0;
	gnw_m68k_cnt = GNW_M68K_PERIOD;
	gnw_m68k_last = *(volatile unsigned int *)0xE0001004;
	gnw_m68k_armed = 1;
}

static void __attribute__((noinline)) gnw_m68k_sample(unsigned int pc)
{
	unsigned int now = *(volatile unsigned int *)0xE0001004;
	unsigned int d;

	if (!gnw_m68k_armed) {
		gnw_m68k_cnt = 1 << 30;
		return;
	}
	gnw_m68k_cnt = GNW_M68K_PERIOD;
	d = now - gnw_m68k_last;
	gnw_m68k_last = now;
	gnw_m68k_samples++;

	pc = (pc & 0xffffff) - GNW_M68K_WIN_BASE;
	if (pc < ((unsigned int)GNW_M68K_NBUCK << GNW_M68K_PAGE_SHIFT))
		gnw_m68k_hist_p[pc >> GNW_M68K_PAGE_SHIFT] += d;
	else
		gnw_m68k_hist_p[GNW_M68K_NBUCK] += d;
}
#define GNW_M68K_TICK(pc) do {                                               \
	gnw_m68k_insns++;                                                    \
	if (--gnw_m68k_cnt <= 0) gnw_m68k_sample(pc);                        \
} while (0)
#define GNW_M68K_ENTER() (gnw_m68k_last = *(volatile unsigned int *)0xE0001004)
#else
#define GNW_M68K_TICK(pc) ((void)0)
#define GNW_M68K_ENTER()  ((void)0)
#endif

/* GNW_M68K_IDLE_FOLD: sleep the 68K in the spin loops picodrive already knows
 * how to recognise but this CPU core never asked about.
 *
 * pico/sek.c carries upstream's idle-loop whitelist (SekIsIdleCode) and two
 * backends that use it -- Cyclone and FAME -- both by PATCHING the branch
 * opcode. gwenesis' jump table is compile-time const, so it was never wired,
 * and the cost of that is not theoretical: on Doom's gameplay anchor the rig
 * puts 75.2% of all 68K instructions in ONE two-instruction loop,
 *
 *     0x8832a2  tst.b  $ff1134.l      ; a flag in the 68K's own work RAM
 *     0x8832a8  beq.b  $8832a2        ; wait for the VInt handler to set it
 *
 * which is `SekIsIdleCode`'s 6-byte `tst.b ($xxxxxxxx)` case, verbatim.
 * picodrive's other stop path (p32x's 32X-register poll detect, which does
 * call SekSetStop) cannot see this one: the address it polls is RAM, not a
 * 32X register.
 *
 * Rather than patch, detect at the branch: a TAKEN short backward Bcc/BRA
 * whose body passes SekIsIdleCode, verdict cached per branch site so the
 * check runs once per site and never in the loop. The action is exactly what
 * the 32X poll path already does -- SekSetStop(1) -- so the CPU consumes the
 * rest of its timeslice without executing and wakes on the next interrupt,
 * which is the only thing that can set a flag in its own RAM anyway.
 *
 * Two guards. It never stops with interrupts masked at or above the level of
 * VInt (FLAG_INT_MASK >= 6), because then nothing could wake it. And the
 * verdict cache is keyed on the branch target, so a site that fails the
 * whitelist is asked once and never again.
 *
 * -DGNW_M68K_NO_IDLE_FOLD prices it. Only for GNW_32X_CORE. */
#if defined(GNW_32X_CORE) && !defined(GNW_M68K_NO_IDLE_FOLD)
#define GNW_M68K_IDLE_FOLD 1
#endif

#ifdef GNW_M68K_IDLE_FOLD
extern int SekIsIdleCode(unsigned short *dst, int bytes);

/* Two slots of four bytes, and not one byte more: this lives in the md32x
 * overlay's BSS, whose measured headroom is **12 bytes**
 * (_OVERLAY_MD32X_BSS_END = 0x240ffff4 against __RAM_EMU_END__ = 0x24100000,
 * read out of the ELF -- the build log does not print it). A struct-per-slot
 * version cost 260 B and failed the link outright; four slots at 16 B failed
 * it too, by four bytes.
 *
 * 68K instructions are word-aligned, so a branch target's bit 0 is always
 * zero and is free to carry the verdict. Slot 0 means "nothing cached here".
 * A miss only costs a re-probe, never correctness -- and the census that
 * motivated this fold found Doom's spin to be a single site, so two slots is
 * not the compromise it looks like. */
#define GNW_M68K_IDLE_SLOTS 2
static unsigned int gnw_m68k_idle_cache[GNW_M68K_IDLE_SLOTS];

static int __attribute__((noinline)) gnw_m68k_idle_probe(unsigned int target,
                                                         int bytes)
{
	unsigned int *sl_ =
		&gnw_m68k_idle_cache[(target >> 1) & (GNW_M68K_IDLE_SLOTS - 1)];
	unsigned short body[6];
	int i, n, ok;

	if ((*sl_ & ~1u) == target && *sl_ != 0)
		return (int)(*sl_ & 1u);

	ok = 0;
	if (bytes >= 2 && bytes <= 12 && !(bytes & 1)) {
		n = bytes >> 1;
		for (i = 0; i < n; i++)
			body[i] = (unsigned short)m68ki_read_16(target + i * 2);
		ok = SekIsIdleCode(body, bytes) ? 1 : 0;
	}
	*sl_ = (target & ~1u) | (unsigned)ok;
	return ok;
}
#endif

/* RIG_M68K_HIST: what the 68K is doing, on the rig.
 *
 * The phase table charges Doom's frame ~6% for the 68K and the July device
 * probe put it at 12.9% of the device's, and nobody has ever looked at WHERE
 * those instructions are. If they are a spin the emulator does not recognise
 * as one, that is a lever; if they are spread over real code, the axis is
 * closed. A direct-mapped exact-PC table answers it without DWT, which the rig
 * does not have. Rig-only; off => no code emitted. */
#ifdef RIG_M68K_HIST
#define RIG_M68K_SLOTS 1024
unsigned long long rig_m68k_insns;
struct rig_m68k_slot { unsigned int at, hits; };
struct rig_m68k_slot rig_m68k_tab[RIG_M68K_SLOTS];
unsigned long long rig_m68k_evict;
#define RIG_M68K_TICK(pc) do {                                               \
	unsigned int p_ = (pc) & 0xffffff;                                   \
	struct rig_m68k_slot *sl_ = &rig_m68k_tab[(p_ >> 1) & (RIG_M68K_SLOTS - 1)]; \
	rig_m68k_insns++;                                                    \
	if (sl_->at == p_ || sl_->hits == 0) { sl_->at = p_; sl_->hits++; }  \
	else rig_m68k_evict++;                                               \
} while (0)
#else
#define RIG_M68K_TICK(pc) ((void)0)
#endif

void m68k_run(unsigned int cycles)
{
    //  printf("m68K_run current_cycles=%d add=%d STOP=%x\n",m68k.cycles,cycles,CPU_STOPPED);

  /* Make sure CPU is not already ahead */
  if (m68k.cycles >= cycles)
  {
    return;
  }

  /* Check interrupt mask to process IRQ if needed */
  m68ki_check_interrupts();

  /* Make sure we're not stopped */
  if (CPU_STOPPED)
  {
#ifdef MD32X_DEVICE_PROFILE
    if (gnw_m68k_armed) {
      gnw_m68k_stop_cyc += cycles - m68k.cycles;
      gnw_m68k_stop_hits++;
    }
#endif
    m68k.cycles = cycles;
    return;
  }

  /* Save end cycles count for when CPU is stopped */
  m68k.cycle_end = cycles;
#ifdef MD32X_DEVICE_PROFILE
  if (gnw_m68k_armed)
    gnw_m68k_run_cyc += cycles - m68k.cycles;
  GNW_M68K_ENTER();
#endif

  /* Return point for when we have an address error (TODO: use goto) */
  m68ki_set_address_error_trap() /* auto-disable (see m68kcpu.h) */

#ifdef LOGERROR
  error("[%d][%d] m68k run to %d cycles (%x), irq mask = %x (%x)\n", v_counter, m68k.cycles, cycles, m68k.pc,FLAG_INT_MASK, CPU_INT_LEVEL);
#endif

  /* GNW: compare against m68k.cycle_end (not the call argument) so that
   * picodrive memory handlers can shrink the running timeslice via
   * SekEndRun()/SekSetStop() by lowering cycle_end mid-run. */
#ifdef GNW_M68K_IDLE_FOLD
  unsigned int gnw_br_pc;
#endif
  while (m68k.cycles < m68k.cycle_end)
  {
    /* Set tracing accodring to T1. */
    m68ki_trace_t1() /* auto-disable (see m68kcpu.h) */

    /* Set the address space for reads */
    m68ki_use_data_space() /* auto-disable (see m68kcpu.h) */

#ifdef HOOK_CPU
    /* Trigger execution hook */
    if (cpu_hook)
      cpu_hook(HOOK_M68K_E, 0, REG_PC, 0);
#endif

    /* Decode next instruction */
    REG_IR = m68ki_read_imm_16();

//    printf("PC=%x IR=%x CYCLES=%d \n",m68k.pc,REG_IR,CYC_INSTRUCTION[REG_IR]);

#ifdef GNW_M68K_IDLE_FOLD
    /* Short backward Bcc/BRA (not BSR, not the .w/.l displacement forms).
     * REG_PC is already past the opcode word, so this is the address of the
     * instruction AFTER the branch. */
    gnw_br_pc = (((REG_IR & 0xf000) == 0x6000) && (REG_IR & 0x0080)
                 && (REG_IR & 0x00ff) != 0x00ff
                 && (REG_IR & 0x0f00) != 0x0100) ? REG_PC : 0;
#endif

    /* Execute instruction */
    GNW_M68K_TICK(REG_PC);
    RIG_M68K_TICK(REG_PC);
    m68ki_instruction_jump_table[REG_IR]();
    USE_CYCLES(CYC_INSTRUCTION[REG_IR]);

#ifdef GNW_M68K_IDLE_FOLD
    if (gnw_br_pc && REG_PC < gnw_br_pc && FLAG_INT_MASK < 0x0600
        && gnw_m68k_idle_probe(REG_PC, (int)(gnw_br_pc - 2 - REG_PC))) {
#ifdef MD32X_DEVICE_PROFILE
      gnw_m68k_idle_stops++;
#endif
      /* Set the stop flag and leave; do NOT go through SekSetStop, whose
       * SekEndRun rebases Pico.t.m68c_cnt. Rebasing rewinds the master clock
       * by the unspent cycles, which moves every sound sync point downstream
       * and changed the audio hash (af8c118b -> 8c20ba95) on an otherwise
       * pixel-identical frame. Leaving the counters alone models what the
       * guest was actually doing -- spinning -- so the emulated timeline is
       * unchanged and only the work disappears. The next m68k_run() sees
       * CPU_STOPPED and fast-forwards m68k.cycles to its target; an interrupt
       * clears the flag in m68ki_exception_interrupt(). */
      CPU_STOPPED = STOP_LEVEL_STOP;
      break;
    }
#endif

    /* Trace m68k_exception, if necessary */
    m68ki_exception_if_trace(); /* auto-disable (see m68kcpu.h) */
  }
}

int m68k_cycles(void)
{
  return CYC_INSTRUCTION[REG_IR];
}

int m68k_cycles_run(void)
{
	return m68k.cycle_end - m68k.cycles;
}

int m68k_cycles_master(void)
{
	return m68k.cycles;
}

void m68k_init(void)
{
#ifdef BUILD_TABLES
  static uint emulation_initialized = 0;

  /* The first call to this function initializes the opcode handler jump table */
  if(!emulation_initialized)
  {
    m68ki_build_opcode_table();
    emulation_initialized = 1;
  }
#endif

#ifdef M68K_OVERCLOCK_SHIFT
  m68k.cycle_ratio = 1 << M68K_OVERCLOCK_SHIFT;
#endif

#if M68K_EMULATE_INT_ACK == OPT_ON
  m68k_set_int_ack_callback(NULL);
#endif
#if M68K_EMULATE_RESET == OPT_ON
  m68k_set_reset_instr_callback(NULL);
#endif
#if M68K_TAS_HAS_CALLBACK == OPT_ON
  m68k_set_tas_instr_callback(NULL);
#endif
#if M68K_EMULATE_FC == OPT_ON
  m68k_set_fc_callback(NULL);
#endif
}

/* Pulse the RESET line on the CPU */
void m68k_pulse_reset(void)
{
  /* Clear all stop levels */
  CPU_STOPPED = 0;
#if M68K_EMULATE_ADDRESS_ERROR
  CPU_RUN_MODE = RUN_MODE_BERR_AERR_RESET;
#endif

  /* Turn off tracing */
  FLAG_T1 = 0;
  m68ki_clear_trace()

  /* Interrupt mask to level 7 */
  FLAG_INT_MASK = 0x0700;
  CPU_INT_LEVEL = 0;
  irq_latency = 0;

  /* Go to supervisor mode */
  m68ki_set_s_flag(SFLAG_SET);

  /* Invalidate the prefetch queue */
#if M68K_EMULATE_PREFETCH
  /* Set to arbitrary number since our first fetch is from 0 */
  CPU_PREF_ADDR = 0x1000;
#endif /* M68K_EMULATE_PREFETCH */

  /* Read the initial stack pointer and program counter */
  m68ki_jump(0);
  REG_SP = m68ki_read_imm_32();
  REG_PC = m68ki_read_imm_32();
  m68ki_jump(REG_PC);

#if M68K_EMULATE_ADDRESS_ERROR
  CPU_RUN_MODE = RUN_MODE_NORMAL;
#endif

  USE_CYCLES(CYC_EXCEPTION[EXCEPTION_RESET]);
}

void m68k_pulse_halt(void)
{
  /* Pulse the HALT line on the CPU */
  CPU_STOPPED |= STOP_LEVEL_HALT;
}

void m68k_clear_halt(void)
{
  /* Clear the HALT line on the CPU */
  CPU_STOPPED &= ~STOP_LEVEL_HALT;
}

void gwenesis_m68k_save_state(FILE *file) {
  fwrite((unsigned char *)REG_D, sizeof(REG_D), 1, file);
  {
    unsigned int sr = m68ki_get_sr();
    fwrite((unsigned char *)&sr, 4, 1, file);
  }

  fwrite((unsigned char *)&REG_PC, 4, 1, file);
  fwrite((unsigned char *)&REG_SP, 4, 1, file);
  fwrite((unsigned char *)&REG_USP, 4, 1, file);
  fwrite((unsigned char *)&REG_ISP, 4, 1, file);
  fwrite((unsigned char *)&REG_IR, 4, 1, file);

  fwrite((unsigned char *)&m68k.cycle_end, 4, 1, file);
  fwrite((unsigned char *)&m68k.cycles, 4, 1, file);
  fwrite((unsigned char *)&m68k.int_level, 4, 1, file);
  fwrite((unsigned char *)&m68k.stopped, 4, 1, file);
}

void gwenesis_m68k_load_state(FILE *file, int ss_version) {
  (void)ss_version;
  fread((unsigned char *)REG_D, sizeof(REG_D), 1, file);
  {
    unsigned int sr;
    fread((unsigned char *)&sr, 4, 1, file);
    m68ki_set_sr(sr);
  }

  fread((unsigned char *)&REG_PC, 4, 1, file);
  fread((unsigned char *)&REG_SP, 4, 1, file);
  fread((unsigned char *)&REG_USP, 4, 1, file);
  fread((unsigned char *)&REG_ISP, 4, 1, file);
  fread((unsigned char *)&REG_IR, 4, 1, file);

  fread((unsigned char *)&m68k.cycle_end, 4, 1, file);
  fread((unsigned char *)&m68k.cycles, 4, 1, file);
  fread((unsigned char *)&m68k.int_level, 4, 1, file);
  fread((unsigned char *)&m68k.stopped, 4, 1, file);
}

/* ======================================================================== */
/* ============================== END OF FILE ============================= */
/* ======================================================================== */
