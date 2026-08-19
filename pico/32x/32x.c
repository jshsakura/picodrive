/*
 * PicoDrive
 * (C) notaz, 2009,2010,2013
 * (C) irixxxx, 2019-2024
 *
 * This work is licensed under the terms of MAME license.
 * See COPYING file in the top-level directory.
 */
#include "../pico_int.h"
#include "../sound/ym2612.h"
#include <cpu/sh2/compiler.h>

#if defined(RIG_PHASE_PROF) || defined(MD32X_DEVICE_PROFILE)
/* This file's pprof "draw" windows are the 32X compositor (layer merge over
 * the MD line buffer) — attribute them to their own bucket so the phase
 * table can separate MD VDP line render (pico/draw.c) from it. */
#define pp_draw pp_draw32x
#endif

struct Pico32x Pico32x;
SH2 sh2s[2];

#define SH2_IDLE_STATES (SH2_STATE_CPOLL|SH2_STATE_VPOLL|SH2_STATE_RPOLL|SH2_STATE_SLEEP)

static int REGPARM(2) sh2_irq_cb(SH2 *sh2, int level)
{
  if (sh2->pending_irl > sh2->pending_int_irq) {
    elprintf_sh2(sh2, EL_32X, "ack/irl %d @ %08x",
      level, sh2_pc(sh2));
    return 64 + sh2->pending_irl / 2;
  } else {
    elprintf_sh2(sh2, EL_32X, "ack/int %d/%d @ %08x",
      level, sh2->pending_int_vector, sh2_pc(sh2));
    sh2->pending_int_irq = 0; // auto-clear
    sh2->pending_level = sh2->pending_irl;
    return sh2->pending_int_vector;
  }
}

#ifdef GNW_SSH2_SND_HLE
/* Doom 32X slave sound-service HLE (2026-08-19).
 *
 * The slave's PWM-irq handler (0x020361a4 - the irq is P32XI_PWM, the
 * sole bit ever pending in sh2irqi[1] on this title, measured 2026-08-19)
 * only does three modelled things:
 * clear PWM irq (0x2000401c=0), bump a frame counter at 0x06001178, and
 * call the sound service at 0x0204e81a: tas-lock a struct at 0x0600117c,
 * prime/refill the PWM MONO fifo from the last sample, mix two 36-byte
 * channel structs at 0x0600118c/+36 (seq ptr, volume, sample ptr, 4-deep
 * page counter, reload counter, page base, loopback offset), write the
 * mixed sample back as the new "last sample", release the lock.
 *
 * This HLE runs that entire sequence natively on the host and skips the
 * guest handler, the GBA M4A pattern: the interpreter is paid to emulate
 * an interpreter; the host does the same arithmetic for ~nothing.  The
 * slave's cycle slice is still granted (it just parks in the bra-self
 * idle loop), so the m68k timeline is unchanged - only host work is
 * removed.  Gated on two ROM literals unique to Doom so no other title
 * can hit it, and only when the PWM irq is the slave's sole pending irq so
 * the guest path still runs whenever anything else is pending.  The gate for
 * shipping is the rig snd hash: bit-identical to the non-HLE build. */
u32 REGPARM(2) p32x_sh2_read8(u32 a, SH2 *sh2);
u32 REGPARM(2) p32x_sh2_read16(u32 a, SH2 *sh2);
u32 REGPARM(2) p32x_sh2_read32(u32 a, SH2 *sh2);
void REGPARM(3) p32x_sh2_write8(u32 a, u32 d, SH2 *sh2);
void REGPARM(3) p32x_sh2_write16(u32 a, u32 d, SH2 *sh2);
void REGPARM(3) p32x_sh2_write32(u32 a, u32 d, SH2 *sh2);

/* one channel of the DMX mixer at 0x0204e7ce - bit-exact port */
static int ssh2_hle_mix(u32 ch, SH2 *sh)
{
  u32 r0 = p32x_sh2_read32(ch + 0, sh);            // remaining samples
  if (r0 == 0)
    return 0;                                      // rts path: sext8(0)
  u32 r2 = r0 - 1;
  u32 r1 = p32x_sh2_read32(ch + 24, sh);           // seq ptr
  u32 r3 = p32x_sh2_read32(ch + 4, sh);            // volume
  u32 r4 = p32x_sh2_read32(ch + 8, sh);            // sample read ptr
  u32 r5 = p32x_sh2_read32(ch + 12, sh);           // page counter (4)
  u32 r6 = p32x_sh2_read32(ch + 16, sh);           // reload counter

  if (--r5 != 0)                                   // dt r5; bf sample
    goto have_sample;
  if (--r6 != 0)                                   // dt r6; bf fetch
    goto fetch_index;
  p32x_sh2_write32(ch + 28, r1, sh);               // new page base = seq ptr
  r1 += p32x_sh2_read32(ch + 32, sh);              // loopback jump
  r6 = p32x_sh2_read32(ch + 20, sh);               // reload restore
fetch_index:
  r4 = p32x_sh2_read8(r1, sh); r1++;               // mov.b @r1+
  r5 = p32x_sh2_read32(ch + 28, sh);               // page base
  r4 = ((r4 & 0xffu) << 2) + r5;                   // extu.b (read8 sign-extends) ; shll2 ; add r5
  r5 = 4;
have_sample:
  r0 = p32x_sh2_read8(r4, sh); r4++;               // mov.b @r4+
  // extu.b + add #-64 twice == (u8)v - 128
  s32 mac = (s32)(s16)(u16)r3 * ((s32)(u8)r0 - 128); // muls.w -> MACL
  p32x_sh2_write32(ch + 24, r1, sh);
  p32x_sh2_write32(ch + 0, r2, sh);
  p32x_sh2_write32(ch + 8, r4, sh);
  p32x_sh2_write32(ch + 12, r5, sh);
  p32x_sh2_write32(ch + 16, r6, sh);
  return (s8)(u8)((u32)mac >> 8);                  // sts macl; shlr8; exts.b
}

/* sound service 0x0204e81a, returns 1 if it ran (signature matched) */
/* Timing alignment knobs, tuned against the guest PWM-write trace:
 * LAT shifts the service's write clock relative to the PWM irq event time,
 * WI1/WI2 reproduce the guest's in-group write cadence (+18,+5 observed). */
#ifndef GNW_SSH2_HLE_LAT
#define GNW_SSH2_HLE_LAT 0
#endif
#ifndef GNW_SSH2_HLE_WI1
#define GNW_SSH2_HLE_WI1 18
#endif
#ifndef GNW_SSH2_HLE_WI2
#define GNW_SSH2_HLE_WI2 5
#endif

/* Engine fingerprint for the Doom DMX sound service, resolved once at the
 * first PWM irq. We do NOT key on absolute addresses (they are this ROM
 * build's layout): we verify the service prologue opcode sequence and read
 * the struct/PWM/channel pointers out of its own literal pool, exactly the
 * way the guest loads them. On mismatch we go quiet (state=-1, no printf,
 * no reject side effects) and leave the guest path untouched. */
static struct {
  u32 base, pwm, ch, cnt;
  int state; /* 0=unchecked, 1=fingerprint ok, -1=rejected */
} snd_fp;

static void ssh2_hle_resolve(SH2 *sh)
{
  /* push r12-r14 / mov.l @(0x1c,PC),r14 / mov.l @(4,r14),r0 / cmp/eq #0,r0
   * / bf / tas.b @r14 / bf -- the Doom slave sound service prologue */
  static const u16 op[] = { 0x2fc6, 0x2fd6, 0x2fe6, 0xde1c,
                            0x50e1, 0x8800, 0x8b31, 0x4e1b, 0x8b2f };
  enum { SVC = 0x0204e81a };
  int i;

  for (i = 0; i < (int)(sizeof(op) / sizeof(op[0])); i++)
    if ((u16)p32x_sh2_read16(SVC + 2 * i, sh) != op[i]) {  // read16 sign-extends
      snd_fp.state = -1;
      return;
    }

  /* literal pool slots for r14/r12/r13 loads: ((insn+4)&~3) + disp*4 */
  snd_fp.base = p32x_sh2_read32(((SVC + 6 + 4) & ~3) + 0x1c * 4, sh);
  snd_fp.pwm  = p32x_sh2_read32(((SVC + 0x26 + 4) & ~3) + 0x15 * 4, sh);
  snd_fp.ch   = p32x_sh2_read32(((SVC + 0x38 + 4) & ~3) + 0x12 * 4, sh);

  /* sanity: channel table sits at base+0x10, struct in SDRAM, PWM in the
   * 32x register window -- otherwise this is not the engine we ported */
  if (snd_fp.ch != snd_fp.base + 0x10 ||
      (snd_fp.base & 0xdf000000) != 0x06000000 ||
      (snd_fp.pwm & 0x3ffc0) != 0x4000) {
    snd_fp.state = -1;
    return;
  }
  snd_fp.cnt = snd_fp.base - 4; /* handler's frame counter literal */
  snd_fp.state = 1;
}

static int ssh2_hle_sound_service(SH2 *sh, u32 evt_cyc)
{
  u32 w_idx = 0; /* in-group write index: 0=group start,1=+WI1,2=+WI2 */
  u32 BASE = snd_fp.base, PWM_MONO = snd_fp.pwm, CH = snd_fp.ch;

  // handler prologue side effects
  p32x_sh2_write32(snd_fp.cnt, p32x_sh2_read32(snd_fp.cnt, sh) + 1, sh);
  p32x_sh2_write16(0x2000401c, 0, sh);                // clear PWM irq (32x reg, fixed by spec)

  /* anchor the write clock to the irq event time: without this the
   * write16 calls timestamp at ssh2's stale slice clock, ~39 m68k cycles
   * late vs the guest's handler entry, and the drain interleaving (and
   * thus the audio stream) diverges */
  sh->m68krcycles_done = evt_cyc + GNW_SSH2_HLE_LAT;
  w_idx = 0;

  if (p32x_sh2_read32(BASE + 4, sh) != 0)             // guest: cmp/eq #0; bf exit -- 0 means "run"
    return 1;
  if (p32x_sh2_read8(BASE, sh))                       // tas fails: locked
    return 1;
  p32x_sh2_write8(BASE, 1, sh);                       // tas acquire

  u32 r7 = p32x_sh2_read32(BASE + 8, sh);             // fifo fill count
  u32 last = p32x_sh2_read16(BASE + 12, sh);          // last sample (u16)
  u32 ch = CH;

  for (;;) {
    do {
      u32 st = p32x_sh2_read8(PWM_MONO, sh);          // 0x80 in high byte
      if (st & 0x80)                                  // P32XP_FULL: stop
        goto full;
      sh->m68krcycles_done += (w_idx == 1) ? GNW_SSH2_HLE_WI1 :
                               (w_idx == 2) ? GNW_SSH2_HLE_WI2 : 0;
      if (w_idx < 2) w_idx++;
      p32x_sh2_write16(PWM_MONO, last, sh);
    } while (--r7 != 0);                              // dt r7; bf fill

    int m1 = ssh2_hle_mix(ch, sh);                    // bsr + delay mov #2,r7
    int m2 = ssh2_hle_mix(ch + 36, sh);               // bsr + delay ch += 36
    last = ((((u32)m1 + (u32)m2) + 256) << 1) + 1;
    p32x_sh2_write16(BASE + 12, last, sh);
    r7 = 2;                                           // bra fill with r7=2
  }

full:
  p32x_sh2_write32(BASE + 8, r7, sh);                 // remaining count
  p32x_sh2_write32(BASE + 0, 0, sh);                  // release lock
  return 1;
}
#endif // GNW_SSH2_SND_HLE

// MUST specify active_sh2 when called from sh2 memhandlers
void p32x_update_irls(SH2 *active_sh2, unsigned int m68k_cycles)
{
  int irqs, mlvl = 0, slvl = 0;
  int mrun, srun;

  if ((Pico32x.regs[0] & (P32XS_nRES|P32XS_ADEN)) != (P32XS_nRES|P32XS_ADEN))
    return;

  if (active_sh2 != NULL)
    m68k_cycles = sh2_cycles_done_m68k(active_sh2);

  // find top bit = highest irq number (0 <= irl <= 14/2) by binary search

  // msh2
  irqs = Pico32x.sh2irqi[0];
  if (irqs >= 0x10)     mlvl += 8, irqs >>= 4;
  if (irqs >= 0x04)     mlvl += 4, irqs >>= 2;
  if (irqs >= 0x02)     mlvl += 2, irqs >>= 1;

  // ssh2
  irqs = Pico32x.sh2irqi[1];
  if (irqs >= 0x10)     slvl += 8, irqs >>= 4;
  if (irqs >= 0x04)     slvl += 4, irqs >>= 2;
  if (irqs >= 0x02)     slvl += 2, irqs >>= 1;

#ifdef RIG_SND_STRUCT_DUMP
/* Throwaway rig probe: snapshot the Doom slave sound-state struct at every
 * PWM irq RAISE (edge-detected), before any handler/HLE service mutates it.
 * Runs in both baseline and HLE builds so the struct evolution can be
 * diffed to find the first diverging field. Same magic windows as the HLE
 * gate (0x0600117c struct / 0x0600118c ch1 / +36 ch2). */
{
  static int s_prev_set;
  static u32 s_seq;
  if (Pico32x.sh2irqi[1] & P32XI_PWM) {
    if (!s_prev_set && s_seq < 70000) {
      int c, w;
      printf("[sndst] %u %x %x %x %x",
        s_seq,
        p32x_sh2_read32(0x0600117c + 0, &ssh2),
        p32x_sh2_read32(0x0600117c + 4, &ssh2),
        p32x_sh2_read32(0x0600117c + 8, &ssh2),
        p32x_sh2_read32(0x0600117c + 12, &ssh2));
      for (c = 0; c < 2; c++) {
        printf(" |");
        for (w = 0; w < 9; w++)
          printf(" %x", p32x_sh2_read32(0x0600118c + c*36 + w*4, &ssh2));
      }
      printf("\n");
      s_seq++;
    }
    s_prev_set = 1;
  } else {
    s_prev_set = 0;
  }
}
#endif

#ifdef GNW_SSH2_SND_HLE
  /* replace the Doom slave PWM-irq handler with the native port above;
   * sole-pending-bit condition keeps the guest path for anything else */
  {
    u32 v = Pico32x.sh2irqi[1];
    if ((v & P32XI_PWM) && v == P32XI_PWM) {
      if (snd_fp.state == 0)
        ssh2_hle_resolve(&ssh2);
      if (snd_fp.state == 1 &&
          ssh2_hle_sound_service(&ssh2, m68k_cycles)) {
        /* service ack'ed the PWM bit via the bus write; skip the guest
         * handler entirely (native port replaced it) */
        slvl = 0;
      }
    }
  }
#endif

  mrun = sh2_irl_irq(&msh2, mlvl, msh2.state & SH2_STATE_RUN);
  if (mrun) {
    p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_IDLE_STATES & ~SH2_STATE_SLEEP, m68k_cycles);
    if (msh2.state & SH2_STATE_RUN)
      sh2_end_run(&msh2, 0);
  }

  srun = sh2_irl_irq(&ssh2, slvl, ssh2.state & SH2_STATE_RUN);
  if (srun) {
    p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_IDLE_STATES & ~SH2_STATE_SLEEP, m68k_cycles);
    if (ssh2.state & SH2_STATE_RUN)
      sh2_end_run(&ssh2, 0);
  }

  elprintf(EL_32X, "update_irls: m %d/%d, s %d/%d", mlvl, mrun, slvl, srun);
}

// the mask register is inconsistent, CMD is supposed to be a mask,
// while others are actually irq trigger enables?
// TODO: test on hw..
void p32x_trigger_irq(SH2 *sh2, unsigned int m68k_cycles, unsigned int mask)
{
  Pico32x.sh2irqi[0] |= mask & P32XI_VRES;
  Pico32x.sh2irqi[1] |= mask & P32XI_VRES;
  Pico32x.sh2irqi[0] |= mask & (Pico32x.sh2irq_mask[0] << 3);
  Pico32x.sh2irqi[1] |= mask & (Pico32x.sh2irq_mask[1] << 3);

  p32x_update_irls(sh2, m68k_cycles);
}

void p32x_update_cmd_irq(SH2 *sh2, unsigned int m68k_cycles)
{
  if ((Pico32x.sh2irq_mask[0] & 2) && (Pico32x.regs[2 / 2] & 1))
    Pico32x.sh2irqi[0] |= P32XI_CMD;
  else
    Pico32x.sh2irqi[0] &= ~P32XI_CMD;

  if ((Pico32x.sh2irq_mask[1] & 2) && (Pico32x.regs[2 / 2] & 2))
    Pico32x.sh2irqi[1] |= P32XI_CMD;
  else
    Pico32x.sh2irqi[1] &= ~P32XI_CMD;

  p32x_update_irls(sh2, m68k_cycles);
}

void Pico32xStartup(void)
{
  elprintf(EL_STATUS|EL_32X, "32X startup");

  PicoIn.AHW |= PAHW_32X;
  // TODO: OOM handling
  if (Pico32xMem == NULL) {
#ifdef GNW_32X_CORE
    // Game & Watch: back the 32X memory (sdram 256K + dram 256K + banks/pal/pwm,
    // ~524KB) with a static buffer so it lands in .bss the linker can account for,
    // instead of the heap. calloc-equivalent: memset to zero below.
    static struct Pico32xMem gnw_32xmem __attribute__((aligned(4)));
    Pico32xMem = &gnw_32xmem;
    memset(Pico32xMem, 0, sizeof(struct Pico32xMem));
#else
    Pico32xMem = plat_mmap(0x06000000, sizeof(*Pico32xMem), 0, 0);
    if (Pico32xMem == NULL) {
      elprintf(EL_STATUS, "OOM");
      return;
    }
    memset(Pico32xMem, 0, sizeof(struct Pico32xMem));
#endif

    sh2_init(&msh2, 0, &ssh2);
    msh2.irq_callback = sh2_irq_cb;
    sh2_init(&ssh2, 1, &msh2);
    ssh2.irq_callback = sh2_irq_cb;
  }

  PicoMemSetup32x();
  p32x_pwm_ctl_changed();
  p32x_timers_recalc();

  Pico32x.regs[0] |= P32XS_ADEN;

  Pico32x.sh2_regs[0] = P32XS2_ADEN;
  if (Pico.m.ncart_in)
    Pico32x.sh2_regs[0] |= P32XS2_nCART;

  if (!Pico.m.pal)
    Pico32x.vdp_regs[0] |= P32XV_nPAL;
  else
    Pico32x.vdp_regs[0] &= ~P32XV_nPAL;

  rendstatus_old = -1;

  Pico32xPrepare();
  emu_32x_startup();
}

void Pico32xShutdown(void)
{
  elprintf(EL_STATUS|EL_32X, "32X shutdown");
  Pico32x.sh2_regs[0] &= ~P32XS2_ADEN;
  Pico32x.regs[0] &= ~P32XS_ADEN;

  rendstatus_old = -1;

  PicoIn.AHW &= ~PAHW_32X;
#ifndef GNW_32X_CORE
  if (PicoIn.AHW & PAHW_MCD)
    PicoMemSetupCD();
  else
#endif
    PicoMemSetup();
  emu_32x_startup();
}

void p32x_reset_sh2s(void)
{
  elprintf(EL_32X, "sh2 reset");

  sh2_reset(&msh2);
  sh2_reset(&ssh2);
  sh2_peripheral_reset(&msh2);
  sh2_peripheral_reset(&ssh2);

  // if we don't have BIOS set, perform it's work here.
  // MSH2
  if (p32x_bios_m == NULL) {
    sh2_set_gbr(0, 0x20004000);

    if (!Pico.m.ncart_in) { // copy IDL from cartridge
      unsigned int idl_src, idl_dst, idl_size; // initial data load
      unsigned int vbr;
      // initial data
      idl_src = CPU_BE2(*(u32 *)(Pico.rom + 0x3d4)) & ~0xf0000000;
      idl_dst = CPU_BE2(*(u32 *)(Pico.rom + 0x3d8)) & ~0xf0000000;
      idl_size= CPU_BE2(*(u32 *)(Pico.rom + 0x3dc));
      // copy in guest memory space
      idl_src += 0x2000000;
      idl_dst += 0x6000000;
      while (idl_size >= 4) {
        p32x_sh2_write32(idl_dst, p32x_sh2_read32(idl_src, &msh2), &msh2);
        idl_src += 4, idl_dst += 4, idl_size -= 4;
      }

      // VBR
      vbr = CPU_BE2(*(u32 *)(Pico.rom + 0x3e8));
      sh2_set_vbr(0, vbr);

      // checksum and M_OK
      Pico32x.regs[0x28 / 2] = *(u16 *)(Pico.rom + 0x18e);
    }
    // program will set M_OK
  }

  // SSH2
  if (p32x_bios_s == NULL) {
    unsigned int vbr;

    // GBR/VBR
    vbr = CPU_BE2(*(u32 *)(Pico.rom + 0x3ec));
    sh2_set_gbr(1, 0x20004000);
    sh2_set_vbr(1, vbr);
    // program will set S_OK
  }

  msh2.m68krcycles_done = ssh2.m68krcycles_done = SekCyclesDone();
}

void Pico32xInit(void)
{
}

void PicoPower32x(void)
{
  memset(&Pico32x, 0, sizeof(Pico32x));

  Pico32x.regs[0] = P32XS_REN|P32XS_nRES; // verified
  Pico32x.regs[0x10/2] = 0xffff;
  Pico32x.vdp_regs[0x0a/2] = P32XV_VBLK|P32XV_PEN;
}

void PicoUnload32x(void)
{
  if (PicoIn.AHW & PAHW_32X)
    Pico32xShutdown();

  sh2_finish(&msh2);
  sh2_finish(&ssh2);

#ifndef GNW_32X_CORE
  if (Pico32xMem != NULL)
    plat_munmap(Pico32xMem, sizeof(*Pico32xMem));
#endif
  Pico32xMem = NULL;
}

void PicoReset32x(void)
{
  if (PicoIn.AHW & PAHW_32X) {
    p32x_trigger_irq(NULL, SekCyclesDone(), P32XI_VRES);
    p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_IDLE_STATES, SekCyclesDone());
    p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_IDLE_STATES, SekCyclesDone());
    p32x_pwm_ctl_changed();
    p32x_timers_recalc();
  }
}

static void Pico32xRenderSync(int lines)
{
  if (Pico32xDrawMode != PDM32X_OFF && !PicoIn.skipFrame) {
    int offs;

    pprof_start(draw);

    offs = 8;
    if (Pico.video.reg[1] & 8)
      offs = 0;

    if ((Pico32x.vdp_regs[0] & P32XV_Mx) != 0 && // 32x not blanking
        (!(Pico.video.debug_p & PVD_KILL_32X)))
    {
      int md_bg = Pico.video.reg[7] & 0x3f;

      // we draw lines up to the sync point (not line-by-line)
      PicoDraw32xLayer(offs, lines-Pico32x.sync_line, md_bg);
    }
    else if (Pico32xDrawMode == PDM32X_BOTH)
      PicoDraw32xLayerMdOnly(offs, lines-Pico32x.sync_line);

    pprof_end(draw);
  }
}

void Pico32xDrawSync(SH2 *sh2)
{
#if defined(RIG_PHASE_PROF) || defined(MD32X_DEVICE_PROFILE)
  /* mid-frame draw sync fires from CPU memory handlers — pause whichever CPU
   * accumulator is live so its bucket holds pure interpreter+bus time (the
   * draw work below books itself into pp_draw/pp_draw32x) */
  pprof_start(m68k); pprof_start(msh2); pprof_start(ssh2);
#endif
  // the fast renderer isn't operating on a line-by-line base
  if (sh2 && !(PicoIn.opt & POPT_ALT_RENDERER)) {
    unsigned int cycle = (sh2 ? sh2_cycles_done_m68k(sh2) : SekCyclesDone());
    int line = ((cycle - Pico.t.m68c_frame_start) * (long long)((1LL<<32)/488.5)) >> 32;

    if (Pico32x.sync_line < line && line < (Pico.video.reg[1] & 8 ? 240 : 224)) {
      // make sure the MD image is also sync'ed to this line for merging
      PicoDrawSync(line, 0, 0);

      // pfff... need to save and restore some persistent data for MD renderer
      void *dest = Pico.est.DrawLineDest;
      int incr = Pico.est.DrawLineDestIncr;
      Pico32xRenderSync(line);
      Pico.est.DrawLineDest = dest;
      Pico.est.DrawLineDestIncr = incr;
    }

    // remember line we sync'ed to
    Pico32x.sync_line = line;
  }
#if defined(RIG_PHASE_PROF) || defined(MD32X_DEVICE_PROFILE)
  pprof_end_sub(ssh2); pprof_end_sub(msh2); pprof_end_sub(m68k);
#endif
}

static void p32x_render_frame(void)
{
  if (Pico32xDrawMode != PDM32X_OFF && !PicoIn.skipFrame) {
    int lines;

    pprof_start(draw);

    lines = 224;
    if (Pico.video.reg[1] & 8)
      lines = 240;

    Pico32xRenderSync(lines);

    pprof_end(draw); // was missing: PPROF builds leaked scope + refcount here
  }
}

static void p32x_start_blank(void)
{
  // enter vblank
  Pico32x.vdp_regs[0x0a/2] |= P32XV_VBLK|P32XV_PEN;

  // FB swap waits until vblank
  if ((Pico32x.vdp_regs[0x0a/2] ^ Pico32x.pending_fb) & P32XV_FS) {
    Pico32x.vdp_regs[0x0a/2] ^= P32XV_FS;
    Pico32xSwapDRAM(Pico32x.pending_fb ^ P32XV_FS);
  }

  p32x_trigger_irq(NULL, Pico.t.m68c_aim, P32XI_VINT);
  p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_STATE_VPOLL, Pico.t.m68c_aim);
  p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_STATE_VPOLL, Pico.t.m68c_aim);
}

static void p32x_end_blank(void)
{
  // end vblank
  Pico32x.vdp_regs[0x0a/2] &= ~P32XV_VBLK; // get out of vblank
  if ((Pico32x.vdp_regs[0] & P32XV_Mx) != 0) // no forced blanking
    Pico32x.vdp_regs[0x0a/2] &= ~P32XV_PEN; // no palette access
  if (!(Pico32x.sh2_regs[0] & 0x80)) {
    // NB must precede VInt per hw manual, min 4 SH-2 cycles to pass Mars Check
    Pico32x.hint_counter = (int)(-1.5*0x10);
    p32x_schedule_hint(NULL, Pico.t.m68c_aim);
  }

  p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_STATE_VPOLL, Pico.t.m68c_aim);
  p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_STATE_VPOLL, Pico.t.m68c_aim);
}

void p32x_schedule_hint(SH2 *sh2, unsigned int m68k_cycles)
{
  // rather rough, 32x hint is useless in practice
  int after;
  if (!((Pico32x.sh2irq_mask[0] | Pico32x.sh2irq_mask[1]) & 4))
    return; // nobody cares
  if (!(Pico32x.sh2_regs[0] & 0x80) && (Pico.video.status & PVS_VB2))
    return;

  Pico32x.hint_counter += (Pico32x.sh2_regs[4 / 2] + 1) * (int)(488.5*0x10);
  after = Pico32x.hint_counter >> 4;
  Pico32x.hint_counter &= 0xf;
  if (sh2 != NULL)
    p32x_event_schedule_sh2(sh2, P32X_EVENT_HINT, after);
  else
    p32x_event_schedule(m68k_cycles, P32X_EVENT_HINT, after);
}

/* events */
static void fillend_event(unsigned int now)
{
  Pico32x.vdp_regs[0x0a/2] &= ~P32XV_nFEN;
  p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_STATE_VPOLL, now);
  p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_STATE_VPOLL, now);
}

static void hint_event(unsigned int now)
{
  p32x_trigger_irq(NULL, now, P32XI_HINT);
  p32x_schedule_hint(NULL, now);
}

typedef void (event_cb)(unsigned int now);

/* times are in m68k (7.6MHz) cycles */
unsigned int p32x_event_times[P32X_EVENT_COUNT];
static unsigned int event_time_next;
static event_cb *p32x_event_cbs[P32X_EVENT_COUNT] = {
  p32x_pwm_irq_event, // P32X_EVENT_PWM
  fillend_event,      // P32X_EVENT_FILLEND
  hint_event,         // P32X_EVENT_HINT
};

// schedule event at some time 'after', in m68k clocks
void p32x_event_schedule(unsigned int now, enum p32x_event event, int after)
{
  unsigned int when;

  when = (now + after) | 1;

  elprintf(EL_32X, "32x: new event #%u %u->%u", event, now, when);
  p32x_event_times[event] = when;

  if (event_time_next == 0 || CYCLES_GT(event_time_next, when))
    event_time_next = when;
}

void p32x_event_schedule_sh2(SH2 *sh2, enum p32x_event event, int after)
{
  unsigned int now = sh2_cycles_done_m68k(sh2);
  int left_to_next;

  p32x_event_schedule(now, event, after);

  left_to_next = C_M68K_TO_SH2(sh2, (int)(event_time_next - now));
  if (sh2_cycles_left(sh2) > left_to_next) {
    if (left_to_next < 1)
      left_to_next = 0;
    sh2_end_run(sh2, left_to_next);
  }
}

static void p32x_run_events(unsigned int until)
{
  int oldest, oldest_diff, time;
  int i, diff;

  while (1) {
    oldest = -1, oldest_diff = 0x7fffffff;

    for (i = 0; i < P32X_EVENT_COUNT; i++) {
      if (p32x_event_times[i]) {
        diff = p32x_event_times[i] - until;
        if (diff < oldest_diff) {
          oldest_diff = diff;
          oldest = i;
        }
      }
    }

    if (oldest_diff <= 0) {
      time = p32x_event_times[oldest];
      p32x_event_times[oldest] = 0;
      elprintf(EL_32X, "32x: run event #%d %u", oldest, time);
      p32x_event_cbs[oldest](time);
    }
    else if (oldest_diff < 0x7fffffff) {
      event_time_next = p32x_event_times[oldest];
      break;
    }
    else {
      event_time_next = 0;
      break;
    }
  }

  if (oldest != -1)
    elprintf(EL_32X, "32x: next event #%d at %u",
      oldest, event_time_next);
}

static void run_sh2(SH2 *sh2, unsigned int m68k_cycles)
{
  unsigned int cycles, done;

  pevt_log_sh2_o(sh2, EVT_RUN_START);
  sh2->state |= SH2_STATE_RUN;
  cycles = C_M68K_TO_SH2(sh2, m68k_cycles);
  elprintf_sh2(sh2, EL_32X, "+run %u %d @%08x",
    sh2->m68krcycles_done, cycles, sh2->pc);

  done = sh2_execute(sh2, cycles);

  sh2->m68krcycles_done += C_SH2_TO_M68K(sh2, done);
  sh2->state &= ~SH2_STATE_RUN;
  pevt_log_sh2_o(sh2, EVT_RUN_END);
  elprintf_sh2(sh2, EL_32X, "-run %u %d",
    sh2->m68krcycles_done, done);
}

// sync other sh2 to this one
// note: recursive call
void p32x_sync_other_sh2(SH2 *sh2, unsigned int m68k_target)
{
  SH2 *osh2 = sh2->other_sh2;
  int left_to_event;
  int m68k_cycles;

  if (osh2->state & SH2_STATE_RUN)
    return;

  m68k_cycles = m68k_target - osh2->m68krcycles_done;
  if (m68k_cycles < 200)
    return;

  if (osh2->state & SH2_IDLE_STATES) {
    osh2->m68krcycles_done = m68k_target;
    return;
  }

  elprintf_sh2(osh2, EL_32X, "sync to %u %d",
    m68k_target, m68k_cycles);

  run_sh2(osh2, m68k_cycles);

  // there might be new event to schedule current sh2 to
  if (event_time_next) {
    left_to_event = C_M68K_TO_SH2(sh2, (int)(event_time_next - m68k_target));
    if (sh2_cycles_left(sh2) > left_to_event) {
      if (left_to_event < 1)
        left_to_event = 0;
      sh2_end_run(sh2, left_to_event);
    }
  }
}

#define STEP_LS 24
#define STEP_N 976 // 2 lines: fewer msh2<->ssh2 swaps, less D-cache thrash
                     // (was 528 = 1 line. 488 is the floor; the practical CAP is the
                    //  H-Blank sync period, ~1000 cycles -- past that, titles
                    //  that hand off between the SH-2s inside a line (VR,
                    //  Kolibri) can crash. 2112 was tried first and is unsafe.
                     //  p32x_sh2_poll_detect still forces an early sync whenever
                     //  one SH-2 polls a location the other writes, so a longer
                     //  slice does not by itself break inter-CPU handshakes.)

#define sync_sh2s_normal p32x_sync_sh2s
//#define sync_sh2s_lockstep p32x_sync_sh2s

/* most timing is in 68k clock */
void sync_sh2s_normal(unsigned int m68k_target)
{
  unsigned int now, target, next, timer_cycles;
  int cycles;

  elprintf(EL_32X, "sh2 sync to %u", m68k_target);

  if ((Pico32x.regs[0] & (P32XS_nRES|P32XS_ADEN)) != (P32XS_nRES|P32XS_ADEN)) {
    msh2.m68krcycles_done = ssh2.m68krcycles_done = m68k_target;
    return; // rare
  }

  now = msh2.m68krcycles_done;
  if (CYCLES_GT(now, ssh2.m68krcycles_done))
    now = ssh2.m68krcycles_done;
  timer_cycles = now;

  pprof_start(m68k);
  while (CYCLES_GT(m68k_target, now))
  {
    if (event_time_next && CYCLES_GE(now, event_time_next))
      p32x_run_events(now);

    target = m68k_target;
    if (event_time_next && CYCLES_GT(target, event_time_next))
      target = event_time_next;
    while (CYCLES_GT(target, now))
    {
      next = target;
      if (CYCLES_GT(target, now + STEP_N))
        next = now + STEP_N;
      elprintf(EL_32X, "sh2 exec to %u %d,%d/%d, flags %x", next,
        next - msh2.m68krcycles_done, next - ssh2.m68krcycles_done,
        m68k_target - now, Pico32x.emu_flags);

      pprof_start(ssh2);
      if (!(ssh2.state & SH2_IDLE_STATES)) {
        cycles = next - ssh2.m68krcycles_done;
        if (cycles > 0) {
          run_sh2(&ssh2, cycles > 20U ? cycles : 20U);

          if (event_time_next && CYCLES_GT(target, event_time_next))
            target = event_time_next;
          if (CYCLES_GT(next, target))
            next = target;
        }
      }
      pprof_end(ssh2);

      pprof_start(msh2);
      if (!(msh2.state & SH2_IDLE_STATES)) {
        cycles = next - msh2.m68krcycles_done;
        if (cycles > 0) {
          run_sh2(&msh2, cycles > 20U ? cycles : 20U);

          if (event_time_next && CYCLES_GT(target, event_time_next))
            target = event_time_next;
          if (CYCLES_GT(next, target))
            next = target;
        }
      }
      pprof_end(msh2);

      now = next;
      if (CYCLES_GT(now, msh2.m68krcycles_done)) {
        if (!(msh2.state & SH2_IDLE_STATES))
          now = msh2.m68krcycles_done;
      }
      if (CYCLES_GT(now, ssh2.m68krcycles_done)) {
        if (!(ssh2.state & SH2_IDLE_STATES)) 
          now = ssh2.m68krcycles_done;
      }
      if (CYCLES_GT(now, timer_cycles+STEP_N)) {
        if  (msh2.state & SH2_TIMER_RUN)
          p32x_timer_do(&msh2, now - timer_cycles);
        if  (ssh2.state & SH2_TIMER_RUN)
          p32x_timer_do(&ssh2, now - timer_cycles);
        timer_cycles = now;
      }
    }

    if  (msh2.state & SH2_TIMER_RUN)
      p32x_timer_do(&msh2, now - timer_cycles);
    if  (ssh2.state & SH2_TIMER_RUN)
      p32x_timer_do(&ssh2, now - timer_cycles);
    timer_cycles = now;
  }
  pprof_end_sub(m68k);

  // advance idle CPUs
  if (msh2.state & SH2_IDLE_STATES) {
    if (CYCLES_GT(m68k_target, msh2.m68krcycles_done))
      msh2.m68krcycles_done = m68k_target;
  }
  if (ssh2.state & SH2_IDLE_STATES) {
    if (CYCLES_GT(m68k_target, ssh2.m68krcycles_done))
      ssh2.m68krcycles_done = m68k_target;
  }

  // everyone is in sync now
  Pico32x.comm_dirty = 0;
}

void sync_sh2s_lockstep(unsigned int m68k_target)
{
  unsigned int mcycles;
  
  mcycles = msh2.m68krcycles_done;
  if (CYCLES_GT(mcycles, ssh2.m68krcycles_done))
    mcycles = ssh2.m68krcycles_done;

  while (CYCLES_GT(m68k_target, mcycles)) {
    mcycles += STEP_LS;
    sync_sh2s_normal(mcycles);
  }
}

#ifdef GNW_32X_CORE
// No Sega CD in the 32X core: the 68k always runs directly (never pcd_run_cpus).
#define CPUS_RUN(m68k_cycles) do { \
  SekRunM68k(m68k_cycles); \
  \
  if ((Pico32x.emu_flags & P32XF_Z80_32X_IO) && Pico.m.z80Run \
      && !Pico.m.z80_reset && (PicoIn.opt & POPT_EN_Z80)) \
    PicoSyncZ80(SekCyclesDone()); \
  if (Pico32x.emu_flags & (P32XF_68KCPOLL|P32XF_68KVPOLL)) \
    p32x_sync_sh2s(SekCyclesDone()); \
} while (0)
#else
#define CPUS_RUN(m68k_cycles) do { \
  if (PicoIn.AHW & PAHW_MCD) \
    pcd_run_cpus(m68k_cycles); \
  else \
    SekRunM68k(m68k_cycles); \
  \
  if ((Pico32x.emu_flags & P32XF_Z80_32X_IO) && Pico.m.z80Run \
      && !Pico.m.z80_reset && (PicoIn.opt & POPT_EN_Z80)) \
    PicoSyncZ80(SekCyclesDone()); \
  if (Pico32x.emu_flags & (P32XF_68KCPOLL|P32XF_68KVPOLL)) \
    p32x_sync_sh2s(SekCyclesDone()); \
} while (0)
#endif // GNW_32X_CORE

#define PICO_32X
#define PICO_CD
#include "../pico_cmn.c"

void PicoFrame32x(void)
{
#ifndef GNW_32X_CORE
  if (PicoIn.AHW & PAHW_MCD)
    pcd_prepare_frame();
#endif

  PicoFrameStart();
  Pico32x.sync_line = 0;
  if (Pico32xDrawMode != PDM32X_BOTH)
    Pico.est.rendstatus |= PDRAW_SYNC_NEEDED;
  PicoFrameHints();

  elprintf(EL_32X, "poll: %02x %02x %02x",
    Pico32x.emu_flags & 3, msh2.state, ssh2.state);
}

// calculate multipliers against 68k clock (7670442)
// normally * 3, but effectively slower due to high latencies everywhere
// however using something lower breaks MK2 animations
void Pico32xSetClocks(int msh2_hz, int ssh2_hz)
{
  float m68k_clk = (float)(OSC_NTSC / 7);
  if (msh2_hz > 0) {
    msh2.mult_m68k_to_sh2 = (int)((float)msh2_hz * (1 << CYCLE_MULT_SHIFT) / m68k_clk);
    msh2.mult_sh2_to_m68k = (int)(m68k_clk * (1 << CYCLE_MULT_SHIFT) / (float)msh2_hz);
  }
  if (ssh2_hz > 0) {
    ssh2.mult_m68k_to_sh2 = (int)((float)ssh2_hz * (1 << CYCLE_MULT_SHIFT) / m68k_clk);
    ssh2.mult_sh2_to_m68k = (int)(m68k_clk * (1 << CYCLE_MULT_SHIFT) / (float)ssh2_hz);
  }
}

void Pico32xStateLoaded(int is_early)
{
  if (is_early) {
    Pico32xMemStateLoaded();
    return;
  }

  if (CYCLES_GE(sh2s[0].m68krcycles_done - Pico.t.m68c_aim, 500) ||
      CYCLES_GE(sh2s[1].m68krcycles_done - Pico.t.m68c_aim, 500))
    sh2s[0].m68krcycles_done = sh2s[1].m68krcycles_done = SekCyclesDone();
  p32x_update_irls(NULL, SekCyclesDone());
  p32x_timers_recalc();
  p32x_pwm_state_loaded();
  p32x_run_events(SekCyclesDone());

  // TODO wakeup CPUs for now. poll detection stuff must go to the save state!
  p32x_m68k_poll_event(0, -1);
  p32x_sh2_poll_event(msh2.poll_addr, &msh2, SH2_IDLE_STATES, msh2.m68krcycles_done);
  p32x_sh2_poll_event(ssh2.poll_addr, &ssh2, SH2_IDLE_STATES, ssh2.m68krcycles_done);
}

void Pico32xPrepare(void)
{
  // fallback in case it was missing in saved config
  if (msh2.mult_m68k_to_sh2 == 0 || msh2.mult_sh2_to_m68k == 0)
    Pico32xSetClocks(PICO_MSH2_HZ, 0);
  if (ssh2.mult_m68k_to_sh2 == 0 || ssh2.mult_sh2_to_m68k == 0)
    Pico32xSetClocks(0, PICO_SSH2_HZ);

  sh2_execute_prepare(&msh2, PicoIn.opt & POPT_EN_DRC);
  sh2_execute_prepare(&ssh2, PicoIn.opt & POPT_EN_DRC);
}

// vim:shiftwidth=2:ts=2:expandtab
