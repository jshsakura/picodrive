/*
 * PicoDrive
 * (C) notaz, 2009,2010
 * (C) irixxxx, 2019-2024
 *
 * This work is licensed under the terms of MAME license.
 * See COPYING file in the top-level directory.
 */
#include "../pico_int.h"

// NB: 32X officially doesn't support H32 mode. However, it does work since the
// cartridge slot carries the EDCLK signal which is always H40 clock and is used
// as video clock by the 32X. The H32 MD image is overlaid with the 320 px 32X
// image which has the same on-screen width. How the /YS signal on the cartridge
// slot (signalling the display of background color) is processed in this case
// is however unclear and might lead to glitches due to race conditions by the
// different video clocks for H32 and H40.
// NB: there is an offset of 4 pixels between MD and 32X layers in H32 mode.
#define H32_OFFSET	4

// BGR555 to native conversion
#if defined(USE_BGR555)
#define PXCONV(t)   ((t)&(mr|mg|mb|mp))
#define PXPRIO      0x8000  // prio in MSB
#elif defined(USE_BGR565)
#define PXCONV(t)   (((t)&mr)  | (((t)&(mg|mb)) << 1) | (((t)&mp) >> 10))
#define PXPRIO      0x0020  // prio in LS green bit
#else // RGB565 
#define PXCONV(t)   ((((t)&mr) << 11) | (((t)&mg) << 1) | (((t)&(mp|mb)) >> 10))
#define PXPRIO      0x0020  // prio in LS green bit
#endif

int (*PicoScan32xBegin)(unsigned int num);
int (*PicoScan32xEnd)(unsigned int num);
int Pico32xDrawMode;

void *DrawLineDestBase32x;
int DrawLineDestIncrement32x;

static void convert_pal555(int invert_prio)
{
  u32 *ps = (void *)Pico32xMem->pal;
  u32 *pd = (void *)Pico32xMem->pal_native;
  u32 mr = 0x001f001f; // masks for red, green, blue, prio
  u32 mg = 0x03e003e0;
  u32 mb = 0x7c007c00;
  u32 mp = 0x80008000;
  u32 inv = 0;
  int i;

  if (invert_prio)
    inv = 0x80008000;

  for (i = 0x100/2; i > 0; i--, ps++, pd++) {
    u32 t = *ps ^ inv;
    *pd = PXCONV(t);
  }

  Pico32x.dirty_pal = 0;
}

// direct color mode
#define do_line_dc(pd, p32x, pmd, inv, pmd_draw_code)             \
{                                                                 \
  const u16 mr = 0x001f;                                          \
  const u16 mg = 0x03e0;                                          \
  const u16 mb = 0x7c00;                                          \
  const u16 mp = 0x0000;                                          \
  unsigned short t;                                               \
  int i = 320;                                                    \
                                                                  \
  while (i > 0) {                                                 \
    for (; i > 0 && (*pmd & 0x3f) == mdbg; pd++, pmd++, i--) {    \
      t = *p32x++;                                                \
      *pd = PXCONV(t);                                            \
    }                                                             \
    for (; i > 0 && (*pmd & 0x3f) != mdbg; pd++, pmd++, i--) {    \
      t = *p32x++ ^ inv;                                          \
      if (t & 0x8000)                                             \
        *pd = PXCONV(t);                                          \
      else                                                        \
        pmd_draw_code;                                            \
    }                                                             \
  }                                                               \
}

// packed pixel mode
//
// 2-pixel unroll with combined 32-bit stores, plus solid-run detection
// (mirrors draw_arm.S labels 5-9). The output line buffer `pd` (DrawLineDest)
// is the launcher's RGB565 framebuffer: AHB-allocated and 4-byte aligned
// (320 px * 2 B), so two adjacent pixels may always be written as one 32-bit
// store. 32X DRAM holds big-endian packed pixels; when the read pointer is
// 2-byte aligned a single u16 load fetches both pixel bytes (high byte = pixel
// N, low byte = pixel N+1), otherwise two byte loads are used. 320 is even, so
// there is never an odd-pixel remainder. The run-length merge semantics
// (MD-background check + 32X priority) are preserved exactly: the
// MD-background run always writes the 32X pixel, the non-background run writes
// it only when PXPRIO is set (else the per-pixel pmd_draw_code fallback, which
// handles all three md_code variants correctly).
//
// Solid-run detection: before the per-pixel loops, 4 leading 32X pixels are
// probed (short-circuit &&, so non-run pixels cost ~2 byte reads).  When all
// four match, the palette entry is converted ONCE and the run is counted with
// u16 loads (2 pixels/iteration, like the arm ldrh loop) when p32x is aligned.
// The run is blasted with paired 32-bit stores (4 pixels/iteration) when the
// merge rule guarantees a 32X write for every pixel: 32X priority set (wins
// regardless of MD) OR every pixel sits over MD-background.  Mixed-bg runs
// are handled per-pixel with the pre-converted color (no pal[] lookup per
// pixel).  The entire run is always consumed, so detection never re-fires
// O(n^2) on the same identical pixels.
/* Ablation switch for the solid-run detector below. On a textured 3D scene the
 * four-pixel probe mostly fails, and a failed probe is pure tax on the
 * per-pixel path that follows. Whether the runs it does catch pay for that is a
 * measurement, not an opinion: build with -DGNW_PP_NO_RUNDET to price it. */
#ifdef GNW_PP_NO_RUNDET
#define GNW_PP_RUNDET 0
#else
#define GNW_PP_RUNDET 1
#endif
#ifdef GNW_PP_NO_QUAD
#define GNW_PP_QUAD 0
#else
#define GNW_PP_QUAD 1
#endif
/* DEFAULT OFF, and not because it lost. It wins on the rig (-1.02%) and the
 * device has never seen it: the md32x overlay has 12 bytes of RAM_EMU
 * headroom (_OVERLAY_MD32X_BSS_END = 0x240ffff4, read from the ELF), and this
 * path costs 592 -- its ~150 B body is instantiated once per make_do_loop
 * variant inside PicoDraw32xLayer, which the linker script pins in RAM_EMU.
 * A flagless build must link, so it is off until the bytes exist. Turn it on
 * with -DGNW_PP_OCTA_ON and bench it on the device; 32X_CLOSED.md says where
 * the bytes would have to come from. */
#if defined(GNW_PP_OCTA_ON) && !defined(GNW_PP_NO_OCTA) && !defined(GNW_PP_NO_QUAD)
#define GNW_PP_OCTA 1
#else
#define GNW_PP_OCTA 0
#endif

/* ONE COPY, NOT EIGHT.
 *
 * This used to be a macro, and `make_do_loop` instantiates its callers six
 * times (bare, _md, _h32, _scan, _scan_h32, _scan_md) with FinalizeLine32xRGB555
 * expanding it twice more. Every one of those copies landed in RAM_EMU, because
 * the linker script pins PicoDraw32xLayer / PicoDraw32xLayerMdOnly /
 * FinalizeLine32xRGB555 there and the do_loop_* statics inline into them. That
 * is where the overlay's headroom went: the solid-run detector alone occupies
 * 2,604 bytes of a region with tens of bytes free, and it is about 100 bytes of
 * code multiplied by the instantiation count.
 *
 * The variants differed in exactly one thing -- what to write where the MD
 * layer is NOT background -- and that has three forms. It is a parameter now.
 * The cost is one compare on md_mode per non-background pixel, which for a 3D
 * view is the cold half of the loop; the gain is spendable bytes.
 *
 * NOTE FOR THE LINKER SCRIPT: this function must be listed explicitly in
 * .overlay_md32x (`build/md32x/pico__32x__draw.o (.text.gnw_line_pp)`),
 * or it lands in XIP and the compositor runs out of external flash. */
#define GNW_PP_MD_NONE 0
#define GNW_PP_MD_PAL  1
#define GNW_PP_MD_H32  2

#define GNW_PP_MD_WRITE() do {                                    \
    if (md_mode == GNW_PP_MD_PAL)      *pd = palmd[*pmd];         \
    else if (md_mode == GNW_PP_MD_H32) *pd = pd[H32_OFFSET];      \
  } while (0)

static void gnw_line_pp(unsigned short *pd, unsigned char *p32x,
                        unsigned char *pmd, unsigned short *pal,
                        unsigned short *palmd, int mdbg, int md_mode)
{
  unsigned short t, t0, t1, v;
  const u32 mdbg4 = (u32)(mdbg & 0x3f) * 0x01010101u;
  int i = 320;
  _Static_assert(320 % 2 == 0, "line must be an even pixel count");
  while (i > 0) {
    /* --- solid-run detection (draw_arm.S labels 5-9) --- */
    if (GNW_PP_RUNDET && i >= 4) {
      unsigned char b0 = *(unsigned char *)(MEM_BE2((uintptr_t)(p32x)));
      if (b0 == *(unsigned char *)(MEM_BE2((uintptr_t)(p32x+1))) &&
          b0 == *(unsigned char *)(MEM_BE2((uintptr_t)(p32x+2))) &&
          b0 == *(unsigned char *)(MEM_BE2((uintptr_t)(p32x+3)))) {
        unsigned short sv = pal[b0];
        int run = 4, n;
        if (sv & PXPRIO) {
          /* prio: 32X wins everywhere. Count pure 32X, blast. */
          if (!((uintptr_t)(p32x) & 1)) {
            u16 exp = (u16)(b0 | (b0 << 8));
            while (run + 1 < i && *(u16 *)(p32x + run) == exp)
              run += 2;
            if (run < i && *(unsigned char *)(MEM_BE2((uintptr_t)(p32x+run))) == b0) run++;
          } else
            while (run < i && *(unsigned char *)(MEM_BE2((uintptr_t)(p32x+run))) == b0) run++;
          { u32 pair = (u32)sv | ((u32)sv << 16);
            /* pd can be 2 mod 4 (odd line offs / H32 stride). A fused
             * pair of u32 stores compiles to STRD, which faults on
             * M-profile unless 4-byte aligned (same bug class as SM
             * ClearBackdrop). Align first; the blast is then strd-safe. */
            n = run;
            if ((uintptr_t)pd & 3) { *pd = sv; pd++; n--; }
            { u32 *p32 = (u32 *)(void *)pd;
              for (; n >= 4; n -= 4) { p32[0] = pair; p32[1] = pair; p32 += 2; pd += 4; }
              for (; n >= 2; n -= 2) { *p32++ = pair; pd += 2; }
            }
            if (n) { *pd = sv; pd++; }
          }
          pmd += run; p32x += run; i -= run;
          continue;
        }
        /* no prio: count bg+identical (u16 for aligned) */
        if ((pmd[0] & 0x3f) == mdbg && (pmd[1] & 0x3f) == mdbg &&
            (pmd[2] & 0x3f) == mdbg && (pmd[3] & 0x3f) == mdbg) {
          if (!((uintptr_t)(p32x) & 1)) {
            u16 exp = (u16)(b0 | (b0 << 8));
            while (run + 1 < i && *(u16 *)(p32x + run) == exp &&
                   (pmd[run] & 0x3f) == mdbg && (pmd[run+1] & 0x3f) == mdbg)
              run += 2;
            if (run < i && (pmd[run] & 0x3f) == mdbg &&
                *(unsigned char *)(MEM_BE2((uintptr_t)(p32x+run))) == b0) run++;
          } else
            while (run < i && (pmd[run] & 0x3f) == mdbg &&
                   *(unsigned char *)(MEM_BE2((uintptr_t)(p32x+run))) == b0) run++;
          /* blast bg+identical run */
          { u32 pair = (u32)sv | ((u32)sv << 16);
            /* pd can be 2 mod 4 (odd line offs / H32 stride). A fused
             * pair of u32 stores compiles to STRD, which faults on
             * M-profile unless 4-byte aligned (same bug class as SM
             * ClearBackdrop). Align first; the blast is then strd-safe. */
            n = run;
            if ((uintptr_t)pd & 3) { *pd = sv; pd++; n--; }
            { u32 *p32 = (u32 *)(void *)pd;
              for (; n >= 4; n -= 4) { p32[0] = pair; p32[1] = pair; p32 += 2; pd += 4; }
              for (; n >= 2; n -= 2) { *p32++ = pair; pd += 2; }
            }
            if (n) { *pd = sv; pd++; }
          }
          pmd += run; p32x += run; i -= run;
          /* consume remaining solid-run pixels (non-bg portion) */
          while (i > 0 && *(unsigned char *)(MEM_BE2((uintptr_t)(p32x))) == b0) {
            if ((*pmd & 0x3f) == mdbg) *pd = sv;
            else GNW_PP_MD_WRITE();
            pd++; pmd++; p32x++; i--;
          }
          continue;
        }
        /* mixed-bg (not all bg): per-pixel with constant sv */
        while (run < i && *(unsigned char *)(MEM_BE2((uintptr_t)(p32x+run))) == b0) run++;
        for (n = 0; n < run; n++) {
          if ((*pmd & 0x3f) == mdbg) *pd = sv;
          else GNW_PP_MD_WRITE();
          pd++; pmd++;
        }
        p32x += run; i -= run;
        continue;
      }
    }
    /* MD-background run: 32X pixel shown unconditionally */
    for (; i > 0 && (*pmd & 0x3f) == mdbg; ) {
      /* Four at a time. Doom's 3D view leaves the MD layer at
       * background for whole lines, so this is the path that
       * actually runs, and the pair loop below asks the MD byte
       * twice per two pixels -- two loads, two ANDs, two
       * compares. One word load answers for four. Guarded on the
       * three pointers being aligned, which is the common case;
       * anything else falls through to the pair path unchanged.
       * GNW_PP_NO_QUAD prices it. */
      /* Eight at a time, same test twice over. The quad path measures
       * 3.0% of the frame on the rig -- and the rig UNDER-prices wide
       * stores, having no cache or write buffer (2026-08-20: it called
       * removing the solid-run detector a 0.48% gain and the device
       * charged 2.95%). So the octa path is deliberately built for the
       * device to judge, not the rig. -DGNW_PP_NO_OCTA prices it. */
      if (GNW_PP_OCTA && i >= 8
          && !(((uintptr_t)pmd | (uintptr_t)pd) & 3)
          && !((uintptr_t)(p32x) & 1)
          && (((u32 *)pmd)[0] & 0x3f3f3f3fu) == mdbg4
          && (((u32 *)pmd)[1] & 0x3f3f3f3fu) == mdbg4) {
        u32 q0 = ((u32 *)(p32x))[0], q1 = ((u32 *)(p32x))[1];
        u16 b0 = pal[(q0 >>  8) & 0xff], b1 = pal[q0 & 0xff];
        u16 b2 = pal[(q0 >> 24) & 0xff], b3 = pal[(q0 >> 16) & 0xff];
        u16 b4 = pal[(q1 >>  8) & 0xff], b5 = pal[q1 & 0xff];
        u16 b6 = pal[(q1 >> 24) & 0xff], b7 = pal[(q1 >> 16) & 0xff];
        ((u32 *)pd)[0] = (u32)b0 | ((u32)b1 << 16);
        ((u32 *)pd)[1] = (u32)b2 | ((u32)b3 << 16);
        ((u32 *)pd)[2] = (u32)b4 | ((u32)b5 << 16);
        ((u32 *)pd)[3] = (u32)b6 | ((u32)b7 << 16);
        pd += 8; pmd += 8; p32x += 8; i -= 8;
        continue;
      }
      if (GNW_PP_QUAD && i >= 4
          && !(((uintptr_t)pmd | (uintptr_t)pd) & 3)
          && !((uintptr_t)(p32x) & 1)
          && (*(u32 *)pmd & 0x3f3f3f3fu) == mdbg4) {
        u32 q = *(u32 *)(p32x);
        u16 a0 = pal[(q >>  8) & 0xff], a1 = pal[q & 0xff];
        u16 a2 = pal[(q >> 24) & 0xff], a3 = pal[(q >> 16) & 0xff];
        ((u32 *)pd)[0] = (u32)a0 | ((u32)a1 << 16);
        ((u32 *)pd)[1] = (u32)a2 | ((u32)a3 << 16);
        pd += 4; pmd += 4; p32x += 4; i -= 4;
        continue;
      }
      if (i >= 2 && (pmd[1] & 0x3f) == mdbg) {
        if (!((uintptr_t)(p32x) & 1)) {
          v = *(u16 *)(p32x);
          t0 = pal[(v >> 8) & 0xff];
          t1 = pal[v & 0xff];
        } else {
          t0 = pal[*(unsigned char *)(MEM_BE2((uintptr_t)(p32x+0)))];
          t1 = pal[*(unsigned char *)(MEM_BE2((uintptr_t)(p32x+1)))];
        }
        *(u32 *)(pd) = (u32)t0 | ((u32)t1 << 16);
        pd += 2; pmd += 2; p32x += 2; i -= 2;
      } else {
        t = pal[*(unsigned char *)(MEM_BE2((uintptr_t)(p32x++)))];
        *pd++ = t; pmd++; i--;
      }
    }
    /* non-background run: 32X pixel only if priority bit set */
    for (; i > 0 && (*pmd & 0x3f) != mdbg; ) {
      if (i >= 2 && (pmd[1] & 0x3f) != mdbg) {
        if (!((uintptr_t)(p32x) & 1)) {
          v = *(u16 *)(p32x);
          t0 = pal[(v >> 8) & 0xff];
          t1 = pal[v & 0xff];
        } else {
          t0 = pal[*(unsigned char *)(MEM_BE2((uintptr_t)(p32x+0)))];
          t1 = pal[*(unsigned char *)(MEM_BE2((uintptr_t)(p32x+1)))];
        }
        if ((t0 & PXPRIO) && (t1 & PXPRIO)) {
          *(u32 *)(pd) = (u32)t0 | ((u32)t1 << 16);
          pd += 2; pmd += 2; p32x += 2; i -= 2;
        } else {
          /* per-pixel fallback: md_code needs sequential advance */
          if (t0 & PXPRIO) *pd = t0; else GNW_PP_MD_WRITE();
          pd++; pmd++; p32x++; i--;
          if (t1 & PXPRIO) *pd = t1; else GNW_PP_MD_WRITE();
          pd++; pmd++; p32x++; i--;
        }
      } else {
        t = pal[*(unsigned char *)(MEM_BE2((uintptr_t)(p32x++)))];
        if (t & PXPRIO) *pd = t; else GNW_PP_MD_WRITE();
        pd++; pmd++; i--;
      }
    }
  }
}

/* The macro survives as the call, and advances what the old one advanced:
 * callers relied on pd and pmd coming back 320 pixels further on. */
#define do_line_pp(pd_, p32x_, pmd_, md_mode_) do {               \
    gnw_line_pp((pd_), (p32x_), (pmd_), pal, palmd, mdbg, (md_mode_)); \
    (pd_) += 320; (pmd_) += 320;                                  \
  } while (0)

// run length mode
#define do_line_rl(pd, p32x, pmd, pmd_draw_code)                  \
{                                                                 \
  unsigned short len, t;                                          \
  int i;                                                          \
  for (i = 320; i > 0; p32x++) {                                  \
    t = pal[*p32x & 0xff];                                        \
    for (len = (*p32x >> 8) + 1; len > 0 && i > 0; len--, i--, pd++, pmd++) { \
      if ((*pmd & 0x3f) == mdbg || (t & PXPRIO))                  \
        *pd = t;                                                  \
      else                                                        \
        pmd_draw_code;                                            \
    }                                                             \
  }                                                               \
}

#define MD_LAYER_CODE_H32 \
  *dst = dst[H32_OFFSET]

// this is almost never used (Wiz and menu bg gen only)
void FinalizeLine32xRGB555(int sh, int line, struct PicoEState *est)
{
  unsigned short *dst = est->DrawLineDest;
  unsigned short *pal = Pico32xMem->pal_native;
  unsigned char  *pmd = est->HighCol + 8;
  unsigned short *palmd = est->HighPal;
  unsigned short *dram, *p32x;
  unsigned char   mdbg;
  int h32 = !(Pico.video.reg[12] & 0x1);

  FinalizeLine555(sh, line, est);

  if ((Pico32x.vdp_regs[0] & P32XV_Mx) == 0 || // 32x blanking
      (Pico.video.debug_p & PVD_KILL_32X))
  {
    return;
  }

  dram = (void *)Pico32xMem->dram[Pico32x.vdp_regs[0x0a/2] & P32XV_FS];
  p32x = dram + dram[line];
  mdbg = Pico.video.reg[7] & 0x3f;
  if (h32) pmd += H32_OFFSET;

  if ((Pico32x.vdp_regs[0] & P32XV_Mx) == 2) { // Direct Color Mode
    int inv_bit = (Pico32x.vdp_regs[0] & P32XV_PRI) ? 0x8000 : 0;
    if (h32) {
      do_line_dc(dst, p32x, pmd, inv_bit, MD_LAYER_CODE_H32);
    } else
      do_line_dc(dst, p32x, pmd, inv_bit,);
    return;
  }

  if (Pico32x.dirty_pal)
    convert_pal555(Pico32x.vdp_regs[0] & P32XV_PRI);

  if ((Pico32x.vdp_regs[0] & P32XV_Mx) == 1) { // Packed Pixel Mode
    unsigned char *p32xb = (void *)p32x;
    if (Pico32x.vdp_regs[2 / 2] & P32XV_SFT)
      p32xb++;
    if (h32) {
      do_line_pp(dst, p32xb, pmd, GNW_PP_MD_H32);
    } else
      do_line_pp(dst, p32xb, pmd, GNW_PP_MD_NONE);
  }
  else { // Run Length Mode
    if (h32) {
      do_line_rl(dst, p32x, pmd, MD_LAYER_CODE_H32);
    } else
      do_line_rl(dst, p32x, pmd,);
  }
}

#define MD_LAYER_CODE \
  *dst = palmd[*pmd]

#define PICOSCAN_PRE \
  PicoScan32xBegin(l + (lines_sft_offs & 0xff)); \
  dst = Pico.est.DrawLineDest; \

#define PICOSCAN_POST \
  PicoScan32xEnd(l + (lines_sft_offs & 0xff)); \
  Pico.est.DrawLineDest = (char *)Pico.est.DrawLineDest + DrawLineDestIncrement32x; \

/* RIG_PP_CENSUS: the two facts that decide whether the LTDC's L8 mode can
 * replace this compositor (see docs/32X_NEXT_SESSION.md).
 *
 *  1. How many DISTINCT 32X packed-pixel indices a scene uses. The LTDC CLUT
 *     holds 256; if the game leaves some free, the MD layer's colours can have
 *     them and hardware scanout becomes possible without a second layer.
 *  2. Where the MD layer is non-background. If it is confined to a rectangle
 *     (Doom's status bar), a second LTDC layer covering only that rectangle is
 *     the other way in.
 *
 * Whole-run figures, not per-frame: a CLUT and a layer rectangle have to serve
 * every frame of a scene. Rig-only; off => no code emitted. */
#ifdef RIG_PP_CENSUS
unsigned char rig_pp_idx_seen[256];
int rig_pp_md_l0 = 9999, rig_pp_md_l1 = -1;
int rig_pp_md_x0 = 9999, rig_pp_md_x1 = -1;
unsigned long long rig_pp_md_px, rig_pp_px;
static void rig_pp_census(int l, const unsigned char *p32x,
                          const unsigned char *pmd, int mdbg)
{
  int x;
  for (x = 0; x < 320; x++) {
    rig_pp_idx_seen[p32x[(unsigned)x ^ 1]] = 1;
    rig_pp_px++;
    if ((pmd[x] & 0x3f) != mdbg) {
      rig_pp_md_px++;
      if (l < rig_pp_md_l0) rig_pp_md_l0 = l;
      if (l > rig_pp_md_l1) rig_pp_md_l1 = l;
      if (x < rig_pp_md_x0) rig_pp_md_x0 = x;
      if (x > rig_pp_md_x1) rig_pp_md_x1 = x;
    }
  }
}
#define RIG_PP_CENSUS(l, p32x, pmd, mdbg) rig_pp_census(l, p32x, pmd, mdbg)
#else
#define RIG_PP_CENSUS(l, p32x, pmd, mdbg) ((void)0)
#endif

#define make_do_loop(name, pre_code, post_code, md_code, pp_mode) \
/* Direct Color Mode */                                         \
static void do_loop_dc##name(unsigned short *dst,               \
    unsigned short *dram, unsigned lines_sft_offs, int mdbg)    \
{                                                               \
  int inv_bit = (Pico32x.vdp_regs[0] & P32XV_PRI) ? 0x8000 : 0; \
  unsigned char  *pmd = Pico.est.Draw2FB +                      \
                          328 * (lines_sft_offs & 0xff) + 8;    \
  unsigned short *palmd = Pico.est.HighPal;                     \
  unsigned short *p32x;                                         \
  int lines = (lines_sft_offs >> 16) & 0xff;                    \
  int l;                                                        \
  if (lines_sft_offs & (2<<8)) pmd += H32_OFFSET;               \
  (void)palmd;                                                  \
  for (l = 0; l < lines; l++, pmd += 8) {                       \
    pre_code;                                                   \
    p32x = dram + dram[l + (lines_sft_offs >> 24)];             \
    do_line_dc(dst, p32x, pmd, inv_bit, md_code);               \
    post_code;                                                  \
    dst += DrawLineDestIncrement32x/2 - 320;                    \
  }                                                             \
}                                                               \
                                                                \
/* Packed Pixel Mode */                                         \
static void do_loop_pp##name(unsigned short *dst,               \
    unsigned short *dram, unsigned lines_sft_offs, int mdbg)    \
{                                                               \
  unsigned short *pal = Pico32xMem->pal_native;                 \
  unsigned char  *pmd = Pico.est.Draw2FB +                      \
                          328 * (lines_sft_offs & 0xff) + 8;    \
  unsigned short *palmd = Pico.est.HighPal;                     \
  unsigned char  *p32x;                                         \
  int lines = (lines_sft_offs >> 16) & 0xff;                    \
  int l;                                                        \
  if (lines_sft_offs & (2<<8)) pmd += H32_OFFSET;               \
  (void)palmd;                                                  \
  for (l = 0; l < lines; l++, pmd += 8) {                       \
    pre_code;                                                   \
    p32x = (void *)(dram + dram[l + (lines_sft_offs >> 24)]);   \
    p32x += (lines_sft_offs >> 8) & 1;                          \
    RIG_PP_CENSUS(l, p32x, pmd, mdbg);                          \
    do_line_pp(dst, p32x, pmd, pp_mode);                        \
    post_code;                                                  \
    dst += DrawLineDestIncrement32x/2 - 320;                    \
  }                                                             \
}                                                               \
                                                                \
/* Run Length Mode */                                           \
static void do_loop_rl##name(unsigned short *dst,               \
    unsigned short *dram, unsigned lines_sft_offs, int mdbg)    \
{                                                               \
  unsigned short *pal = Pico32xMem->pal_native;                 \
  unsigned char  *pmd = Pico.est.Draw2FB +                      \
                          328 * (lines_sft_offs & 0xff) + 8;    \
  unsigned short *palmd = Pico.est.HighPal;                     \
  unsigned short *p32x;                                         \
  int lines = (lines_sft_offs >> 16) & 0xff;                    \
  int l;                                                        \
  if (lines_sft_offs & (2<<8)) pmd += H32_OFFSET;               \
  (void)palmd;                                                  \
  for (l = 0; l < lines; l++, pmd += 8) {                       \
    pre_code;                                                   \
    p32x = dram + dram[l + (lines_sft_offs >> 24)];             \
    do_line_rl(dst, p32x, pmd, md_code);                        \
    post_code;                                                  \
    dst += DrawLineDestIncrement32x/2 - 320;                    \
  }                                                             \
}

#ifdef _ASM_32X_DRAW
#undef make_do_loop
#define make_do_loop(name, pre_code, post_code, md_code, pp_mode) \
extern void do_loop_dc##name(unsigned short *dst,        \
    unsigned short *dram, unsigned lines_offs, int mdbg);\
extern void do_loop_pp##name(unsigned short *dst,        \
    unsigned short *dram, unsigned lines_offs, int mdbg);\
extern void do_loop_rl##name(unsigned short *dst,        \
    unsigned short *dram, unsigned lines_offs, int mdbg);
#endif

make_do_loop(,,,,GNW_PP_MD_NONE)
make_do_loop(_md, , , MD_LAYER_CODE, GNW_PP_MD_PAL)
make_do_loop(_h32, , , MD_LAYER_CODE_H32, GNW_PP_MD_H32)
make_do_loop(_scan, PICOSCAN_PRE, PICOSCAN_POST, , GNW_PP_MD_NONE)
make_do_loop(_scan_h32, PICOSCAN_PRE, PICOSCAN_POST, MD_LAYER_CODE_H32, GNW_PP_MD_H32)
make_do_loop(_scan_md, PICOSCAN_PRE, PICOSCAN_POST, MD_LAYER_CODE, GNW_PP_MD_PAL)

typedef void (*do_loop_func)(unsigned short *dst, unsigned short *dram, unsigned lines, int mdbg);
enum { DO_LOOP, DO_LOOP_H32, DO_LOOP_MD, DO_LOOP_SCAN, DO_LOOP_H32_SCAN, DO_LOOP_MD_SCAN };

static const do_loop_func do_loop_dc_f[] = { do_loop_dc, do_loop_dc_h32, do_loop_dc_md, do_loop_dc_scan, do_loop_dc_scan_h32, do_loop_dc_scan_md };
static const do_loop_func do_loop_pp_f[] = { do_loop_pp, do_loop_pp_h32, do_loop_pp_md, do_loop_pp_scan, do_loop_pp_scan_h32, do_loop_pp_scan_md };
static const do_loop_func do_loop_rl_f[] = { do_loop_rl, do_loop_rl_h32, do_loop_rl_md, do_loop_rl_scan, do_loop_rl_scan_h32, do_loop_rl_scan_md };

void PicoDraw32xLayer(int offs, int lines, int md_bg)
{
#ifdef GNW_MD_ABLATE_SPR
  /* Measurement build only: kill just the MD sprite layers, leaving the planes.
   * Prices the sprite pipeline on its own -- if Doom's MD layer is a plane-only
   * HUD, every cycle here is pure overhead and the number says how much. */
  Pico.video.debug_p |= PVD_KILL_S_LO | PVD_KILL_S_HI;
#endif
#ifdef GNW_MD_ABLATE_PLANES
  /* ... and the mirror: planes only killed, sprites left. */
  Pico.video.debug_p |= PVD_KILL_A | PVD_KILL_B;
#endif
#ifdef GNW_MD_ABLATE
  /* Measurement build only: kill every MD VDP layer so the pprof "draw" bucket
   * collapses to whatever is irreducible. Prices the ceiling on any MD-side
   * optimisation, and shows whether the MD layer contributes anything the 32X
   * does not already cover (compare framebuffer hashes). Never define this in a
   * shipping build -- it deletes the layer, it does not speed it up. */
  Pico.video.debug_p |= PVD_KILL_A | PVD_KILL_B | PVD_KILL_S_LO | PVD_KILL_S_HI;
#endif
  int have_scan = PicoScan32xBegin != NULL && PicoScan32xEnd != NULL;
  const do_loop_func *do_loop;
  unsigned short *dram;
  int lines_sft_offs;
  int which_func;

  offs += Pico32x.sync_line;

  Pico.est.DrawLineDest = (char *)DrawLineDestBase32x + offs * DrawLineDestIncrement32x;
  Pico.est.DrawLineDestIncr = DrawLineDestIncrement32x;
  dram = Pico32xMem->dram[Pico32x.vdp_regs[0x0a/2] & P32XV_FS];

  if (Pico32xDrawMode == PDM32X_BOTH)
    PicoDrawUpdateHighPal();

  if ((Pico32x.vdp_regs[0] & P32XV_Mx) == 2)
  {
    // Direct Color Mode
    do_loop = do_loop_dc_f;
    goto do_it;
  }

  if (Pico32x.dirty_pal)
    convert_pal555(Pico32x.vdp_regs[0] & P32XV_PRI);

  if ((Pico32x.vdp_regs[0] & P32XV_Mx) == 1)
  {
    // Packed Pixel Mode
    do_loop = do_loop_pp_f;
  }
  else
  {
    // Run Length Mode
    do_loop = do_loop_rl_f;
  }

do_it:
  // In 8bit modes MD+32X layers are merged together in 32X rendering, while in
  // 16bit mode the MD layer is directly created in the target buffer and the
  // 32X layer is overlaid onto that.
  if (Pico32xDrawMode == PDM32X_BOTH)
    which_func = have_scan ? DO_LOOP_MD_SCAN : DO_LOOP_MD;
  else if (!(Pico.video.reg[12] & 1)) // H32, mind 4 px offset
    which_func = have_scan ? DO_LOOP_H32_SCAN : DO_LOOP_H32;
  else
    which_func = have_scan ? DO_LOOP_SCAN : DO_LOOP;
  lines_sft_offs = (Pico32x.sync_line << 24) | (lines << 16) | offs;
  if (Pico32x.vdp_regs[2 / 2] & P32XV_SFT)
    lines_sft_offs |= 1 << 8;
  if (!(Pico.video.reg[12] & 1)) // offset flag for H32
    lines_sft_offs |= 2 << 8;

  do_loop[which_func](Pico.est.DrawLineDest, dram, lines_sft_offs, md_bg);
}

// mostly unused, games tend to keep 32X layer on
void PicoDraw32xLayerMdOnly(int offs, int lines)
{
  int have_scan = PicoScan32xBegin != NULL && PicoScan32xEnd != NULL;
  unsigned short *dst = (void *)((char *)DrawLineDestBase32x + offs * DrawLineDestIncrement32x);
  unsigned char  *pmd = Pico.est.Draw2FB + 328 * offs + 8;
  unsigned short *pal = Pico.est.HighPal;
  int plen = 320;
  int l, p;

  PicoDrawUpdateHighPal();

  offs += Pico32x.sync_line;
  dst += Pico32x.sync_line * DrawLineDestIncrement32x;

  for (l = 0; l < lines; l++) {
    if (have_scan) {
      PicoScan32xBegin(l + offs);
      dst = (unsigned short *)Pico.est.DrawLineDest;
    }
    for (p = 0; p < plen; p += 4) {
      dst[p + 0] = pal[*pmd++];
      dst[p + 1] = pal[*pmd++];
      dst[p + 2] = pal[*pmd++];
      dst[p + 3] = pal[*pmd++];
    }
    dst = Pico.est.DrawLineDest = (char *)dst + DrawLineDestIncrement32x;
    pmd += 328 - plen;
    if (have_scan)
      PicoScan32xEnd(l + offs);
  }
}

void PicoDrawSetOutFormat32x(pdso_t which, int use_32x_line_mode)
{
  if (which == PDF_RGB555) {
    // CLUT pixels needed as well, for layer priority
    PicoDrawSetInternalBuf(Pico.est.Draw2FB, 328);
    PicoDrawSetOutBufMD(NULL, 0);
  } else {
    // store CLUT pixels, same layout as alt renderer
    PicoDrawSetInternalBuf(NULL, 0);
    PicoDrawSetOutBufMD(Pico.est.Draw2FB, 328);
  }

  if (use_32x_line_mode)
    // we'll draw via FinalizeLine32xRGB555 (rare)
    Pico32xDrawMode = PDM32X_OFF;
  else
    // in RGB555 mode the 32x layer is overlaid on the MD layer, in the other
    // modes 32x and MD layer are merged together by the 32x renderer
    Pico32xDrawMode = (which == PDF_RGB555) ? PDM32X_32X_ONLY : PDM32X_BOTH;
}

void PicoDrawSetOutBuf32X(void *dest, int increment)
{
  DrawLineDestBase32x = dest;
  DrawLineDestIncrement32x = increment;
  // in RGB555 mode this buffer is also used by the MD renderer
  if (Pico32xDrawMode != PDM32X_BOTH)
    PicoDrawSetOutBufMD(DrawLineDestBase32x, DrawLineDestIncrement32x);
}

// vim:shiftwidth=2:ts=2:expandtab
