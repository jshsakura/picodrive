#ifndef _MACROS_H_
#define _MACROS_H_

/* PICODRIVE STORAGE CONVENTION (this g68k exists only as picodrive's 68K):
 * guest memory (ROM/RAM the memory_map bases point into) is stored 16-bit
 * BYTESWAPPED on little-endian hosts — a u16 read at an even address is a
 * native *(u16*) load, but a BYTE lives at (ADDR)^1 (picodrive's MEM_BE2).
 *
 * Neither original gwenesis variant matches that layout:
 *  - its LSB_FIRST set does big-endian byte-lane math for WORDS (wrong here:
 *    words are stored native-LE),
 *  - its big-endian set indexes bytes WITHOUT ^1 (wrong here: every direct
 *    byte access reads/writes the swapped neighbour — VF read its VDP-reg
 *    init table as 04/24 swapped and never enabled VINT; found 2026-07-16).
 * So define exactly one set, matching pico/pico_port.h MEM_BE2/CPU_BE2. */

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#error "g68k memory macros assume a little-endian host (picodrive byteswapped storage)"
#endif

#define READ_BYTE(BASE, ADDR) (BASE)[(ADDR)^1]
#define READ_WORD(BASE, ADDR) *(uint16 *)((BASE) + (ADDR))
#define READ_WORD_LONG(BASE, ADDR) (((uint32)*(uint16 *)((BASE) + (ADDR)) << 16) | \
                                    *(uint16 *)((BASE) + (ADDR) + 2))
#define WRITE_BYTE(BASE, ADDR, VAL) (BASE)[(ADDR)^1] = (VAL) & 0xff
#define WRITE_WORD(BASE, ADDR, VAL) *(uint16 *)((BASE) + (ADDR)) = (VAL) & 0xffff
#define WRITE_WORD_LONG(BASE, ADDR, VAL) { \
    *(uint16 *)((BASE) + (ADDR)) = ((VAL) >> 16) & 0xffff; \
    *(uint16 *)((BASE) + (ADDR) + 2) = (VAL) & 0xffff; \
  }

/* C89 compatibility */
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327f
#endif /* M_PI */

/* Set to your compiler's static inline keyword to enable it, or
 * set it to blank to disable it.
 * If you define INLINE in makefile or osd.h, it will override this value.
 * NOTE: not enabling inline functions will SEVERELY slow down emulation.
 */
#ifndef INLINE
#define INLINE static __inline__
#endif /* INLINE */

/* Alignment macros for cross compiler compatibility */
#if defined(_MSC_VER)
#define ALIGNED_(x) __declspec(align(x))
#elif defined(__GNUC__)
#define ALIGNED_(x) __attribute__ ((aligned(x)))
#endif

/* Default CD image file access (read-only) functions */
/* If you need to override default stdio.h functions with custom filesystem API,
   redefine following macros in platform specific include file (osd.h) or Makefile
*/
#ifndef cdStream
#define cdStream            FILE
#define cdStreamOpen(fname) fopen(fname, "rb")
#define cdStreamClose       fclose
#define cdStreamRead        fread
#define cdStreamSeek        fseek
#define cdStreamTell        ftell
#define cdStreamGets        fgets
#endif

#endif /* _MACROS_H_ */
