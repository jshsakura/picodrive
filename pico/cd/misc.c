/*
 * PicoDrive
 * (C) notaz, 2007
 *
 * This work is licensed under the terms of MAME license.
 * See COPYING file in the top-level directory.
 */

#include "../pico_int.h"

unsigned char formatted_bram[4*0x10] =
{
#if 0
	0x00, 0xd4, 0x63, 0x00, 0x00, 0x03, 0x03, 0x00, 0x03, 0x03, 0x03, 0x00, 0x03, 0x00, 0x00, 0x03,
	0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x53, 0xd2, 0xf5, 0x3a, 0x48, 0x50, 0x35, 0x0f,
	0x47, 0x14, 0xf5, 0x7e, 0x5c, 0xd4, 0xf3, 0x03, 0x00, 0x03, 0x12, 0x00, 0x0a, 0xff, 0xca, 0xa6,
	0xf5, 0x27, 0xed, 0x22, 0x47, 0xfa, 0x22, 0x96, 0x6c, 0xa5, 0x88, 0x14, 0x48, 0x48, 0x0a, 0xbb,
#endif
	0x5f, 0x5f, 0x5f, 0x5f, 0x5f, 0x5f, 0x5f, 0x5f, 0x5f, 0x5f, 0x5f, 0x00, 0x00, 0x00, 0x00, 0x40,
	0x00, 0x7d, 0x00, 0x7d, 0x00, 0x7d, 0x00, 0x7d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x53, 0x45, 0x47, 0x41, 0x5f, 0x43, 0x44, 0x5f, 0x52, 0x4f, 0x4d, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x52, 0x41, 0x4d, 0x5f, 0x43, 0x41, 0x52, 0x54, 0x52, 0x49, 0x44, 0x47, 0x45, 0x5f, 0x5f, 0x5f,
	// SEGA_CD_ROM.....RAM_CARTRIDGE___
};


// offs | 2Mbit  | 1Mbit  |
//   0  | [ 2M   | unused |
// 128K |  bit ] | bank0  |
// 256K | unused | bank1  |

#ifndef _ASM_MISC_C
PICO_INTERNAL_ASM void wram_2M_to_1M(unsigned char *m)
{
#ifdef GNW_MCD_SPLIT
	/* Deinterleave 16-bit words in place:
	 *   a0,b0,a1,b1,... -> a0,a1,...,b0,b1,...
	 * The stock routine uses an extra 128K overlap area. */
	unsigned short *p = (unsigned short *)m;
	unsigned int pairs, half, block, i;

	for (block = 2; block <= 0x20000; block <<= 1) {
		pairs = block >> 1;
		half = pairs >> 1;
		if (half == 0)
			continue;
		for (i = 0; i < 0x20000; i += block) {
			unsigned int j;
			for (j = 0; j < half; j++) {
				unsigned short t = p[i + half + j];
				p[i + half + j] = p[i + pairs + j];
				p[i + pairs + j] = t;
			}
		}
	}
#else
	unsigned short *m1M_b0, *m1M_b1;
	unsigned int i, tmp, *m2M;

	m2M = (unsigned int *) (m + 0x40000);
	m1M_b0 = (unsigned short *) m2M;
	m1M_b1 = (unsigned short *) (m + 0x60000);

	for (i = 0x40000/4; i; i--)
	{
		tmp = *(--m2M);
		*(--m1M_b0) = tmp;
		*(--m1M_b1) = tmp >> 16;
	}
#endif
}

PICO_INTERNAL_ASM void wram_1M_to_2M(unsigned char *m)
{
#ifdef GNW_MCD_SPLIT
	/* Reverse the stages above to interleave the two contiguous 128K banks. */
	unsigned short *p = (unsigned short *)m;
	unsigned int block, pairs, half, i;

	for (block = 0x20000; block >= 2; block >>= 1) {
		pairs = block >> 1;
		half = pairs >> 1;
		if (half == 0)
			continue;
		for (i = 0; i < 0x20000; i += block) {
			unsigned int j;
			for (j = 0; j < half; j++) {
				unsigned short t = p[i + half + j];
				p[i + half + j] = p[i + pairs + j];
				p[i + pairs + j] = t;
			}
		}
	}
#else
	unsigned short *m1M_b0, *m1M_b1;
	unsigned int i, tmp, *m2M;

	m2M = (unsigned int *) m;
	m1M_b0 = (unsigned short *) (m + 0x20000);
	m1M_b1 = (unsigned short *) (m + 0x40000);

	for (i = 0x40000/4; i; i--)
	{
		tmp = *m1M_b0++ | (*m1M_b1++ << 16);
		*m2M++ = tmp;
	}
#endif
}
#endif
