/*
 * EMU_G68K memory bridge: picodrive page tables -> gwenesis m68k.memory_map.
 *
 * picodrive keeps four independent 64 KB-page tables (m68k_read8_map,
 * m68k_read16_map, m68k_write8_map, m68k_write16_map); an entry is either
 * (host_base - region_start) >> 1 for memory, or (handler >> 1) | MAP_FLAG.
 * The gwenesis core keeps one cpu_memory_map per page: a shared `base`
 * pointer (used for opcode fetch and for any accessor left NULL) plus four
 * optional handler pointers whose signatures happen to match picodrive's
 * cpu68k_read_f / cpu68k_write_f exactly, so decoded handlers are installed
 * directly - no trampolines.
 *
 * Fetch discipline mirrors FAME: opcode fetch only works from memory-mapped
 * (base) pages, exactly the pages cpu68k_map_set(!is_func) feeds FAME's
 * fetchmap. When a page is switched to handlers its previous base is kept
 * (FAME keeps stale Fetch[] entries the same way); before any mapping exists
 * the base points at PicoMem.ram so a stray fetch reads garbage instead of
 * faulting.
 *
 * If a page mixes memory mappings with DIFFERENT bases across the four
 * tables (picodrive allows it, gwenesis's single base cannot express it),
 * the odd ones out fall back to picodrive's generic dispatchers
 * (m68k_read8/...), which are always correct.
 */

#include "../../pico/pico_int.h"
#include "../../pico/memory.h"

int g68k_not_polling;
int g68k_s68k_stub[2];

/* picodrive's generic dispatchers use u8/u16 data params; gwenesis handler
 * pointers are (unsigned int, unsigned int) - wrap instead of casting. */
static void g68k_disp_write8(unsigned int a, unsigned int d)
{
  m68k_write8(a, (u8)d);
}

static void g68k_disp_write16(unsigned int a, unsigned int d)
{
  m68k_write16(a, (u16)d);
}

static unsigned char *g68k_page_base(uptr v, unsigned int page)
{
  return (unsigned char *)((v << 1) + ((uptr)page << 16));
}

void g68k_map_sync_range(unsigned int start_addr, unsigned int end_addr)
{
  unsigned int page = (start_addr & 0xffffff) >> M68K_MEM_SHIFT;
  unsigned int last = (end_addr & 0xffffff) >> M68K_MEM_SHIFT;

  for (; page <= last; page++)
  {
    cpu_memory_map *mm = &m68k.memory_map[page];
    uptr r8 = m68k_read8_map[page];
    uptr r16 = m68k_read16_map[page];
    uptr w8 = m68k_write8_map[page];
    uptr w16 = m68k_write16_map[page];
    unsigned char *base = NULL;

    /* entry 0 = a never-registered page (the maps are BSS): NOT a real
     * mapping (that would mean host_ptr == guest page start). Installing it
     * as a base makes opcode fetch dereference the raw guest address — a
     * wild read on the host, a Hardfault on the device (VF's MD-mode probe
     * of 0x880000 found this). Treat it as unmapped: keep the previous
     * fetch base and go through the generic dispatchers, which reproduce
     * upstream's behaviour for unregistered pages. */
    int r16_mem = !map_flag_set(r16) && r16 != 0;
    int r8_mem  = !map_flag_set(r8)  && r8 != 0;

    /* choose the fetch/base pointer: 16-bit read base has priority (opcode
     * fetches go through it), then 8-bit read base */
    if (r16_mem)
      base = g68k_page_base(r16, page);
    else if (r8_mem)
      base = g68k_page_base(r8, page);

    if (base)
      mm->base = base; /* else: keep previous base (FAME stale-fetch rule) */
#ifdef GNW_32X_CORE
    else if (map_flag_set(r16))
      /* Handler-mapped page with no memory base (the 32X page-0 stub split):
       * a stale base FETCHES the wrong program — Chaotix/VRDx jsr into the
       * stub helpers. NULL routes fetches through the generic dispatchers
       * (see m68k_read_immediate_16's GNW fallback). */
      mm->base = NULL;
#endif

    if (map_flag_set(r16))
      mm->read16 = (unsigned int (*)(unsigned int))MAP_FUNC(r16);
    else if (r16_mem)
      mm->read16 = NULL; /* direct: r16 base IS mm->base by construction */
    else
      mm->read16 = m68k_read16; /* unregistered page: generic dispatch */

    if (map_flag_set(r8))
      mm->read8 = (unsigned int (*)(unsigned int))MAP_FUNC(r8);
    else if (r8_mem && base && g68k_page_base(r8, page) == base)
      mm->read8 = NULL;
    else
      mm->read8 = m68k_read8; /* different base or unregistered: dispatch */

    if (map_flag_set(w16))
      mm->write16 = (void (*)(unsigned int, unsigned int))MAP_FUNC(w16);
    else if (base && g68k_page_base(w16, page) == base)
      mm->write16 = NULL;
    else
      mm->write16 = g68k_disp_write16;

    if (map_flag_set(w8))
      mm->write8 = (void (*)(unsigned int, unsigned int))MAP_FUNC(w8);
    else if (base && g68k_page_base(w8, page) == base)
      mm->write8 = NULL;
    else
      mm->write8 = g68k_disp_write8;
  }
}

void g68k_map_sync_all(void)
{
  g68k_map_sync_range(0x000000, 0xffffff);
}

void g68k_bus_init(void)
{
  int page;

  for (page = 0; page < 256; page++)
  {
    cpu_memory_map *mm = &m68k.memory_map[page];
    mm->base = PicoMem.ram; /* harmless 64 KB fetch target until mapped */
    mm->read8 = m68k_read8;
    mm->read16 = m68k_read16;
    mm->write8 = g68k_disp_write8;
    mm->write16 = g68k_disp_write16;
  }
}
