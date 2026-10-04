/* Bounded PCM sample/position update. Recognise current SDRAM opcodes,
 * including both directions; no ROM name, hash, or fixed guest PC.
 * Stop before the boundary-handling path. MMIO, pending IRQ, delay,
 * exhausted slices and self-modifying voice records use the interpreter. */
#ifdef PCM_SPAN_SHADOW_PROOF
static unsigned pcm_span_proofs;
static int pcm_span_proof_active;
#else
#define pcm_span_proof_active 0
#endif
#ifdef RIG_SH2_COUNT
unsigned pcm_span_stats[4];
#endif
static unsigned __attribute__((noinline, section(".gnw_xip_cold")))
pcm_span_native(SH2 *s, unsigned pc)
{
  if (pcm_span_proof_active || s->delay || s->test_irq || s->icount < 21 ||
      !s->p_sdram || !control_ram(pc) || (pc & 1) || (pc & 0x3ffffu) < 14 ||
      (pc & 0x3ffffu) > 0x3ffffu - 0x7c) return 0;
  unsigned entry=pc; pc-=14;
  unsigned voice = s->r[6], vo = voice & 0x3ffffu, co = pc & 0x3ffffu;
  if (!control_ram(voice) || (voice & 3) || vo > 0x40000u - 44 ||
      (vo < co + 0x7c && vo + 44 > co)) return 0;
  const unsigned short *p = (const unsigned short *)((const unsigned char *)s->p_sdram + co);
  static const unsigned short head[] = {
    0x8469,0xc840,0x8b5a,0x6103,0x5260,0x6320,0x8468,
    0x230f,0x6013,0x031a,0x5168,0x546a,0xc820,0x8925
  };
  static const unsigned short forward[] = {
    0x373c,0x341c,0x6149,0x644d,0x611d,0x164a,0x321c,0x5469,0x3422,0x8947
  };
  static const unsigned short backward[] = {
    0x3738,0x341c,0x6149,0x644d,0x611d,0x164a,0x3218,0x5469,0x3242,0x8921
  };
  if (!hw_eq(p,head,14) || !hw_eq(p+14,forward,10) ||
      !hw_eq(p+0x34,backward,10)) return 0;
  /* The sample and volume loads have already executed. Fold only
   * MULS and the position update, using the live entry registers. */
  unsigned r0=s->r[1],r2=s->r[2];
  unsigned macl=(unsigned)((int)(short)s->r[3] * (short)s->r[0]);
  unsigned sum = control_rd32(s,voice+40) + control_rd32(s,voice+32);
  unsigned r1 = sum >> 16, frac = sum & 65535, reverse = !(r0 & 32);
  unsigned r7 = reverse ? s->r[7]-macl : s->r[7]+macl;
  r2 = reverse ? r2-r1 : r2+r1;
  unsigned r4 = control_rd32(s,voice+36);
  unsigned taken = reverse ? r2 >= r4 : r4 >= r2;
  unsigned cycles = 17 + 2*reverse + 2*taken;
#ifdef PCM_SPAN_SHADOW_PROOF
  SH2 reference;
  unsigned before = control_rd32(s,voice+40), expected = 0;
  int proof = pcm_span_proofs < 4096;
  if (proof) {
    reference = *s; reference.pc = entry;
    pcm_span_proof_active = 1;
    sh2_execute_interpreter(&reference,cycles);
    pcm_span_proof_active = 0;
    reference.icount = s->icount-cycles;
    expected = control_rd32(s,voice+40);
    WL(s,voice+40,before);
  }
#endif
  /* Use the original bus write for cache/poll bookkeeping. */
  WL(s,voice+40,frac);
  s->r[0]=r0; s->r[1]=r1; s->r[2]=r2; s->r[3]=macl;
  s->r[4]=r4; s->r[7]=r7; s->macl=macl; s->t_flag=taken;
  s->ppc=pc+(reverse ? 0x7a : 0x2e);
  s->pc=pc+(taken ? 0xc0 : reverse ? 0x7c : 0x30);
  s->ea=taken ? s->pc : voice+36;
  s->icount-=cycles;
#ifdef PCM_SPAN_SHADOW_PROOF
  if (proof) {
    if (memcmp(s,&reference,sizeof(*s)) || control_rd32(s,voice+40)!=expected) {
      printf("PCM_SPAN_FAIL pc=%08x reverse=%u cycles=%u memory=%08x/%08x\n",
             pc,reverse,cycles,control_rd32(s,voice+40),expected);
      const unsigned *a=(const unsigned *)s,*b=(const unsigned *)&reference;
      for (unsigned i=0;i<sizeof(*s)/4;i++)
        if (a[i]!=b[i]) printf("PCM_DIFF word=%u actual=%08x reference=%08x\n",i,a[i],b[i]);
      exit(2);
    }
    pcm_span_proofs++;
    if (pcm_span_proofs==4096) printf("PCM_SPAN_PROOF checks=4096 context=%u\n",(unsigned)sizeof(*s));
  }
#endif
#ifdef RIG_SH2_COUNT
  pcm_span_stats[0]++; pcm_span_stats[1+reverse]++; pcm_span_stats[3]+=taken;
#endif
  return cycles;
}
