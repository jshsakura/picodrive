#ifdef PCM_SPAN_SHADOW_PROOF
void pcm_span_synthetic_proof(void)
{
  extern SH2 *sh2s;
  SH2 *base=&sh2s[1];
  unsigned char *ram=base->p_sdram;
  unsigned char *saved=malloc(0x40000);
  if (!ram || !saved) { printf("PCM_SYNTH_SETUP_FAIL\n"); exit(2); }
  memcpy(saved,ram,0x40000);
  static const unsigned short head[]={0x8469,0xc840,0x8b5a,0x6103,0x5260,0x6320,0x8468,0x230f,0x6013,0x031a,0x5168,0x546a,0xc820,0x8925,0x373c,0x341c,0x6149,0x644d,0x611d,0x164a,0x321c,0x5469,0x3422,0x8947};
  static const unsigned short back[]={0x3738,0x341c,0x6149,0x644d,0x611d,0x164a,0x3218,0x5469,0x3242,0x8921};
  static const unsigned edge[]={0,1,0x7fff,0x8000,0xffff,0x10000,0x7fffffff,0x80000000,0xffffffff};
  memcpy(ram+0x200,head,sizeof head); memcpy(ram+0x268,back,sizeof back);
  unsigned seed=0xc010b1,checks=0,declines=0,directions[2]={0},branches[2]={0};
  for (unsigned j=0;j<12288;j++) {
    SH2 a=*base,b,original;
    unsigned pc=0x0600020eu | (j&1 ? 0x20000000u : 0);
    a.pc=pc+2;a.ppc=pc;a.delay=0;a.test_irq=0;a.icount=j%40;
    for (unsigned k=0;k<16;k++) { seed=seed*1664525u+1013904223u;a.r[k]=seed; }
    a.r[6]=0x06030000u | (j&2 ? 0x20000000u : 0);
    a.t_flag=j&1;
    unsigned voice=a.r[6];
    WL(&a,voice,0x06031000u);
    WB(&a,0x06031000u,j);
    WB(&a,voice+8,j>>1);
    WB(&a,voice+9,j%3==0 ? 64 : (j&4 ? 32 : 0));
    WL(&a,voice+32,edge[(j/4)%9]);
    WL(&a,voice+40,edge[(j/16)%9]);
    WL(&a,voice+36,edge[(j/32)%9]);
    switch ((j/40)%16) {
      case 1:a.delay=pc;break;
      case 2:a.test_irq=1;break;
      case 3:a.p_sdram=0;break;
      case 4:a.r[6]|=2;break;
      case 5:a.r[6]=pc-14;break;
      case 6:a.r[6]=0x0603fff0;break;
      case 7:WL(&a,voice,0x20004000);break;
      case 8:pc|=1;break;
      case 9:pc=0x0603fff0;break;
      case 10:pc=0x02000200;break;
      case 11:a.icount=-1;break;
    }
    unsigned short *code=(unsigned short *)(ram+0x206);
    if ((j/40)%16==12) *code=0x6102;
    else *code=0x6103;
    a.r[0]=control_rd8(base,voice+8);a.r[1]=control_rd8(base,voice+9);a.r[2]=control_rd32(base,voice);a.r[3]=control_rd8(base,0x06031000u);
    a.pc=pc+2;a.ppc=pc;original=a;
    unsigned before=control_rd32(base,voice+40);
    unsigned used=pcm_span_native(&a,pc);
    if (!used) {
      if (memcmp(&a,&original,sizeof a) || before!=control_rd32(base,voice+40)) {
        printf("PCM_SYNTH_DECLINE_FAIL case=%u\n",j);exit(2);
      }
      declines++;continue;
    }
    unsigned actual=control_rd32(base,voice+40);
    WL(base,voice+40,before);
    b=original;b.pc=pc;pcm_span_proof_active=1;
    sh2_execute_interpreter(&b,used);
    pcm_span_proof_active=0;b.icount=original.icount-used;
    if (memcmp(&a,&b,sizeof a) || actual!=control_rd32(base,voice+40)) {
      printf("PCM_SYNTH_FAIL case=%u used=%u\n",j,used);exit(2);
    }
    checks++;directions[!(a.r[0]&32)]++;branches[a.t_flag]++;
  }
  memcpy(ram,saved,0x40000);free(saved);
  pcm_span_proofs=0;
  memset(pcm_span_stats,0,sizeof pcm_span_stats);
  printf("PCM_SYNTH_PASS checks=%u declines=%u forward=%u reverse=%u branch0=%u branch1=%u context=%u\n",
         checks,declines,directions[0],directions[1],branches[0],branches[1],(unsigned)sizeof(SH2));
}
#endif
