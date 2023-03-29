#ifndef mACQ_H
#define mACQ_H

  #define FSAMP 48000
  
  void i2s_setup(void);
  void dma_setup(void);

  extern uint32_t procCount;
  extern uint32_t procMiss;

#endif