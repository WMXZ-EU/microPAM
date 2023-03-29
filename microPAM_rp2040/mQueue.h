#ifndef mQUEUE_H
#define mQUEUE_H

  #define NBUF_ACQ 128
  #define MAXBUF 128

  uint16_t getDataCount ();
  uint16_t pushData(uint32_t * src);
  uint16_t pullData(uint32_t * dst);

#endif