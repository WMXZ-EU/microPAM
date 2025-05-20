#ifndef GLOBAL_H
#define GLOBAL_H

#include "../config.h"
  //#define Version "1.00"
  #define Version "1.01" // changed adalogger.int to first change cpu frequency

  // definitions for acquisition and filing
  #define NCH       1
  #define MBIT      32
  #define NBUF_I2S  24000 // buffer in samples for acquisition and filing

  #define WAIT      0     // seconds to wait for serial (0 do not wait)

  // program states
  enum status_t  {CLOSED, RECORDING, MUST_STOP, STOPPED};

  // in filing.cxx
  extern uint32_t t_acq;   // seconds (each file)
  extern uint32_t t_on;    // minutes (each on period)
  extern uint32_t t_rep;   // minutes (for continuous recording set t_rep < t_acq)

#endif