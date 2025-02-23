#ifndef GLOBAL_H
#define GLOBAL_H

#include "../config.h"

  // definitions for acquisition and filing
  #define NCH       1
  #define MBIT      32
  #define NBUF_I2S  24000 // buffer in samples for acquisition and filing

  #define WAIT      0     // seconds to wait for serial (0 do not wait)

  // program states
  enum status_t  {CLOSED, RECORDING, MUST_STOP, STOPPED};
#endif