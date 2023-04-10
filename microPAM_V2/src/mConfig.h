/* microPAM 
 * Copyright (c) 2023, Walter Zimmer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef mConfig_H
#define mCONFIG_H
  #include <stdint.h>
  
  #define START_MODE 0  // -1 is stopped; 0 is closed (ready to open file)
  #define ICS3434 0     // use 0 when Adafruit I2S MEMS

  // for mAcq
  #define FSAMP 44100 // sampling frequency
  #define MBIT 32     // number of bits / sample
  #define NCH 1       // number of channels
  #define ICH 0       // selected channel

  #define NBUF_ACQ 128  // number of samples in acq buffer
  #define NBUF_I2S (2*NBUF_ACQ)

  // for mFiling
  #define MIN_SPACE 2000  // number of disk clusters to keep free
  #define DirPrefix "D"   // prefix for directory
  #define FilePrefix "F"  // prefix fir fileName

  // for mQueue
  #define MAXBUF 128       // Queue length

  // pocess mode
  #define PROC_MODE 1        // 0: raw data 1; compress
  #define MB 24

  // mAcq
  #define SHIFT (8+4)
  #if ICS3434==1
    #define BIAS (0)
  #else
    #define BIAS (-27000<<SHIFT)
  #endif

  #if defined(__IMXRT1062__)
    #define __not_in_flash_func(func_name) func_name
  #endif

  // extern (global) parameters
  extern int32_t fsamp;  // mAcq.cpp
  extern int16_t shift;  // mAcq.cpp
  extern int16_t proc;   // mAcq.cpp

  extern class AudioIF acqIF;
#endif