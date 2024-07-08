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
 /*
  * The connections for the RP2040 are
  * Mic:
  *SD  -> GP0
  *SCK -> GP1
  *WS  -> GP2
  #L/R -> GND (left) or 3.3V (right)
  *
  * SD-Card:
  *Di  -> GP3
  *DO  -> GP4
  *CS  -> GP5
  *SCK -> GP6
  *
  * RTC:
  *SDA -> GP8
  *SCL -> GP9
  *
  * for all:
  *3V3/VCC/Vin -> 3.3V
  *GND -> GND
  *
  * The connections for the Teensy 4.1 are
  * MIC 
  *SD  -> P8
  *SCK -> P21
  *L/R -> P20
  *3V3 -> 3.3V
  *GND -> GND
*/

#ifndef mConfig_H
#define mCONFIG_H
  #include <stdint.h>
  #include "pins_arduino.h"
  
  #define START_MODE 0  // -1 is stopped; 0 is closed (ready to open file)

  #define T_ACQ 20  // size in seconds of each file
  #define T_ON 600  // duration of each aquisition chunk >= T_ACQ (duty cycling not implemented)
  #define T_OFF 0   // duration between acquisition chunks: 0 continuous recording (duty cycling not implemented)

  // for mAcq
  #define FSAMP 48000 // sampling frequency
  #define MBIT 32     // number of bits / sample (do not change)
  #define NCH 1       // number of channels
  #define ICH 0       // selected channel

  #define NBUF_ACQ 128  // number of samples in acq buffer
  #define NBUF_I2S (2*NBUF_ACQ)

  // for mFiling
  #define MIN_SPACE 2000  // number of disk clusters to keep free (can be smaller)
  #define DirPrefix "D"   // prefix for directory
  #define FilePrefix "F"  // prefix fir fileName
  #define NBITS_WAV 16    // bit depth in wav files (32/24/16) 
  #define HourDir 1       // usse day/hour directories (if 0 use only day/)

  // for mQueue
  #define NDBL 12
  #define MAXBUF (16*NDBL)       // Queue length
  
  // pocess mode
  #define PROC_MODE 0     // 0: wav data 1; compress
  #define MB 24           // maximal bits for compression

  // mAcq
  #if PROC_MODE==0
    #define SHIFT 0       // no shift is needed
  #else
    #define SHIFT (8+4)   // shift data to right to improve compression (8+4 meand top 20 bits are good)
  #endif

  #if defined(__IMXRT1062__)
    #define __not_in_flash_func(func_name) func_name
  #endif

  // extern (global) parameters
  extern int32_t fsamp;  // mAcq.cpp
  extern int16_t shift;  // mAcq.cpp
  extern int16_t proc;   // mAcq.cpp
  extern volatile uint32_t t_acq; // mFiling.cpp

  extern class AudioIF acqIF;
#endif