/* microPAM 
 * Copyright (c) 2023/2024, Walter Zimmer
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
  * File: Config.h
 */

#ifndef CONFIG_H
#define CONFIG_H
  #include <stdint.h>

  // wait for serial (n seconds 0 continue immediately)
  #define WAIT_SERIAL 0

  // use MTP 
  #if defined(ARDUINO_ARCH_RP2040)
    #define USE_MTP 0             // not implemented yet on RP2040
  #elif defined(__IMXRT1062__)
    #define USE_MTP 1
  #endif

  #define USE_SDIO  0             // faster SDIO for RP2040

  // for mAcq
  #if defined(AUDIO_INTERFACE)    // for Teensy 4.1
    #define FSAMP 44100           // for audio interface force 44100
  #else
    #define FSAMP 48000           // general sampling frequency
  #endif

  #define I2S           0   // I2S microphone
  #define TLV320ADC6140 1   // ADC6140 ADC

  #if defined(ARDUINO_ARCH_RP2040) // no ADC6149 on RP2040
    #define ADC_MODEL I2S
  #else
    #define ADC_MODEL I2S
    //#define ADC_MODEL TLV320ADC6140
  #endif

  #define NCHAN_ACQ  1    // number of channels

  #if NCHAN_ACQ == 1
    #if ADC_MODEL== I2S
      #define NCHAN_I2S  2    // number of I2S channels 
        #define ICH      0    // selected channel (set to -1 to disable monochannel extraction)
    #else
      #define NCHAN_I2S  4    // number of I2S channels 
        #define ICH      2    // selected channel (set to -1 to disable monochannel extraction)
    #endif
  #else
    #define ICH     -1    // disable monochannel extraction
  #endif


  #if NCHAN_ACQ > NCHAN_I2S
    #error "NCHAN_ACQ > NCHAN_I2S"
  #endif

  #if ICH >= NCHAN_I2S
    #error "ICH > NCHAN_I2S-1"
  #endif

  #define MBIT      32      // number of bits / sample from ADC
  #define MDIV       1      // MCLK divider (MCLK = 2*MDIV*BCLK)
  #define NSAMP    128      // number of samples in acq buffer
  #define NBUF_ACQ (NCHAN_ACQ*NSAMP)
  #define NBUF_I2S (NCHAN_I2S*NSAMP)

  // for mFiling
  #define MIN_SPACE   2000  // number of disk clusters to keep free  (e.g. 256 MB if cluster=128 kB)
  #define DirPrefix    "T"  // prefix for directory
  #define FilePrefix   "F"  // prefix fir fileName
  #define HourDir        1  // use date/hour/file structure (0 for date/file stucture)

  // for mQueue
  #define HAVE_PSRAM      0
  // adapt to available memory
  #if defined(ARDUINO_ARCH_RP2040)
    #define USE_PSRAM     0
    #define NDBL          12         // number of acuisition buffers fetched from queue for disk storage
    #define MAXBUF        (12*NDBL)  // resulting queue length in multiple of disk buffer
  #elif defined(__IMXRT1062__)
    #if HAVE_PSRAM==0
      #define USE_PSRAM     0
      #define NDBL          12         // number of acuisition buffers fetched from queue for disk storage
      #define MAXBUF        (24*NDBL)  // resulting queue length in multiple of disk buffer
    #else
      #define USE_PSRAM     1
      #define NDBL          48         // number of acuisition buffers fetched from queue for disk storage
      #define MAXBUF        (96*NDBL)  // resulting queue length in multiple of disk buffer
    #endif
  #endif
  
  // pocess mode
  #define PROC_MODE      0  // 0: wav data 1; compress 
  #if PROC_MODE==0
    #define SHIFT         0     // extraction is always from top bit
  #else
    #define SHIFT        (8+4)   // shift data to right to improve compression
  #endif
  
  #define NBITS         32  // 32,24,16 number of bits in wav file (will be used in saveData)
  #if PROC_MODE==1
    #if NBITS<32 
      #ERROR("wrong NBITS for compressed mode")
    #endif
  #endif

  // Acq
  #if ADC_MODEL == I2S
    #define AGAIN          0  // cannot be changed In MEMS
    #define DGAIN          0  // cannot be changed
  #else
    #define AGAIN         10  // analog gain in dB   // 0:42
    #define DGAIN          0  // digital gain in dB  // (-200:54)/2 
  #endif

/******************** Acquisition scheduling *************************/
  #define T_ACQ  20   // duration in sec of each acquisition file
  #define T_ON   60   // duration in sec  of acquisition
  #define T_REP   0   // repetition in sec of acquisition window (0 is continuous)
  #define H_1     0   // start hour of first acquisition block
  #define H_2    12   // stop hour of first acquisition block
  #define H_3    12   // start hour of second acquisition block
  #define H_4    24   // stop hour of second acquisition block
  #define D_ON    1   // duration in days  of acquisition
  #define D_REP   0   // repetition in days of acquisition (0 is continuous)
  #define D_0     0   // start day of acquisition (counted from 4-10-2024 (D_REF = 20000))

  // local reference day (4-october-2024)
  #if defined(ARDUINO_ARCH_RP2040)
    #define D_REF 9043  // based on 2000 (DS3231)
  #elif defined(__IMXRT1062__)
    #define D_REF 20000 // based on 1970
  #endif

  // for external RTC
  #define NONE 0
  #define DS1307 1
  #define DS3231 2
  #define PCF8523 3
  #define RV3028 4

  #if defined(ARDUINO_ARCH_RP2040)
    #define USE_EXT_RTC 1
    #define XRTC DS3231 // maybe NONE, DS1307, DS3231, PCF8523, RV3028
    #define XRTC_INT_PIN A2
  #elif defined(__IMXRT1062__)
    #define USE_EXT_RTC 0
    #define XRTC None // maybe NONE, DS1307, DS3231, PCF8523, RV3028
  #endif

/******************************** HW ****************************/
//ARDUINO_RASPBERRY_PI_PICO
//BOARD_NAME="RASPBERRY_PI_PICO"
//
//ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER
//BOARD_NAME="ADAFRUIT_FEATHER_RP2040_ADALOGGER"
//
//ARDUINO_ADAFRUIT_KB2040_RP2040 
//BOARD_NAME="ADAFRUIT_KB2040_RP2040"

 /***********************************************************************
  * The connections for the RP2040 are
  * I2S-Mic:
  *SD  -> GP0
  *SCK -> GP1
  *WS  -> GP2
  *L/R -> GND (left) or 3.3V (right)
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
  * I2S-MIC 
  *SD  -> P8
  *SCK -> P21
  *WS  -> P20
  *L/R -> GND (left) or 3.3V (right)
  *3V3 -> 3.3V
  *GND -> GND
  * RTC:
  *SDA -> P18
  *SCL -> P19
*/

  #if defined(ARDUINO_ARCH_RP2040)
    #if defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER)
      // for I2S
      #define I2S_DOUT  11
      #define I2S_BCLK  9
      #define I2S_FSYNC (I2S_BCLK+1) // must always be so
      // for SPI
      #define SPI_SCK   18
      #define SPI_MOSI  19
      #define SPI_MISO  20
      #define SPI_CS    23
      // for RTC
      #define RTC_SDA   2
      #define RTC_SCL   3

      #if USE_SDIO==0
        #define mSPI    SPI1
      #endif
      #define LED       13  

    #else //standard RP2040
      // for I2S
      #define I2S_DOUT  0
      #define I2S_BCLK  1
      #define I2S_FSYNC (I2S_BCLK+1) // must always be so
      // for SPI
      #define SPI_MOSI  3
      #define SPI_MISO  4
      #define SPI_CS    5
      #define SPI_SCK   6
      // for RTC
      #define RTC_SDA   8
      #define RTC_SCL   9

      #define mSPI      SPI
      #define LED       25
    #endif

  #elif defined(__IMXRT1062__)
    #if defined(ARDUINO_TEENSY41)
      // for RTC
      #define RTC_SDA 18
      #define RTC_SCL 19
    #else
      // for RTC
      #define RTC_SDA 18
      #define RTC_SCL 19
    #endif
    #define LED       13
  #endif


// extern (global) parameters
  extern volatile int32_t fsamp;  // Acq.cpp 
  extern volatile int16_t shift;  // Acq.cpp 
  extern volatile int16_t  proc;  // Acq.cpp 
  extern volatile int16_t again;  // Acq.cpp
  extern volatile int16_t dgain;  // Acq.cpp

  extern volatile uint16_t t_acq; // Filing.cpp 
  extern volatile uint16_t t_on;  // Filing.cpp 
  extern volatile uint16_t t_rep; // Filing.cpp 

  extern volatile uint16_t h_1; // Filing.cpp 
  extern volatile uint16_t h_2; // Filing.cpp
  extern volatile uint16_t h_3; // Filing.cpp
  extern volatile uint16_t h_4; // Filing.cpp

  extern volatile uint16_t d_on;  // Filing.cpp 
  extern volatile uint16_t d_rep; // Filing.cpp 
  extern volatile  int16_t d_0;   // Filing.cpp 

  extern volatile char b_string[];  // Filing.cpp 
  extern volatile char k_string[];  // Filing.cpp 
  extern volatile char n_string[];  // Filing.cpp 

  extern volatile uint16_t *params0; //menu.cpp

  extern uint32_t SerNum;     // Filing.cpp

  extern class AudioIF acqIF;

  #if defined(__IMXRT1062__)
    #define __not_in_flash_func(func_name) func_name
  #endif

#endif