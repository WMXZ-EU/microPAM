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
 
#include "Arduino.h"
#if defined(__IMXRT1062__)
  #include <CrashReport.h>
#endif

#if defined(TARGET_RP2040)
  #include "pico/stdlib.h"
#endif

#include "mConfig.h"
#include "mQueue.h"
#include "mAcq.h"
#include "mRTC.h"
#include "mCompress.h"
#include "mFiling.h"

/***************************************************************************/
void setup() 
{
  // put your setup code here, to run once:
  #if defined(TARGET_RP2040)
    set_sys_clock_khz(48000, true);
  #endif
  Serial.begin(115200);

  // wait for a minute to allow USB-Serial connection
  while(millis()<60000) if(Serial) break;

  // Teensy has a crash report
  #if defined(__IMXRT1062__)
    if(CrashReport) Serial.print(CrashReport);
  #endif

  if(!rtc_setup()) Serial.println("RTC Lost Power");

  i2s_setup();
  dma_setup();

  uint32_t to= rtc_get();
  //datetime_t t;
  time2date(to, &t);
  Serial.printf("Now: %4d-%02d-%02d %02d:%02d:%02d",
                      t.year,t.month,t.day,t.hour,t.min,t.sec); Serial.println();
  Serial.print("Week Day "); Serial.println(t.dotw);

  filing_init();
  Serial.println("Setup done");
}

void loop() 
{
  // put your main code here, to run repeatedly:
  static uint32_t loopCount=0;
  loopCount++;

  static int16_t status=STOPPED;

  // basic menu to start and stop archiving
  if(Serial.available())
  {
    char ch=Serial.read();
    if(ch=='s') status=CLOSED;
    if(ch=='e') status=MUSTSTOP;
  }

  // obtain some statistics on Queue usage
  static uint16_t mxb=0;
  uint16_t nb = getDataCount();
  if(nb>mxb) mxb=nb;

  // save data (filing will be handled inside saveData)
  status=saveData(status);  

  // once a second provide some information to User
  static uint32_t t0=0;
  uint32_t t1;
  if((t1=millis())>(t0+1000))
  { datetime_t t;
    rtc_get_datetime(&t);

    Serial.printf("\n%4d-%02d-%02d %02d:%02d:%02d %d",
                   t.year,t.month,t.day,t.hour,t.min,t.sec,t.dotw); Serial.print(" : ");

    Serial.print(loopCount); Serial.print(" ");
    Serial.print(procCount); Serial.print(" ");
    Serial.print(procMiss); Serial.print(" ");
    Serial.printf("%3d",mxb); Serial.print("  ");
    Serial.print(status); Serial.print(" ");

    loopCount=0;
    procCount=0;
    procMiss=0;
    mxb=0;

    #if PROC_MODE==0
      for(int ii=0; ii<8;ii++){ Serial.printf("%8X ",logBuffer[ii]);}
    #else
      for(int ii=0; ii<MB;ii++){ Serial.printf("%2d ",proc_stat[ii]);}
      Serial.printf("%2d",max_stat);

      for(int ii=0; ii<MB;ii++){ proc_stat[ii]=0;}
      max_stat=0;
    #endif

    t0=t1;
  }
}

