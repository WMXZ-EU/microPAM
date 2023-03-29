
#include "Arduino.h"
#if defined(__IMXRT1062__)
  #include <CrashReport.h>
#endif

#if defined(TARGET_RP2040)
  #include "pico/stdlib.h"
#endif
#include "mAcq.h"
#include "mQueue.h"
#include "mRTC.h"
#include "mFiling.h"

/***************************************************************************/
void setup() 
{
  // put your setup code here, to run once:
  #if defined(TARGET_RP2040)
    set_sys_clock_khz(48000, true);
  #endif
  Serial.begin(115200);

  while(millis()<60000) if(Serial) break;

  #if defined(__IMXRT1062__)
    if(CrashReport) Serial.print(CrashReport);
  #endif

  if(!rtc_setup()) Serial.println("RTC Lost Power");
  i2s_setup();
  dma_setup();

  uint32_t to= rtc_get();
  datetime_t t;
  time2date(to, &t);
  Serial.printf("Now: %4d-%02d-%02d %02d:%02d:%02d",t.year,t.month,t.day,t.hour,t.min,t.sec); Serial.println();
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

  static uint16_t mxb=0;
  uint16_t nb = getDataCount();
  if(nb>mxb) mxb=nb;

  // check end of file
  status=saveData(status);  


  static uint32_t t0=0;
  uint32_t t1;
  if((t1=millis())>(t0+1000))
  { datetime_t t;
    rtc_get_datetime(&t);

    Serial.printf("\n%4d-%02d-%02d %02d:%02d:%02d %d",t.year,t.month,t.day,t.hour,t.min,t.sec,t.dotw); Serial.print(" : ");

    Serial.print(loopCount); Serial.print(" ");
    Serial.print(procCount); Serial.print(" ");
    Serial.print(procMiss); Serial.print(" ");
    Serial.printf("%3d",mxb); Serial.print("  ");
    Serial.print(status); Serial.print(" ");
    
    for(int ii=0; ii<8;ii++){ Serial.printf("%8X ",logBuffer[ii]);}
    
    loopCount=0;
    procCount=0;
    procMiss=0;
    mxb=0;

    t0=t1;
  }
}

