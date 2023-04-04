#include "Arduino.h"
#include "menue.h"
#include "mRTC.h"

static int menuGetInt(int *val)
{ char buffer[40];
  while(!Serial.available()) continue;
  int count = Serial.readBytesUntil('\r',buffer,40);
  buffer[count]=0;
  return sscanf(buffer,"%d",val);
}

static int menuGet3Int(int *val1, int *val2, int *val3)
{ char buffer[40];
  while(!Serial.available()) continue;
  int count = Serial.readBytesUntil('\r',buffer,40);
  buffer[count]=0;
  char c1,c2;
  return sscanf(buffer,"%d%c%d%c%d",val1,&c1,val2,&c2,val3);
}

static uint8_t rtcBuffer[7] = {0,27,15,1,3,4,23}; // adapt to better time (secs,min, ...., year)
static datetime_t t;

void syncRTC(void)
{
    mgetRTC(rtcBuffer,7);
    t.year=rtcBuffer[6]+2000;
    t.month=rtcBuffer[5]&0x7f;
    t.day=rtcBuffer[4];

    t.hour=rtcBuffer[2];
    t.min=rtcBuffer[1];
    t.sec=rtcBuffer[0] &0xff;

    rtc_set_datetime(&t);
}
void menue(void)
{
    while(!Serial.available()) ;
    char ch=Serial.read();
    if(ch=='d') // set date
    { int year,month,day;
      menuGet3Int(&year,&month,&day);

      mgetRTC(rtcBuffer,7);
      rtcBuffer[6]=year-2000;
      rtcBuffer[5]=month;
      rtcBuffer[4]=day;
      msetRTC(rtcBuffer,7);
      syncRTC();
   } 
    else if(ch=='t') // set time
    { int hour,minutes,seconds;
      menuGet3Int(&hour,&minutes,&seconds);
      //
      mgetRTC(rtcBuffer,7);
      rtcBuffer[2]=hour;
      rtcBuffer[3]=minutes;
      rtcBuffer[0]=seconds;
      msetRTC(rtcBuffer,7);
      syncRTC();
    } 
}