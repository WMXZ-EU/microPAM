#include "Arduino.h"
#include "mConfig.h"
#include "menu.h"
#include "mRTC.h"

static int menuGetInt16(int16_t *val)
{ char buffer[40];
  while(!Serial.available()) continue;
  int count = Serial.readBytesUntil('\r',buffer,40);
  buffer[count]=0;
  int tmp;
  sscanf(buffer,"%d",&tmp); *val=(int16_t) tmp;
  return 1;
}

static int menuGetInt32(int32_t *val)
{ char buffer[40];
  while(!Serial.available()) continue;
  int count = Serial.readBytesUntil('\r',buffer,40);
  buffer[count]=0;
  int tmp;
  sscanf(buffer,"%d",&tmp); *val=(int32_t) tmp;
  return 1;
}

static int menuGet3Int(int *val1, int *val2, int *val3)
{ char buffer[40];
  while(!Serial.available()) continue;
  int count = Serial.readBytesUntil('\r',buffer,40);
  buffer[count]=0;
  char c1,c2;
  return sscanf(buffer,"%d%c%d%c%d",val1,&c1,val2,&c2,val3);
}

void syncRTC(datetime_t *t, uint8_t *rtcBuffer)
{
    mgetRTC(rtcBuffer,7);
    t->year=rtcBuffer[6]+2000;
    t->month=rtcBuffer[5]&0x7f;
    t->day=rtcBuffer[4];

    t->hour=rtcBuffer[2];
    t->min=rtcBuffer[1];
    t->sec=rtcBuffer[0] &0xff;

    rtc_set_datetime(t);
}

void menu1(void)
{   // have ':'
    while(!Serial.available()) ;
    char ch=Serial.read();
    if(ch=='x') return;
}

void menu2(void)
{   // have '?'
    datetime_t t;

    while(!Serial.available()) ;
    char ch=Serial.read();
    if(ch=='p') // get parameters
    {
      rtc_get_datetime(&t);
      Serial.printf("\n%4d-%02d-%02d %02d:%02d:%02d %d\n",
                   t.year,t.month,t.day,t.hour,t.min,t.sec,t.dotw);
      Serial.print("fsamp (f) = "); Serial.println(fsamp);
      Serial.print("shift (s) = "); Serial.println(shift);
      Serial.print("proc  (c) = "); Serial.println(proc);
    }
    if(ch=='d') // get date
    {
      rtc_get_datetime(&t);
      Serial.printf("date (d) = %4d-%02d-%02d\n", t.year,t.month,t.day);
    }
    else if(ch=='t') // get time
    {
      rtc_get_datetime(&t);
      Serial.printf("time (t) = %02d:%02d:%02d\n", t.hour,t.min,t.sec);
    }
    else if(ch=='f')
    {
      Serial.print("fsamp (f) = "); Serial.println(fsamp);
    }
    else if(ch=='s')
    {
      Serial.print("shift (s) = "); Serial.println(shift);
    }
    else if(ch=='c')
    {
      Serial.print("proc  (c) = "); Serial.println(proc);
    }
    while(Serial.available()) ch=Serial.read();

}

void menu3(void)
{   // have '!'
    uint8_t rtcBuffer[7] = {0};//{0,27,15,1,3,4,23}; // adapt to better time (secs,min, ...., year)
    datetime_t t;

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
      syncRTC(&t, rtcBuffer);
    } 
    else if(ch=='t') // set time
    { int hour,minutes,seconds;
      menuGet3Int(&hour,&minutes,&seconds);
      //
      mgetRTC(rtcBuffer,7);
      rtcBuffer[2]=hour;
      rtcBuffer[1]=minutes;
      rtcBuffer[0]=seconds;
      msetRTC(rtcBuffer,7);
      syncRTC(&t, rtcBuffer);
    } 
    else if(ch=='f')
    {
      menuGetInt32(&fsamp);
    }
    else if(ch=='s')
    {
      menuGetInt16(&shift); if(shift<0) shift=0;
    }
    else if(ch=='c')
    {
      menuGetInt16(&proc);
    }
}
