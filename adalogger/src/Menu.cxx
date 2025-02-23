#include "Arduino.h"
#include "global.h"
#include "Menu.h"
#include "RTC.h"

// User Interface
static char * menuGetLine(void)
{
  static char buffer[40];
  while(!Serial.available()) continue;
  Serial.setTimeout(5000);
  int count;
  count = Serial.readBytesUntil('\r',buffer,40);
  buffer[count]=0;
  Serial.println(buffer);
  return buffer;
}
static uint16_t menuGetTime(datetime_t *t)
{
  char *buffer=menuGetLine();
  int v1,v2,v3,v4,v5,v6;
  char c1,c2,c3,c4,c5;

  if(strlen(buffer)<19) return 0;
  //
  sscanf(buffer,"%d%c%d%c%d%c%d%c%d%c%d",
                &v1,&c1, &v2,&c2, &v3,&c3, &v4,&c4, &v5,&c5, &v6);
  t->year =v1;
  t->month=v2;
  t->day  =v3;
  t->hour =v4;
  t->min  =v5;
  t->sec  =v6;
  return 1;
}
status_t menu(status_t status)
{
  if(Serial)
  {
    if(Serial.available())
    {
      char ch;
      ch=Serial.read();
      if(ch=='s') // start acquisition
      { Serial.println("start");
        status=CLOSED;
      }
      else if(ch=='e')  // stop aquisition
      { Serial.print("stop ");
        status=MUST_STOP;
      }
      else if(ch=='c')  // check and correct RTC time
      {
        if(status==STOPPED) // only if we are stopped
        {
          while(Serial.available()) {volatile char c = Serial.read(); (void) c;}
          // print time stamp
          datetime_t t;
          XRTCgetDatetime(&t);    
          Serial.println("Actual time on DS3231 is");
          printDatetime("DS3231",&t);
          Serial.println("If correct press return, otherwise enter correct date and time");

          // correct RTC time is required
          if(menuGetTime(&t))
          {
            XRTCsetDatetime(&t);  
            delay(10);
            // write also to local rtc
            rtcSetDatetime(&t);  
            Serial.println("Corrected time is");
            printDatetime("rtc",&t);
          }
          else
          {
            Serial.println("Time has not been corrected");
          }
        }
      }
    }
  }
  return status;
}
