#include "Arduino.h"
#include <EEPROM.h>

#include "global.h"
#include "Menu.h"
#include "RTC.h"
#include "rp2040.h"

static uint16_t eeprom=0;
void eepromInit() { EEPROM.begin(256);}
void eepromUpdate() { EEPROM.commit();}

void eepromWrite(byte a, uint32_t v)
{
    EEPROM.write(a,v&0xff);
    EEPROM.write(a+1,(v>>8) &0xff);
}
uint16_t eepromRead(byte a)
{ uint16_t v;
    v = EEPROM.read(a);
    v |= EEPROM.read(a+1)<<8;
    return v;
}

void parameterInit(void)
{ // load parameters from EEPROM
  eepromInit();
  eeprom=EEPROM.read(0);
  if(eeprom==1)
  { 
    t_acq = eepromRead(1);
    t_on  = eepromRead(3);
    t_rep = eepromRead(5);
  }
}

void parameterPrint(void)
{ Serial.print("\nversion    "); Serial.println(Version);
  getUID();
  Serial.print("UID        "); Serial.println(uid_str);
  Serial.print("eeprom (w) "); Serial.print(eeprom); Serial.println();
  Serial.print("t_acq  (a) "); Serial.print(t_acq); Serial.println(" sec");
  Serial.print("t_on   (o) "); Serial.print(t_on);  Serial.println(" min");
  Serial.print("t_rep  (r) "); Serial.print(t_rep); Serial.println(" min");

  for(int ii=0;ii<7;ii++) {Serial.print(EEPROM.read(ii)); Serial.print(' ');} Serial.println();
}

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
static int menuGetInt16(uint16_t *val)
{ char *buffer=menuGetLine();
  int tmp;
  sscanf(buffer,"%d",&tmp); *val=(uint16_t) tmp;
  return 1;
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
      else if(ch=='p')  // print parameters
      {  parameterPrint();
      }
      else if(ch=='!')  // modify parameters
      {
        while(!Serial.available()) delay(10);
        ch=Serial.read();
        if(ch=='a') 
        { menuGetInt16((uint16_t*)&t_acq); 
          eepromWrite(1,t_acq);
          }
        if(ch=='o') 
        { menuGetInt16((uint16_t*)&t_on);
          eepromWrite(3,t_on);
        }
        if(ch=='r') 
        { menuGetInt16((uint16_t*)&t_rep);
          eepromWrite(5,t_rep);
        }
        if(ch=='w') 
        { menuGetInt16((uint16_t*)&eeprom);
          EEPROM.write(0,eeprom&0xff);
          eepromUpdate();
        }
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
