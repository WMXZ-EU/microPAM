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
  * File: Menu.cpp
 */
 #include "Arduino.h"
#include "Version.h"
#include "Config.h"
#include "Menu.h"
#include "RTC.h"
#include "Acq.h"
#include "Adc.h"
#include "Hibernate.h"

static uint16_t store[16] = {0};

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

static int menuGetInt16(int16_t *val)
{ char *buffer=menuGetLine();
  int tmp;
  sscanf(buffer,"%d",&tmp); *val=(int16_t) tmp;
  return 1;
}

static int menuGetInt32(int32_t *val)
{ char *buffer=menuGetLine();
  int tmp;
  sscanf(buffer,"%d",&tmp); *val=(int32_t) tmp;
  return 1;
}

static int menuGet3Int(int *val1, int *val2, int *val3)
{ char *buffer=menuGetLine();
  char c1,c2;
  return sscanf(buffer,"%d%c%d%c%d",val1,&c1,val2,&c2,val3);
}

static int menuGetString(char *txt)
{ char *buffer=menuGetLine();
  return sscanf(buffer,"%s",txt);
}

#include "Filing.h"
void resetMTP();
void resetUSB(void);
void adcStatus(void);
void setAGain(int8_t again);
void reboot(void) ;

int16_t monitor=0;
int16_t menu(int16_t status)
{
  // basic menu to start and stop archiving  
  if(Serial.available())
  {
    char ch=Serial.read();
    // some basic options (without ":")
    if(ch=='s') {Serial.print("\n Start"); adc_init(); adcStatus(); status=CLOSED;}
    else if(ch=='e') {Serial.print("\n Stop"); status=MUSTSTOP;}
    else if(ch=='m') {monitor=1-monitor; Serial.print("\n Monitor "); Serial.print(monitor); }
    else if(ch=='r') {resetUSB();}
    else if(ch=='b') {reboot();}
    else if(ch=='x') {powerDown();}
    else if(ch=='t') {hibernate_init(); hibernate_now(rtc_get()+30);}
    // detailed options
    else if(ch==':') status=menu1(status); 
    else if(ch=='?') menu2(); 
    else if(ch=='!') menu3(); 
//    else Serial.clear();
    else {Serial.print(ch); Serial.println("\nAllowed commands: 's','e','m','r'. See also '?p' for parameters"); }
    while(Serial.available()) { ch=Serial.read(); Serial.print(ch);} // clean-up
  }

  return status;
}

int16_t menu1(int16_t status)
{   // have ':'
    while(!Serial.available()) ;
    char ch;
    ch=Serial.read();
    Serial.println(ch);
    if(ch=='w') 
    { saveParameters();
      Serial.println("Save parameters");
    }
    else if(ch=='m') // control monitor (needed for gui)
    {
      menuGetInt16((int16_t *)&monitor);
    }
    else if(ch=='c') // transfer internal rtc to external rtc
    { 
      #if defined(ARDUINO_ARCH_RP2040)
        datetime_t t;
        //XRTCprintTime();
        XRTCsyncTime();
        //XRTCprintTime();
      #endif
    }
    else
    {
      Serial.print(ch); Serial.println(" not recognized");
    }
    while(Serial.available()) ch=Serial.read();
    return status;
}

void printPar()
{
      Serial.print("SerNum(h) = "); Serial.println(SerNum,HEX);
      Serial.print("fsamp (f) = "); Serial.println(fsamp);
      Serial.print("shift (s) = "); Serial.println(shift);
      Serial.print("proc  (c) = "); Serial.println(proc);
      Serial.print("again (g) = "); Serial.println(again);
      Serial.print("t_acq (a) = "); Serial.println(t_acq);
      Serial.print("t_on  (o) = "); Serial.println(t_on);
      Serial.print("t_rep (r) = "); Serial.println(t_rep);
      Serial.print("h_1   (1) = "); Serial.println(h_1);
      Serial.print("h_2   (2) = "); Serial.println(h_2);
      Serial.print("h_3   (3) = "); Serial.println(h_3);
      Serial.print("h_4   (4) = "); Serial.println(h_4);
      Serial.print("d_on  (5) = "); Serial.println(d_on);
      Serial.print("d_rep (6) = "); Serial.println(d_rep);
      Serial.print("d_0   (0) = "); Serial.println(d_0);
      Serial.print("b     (b) = "); Serial.println((char *)b_string); 
      Serial.print("k     (k) = "); Serial.println((char *)k_string); 
      Serial.print("n     (n) = "); Serial.println((char *)n_string); 
}

void menu2(void)
{   // have '?'
    datetime_t t;

    while(!Serial.available()) ;
    char ch;
    ch=Serial.read();
    if(ch=='p') // get parameters
    {
      Serial.println();
      Serial.println(version);

      rtc_get_datetime(&t);
      Serial.printf("Now:\n%4d-%02d-%02d %02d:%02d:%02d %d\n",
                   t.year,t.month,t.day,t.hour,t.min,t.sec,t.dotw);
      XRTCprintTime();
      
      #if defined(__IMXRT1062__)
      Serial.println(rtcGetTimestamp());
      #endif
      printPar();
    }
    else if(ch=='d') // get date
    {
      rtc_get_datetime(&t);
      Serial.printf("date (d) = %4d-%02d-%02d\n", t.year,t.month,t.day);
    }
    else if(ch=='t') // get time
    {
      rtc_get_datetime(&t);
      Serial.printf("time (t) = %02d:%02d:%02d\n", t.hour,t.min,t.sec); 
    }
    else if(ch=='h') { Serial.print("SerNum(h) = "); Serial.println(SerNum,HEX); }
    else if(ch=='a') { Serial.print("t_acq (a) = "); Serial.println(t_acq); }
    else if(ch=='o') { Serial.print("t_on  (o) = "); Serial.println(t_on); }
    else if(ch=='r') { Serial.print("t_rep (r) = "); Serial.println(t_rep); }
    else if(ch=='f') { Serial.print("fsamp (f) = "); Serial.println(fsamp); }
    else if(ch=='s') { Serial.print("shift (s) = "); Serial.println(shift); }
    else if(ch=='c') { Serial.print("proc  (c) = "); Serial.println(proc); }
    else if(ch=='g') { Serial.print("again (g) = "); Serial.println(again); }
    else if(ch=='1') { Serial.print("h_1   (1) = "); Serial.println(h_1); }
    else if(ch=='2') { Serial.print("h_2   (2) = "); Serial.println(h_2); }
    else if(ch=='3') { Serial.print("h_3   (3) = "); Serial.println(h_3); }
    else if(ch=='4') { Serial.print("h_4   (4) = "); Serial.println(h_4); }
    else if(ch=='5') { Serial.print("d_on  (5) = "); Serial.println(d_on); }
    else if(ch=='6') { Serial.print("d_rep (6) = "); Serial.println(d_rep); }
    else if(ch=='0') { Serial.print("d_0   (0) = "); Serial.println(d_0); }
    else if(ch=='w') { uint16_t *params=loadParameters(); Serial.print("params[0]  = "); Serial.println(params[0]); }
    else if(ch=='b') { Serial.print("b     (b) = "); Serial.println((char *)b_string); }
    else if(ch=='k') { Serial.print("k     (k) = "); Serial.println((char *)k_string); }
    else if(ch=='n') { Serial.print("n     (n) = "); Serial.println((char *)n_string); }
    //
    while(Serial.available()) ch=Serial.read();

}

void menu3(void)
{   // have '!'

    while(!Serial.available()) ;
    char ch;
    ch=Serial.read();
    if(ch=='d') // set date
    { int year,month,day;
      menuGet3Int(&year,&month,&day);
      rtcSetDate(year,month,day);
    } 
    else if(ch=='t') // set time
    { int hour,minutes,seconds;
      menuGet3Int(&hour,&minutes,&seconds);
      rtcSetTime(hour,minutes,seconds);
    } 
    else if(ch=='a') { menuGetInt16((int16_t *)&t_acq); }
    else if(ch=='o') { menuGetInt16((int16_t *)&t_on); }
    else if(ch=='r') { menuGetInt16((int16_t *)&t_rep); }
    else if(ch=='f') { menuGetInt32((int32_t *)&fsamp); acqModifyFrequency(fsamp); }
    else if(ch=='s') { menuGetInt16((int16_t *)&shift); if(shift<0) shift=0; }
    else if(ch=='c') { menuGetInt16((int16_t *)&proc); }
    else if(ch=='g') { menuGetInt16((int16_t *)&again); setAGain(again); }
    else if(ch=='w') { menuGetInt16((int16_t *)&store[0]); }
    else if(ch=='1') { menuGetInt16((int16_t *)&h_1); }
    else if(ch=='2') { menuGetInt16((int16_t *)&h_2); }
    else if(ch=='3') { menuGetInt16((int16_t *)&h_3); }
    else if(ch=='4') { menuGetInt16((int16_t *)&h_4); }
    else if(ch=='5') { menuGetInt16((int16_t *)&d_on); }
    else if(ch=='6') { menuGetInt16((int16_t *)&d_rep); }
    else if(ch=='0') { menuGetInt16((int16_t *)&d_0); }
    else if(ch=='b') { menuGetString((char *)&b_string[0]);}
    else if(ch=='k') { menuGetString((char *)&k_string[0]);}
    else if(ch=='n') { menuGetString((char *)&n_string[0]);}
}

/******************** Parameter ******************************/
void storeConfig(uint16_t *store, int ns)
{ 
  #if defined(__IMXRT1062__)
    eeprom_write_block(store, 0, ns*sizeof(store[0]));  
  #endif
}

void loadConfig(uint16_t *store, int ns)
{
  #if defined(__IMXRT1062__)
    eeprom_read_block(store, 0, ns*sizeof(store[0]));  
  #endif
}

void saveParameters(void)
{
  store[1]  = t_acq;
  store[2]  = t_on;
  store[3]  = t_rep;
  store[4]  = proc;
  store[5]  = shift;
  store[6]  = h_1;
  store[7]  = h_2;
  store[8]  = h_3;
  store[9]  = h_4;
  store[10] = d_on;
  store[11] = d_rep;
  store[12] = fsamp/1000;
  store[13] = again;
  store[14] = dgain;
  store[15] = d_0;
  
  storeConfig(store, 16);
}

uint16_t *loadParameters(void)
{
  #if defined(__IMXRT1062__)
  loadConfig(store,16);
  Serial.println(store[0]);
  store[0]=0;
  if(store[0]==1)
  {
    t_acq   = store[1];
    t_on    = store[2];
    t_rep   = store[3];
    proc    = store[4];
    shift   = store[5];
    h_1     = store[6];
    h_2     = store[7];
    h_3     = store[8];
    h_4     = store[9];
    d_on    = store[10];
    d_rep   = store[11];
    fsamp   = store[12]*1000;
    again   = store[13];
    dgain   = store[14];
    d_0     = store[15];
  }
  else
  #endif
  {
    store[0]  = 0;
    store[1]  = t_acq    = T_ACQ;
    store[2]  = t_on     = T_ON;
    store[3]  = t_rep    = T_REP;
    store[4]  = proc     = PROC_MODE;
    store[5]  = shift    = SHIFT;
    store[6]  = h_1      = H_1;
    store[7]  = h_2      = H_2;
    store[8]  = h_3      = H_3;
    store[9]  = h_4      = H_4;
    store[10] = d_on     = D_ON;
    store[11] = d_rep    = D_REP;
    store[12] = (fsamp   = FSAMP)/1000; 
    store[13] = again    = AGAIN;
    store[14] = dgain    = DGAIN;
    store[15] = d_0      = 0; 
  }
  return store;
}

uint16_t *getStore(void) {return store;}

/***********************config from file*********************************************/
/*
 * format (example)
 * a = 20; t_acq
 * o = 60; t_on
 * r = 0; t_rep
 * f = 48000; fsamp
 * s = 12; shift
 * c = 0; proc
 * g = 10; again
 * 1 = 0; h_1
 * 2 = 12; h_2
 * 3 = 12; h_3
 * 4 = 24; h_4
 * 5 = 1; d_on
 * 6 = 0; d_rep
 * 0 = 0; d_0
 * b = WMXZ; owner
 * k = cinqueTerre; cruise
 * n = boa1; sensor
 */

int16_t configGetInt16(char *txt)
{ while(*txt++ !='=') continue;
  int16_t val;
  sscanf(txt,"%hd",&val);
  return val;
}

int32_t configGetInt32(char *txt)
{ while(*txt++ !='=') continue;
  int32_t val;
  sscanf(txt,"%ld",&val);
  return val;
}

void configGetString(char *txt, char *str)
{ while(*txt++ !='=') continue;
  while(*txt == ' ') txt++;
  while (*txt !=';') *str++=*txt++;
  *str='\0';
}

char *skipEOL(char *ptr){ while(*ptr++>=' ') continue; return ptr; }

void decodeConfigfromFile(char *configText)
{
  if(configText)
  {
    char *cptr=configText;
    char *eptr=cptr+strlen(configText);
    Serial.println(configText);
    while(cptr<eptr)
    {
      char ch=*cptr++;
      if(ch<=' ') continue;
      else if(ch==';') cptr=skipEOL(cptr);
      else if(ch=='w') { store[0]=configGetInt16(cptr); cptr=skipEOL(cptr); }
      else if(ch=='a') { store[1]=t_acq=configGetInt16(cptr); cptr=skipEOL(cptr);}
      else if(ch=='o') { store[2]=t_on= configGetInt16(cptr); cptr=skipEOL(cptr);}
      else if(ch=='r') { store[3]=t_rep=configGetInt16(cptr); cptr=skipEOL(cptr);}
      else if(ch=='c') { store[4]=proc= configGetInt16(cptr); cptr=skipEOL(cptr);}
      else if(ch=='s') { store[5]=shift=configGetInt16(cptr); cptr=skipEOL(cptr);if(shift<0) shift=0; }
      else if(ch=='1') { store[6]=h_1=  configGetInt16(cptr); cptr=skipEOL(cptr); }
      else if(ch=='2') { store[7]=h_2=  configGetInt16(cptr); cptr=skipEOL(cptr); }
      else if(ch=='3') { store[8]=h_3=  configGetInt16(cptr); cptr=skipEOL(cptr); }
      else if(ch=='4') { store[9]=h_4=  configGetInt16(cptr); cptr=skipEOL(cptr); }
      else if(ch=='5') { store[10]=d_on= configGetInt16(cptr); cptr=skipEOL(cptr); }
      else if(ch=='6') { store[11]=d_rep=configGetInt16(cptr); cptr=skipEOL(cptr); }
      else if(ch=='f') { fsamp=configGetInt32(cptr); cptr=skipEOL(cptr); acqModifyFrequency(fsamp); store[12]= fsamp/1000; }
      else if(ch=='g') { store[13]=again= configGetInt16(cptr); cptr=skipEOL(cptr); setAGain(again);      }
      else if(ch=='0') { store[15]=d_0=  configGetInt16(cptr); cptr=skipEOL(cptr); }
      else if(ch=='b') { configGetString(cptr,(char *)&b_string[0]); cptr=skipEOL(cptr); }
      else if(ch=='k') { configGetString(cptr,(char *)&k_string[0]); cptr=skipEOL(cptr); }
      else if(ch=='n') { configGetString(cptr,(char *)&n_string[0]); cptr=skipEOL(cptr); }
    }
    printPar();
  }
}
