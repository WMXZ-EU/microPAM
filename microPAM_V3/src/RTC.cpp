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
  * File: RTC.cpp
 */

#include "Arduino.h"
#include "Wire.h"

#include "Config.h"
#include "RTC.h"

#if USE_EXT_RTC > 0
  static int16_t i2c_setup(uint8_t sda, uint8_t scl)
  {
    Wire.setSDA(sda);
    Wire.setSCL(scl);
    Wire.begin();
    delay(100);
    return 1;
  }

  static int16_t i2c_exists(uint8_t address)
  {
    Wire.beginTransmission(address);
    return !Wire.endTransmission();
  }

  static void i2c_write_register(uint8_t address, uint8_t reg, uint8_t val) 
  {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  static uint8_t i2c_read_register(uint8_t address, uint8_t reg) 
  {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();

  //  uint8_t buffer[1];
    Wire.requestFrom(address, (uint8_t)1);
    return Wire.read();
  }

  static void i2c_write_data(uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t nbuf)
  {
    Wire.beginTransmission(address);
    Wire.write(reg);
    for(int ii=0; ii<nbuf; ii++) Wire.write(buffer[ii]);
    Wire.endTransmission();  
  }

  static void i2c_read_data(uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t nbuf)
  {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(address, (uint8_t)nbuf);
    for(int ii=0;ii<nbuf;ii++) buffer[ii]=Wire.read();
  }

  /**************************************************************************************/
  static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }
  static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
#endif

#if XRTC==DS1307
  #define DS1307_ADDRESS   0x68   ///< I2C address for DS1307
  #define DS1307_CONTROL   0x07   ///< Control register
  #define DS1307_NVRAM   0x08   ///< Start of RAM registers - 56 bytes, 0x08 to 0x3f

  static uint8_t address = DS1307_ADDRESS;
  static uint8_t status_reg = 0;
  static uint8_t control_reg = 0;
  static uint8_t time_reg = 0;

  int16_t initXRTC(uint8_t sda, uint8_t scl)
  {
    i2c_setup(sda,scl);
    return i2c_exists(address);
  }

  int16_t lostPowerXRTC(void) { return 0;}  // not implemented

  uint8_t *getXRTC(uint8_t *buffer,uint16_t nbuf)
  { // read time from RTC
    i2c_read_data(address,time_reg,buffer,nbuf);
    for(int ii=0;ii<nbuf;ii++) buffer[ii]=bcd2bin(buffer[ii]);
    
    return buffer;
  }

  uint8_t *setXRTC(uint8_t *buffer, uint16_t nbuf)
  { // write time to RTC
    for(int ii=0;ii<nbuf;ii++) buffer[ii]=bin2bcd(buffer[ii]);
    i2c_write_data(address,time_reg,buffer,nbuf);

    return buffer;
  }

  void XRTCclearAlarm(void)
  {
    // clear alarm1
    Serial.println("not implemented");
  }

  void XRTCsetAlarm(uint32_t secs)
  { 
    // set alarm1
    Serial.println("not implemented");
  }
  float XRTCgetTemperature(void) {return 0.0;}
#elif XRTC==DS3231

  #define DS3231_ADDRESS   0x68   ///< I2C address for DS3231
  #define DS3231_TIME      0x00   ///< Time register
  #define DS3231_ALARM1    0x07   ///< Alarm 1 register
  #define DS3231_ALARM2    0x0B   ///< Alarm 2 register
  #define DS3231_CONTROL   0x0E   ///< Control register
  #define DS3231_STATUSREG 0x0F   ///< Status register
  #define DS3231_TEMPERATUREREG 0x11 

  static uint8_t address = DS3231_ADDRESS;
  static uint8_t status_reg = DS3231_STATUSREG;
  static uint8_t time_reg = DS3231_TIME;
  static uint8_t control_reg = DS3231_CONTROL;
  static uint8_t alarm1_reg = DS3231_ALARM1;
  static uint8_t alarm2_reg = DS3231_ALARM2;

  int16_t initXRTC(uint8_t sda, uint8_t scl)
  {
    i2c_setup(sda,scl);
    return i2c_exists(address);
  }

  int16_t lostPowerXRTC(void) 
  {
    return i2c_read_register(address,status_reg) >> 7;
  }

  uint8_t *getXRTC(uint8_t *buffer,uint16_t nbuf)
  { // read time from RTC
    i2c_read_data(address,time_reg,buffer,nbuf);
    for(int ii=0;ii<nbuf;ii++) buffer[ii]=bcd2bin(buffer[ii]);
    
    return buffer;
  }

  uint8_t *setXRTC(uint8_t *buffer, uint16_t nbuf)
  { // write time to RTC
    for(int ii=0;ii<nbuf;ii++) buffer[ii]=bin2bcd(buffer[ii]);
    i2c_write_data(address,time_reg,buffer,nbuf);
    
    uint8_t statreg = i2c_read_register(address,status_reg);
    Serial.println(statreg,BIN);
    statreg &= ~0x80; // flip OSF bit
    i2c_write_register(address,status_reg, statreg);

    return buffer;
  }

  void XRTCclearAlarm(void)
  {
    // clear alarm1
    uint8_t regval=i2c_read_register(address,status_reg);
    regval = regval & 0b11111110;
    i2c_write_register(address,status_reg,regval);
  }

  void XRTCsetAlarm(uint32_t secs)
  { //
    uint8_t buffer[7]={0};
    uint16_t nbuf;

    //uint32_t secso=rtc_get();

  /*
    Serial.print(i2c_read_register(address,alarm1_reg),BIN); Serial.print(' ');
    Serial.print(i2c_read_register(address,alarm1_reg+1),BIN); Serial.print(' ');
    Serial.print(i2c_read_register(address,alarm1_reg+2),BIN); Serial.print(' ');
    Serial.print(i2c_read_register(address,alarm1_reg+3),BIN); Serial.println(' ');
  */
    //
    // turn off alarm
    uint8_t regval=i2c_read_register(address,control_reg);
    regval = regval & 0b11111110;
    i2c_write_register(address,control_reg, regval);

    //
    // write alarm time to rtc
    secs=secs % (24*3600); // only seconds in day

    Serial.println(secs);
    //Serial.print(secso % (24*3600)); Serial.print(' -> ');
    //Serial.println(secs % (24*3600));

    int hh,mm,ss;
    buffer[4] = 3+(secs /(24*3600)) % 7;    // dow 1=sunday
    buffer[3] = buffer[4] | (1<<6);         // use dow
    buffer[3] = 1<<7;                       // use only hour,min,sec
    buffer[2] = secs/3600;
    buffer[1] = (secs % 3600)/60;
    buffer[0] = (secs % 60);
    
    Serial.print("Alarm : ");
    Serial.print(buffer[4]); Serial.print(' ');
    Serial.print(buffer[3],BIN); Serial.print(' ');
    Serial.print(buffer[2]); Serial.print(' ');
    Serial.print(buffer[1]); Serial.print(' ');
    Serial.print(buffer[0]); Serial.println(' ');
    Serial.flush();

    for(int ii=0;ii<4;ii++) buffer[ii]=bin2bcd(buffer[ii]);
    i2c_write_data(address,alarm1_reg,buffer,4);

  /*
    Serial.print(i2c_read_register(address,alarm1_reg),BIN); Serial.print(' ');
    Serial.print(i2c_read_register(address,alarm1_reg+1),BIN); Serial.print(' ');
    Serial.print(i2c_read_register(address,alarm1_reg+2),BIN); Serial.print(' ');
    Serial.print(i2c_read_register(address,alarm1_reg+3),BIN); Serial.println(' ');
  */
    // clear alarm1
    regval=i2c_read_register(address,status_reg);
    regval = regval & 0b11111110;
    i2c_write_register(address,status_reg,regval);

    // turn on alarm
    regval=i2c_read_register(address,control_reg);
    regval = regval | 0b00000101;
    i2c_write_register(address,control_reg, regval);
  }

  float XRTCgetTemperature(void) {
    uint8_t buffer[2] = {DS3231_TEMPERATUREREG, 0};
    i2c_read_data(address,DS3231_TEMPERATUREREG,buffer,2);
    //i2c_dev->write_then_read(buffer, 1, buffer, 2);
    return (float)buffer[0] + (buffer[1] >> 6) * 0.25f;
  }

#elif XRTC==PCF8523
  #define PCF8523_ADDRESS   0x68   ///< I2C address for PCF8523
  #define PCF8523_CONTROL1  0x00  ///< Control and status register 1
  #define PCF8523_CONTROL2  0x01  ///< Control and status register 2
  #define PCF8523_CONTROL3  0x02  ///< Control and status register 3
  #define PCF8523_STATUSREG 0x0F3  ///< Status register
  #define PCF8523_OFFSET    0x0E  ///< Offset register
  #define PCF8523_CLKOUTCONTROL 0x0F  ///< Timer and CLKOUT control register
  #define PCF8523_TIMER_B_FRCTL 0x12   ///< Timer B source clock frequency control
  #define PCF8523_TIMER_B_VALUE 0x13   ///< Timer B value (number clock periods)

  static uint8_t address = PCF8523_ADDRESS;
  static uint8_t status_reg = PCF8523_STATUSREG;
  static uint8_t time_reg = PCF8523_OFFSET;

  int16_t initXRTC(uint8_t sda, uint8_t scl)
  {
    i2c_setup(sda,scl);
    return i2c_exists(address);
  }

  int16_t lostPowerXRTC(void) 
  {
    return i2c_read_register(address,status_reg) >> 7;
  }

  uint8_t *getXRTC(uint8_t *buffer,uint16_t nbuf)
  { // read time from RTC
    uint8_t tmp[7]
    i2c_read_data(address,time_reg,tmp,7);
    for(int ii=0;ii<nbuf;ii++) buffer[ii]=bcd2bin(tmp[6-ii]);
    
    return buffer;
  }

  uint8_t *setXRTC(uint8_t *buffer, uint16_t nbuf)
  { // write time to RTC
    uint8_t tmp[7]
    for(int ii=0;ii<nbuf;ii++) tmp[6-ii]=bin2bcd(buffer[ii]);
    i2c_write_data(address,time_reg,buffer,nbuf);

    // set to battery switchover mode
    i2c_write_register(address,PCF8523_CONTROL3, 0x00);

    // start ext RTC in case it is stopped
    uint8_r reg= i2c_read_register(address,PCF8523_CONTROL1);
    if(reg & (1<<5))
      i2c_write_register(address,PCF8523_CONTROL1, reg & ~(1<<5));

    return buffer;
  }

  void XRTCclearAlarm(void)
  {
    // clear alarm1
    Serial.println("not implemented");
  }

  void XRTCsetAlarm(uint32_t secs)
  { 
    // set alarm1
    Serial.println("not implemented");
  }
  float XRTCgetTemperature(void) {return 0.0;}

#elseif XRTC== RV3028
  #include "RV-3028-C7.h"
  RV3028 rtc;
  // needed functions
  // rtc.begin()
  // rtc.setUNIX(...)
  // rtc.setTime(...)
  // rtc.getUNIX()
  // rtc.updateTime()
  // rtc.stringTimeStamp()

  int16_t initXRTC(uint8_t sda, uint8_t scl)
  { return 0;
  }
  int16_t lostPowerXRTC(void) 
  { return 0;
  }
  uint8_t *getXRTC(uint8_t *buffer,uint16_t nbuf)
  {   return 0;
  }
  uint8_t *setXRTC(uint8_t *buffer, uint16_t nbuf)
  {  return 0;
  }

  void XRTCclearAlarm(void)
  {
    // clear alarm1
    Serial.println("not implemented");
  }

  void XRTCsetAlarm(uint32_t secs)
  { 
    // set alarm1
    Serial.println("not implemented");
  }

  float XRTCgetTemperature(void) {return 0.0;}

#else
  // unknown or no xrtc
  int16_t initXRTC(uint8_t sda, uint8_t scl){ return 0;}
  int16_t lostPowerXRTC(void) { return 0;}
  uint8_t *getXRTC(uint8_t *buffer,uint16_t nbuf){   return 0;}
  uint8_t *setXRTC(uint8_t *buffer, uint16_t nbuf){  return 0;}

  void XRTCclearAlarm(void) {}
  void XRTCsetAlarm(uint32_t secs) {}
  float XRTCgetTemperature(void) {return 0.0;}

#endif

/******************************************************************/

/******************************************************************/
#if defined(ARDUINO_ARCH_RP2040)

  const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) 
  {
    if (y >= 2000U) y -= 2000U;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i) days += daysInMonth[i - 1];
    if (m > 2 && y % 4 == 0) ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
  }

  uint32_t time2seconds(uint16_t days, uint8_t h, uint8_t m, uint8_t s) 
  {
    return ((days * 24UL + h) * 60 + m) * 60 + s;
  }
/*
  static uint8_t conv2d(const char *p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')  v = *p - '0';
    return 10 * v + *++p - '0';
  }

  void adjustXRTC(char *date, char *time)
  { // Set ext RTC
    //Mar 20 2023 17:34:05
    uint8_t rtcBuffer[7];
    
    char buff[11];
    int m;
    memcpy(buff, date, 11);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (buff[0]) {
    case 'J':
      m = (buff[1] == 'a') ? 1 : ((buff[2] == 'n') ? 6 : 7);
      break;
    case 'F':
      m = 2;
      break;
    case 'A':
      m = buff[2] == 'r' ? 4 : 8;
      break;
    case 'M':
      m = buff[2] == 'r' ? 3 : 5;
      break;
    case 'S':
      m = 9;
      break;
    case 'O':
      m = 10;
      break;
    case 'N':
      m = 11;
      break;
    case 'D':
      m = 12;
      break;
    }
    rtcBuffer[6] = conv2d(buff + 9);
    rtcBuffer[5] = m;
    rtcBuffer[4] = conv2d(buff + 4);
    //
    memcpy(buff, time, 8);
    rtcBuffer[2] = conv2d(buff);
    rtcBuffer[1] = conv2d(buff + 3);
    rtcBuffer[0] = conv2d(buff + 6);

    rtcBuffer[3] = 0;

    setXRTC(rtcBuffer, 7);
  }
*/
  static volatile uint16_t haveExtRTC=0;
  int16_t rtc_setup(uint8_t sda, uint8_t scl)
  {
    rtc_init(); // hardware (MCU) rtc
    haveExtRTC=initXRTC(sda, scl); // external rtc clock (XRTC==NONE return 0)
    Serial.print("extClock "); Serial.println(haveExtRTC);
    delay(10);

    // get time from ext RTC
    if(haveExtRTC)
    {
      rtcSyncfromXRTC();
    }
    return 1;
  }

  void rtc_print_datetime(void)
  { // print local datetime
    datetime_t t;
    rtc_get_datetime(&t);
    Serial.printf("RTC: %4d-%02d-%02d %02d:%02d:%02d",
                      t.year,t.month,t.day,t.hour,t.min,t.sec); Serial.println();    
  }

  uint32_t rtc_get(void)
  { // get seconds since epoch time
    datetime_t t;
    // get local time
    rtc_get_datetime(&t);    
    uint16_t day;
    day=date2days(t.year, t.month, t.day);
    return time2seconds(day, t.hour, t.min, t.sec);
  }

  void rtcSyncfromXRTC(void)
  {// sync local RTC from ext RTC
    uint8_t rtcBuffer[7] ={0};
    datetime_t t;
    if(haveExtRTC)
    {
      getXRTC(rtcBuffer,7);
      t.year =rtcBuffer[6]+2000;
      t.month=rtcBuffer[5]&0x7f;
      t.day  =rtcBuffer[4];

      t.hour=rtcBuffer[2];
      t.min =rtcBuffer[1];
      t.sec =rtcBuffer[0] &0xff;

      rtc_set_datetime(&t);
      delay(10);
    }
  }

  void rtcSetDate(int year,int month,int day)
  { //gets dateTime from XRTC updates date
    uint8_t rtcBuffer[7] = {0};//{0,27,15,1,3,4,23}; // adapt to better time (secs,min, ...., year)
    datetime_t t;
    if(haveExtRTC)
    {
      getXRTC(rtcBuffer,7);   // get time from ExtRTC
      rtcBuffer[6]=year-2000;
      rtcBuffer[5]=month;
      rtcBuffer[4]=day;
      setXRTC(rtcBuffer,7);   // set time to ExtRTC
      rtcSyncfromXRTC();              // syn MCU-RTC from ExtRTC
    }
    else
    {
      rtc_get_datetime(&t);   
      t.year=year-2000;
      t.month=month;
      t.day=day;   
      rtc_set_datetime(&t);      
    }
  }

  void rtcSetTime(int hour,int minutes,int seconds)
  { //gets dateTime from XRTC updates time
    uint8_t rtcBuffer[7] = {0};//{0,27,15,1,3,4,23}; // adapt to better time (secs,min, ...., year)
    datetime_t t;
    if(haveExtRTC)
    {
      getXRTC(rtcBuffer,7);  // read from ExtRTC
      rtcBuffer[2]=hour;
      rtcBuffer[1]=minutes;
      rtcBuffer[0]=seconds;
      setXRTC(rtcBuffer,7);  // write to ExtRTC
      rtcSyncfromXRTC(); // sync MCU-RTC from ExtRTC
    }
    else
    {
      rtc_get_datetime(&t);   
      t.hour=hour;
      t.min=minutes;
      t.sec=seconds;   
      rtc_set_datetime(&t);      
    }
  }

  void XRTCsyncTime(void)
  { // transfer time from local to ext RTC
    uint8_t rtcBuffer[7] = {0};//{0,27,15,1,3,4,23}; // adapt to better time (secs,min, ...., year)
    datetime_t t;
    if(haveExtRTC)
    {     
      rtc_get_datetime(&t);
      rtcBuffer[6]=t.year-2000;
      rtcBuffer[5]=t.month;
      rtcBuffer[4]=t.day;
      rtcBuffer[2]=t.hour;
      rtcBuffer[1]=t.min;
      rtcBuffer[0]=t.sec;
      for(int ii=0;ii<7;ii++) {Serial.print(rtcBuffer[ii]); Serial.print(' ');} Serial.println();
      setXRTC(rtcBuffer,7);  // write to ExtRTC
    }
  }

  void XRTCprintTime(void)
  {
    uint8_t rtcBuffer[7] = {0};//{0,27,15,1,3,4,23}; // adapt to better time (secs,min, ...., year)
    if(haveExtRTC)
    {     
      getXRTC(rtcBuffer,7);  // write to ExtRTC
      Serial.print(rtcBuffer[6]+2000); Serial.print(' ');
      Serial.print(rtcBuffer[5]); Serial.print(' ');
      Serial.print(rtcBuffer[4]); Serial.print(' ');
      Serial.print(rtcBuffer[2]); Serial.print(' ');
      Serial.print(rtcBuffer[1]); Serial.print(' ');
      Serial.print(rtcBuffer[0]); Serial.println(' ');
    }
  }


#elif defined(__IMXRT1062__)
  // use only MCU-RTC
  #define YEAR0 1970
  #define LEAP_YEAR(Y)  ( ((YEAR0+(Y))>0) && !((YEAR0+(Y))%4) && ( ((YEAR0+(Y))%100) || !((YEAR0+(Y))%400) ) )

  //convenience macros to convert to and from tm years 
  #define  tmYearToCalendar(Y) ((Y) + YEAR0)  // full four digit year 
  #define  CalendarYrToTm(Y)   ((Y) - YEAR0)

  #define SECS_PER_MIN  ((uint32_t)(60UL))
  #define SECS_PER_HOUR ((uint32_t)(3600UL))
  #define SECS_PER_DAY  ((uint32_t)(SECS_PER_HOUR * 24UL))

  static  const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31}; // API starts months from 1, this array starts from 0
 
  void time2date(uint32_t time, datetime_t *tm)
  {
  // break the given time_t into time components
  // this is a more compact version of the C library localtime function
  // note that year is offset from YEAR0 !!!

    uint8_t year;
    uint8_t month, monthLength;
    unsigned long days;

    tm->sec  = time % 60; time /= 60; // now time is minutes
    tm->min  = time % 60; time /= 60; // now time is hours
    tm->hour = time % 24; time /= 24; // now time is days

    tm->dotw = ((time + 4) % 7) ;  // Sunday is day 0 // 1-1-1970 was Thursday
    
    year = 0;  
    days = 0;
    while((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) {
      year++;
    }
    tm->year = year+YEAR0;
    
    days -= LEAP_YEAR(year) ? 366 : 365;
    time -= days; // now time is days in this year, starting at 0
    
    days=0;
    month=0;
    monthLength=0;
    for (month=0; month<12; month++) {
      if (month==1) { // february
        if (LEAP_YEAR(year)) {
          monthLength=29;
        } else {
          monthLength=28;
        }
      } else {
        monthLength = monthDays[month];
      }
      
      if (time >= monthLength) {
        time -= monthLength;
      } else {
          break;
      }
    }
    tm->month = month + 1;  // jan is month 1  
    tm->day = time + 1;     // day of month
  }

  uint32_t date2time(datetime_t *tm)
  {
    int ii;
    uint32_t seconds;

    uint8_t year;
    year=tm->year-YEAR0; // year after 1-jan-1970 (YEAR0)
    #if 0
    seconds= year*(SECS_PER_DAY * 365);
    for (ii = 0; ii < year; ii++) {
      if (LEAP_YEAR(ii)) {
        seconds += SECS_PER_DAY;   // add extra days for leap years
      }
    }
    #endif
    uint32_t days= year*365;
    for (ii = 0; ii < year; ii++) if (LEAP_YEAR(ii)) days++;  // add extra days for leap years
    seconds = days*SECS_PER_DAY;

    // add days for this year, months start from 1
    for (ii = 1; ii < tm->month; ii++) {
      if ( (ii == 2) && LEAP_YEAR(year)) { 
        seconds += SECS_PER_DAY * 29;
      } else {
        seconds += SECS_PER_DAY * monthDays[ii-1];  //monthDay array starts from 0
      }
    }
    seconds+= (tm->day-1) * SECS_PER_DAY;
    seconds+= tm->hour * SECS_PER_HOUR;
    seconds+= tm->min * SECS_PER_MIN;
    seconds+= tm->sec;
    return seconds; 
  }

  #if USE_EXT_RTC==1 && XRTC==RV3028
    #include "RV-3028-C7.h"
    RV3028 xrtc;
    // needed functions
    // rtc.begin()
    // rtc.setUNIX(...)
    // rtc.setTime(...)
    // rtc.getUNIX()
    // rtc.updateTime()
    // rtc.stringTimeStamp()
  #endif

  int16_t rtc_setup(uint8_t sda, uint8_t scl)
  {
    #if USE_EXT_RTC==1 && XRTC==RV3028
      Wire.begin();
      if (xrtc.begin() == false) {
      Serial.println("RTC offline!");
      }
      else
      {
        Serial.println("RTC online!");
      }
    #endif
    
    return 1;
  }


  void rtcXferTime(void)
  {
    #if USE_EXT_RTC==1 && XRTC==RV3028
      xrtc.setUNIX(rtc_get());
      datetime_t t;
      rtc_get_datetime(&t);
      if (xrtc.setTime(t.sec, t.min, t.hour, t.day, t.day, t.month, t.year) == false) 
      {
            Serial.println("Something went wrong setting the time");
      }
    #endif
  }
  void rtcSync(void)
  { // update local RTC if different from XRTC
    #if USE_EXT_RTC==1 && XRTC==RV3028
      uint32_t to;
      to=rtc_get();
      uint32_t tx;
      tx=xrtc.getUNIX();
      if (to<tx) rtc_set(tx);
    #endif
  }

  char * rtcGetTimestamp(void)
  { 
    #if USE_EXT_RTC==1 && XRTC==RV3028
    //PRINT TIME
    if (xrtc.updateTime() == false) //Updates the time variables from RTC
    {
      Serial.println("RTC failed to update");
      return "";
    } else {
      return xrtc.stringTimeStamp();
    }
    #else
      return 0;
    #endif
  }

  bool rtc_get_datetime(datetime_t *t)
  {
    time2date(rtc_get(), t);
    return 1;
  }

  bool rtc_set_datetime(datetime_t *t)
  {
    rtc_set(date2time(t));
    return 1;
  }

  void rtcSetDate(int year,int month,int day)
  { datetime_t t;
    rtc_get_datetime(&t);
    t.year=year;
    t.month=month;
    t.day=day;
    rtc_set_datetime(&t);
  }

  void rtcSetTime(int hour,int minutes,int seconds)
  { datetime_t t;
    rtc_get_datetime(&t);
    t.hour=hour;
    t.min=minutes;
    t.sec=seconds;
    rtc_set_datetime(&t);
  }

  void XRTCprintTime(void) {}

#endif
    
