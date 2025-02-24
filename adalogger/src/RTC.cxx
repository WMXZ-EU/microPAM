
#include <Wire.h>
#include "RTC.h"

#if USE_EXT_RTC > 0
  #define mWire Wire

  static int16_t i2c_setup(uint8_t sda, uint8_t scl)
  {
    mWire.setSDA(sda);
    mWire.setSCL(scl);
    mWire.begin();
    delay(100);
    return 1;
  }

  static int16_t i2c_exists(uint8_t address)
  {
    mWire.beginTransmission(address);
    return !mWire.endTransmission();
  }

  static void i2c_write_register(uint8_t address, uint8_t reg, uint8_t val) 
  {
    mWire.beginTransmission(address);
    mWire.write(reg);
    mWire.write(val);
    mWire.endTransmission();
  }

  static uint8_t i2c_read_register(uint8_t address, uint8_t reg) 
  {
    mWire.beginTransmission(address);
    mWire.write(reg);
    mWire.endTransmission();

  //  uint8_t buffer[1];
    mWire.requestFrom(address, (uint8_t)1);
    return mWire.read();
  }

  static void i2c_write_data(uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t nbuf)
  {
    mWire.beginTransmission(address);
    mWire.write(reg);
    for(int ii=0; ii<nbuf; ii++) mWire.write(buffer[ii]);
    mWire.endTransmission();  
  }

  static void i2c_read_data(uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t nbuf)
  {
    mWire.beginTransmission(address);
    mWire.write(reg);
    mWire.endTransmission();

    mWire.requestFrom(address, (uint8_t)nbuf);
    for(int ii=0;ii<nbuf;ii++) buffer[ii]=mWire.read();
  }

#endif

  /**************************************************************************************/
  static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }
  static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }


#if XRTC==DS3231

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
    //Serial.println(statreg,BIN);
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
    Serial.print(buffer[3]&(1<<6-1)); Serial.print(' ');
    Serial.print(buffer[2]); Serial.print(' ');
    Serial.print(buffer[1]); Serial.print(' ');
    Serial.print(buffer[0]); Serial.println(' ');
    Serial.flush();

    for(int ii=0;ii<4;ii++) buffer[ii]=bin2bcd(buffer[ii]);
    i2c_write_data(address,alarm1_reg,buffer,4);

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
    //
    return (float)buffer[0] + (buffer[1] >> 6) * 0.25f;
  }
#endif

  #define T2000 946684800 // seconds different between 1-jan-1970 and 1-jan-2000
  #define D2000 10957     // day different between 1-jan-1970 and 1-jan-2000

  #define LEAP_YEAR(Y)  (((Y % 4) == 0) && (((Y % 100) != 0) || ((Y % 400) == 0)))

  const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  void time2date(uint32_t seconds, datetime_t *tm, uint16_t epoch)
  {
    uint32_t days, year, month, dotw;

    uint32_t time = seconds;
   //Retrieve hours, minutes and seconds
    tm->sec =  time % 60;  time /= 60;
    tm->min  = time % 60;  time /= 60;
    tm->hour = time % 24;  time /= 24;

    // time counts now days since epoch (e.g. 1970)
    if(epoch==1970)  dotw = 4; else if(epoch==2000)  dotw = (D2000+4)%7;
    tm->dotw = (time+dotw)%7;    //Unix time starts in 1970 on a Thursday (4)

    year = epoch;
    days = 0;
    while((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) year++;
    tm->year = year;

    days -= LEAP_YEAR(year) ? 366 : 365;
    days = time - days; // now time is days in this year, starting at 0
    Serial.print(days); Serial.print(" ");

    month=0;
    uint16_t monthLength=0;
    for (month=1; month<12; month++) 
    {
      monthLength = daysInMonth[month-1];  
      if ((month==2) & LEAP_YEAR(year)) monthLength++; // february
      
      if (days< monthLength) break;
      days -= monthLength;
    }
    tm->day    = days+1;  // days started from 0
    tm->month  = month;   // is already incremented
  }

  uint32_t date2time(datetime_t *tm, uint16_t epoch)
  {
    uint32_t seconds;

    uint16_t year;
    year=tm->year; 
    uint32_t days= 0;
    for (int y = epoch; y < year; y++)  days += (LEAP_YEAR(y)? 366: 365);

    // add days for this year, months start from 1
    for (int ii = 1; ii < tm->month; ii++) 
    {
      days +=  daysInMonth[ii-1];  
      if ( (ii == 2) && LEAP_YEAR(year)) days++;
    }
    seconds= days*86400;
    seconds+= (tm->day -1) * 86400;
    seconds+= tm->hour * 3600;
    seconds+= tm->min * 60;
    seconds+= tm->sec;
    return seconds; 
  }


  static volatile uint16_t haveExtRTC=0;

  void XRTCgetDatetime(datetime_t *t)
  {
    uint8_t rtcBuffer[7] = {0};//{0,27,15,1,3,4,23}; // adapt to better time (secs,min, ...., year)
    if(haveExtRTC)
    {
      getXRTC(rtcBuffer,7);
      t->year =rtcBuffer[6]+2000; //XRTC reference from y2000
      t->month=rtcBuffer[5]&0x7f;
      t->day  =rtcBuffer[4];

      t->hour=rtcBuffer[2];
      t->min =rtcBuffer[1];
      t->sec =rtcBuffer[0] &0xff;
    }
  }
  void XRTCsetDatetime(datetime_t *t)
  {
    uint8_t rtcBuffer[7] = {0};//{0,27,15,1,3,4,23}; // adapt to better time (secs,min, ...., year)

    if(haveExtRTC)
    {     
      rtcBuffer[6]=t->year-2000;  //XRTC reference from y2000
      rtcBuffer[5]=t->month;
      rtcBuffer[4]=t->day;
      rtcBuffer[2]=t->hour;
      rtcBuffer[1]=t->min;
      rtcBuffer[0]=t->sec;
      for(int ii=0;ii<7;ii++) {Serial.print(rtcBuffer[ii]); Serial.print(' ');} Serial.println();
      setXRTC(rtcBuffer,7);   // write to ExtRTC
    }
  }

  void printDatetime(const char *str, datetime_t *t)
  {
    Serial.printf("%s %04d-%02d-%02d ",str, t->year,t->month,t->day);
    Serial.printf("%02d:%02d:%02d\n",t->hour,t->min,t->sec);
  }

/**************************************************************************************/  
#if defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER)
  uint32_t rtc_get(void)
  { // get seconds since epoch time
    datetime_t tm;
    // get local time
    rtc_get_datetime(&tm);    
    return date2time(&tm, 2000);
  }

  void rtc_set(uint32_t tt)
  {
    datetime_t tm;
    time2date(tt,&tm,2000);
    // set local time
    rtc_set_datetime(&tm);
  }

#else
  void rtc_init(void) {}

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

#endif

  int16_t rtc_setup(void)
  {
    rtc_init(); // hardware (MCU) rtc
    haveExtRTC=initXRTC(RTC_SDA, RTC_SCL); // external rtc clock (XRTC==NONE return 0)
    Serial.print("extClock "); Serial.println(haveExtRTC);
    delay(10);

    // get time from ext RTC
    if(haveExtRTC)
    {
      datetime_t t;
      XRTCgetDatetime(&t);
      printDatetime("XRTC",&t);
      rtcSetDatetime(&t);
      delay(10);

      rtcGetDatetime(&t);
      printDatetime("rtc",&t);
    }
    return 1;
  }
  
  void rtcSetDatetime(datetime_t *t)
  {
      rtc_set_datetime(t);
  }

  void rtcGetDatetime( datetime_t *t)
  {
    rtc_get_datetime(t);
  }    

