#ifndef RTC_H
#define RTC_H

#if defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER)
  #include "hardware/rtc.h"
#else
  typedef struct {
      int16_t year;    ///< 0..4095
      int8_t month;    ///< 1..12, 1 is January
      int8_t day;      ///< 1..28,29,30,31 depending on month
      int8_t dotw;     ///< 0..6, 0 is Sunday
      int8_t hour;     ///< 0..23
      int8_t min;      ///< 0..59
      int8_t sec;      ///< 0..59
  } datetime_t;
  bool rtc_get_datetime(datetime_t *t);
  bool rtc_set_datetime(const datetime_t *t);
#endif

#define USE_EXT_RTC 1
#define XRTC DS3231

// for XRTC
#define RTC_SDA   2
#define RTC_SCL   3

int16_t rtc_setup(void);
uint32_t rtc_get(void);

void time2date(uint32_t seconds, datetime_t *tm, uint16_t epoch);
uint32_t date2time(datetime_t *tm, uint16_t epoch);

void printDatetime(const char *str, datetime_t *t);

void rtcGetDatetime(datetime_t *t);
void rtcSetDatetime(datetime_t *t);

void XRTCgetDatetime(datetime_t *t);
void XRTCsetDatetime(datetime_t *t);

void XRTCsetAlarm(uint32_t secs);

#endif
