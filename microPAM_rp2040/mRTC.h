#ifndef mRTC_H
#define mRTC_H

#if defined(TARGET_RP2040)
#include "hardware/rtc.h"
  #define SDA 8
  #define SCL 9

  uint32_t rtc_get(void);

#elif defined(__IMXRT1062__)
  #define SDA 18
  #define SCL 19


  /** \struct datetime_t
  *  \ingroup util_datetime
  *  \brief Structure containing date and time information
  *
  *    When setting an RTC alarm, set a field to -1 tells
  *    the RTC to not match on this field
  */
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
#endif

int16_t rtc_setup(uint8_t sda=SDA, uint8_t scl=SCL);
void time2date(uint32_t time, datetime_t *tm);

#endif