
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#include <TimeLib.h>
#include <CrashReport.h>

#define NCHAN_I2S 2
#define NCHAN_ACQ 1

#include "I2S_32.h"
#include "logger.h"

// GUItool: begin automatically generated code
//AudioInputI2S            acq;           //xy=221.23077011108398,209.23077774047852
I2S_32                  acq;           //xy=221.23077011108398,209.23077774047852
AudioOutputUSB          usb1;           //xy=543.2307739257812,207.23076629638672

AudioConnection         patchCord2(acq, 0, usb1, 0);
AudioConnection         patchCord3(acq, 1, usb1, 1);
// GUItool: end automatically generated code

uint32_t SerNum =0;
int32_t fsamp=44100;
int32_t t_acq=60,t_on=300,t_off=0;
int32_t shift = 8;
int32_t proc = 0;

#define SDCARD_CS_PIN    BUILTIN_SDCARD
time_t getTime() { return Teensy3Clock.get(); }

extern "C"
{
  uint32_t set_arm_clock(uint32_t frequency);
}
void setup() {
  // put your setup code here, to run once:

  setSyncProvider(getTime);

  set_arm_clock(24000000);
  
  AudioMemory(10);
  acq.digitalShift(shift);

  while(!Serial);
  if(CrashReport) Serial.print(CrashReport);
  
  if (!(SD.begin(SDCARD_CS_PIN)) && !(SD.begin(SDCARD_CS_PIN))) 
  {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
    //while(1);
  }
  else
    Serial.println("SD card found");
  
  pinMode(13,OUTPUT);
}

extern uint32_t disk_count;
extern uint32_t diskBuffer[];
int32_t *data= (int32_t *)diskBuffer;
//
void loop() {
  // put your main code here, to run repeatedly:
  static int16_t status=0;

  int m_sec=(second() % t_acq); // close file every t_acq (60) seconds (on the minute)
  static int m_seco=0;
  if( (m_seco>0) && (m_sec<m_seco)) status=DOCLOSE;
  m_seco=m_sec; 

  status=saveData(status);
  
  static uint32_t t0=0;
  static uint32_t ic=0;
  if(millis()-t0>1000)
  { digitalWriteFast(13,!digitalReadFast(13));
    t0=millis();
    Serial.printf("\n%10d %2d %2d %10d %10d",ic++, second(), disk_count, data[0],data[1]);
    disk_count=0;
  }
//  asm ("WFI");
}
