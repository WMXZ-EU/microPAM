
//#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
//#include <SerialFlash.h>

#include <TimeLib.h>
#include <CrashReport.h>

#include <usb_audio.h>
#include "I2S_32.h"
#include "record_queue.h"

// GUItool: begin automatically generated code
//AudioInputI2S            acq;           //xy=221.23077011108398,209.23077774047852
I2S_32                   acq;           //xy=221.23077011108398,209.23077774047852
//AudioInputPDM            acq;           //xy=238.23077392578125,285.2307662963867
AudioOutputUSB           usb1;           //xy=543.2307739257812,207.23076629638672
AudioRecordQueue         queue1;         //xy=550.2307739257812,276.2307662963867

AudioConnection          patchCord1(acq, 0, queue1, 0);
AudioConnection          patchCord2(acq, 0, usb1, 0);
AudioConnection          patchCord3(acq, 1, usb1, 1);

// GUItool: end automatically generated code

// custom audiobuffer macro (in lieu of stock AudioMemory

  #define ExtAudioMemory(num) ({ \
  static EXTMEM audio_block_t data[num]; \
  AudioStream::initialize_memory(data, num); \
  })

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
  
  AudioMemory(60);
//  acq.digitalShift(8);

  while(!Serial);
  if(CrashReport) Serial.print(CrashReport);
  
  if (!(SD.begin(SDCARD_CS_PIN)) && !(SD.begin(SDCARD_CS_PIN))) 
  {
    // stop here if no SD card, but print a message
    Serial.println("Unable to access the SD card");
    //while(1);
  }
  Serial.println("SD card found");
  
  queue1.begin();
}

#include "logger.h"

extern uint32_t disk_count;
extern char diskBuffer[];
int16_t *data=(int16_t *) diskBuffer;
//
void loop() {
  // put your main code here, to run repeatedly:
  static int16_t status=0;

  int m_sec=(second() % 60); // close file every 60 seconds (on the minute)
  static int m_seco=0;
  if( (m_seco>0) && (m_sec<m_seco)) status=DOCLOSE;
  m_seco=m_sec; 

  status=saveData(status);
  
  static uint32_t t0=0;
  static uint32_t ic=0;
  if(millis()-t0>1000)
  {
    t0=millis();
    Serial.printf("\n%10d %2d %2d %10d %10d",ic++, second(), disk_count, data[0],data[1]);
    disk_count=0;
  }
//  asm ("WFI");
}
