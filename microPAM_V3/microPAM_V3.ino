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
  microPAM purpose:
  logging of acoustic data 
  use of Teensy 4.1 or RP2040
  use of I2S Microphone (both) or TLV320ADC6140  (T4.1 only)
  working system T4.1 + TLV320ADC6140 
 */
 
#include "Arduino.h"

#include "src/Config.h"
#include "src/Queue.h"
#include "src/Acq.h"
#include "src/RTC.h"
#include "src/Compress.h"
#include "src/Filing.h"
#include "src/Menu.h"

#include "src/Adc.h"

#if defined(__IMXRT1062__) 
  #include <CrashReport.h>

  #if defined(AUDIO_INTERFACE)
    #include <AudioStream.h>
    #include <usb_audio.h>
    #include "src/AudioTrigger.h"
    #include "src/AudioIF.h"

    AudioTrigger    trigger;
    AudioIF         acqIF(FSAMP);
    AudioOutputUSB  usb; 
    AudioConnection patchCord1(acqIF, 0, usb, 0);
    AudioConnection patchCord2(acqIF, 1, usb, 1);
  #endif

  extern "C" uint32_t set_arm_clock(uint32_t frequency); // clockspeed.c

  #if USE_MTP==1
    #include "src/MTP.h"
    #if NEW_MPT==0
      MTPStorage_SD storage;
      MTPD    mtpd(&storage);
    #else
      MTPD_class mtpd;
    #endif
  #endif

  extern "C" void usb_init(void);
  void stopUSB(void)
  { USB1_USBCMD = 0;  // disconnect USB
    delay(100);       // long enough for PC+hubs to detect
  }
  void resetUSB(void)
  {
    stopUSB();
    usb_init();
    #if USE_MTP==1
      while(!Serial) mtpd.loop();
    #endif
  }

  void reboot(void) { *(uint32_t *)0xE000ED0C =  0x5FA0004;}

  #if USE_EVENTS==1
    extern "C" int usb_init_events(void);
    void resetMTP(void) {mtpd.send_DeviceResetEvent();}
  #else
    int usb_init_events(void) {return 0;}
    void resetMTP(void) {}

  #endif

//  uint32_t getTeensySerial(void) { return (HW_OCOTP_MAC0 & 0xFFFFFF); }
  void getSerNum(void)
  {
    SerNum=HW_OCOTP_MAC0 & 0xFFFFFF;
  }

  void lowPowerInit(void)
  {
    // keep memory powered during sleep
    CCM_CGPR |= CCM_CGPR_INT_MEM_CLK_LPM;
    // keep cpu clock on in wait mode (required for systick to trigger wake-up)
    CCM_CLPCR &= ~(CCM_CLPCR_ARM_CLK_DIS_ON_LPM | CCM_CLPCR_LPM(3));
    // set SoC low power mode to wait mode
    CCM_CLPCR |= CCM_CLPCR_LPM(1);
    // ensure above config is done before executing WFI
    asm volatile("dsb");
    //
    for(int ii=0;ii<41;ii++) { pinMode(ii,INPUT_DISABLE); }
  }

  void triggerRTC() {}

  void stopSystem(void)
  { //shutting down power
    stopSD(); 
    adc_exit();
    stopUSB();
  }

#elif defined(ARDUINO_ARCH_RP2040)
  #if 0
    #include "pico/stdlib.h"
    bool core1_separate_stack = true; //when is it needed?
    // for stack changes
    //"Arduino15\packages\rp2040\hardware\rp2040\4.3.0\lib\rp2040\memmap_default.ld"
  #endif

  #ifdef USE_TINYUSB
    #include "Adafruit_TinyUSB.h"
    void stopUSB(void)
    { 
      if (TinyUSBDevice.mounted()) {
        TinyUSBDevice.detach();
        delay(100);
      }
    }
    void resetUSB(void) 
    { stopUSB();
      TinyUSBDevice.attach();
      while(!Serial);    
    }
  #else // USE_TINYUSB
    #include "tusb.h"
    void stopUSB(void)
    {
      tud_disconnect();
      delay(100);
    }
    void resetUSB(void) 
    {
      stopUSB();
      tud_connect();
      while(!Serial);    
    }
  #endif

  void lowPowerInit(void) {}

  #include "pico/unique_id.h"
  void getSerNum(void)
  {
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    Serial.print("unique_board_id: ");
    for (int len = 0; len < 8; len++) 
    { Serial.print(id.id[len],HEX); Serial.print(' '); } Serial.println();  
    memcpy(&SerNum,&id.id[4],4);
    SerNum &= 0xFFFFFF;
  }

  #ifdef USE_TINYUSB
    // for generating interrupts in core 0 to keep up with acquisition while wfi
    class Ticker_class
    {
      struct repeating_timer timer;
      static void tick() {}
      uint32_t us;
      public:
        Ticker_class() {}
        void begin(uint32_t us)
        { this->us=us;
          add_repeating_timer_us(us, (repeating_timer_callback_t) tick, NULL, &timer);
        }
        void end()
        {
          cancel_repeating_timer(&timer);
        }
        void restart()
        {  add_repeating_timer_us(us, (repeating_timer_callback_t) tick, NULL, &timer);
        }
    };

    Ticker_class ticker;
  #endif
  // see also https://github.com/earlephilhower/arduino-pico/discussions/1544
  // on low power (sleep etc)

  void measure_freqs(void) {
      uint f_pll_sys  = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
      uint f_pll_usb  = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
      uint f_rosc     = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
      uint f_clk_sys  = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
      uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
      uint f_clk_usb  = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
      uint f_clk_adc  = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
      uint f_clk_rtc  = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

      Serial.printf("pll_sys  = %dkHz\n", f_pll_sys);
      Serial.printf("pll_usb  = %dkHz\n", f_pll_usb);
      Serial.printf("rosc     = %dkHz\n", f_rosc);
      Serial.printf("clk_sys  = %dkHz\n", f_clk_sys);
      Serial.printf("clk_peri = %dkHz\n", f_clk_peri);
      Serial.printf("clk_usb  = %dkHz\n", f_clk_usb);
      Serial.printf("clk_adc  = %dkHz\n", f_clk_adc);
      Serial.printf("clk_rtc  = %dkHz\n", f_clk_rtc);
      Serial.flush();
      // Can't measure clk_ref / xosc as it is the ref
  }

  void stopSystem(void)
  {
//    measure_freqs();
    //
    stopSD();
    adc_exit();
    #ifdef USE_TINYUSB
      ticker.end(); // needed for TinyUSB
    #endif
    //
    stopUSB();
  }

#endif 

void storage_configure();  // defined below


/***************************************************************************/
static int setup_ready=0; // lock setup1() while setup() is running
static int termon=0;

  void checkInput(void)
  {
    while(!Serial.available()) ;
    char ch;
    ch=Serial.read();
    (void) ch;
  }

void setup1();  // contains acquisition module and is seperated core in RP2040
//
void setup() 
{
  // put your setup code here, to run once:
  #if defined(__IMXRT1062__)
    set_arm_clock(96'000'000);
  #elif defined(ARDUINO_ARCH_RP2040)
    set_sys_clock_khz(48'000, true);
  #endif

  // only in case it is needed set baud rate (is ignored for USB-Serial)
  Serial.begin(115200); // no action if allready running

  // check start or restart
  uint16_t *params=loadParameters();  // get parameters from EEPROM

  // wait for WAIT_SERIAL s to allow USB-Serial connection, otherwise start after setup
  while(millis()<(WAIT_SERIAL*1000)) if(Serial) { termon=1; break;}
  Serial.println("\nSetup");
  
  Serial.print("params[0] = "); Serial.println(params[0]);
  for(int ii=0;ii<16;ii++) {Serial.print(ii); Serial.print(" "); Serial.println(params[ii]);}
  
  #if defined(__IMXRT1062__)
  // Teensy has a crash report
    if(CrashReport) Serial.print(CrashReport);
    //
    #if defined(AUDIO_INTERFACE)
      AudioMemory(8);
    #endif
    //
    #if USE_MTP==1
      // Teensy has MTP events
      #if USE_EVENTS==1
        usb_init_events();
      #endif
    #endif
    //
    // provide power to ADC
    usbPowerSetup();  
    // keep systick during "wfi"
    lowPowerInit();
    //

  #elif defined(ARDUINO_ARCH_RP2040)
    //
    #ifdef USE_TINYUSB
      ticker.begin(1000); // 1000: interrupts every 1000 microseconds (no need for Teensy pico USB)
    #endif
  #endif //__IMXRT1062__
  //
  getSerNum();
  Serial.print("SerNum: "); Serial.println(SerNum,HEX); Serial.flush();

  // setup RT Clock
  #if USE_EXT_RTC==1
    Serial.println("rtc_setup");
    rtc_setup();
    XRTCclearAlarm();

    #if defined(__IMXRT1062__) // maybe rewrite for all processors to sync
      rtcSync();
    #endif
  #endif

  datetime_t t;
  if(!rtc_get_datetime(&t)) Serial.println("failing get_datetime");
  Serial.printf("RTC-main: %4d-%02d-%02d %02d:%02d:%02d",
                           t.year,t.month,t.day,t.hour,t.min,t.sec); 
  Serial.println();
  //

  /*
  #if USE_EXT_RTC==2
    Serial.print("RV3028: ");
    Serial.println(rtcGetTimestamp());
  #endif
  */

  // initialize filing system
  Serial.println("filing_init");
  storage_configure();  
  Serial.println("Setup done"); Serial.flush();

  // release lock for setup1()
  setup_ready=1;

  // in case of single core teensy 4.1 start acquisition, which for rp2040 is in 2nd core
  #if defined(__IMXRT1062__)
    setup1();
  #endif
}

static uint32_t have_disk=0;
void printBin(uint32_t x) 
{ for (int i = 31; i >= 0; i--) { Serial.print(bitRead(x, i)); if(!(i%8)) Serial.print(' '); }}

void loop() 
{
  // put your main code here, to run repeatedly:
  static uint32_t loopCount=0;
  loopCount++;
  
  // obtain some statistics on Queue usage
  static uint16_t mxb=0;
  uint16_t nb;
  nb = getDataCount();
  if(nb>mxb) mxb=nb;

  // set inititial status STOPPED if terminal was attached within 10 s, CLOSED otherwise 
  static int16_t status=-99;
  if(status==-99) status = (termon==1)? STOPPED: CLOSED;
  status = menu(status);

  #if USE_MTP==1
    #if defined(__IMXRT1062__)
      if(status<0)
      { mtpd.loop();
      }
    #endif
  #endif

  //if(status != DOHIBERNATE)
    status=saveData(status);  

  // once a second provide some information to User
  static uint32_t t0=0;
  uint32_t t1;
  if((t1=millis())>(t0+1000))
  { 
    if(monitor)
    {
      datetime_t t;
      rtc_get_datetime(&t);
      Serial.printf("\n%4d-%02d-%02d %02d:%02d:%02d %d",
                    t.year,t.month,t.day,t.hour,t.min,t.sec,t.dotw); Serial.print(" : ");

      Serial.print(loopCount);  Serial.print(" ");
      Serial.print(procCount);  Serial.print(" ");
      Serial.print(procMiss);   Serial.print(" ");
      Serial.printf("%3d",mxb); Serial.print("  ");
      Serial.print(have_disk);  Serial.print("  ");
      Serial.print(disk_count); Serial.print("; ");

      if(proc==0)
      {
        #if 1
          for(int ii=0; ii<8;ii++) {Serial.printf("%9d",logBuffer[ii]);}
        #else
          for(int ii=0; ii<2;ii++) 
          { /* print binary to check data*/ printBin(logBuffer[ii]); Serial.print("; "); }        
        #endif
      }
      else
      {
        for(int ii=0; ii<MB;ii++){ Serial.printf("%2d ",proc_stat[ii]);}
        Serial.printf("%2d",max_stat);

        for(int ii=0; ii<MB;ii++){ proc_stat[ii]=0;}
        max_stat=0;        
      }
    }

    loopCount=0;
    procCount=0;
    procMiss=0;
    mxb=0;
    disk_count=0;

    t0=t1;
  }

  asm("wfi");
}

/******************************************************************************e****/
// rp2040 has two cores. let acquisition run on its own core
// on teensy this will be called from setup()

void setup1()
{ delay(100);
  while(!setup_ready) {delay(10);} // wait for setup() to finish
  Serial.println("Setup1");
  i2s_setup();
  dma_setup();
  adc_init();

  Serial.println("Setup1 done");
}

void loop1(){ asm("wfi"); }  // nothing to be done here; sleep (wait for interrupt) in case it is called

//
/************************************Device specific functions *****************************/
 #include "SdFat.h"

  #ifndef BUILTIN_SDCARD
    #define BUILTIN_SDCARD 254
  #endif

  #if defined(__IMXRT1062__)
    #define SD_CONFIG SdioConfig(FIFO_SDIO)
    void spi_init() {}

  #elif defined(ARDUINO_ARCH_RP2040)
    #if USE_SDIO==0
      #if ENABLE_DEDICATED_SPI !=1
        #error "ENABLE_DEDICATED_SPI !=1: edit SdFatConfig.h"
      #endif

      void spi_init()
      { pinMode(SPI_CS, OUTPUT);
        mSPI.setCS(SPI_CS);
        mSPI.setRX(SPI_MISO);
        mSPI.setTX(SPI_MOSI);
        mSPI.setSCK(SPI_SCK);
      }
      // Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
      #define SD_CONFIG SdSpiConfig(SPI_CS, DEDICATED_SPI, SD_SCK_MHZ(50),(SpiPort_t *) &mSPI)
    #else
      // for sdio
      #define RP_CLK_GPIO SPI_SCK
      #define RP_CMD_GPIO SPI_MOSI
      #define RP_DAT0_GPIO SPI_MISO  // DAT1: GPIO21, DAT2: GPIO22, DAT3: GPIO23.

      #define SD_CONFIG SdioConfig(RP_CLK_GPIO, RP_CMD_GPIO, RP_DAT0_GPIO)
      void spi_init() {}
    #endif
  #endif

  // needed for MTP
  const char *sd_str[]={"sdio"};          // edit to reflect your configuration
  const int cs[] = {BUILTIN_SDCARD};      // edit to reflect your configuration

  const int nsd = sizeof(sd_str)/sizeof(const char *);
  SdFs sdx[nsd];

  void storage_configure()
  {
    // Using SD card for storage
      for(int ii=0; ii<nsd; ii++)
      { 
        /* for spi cards need chip select pin */
        if(cs[ii]<BUILTIN_SDCARD)
        { pinMode(cs[ii],OUTPUT); digitalWrite(cs[ii],HIGH); 
          spi_init();
        }
        int jj;
        for(jj=0;jj<5;jj++) if (sdx[ii].begin(SD_CONFIG)) break; else delay(1000);
        if(jj==5)
        {
          Serial.printf("SD/SDIO Storage %d %d %s failed or missing",ii,cs[ii],sd_str[ii]);  Serial.println();
          have_disk=0;
        }
        else
        {
          uint64_t totalSize = sdx[ii].clusterCount();
          uint64_t freeSize  = sdx[ii].freeClusterCount();
          uint32_t clusterSize = sdx[ii].bytesPerCluster();
          Serial.printf("SDIO Storage %d %d %s ",ii,cs[ii],sd_str[ii]); 
          Serial.print("; total clusters: "); Serial.print(totalSize); 
          Serial.print(" free clusters: "); Serial.print(freeSize);
          Serial.print(" clustersize: "); Serial.print(clusterSize/1024); Serial.println(" kByte");

          #if USE_MTP==1
            #if NEW_MPT==0
              storage.addFilesystem(sdx[ii], sd_str[ii]);
            #else
              mtpd.addFilesystem(sdx[ii], sd_str[ii]);
            #endif
          #endif

          filing_init();
          have_disk=1;
        }
      }
  }
