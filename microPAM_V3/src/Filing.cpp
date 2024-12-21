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
  * File: Filing.cpp
 */

#include "SdFat.h"

#include "Config.h"
#include "Queue.h"
#include "Acq.h"
#include "Adc.h"
#include "RTC.h"
#include "Menu.h"
#include "Filing.h"

#define MAGIC "WMXZ"

volatile uint16_t t_acq= T_ACQ;
volatile uint16_t t_on = T_ON;
volatile uint16_t t_rep= T_REP;
volatile uint16_t h_1  = H_1;
volatile uint16_t h_2  = H_2;
volatile uint16_t h_3  = H_3;
volatile uint16_t h_4  = H_4;
volatile uint16_t d_on = D_ON;
volatile uint16_t d_rep= D_REP;
volatile  int16_t d_0  = D_0;

volatile char b_string[40];
volatile char k_string[40];
volatile char n_string[40];

#if SDFAT_FILE_TYPE != 3
 #error "SDFAT_FILE_TYPE != 3: edit SdFatConfig.h"
#endif

/*
#if defined(ARDUINO_ARCH_RP2040)

  #if 1
    #if ENABLE_DEDICATED_SPI !=1
      #error "ENABLE_DEDICATED_SPI !=1: edit SdFatConfig.h"
    #endif

    // Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
    #define SD_CONFIG SdSpiConfig(SPI_CS, DEDICATED_SPI, SD_SCK_MHZ(50),(SpiPort_t *) &mSPI)

    void msc_setup(void);
    void msc_init(void);

    void spi_init()
    { pinMode(SPI_CS, OUTPUT);
      mSPI.setCS(SPI_CS);
      mSPI.setRX(SPI_MISO);
      mSPI.setTX(SPI_MOSI);
      mSPI.setSCK(SPI_SCK);
    }
  #else
    #define RP_CLK_GPIO 18
    #define RP_CMD_GPIO 19
    #define RP_DAT0_GPIO 20  // DAT1: GPIO21, DAT2: GPIO22, DAT3: GPIO23.

    #define SD_CONFIG SdioConfig(RP_CLK_GPIO, RP_CMD_GPIO, RP_DAT0_GPIO)

    void spi_init() {}
  #endif

#elif defined(__IMXRT1062__)
  #define SD_CONFIG SdioConfig(FIFO_SDIO)
#endif
*/

extern SdFs sdx[];  // defined in storage_configure
SdFs *sd = &sdx[0];
FsFile file;

static HdrStruct wav_hdr;

static int haveStore=0;
uint32_t diskSpace=0;
uint32_t diskSize=0;   

// tempBuffer to keep all 32-bit data 
//#define MAX_TEMP_BUFFER (NDBL*NBUF_ACQ)
//static int32_t tempBuffer0[MAX_TEMP_BUFFER];
//static int32_t tempBuffer1[MAX_TEMP_BUFFER];

// diskBuffer to hold all data to be written to disk (with and without compression
#define MAX_DISK_BUFFER (NDBL*NBUF_ACQ)
static int32_t diskBuffer[MAX_DISK_BUFFER];
uint32_t disk_count=0;
uint32_t nout_dat=0;

uint32_t SerNum=0;

// Call back for file timestamps.  Only called for file create and sync(). needed by SDFat
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) 
{
    datetime_t t;
    rtc_get_datetime(&t);

    *date = FS_DATE(t.year,t.month,t.day);
    *time = FS_TIME(t.hour,t.min,t.sec);
    *ms10 = 0;
}

static char configText[1024];
uint32_t sdSectorCount=0;

int16_t loadConfigfromFile(void)
{
    int ii=0;
    file = sd->open("config.txt"); 
    if(file) 
    { while (file.available()) 
      {
        configText[ii++]=file.read();
      }
      file.close(); 
    }
  return ii;
}
int16_t filing_init(void)
{
  FsDateTime::callback = dateTime;

  sdSectorCount=sd->card()->sectorCount();
  Serial.println(sdSectorCount);
  haveStore=1;
  if(loadConfigfromFile())              // is in Filing
    decodeConfigfromFile(configText);   // in in Menu
  pinMode(LED,OUTPUT);
  return 1;

  /*
  for(int ii=0; ii<5;ii++)
  {
    Serial.print("connect to SDcard :"); Serial.println(ii);
    if (sd->begin(SD_CONFIG)) 
    { Serial.println("card initialized.");
      sdSectorCount=sd->card()->sectorCount();
      haveStore=1;
      if(loadConfigfromFile())    // is in Filing
        decodeConfigfromFile();   // in in Menu
      pinMode(LED,OUTPUT);

      #if defined(ARDUINO_ARCH_RP2040)
        //msc_init();
      #endif

      return 1;
    }
    else
    { Serial.println("still trying..."); 
      delay(100);
    }
  }

  Serial.println("Card failed, or not present");
  // don't do anything more:
  return 0;
  */
}

void stopSD(void)
{
//https://github.com/greiman/SdFat/issues/401
  sd->card()->syncDevice();
}
void filing_exit()
{
  /*
  pinMode(_CS, OUTPUT);
  digitalWrite(_CS, LOW);

  for(int i = 0; i < 10; i++)
  {
    byte v = mSPI.transfer(0xFF);
    if(v == 0xFF)
      break;
  }
  digitalWrite(_CS, HIGH);
  */

  /*
  mSPI.begin();
  mSPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  mSPI.transfer(0XFF);
  mSPI.endTransaction();
  mSPI.end();
  */
  digitalWrite(LED,LOW);
}

char * timeStamp(void)
{
  static char date_time[80];
  
  datetime_t t;
  rtc_get_datetime(&t);

  sprintf(date_time,"%04d%02d%02d_%02d%02d%02d",t.year,t.month,t.day,t.hour,t.min,t.sec);
  return date_time;
}

void writeHeaderInfo(char *info, int serNum)
{
  uint32_t tt=millis();
  int ih=0;
  strcpy(&info[ih], MAGIC); ih+=4;              //4
  strcpy(&info[ih], timeStamp()); ih +=16;      //20
  sprintf(&info[ih]," %8x",serNum); ih +=12;    //32
  memcpy(&info[ih], &tt,4); ih +=4;             //36
  memcpy(&info[ih], (char *)b_string,40); ih+=40;       //76
  memcpy(&info[ih], (char *)k_string,40); ih+=40;       //116
  memcpy(&info[ih], (char *)n_string,40); ih+=40;       //156
  memcpy(&info[ih], getStore(),16*2); ih +=32;  //188
  strcpy(&info[ih]," end");                     //196
}

char * wavHeaderInit(int32_t fsamp, int32_t nchan, int32_t nbits, int serNum)
{
  int nbytes=nbits/8;

  memcpy(wav_hdr.rId,"RIFF",4);
  memcpy(wav_hdr.wId,"WAVE",4);
  memcpy(wav_hdr.fId,"fmt ",4);
  memcpy(wav_hdr.dId,"data",4);
  memcpy(wav_hdr.iId,"info",4);

  wav_hdr.rLen = 512-2*4; // will be updated at closing
  wav_hdr.fLen = 0x10;
  wav_hdr.iLen = 512 - 13*4;
  wav_hdr.dLen = 0; // will be updated at closing

  wav_hdr.nFormatTag=1;
  wav_hdr.nChannels=nchan;
  wav_hdr.nSamplesPerSec=fsamp;
  wav_hdr.nAvgBytesPerSec=fsamp*nbytes*nchan;
  wav_hdr.nBlockAlign=nchan*nbytes;
  wav_hdr.nBitsPerSamples=nbits;

  writeHeaderInfo(&wav_hdr.info[0],serNum);

  return (char *)&wav_hdr;
}

char * wavHeaderUpdate(int32_t nbytes)
{
  wav_hdr.dLen = nbytes;
  wav_hdr.rLen += wav_hdr.dLen;
  return (char *)&wav_hdr;
}

void wavHeaderWrite(char * wav_hdr) 
{ 
  uint64_t fpos;
  fpos = file.curPosition();
  Serial.printf("; fpos=%d ",fpos);
  file.seekSet(0);
  file.write((const uint8_t*)wav_hdr,512);
  file.seekSet(fpos);
}

/*
char * wavHeader(uint32_t fileSize, int32_t fsamp, int32_t nchan, int32_t nbits)
{
//  int fsamp=48000;
//  int fsamp = fs;
//  int nchan=nch;

//  int nbits=16;
  int nbytes=nbits/8;

  int nsamp=fileSize/(nbytes*nchan);
  //
  //  static char wheader[44];
  static char wheader[512];
  //
  strcpy(wheader,"RIFF");
  strcpy(wheader+8,"WAVE");
  strcpy(wheader+12,"fmt ");
  strcpy(wheader+36,"data");
  *(int32_t*)(wheader+16)= 16;// chunk_size
  *(int16_t*)(wheader+20)= 1; // PCM 
  *(int16_t*)(wheader+22)= nchan;// numChannels 
  *(int32_t*)(wheader+24)= fsamp; // sample rate 
  *(int32_t*)(wheader+28)= fsamp*nbytes; // byte rate
  *(int16_t*)(wheader+32)= nchan*nbytes; // block align
  *(int16_t*)(wheader+34)= nbits; // bits per sample 
  *(int32_t*)(wheader+40)= nsamp*nchan*nbytes; 
  *(int32_t*)(wheader+4)= 36+nsamp*nchan*nbytes; 

   return wheader;
}
*/

int16_t makeHeader(int32_t *header)
{
    /**
     * @brief Make file header
     * @param header is pointer to header
     * 
     */
    datetime_t t;
    rtc_get_datetime(&t); 

    sprintf((char *)header,"%s%04d%02d%02d_%02d%02d%02d",
            MAGIC,t.year,t.month,t.day,t.hour,t.min,t.sec);

    //header[5] = 20;          // SW version
    header[5] = 30;            // SW version
    header[6] = SerNum;        // serial number
    header[7] = fsamp;
    header[8] = NCHAN_ACQ;
    header[9] = t_acq;
    header[10] = t_on;
    header[11] = t_rep;
    header[12] = proc;
    header[13] = shift;
    header[14] = again;
    header[15] = dgain;
    header[16] = millis();
    //memcpy(&header[20], getStore(),16*2);
    writeHeaderInfo((char *)&header[20],SerNum);
    header[127]=0x55555555;
    return 1;
}

int16_t checkEndOfFile(int16_t state)
{ 
  static uint32_t dta=0;

  if(state==RUNNING)
  { 
    uint32_t tt = rtc_get();
    //
    uint32_t dt1 = tt % t_acq;
    if(dt1<dta) state = DOCLOSE;  	  	// should close file and continue
    dta = dt1;
    //
    // if file should be closed
    // check also if it should then hibernate 
    if(state == DOCLOSE)                // in case of DOCLOSE
    { 
      if(t_rep>t_on)                      // and if foreseen  check for hibernation
      { uint32_t dt2 = (tt % t_rep);
        if(dt2>=t_on) state=DOHIBERNATE;  // should close file and hibernate
      }
    }
    if(state == DOCLOSE)                // in case of DOCLOSE check dayly protocol
    {
      if(d_rep>d_on)                      // and if foreseen  check for hibernation
      { int32_t dd=tt/(24*3600);
        uint32_t dd2 = (dd % d_rep);
        
        if(dd2>=d_on) state=DOHIBERNATE;  // should close file and hibernate
      }
    }
    if(state == DOCLOSE)                // in case of DOCLOSE check acquisition periods
    {
      uint32_t hh=(tt%((24*3600)/3600));
      if(((hh>=h_1) && (hh<h_2)) || ((hh>=h_3) && (hh<h_4)))
      { state=DOCLOSE;
      }
      else
      { 
        state=DOHIBERNATE;
      }
    }
    if(state == DOCLOSE)                // in case of DOCLOSE check start day
    {
      uint32_t dd=tt/(24*3600);
      if(dd<(uint32_t)(d_0+D_REF)) state=DOHIBERNATE;     // we are too early
    }
  }
  return state;
}

uint16_t checkDiskSpace(void)
{   if((diskSize>0) && ((diskSpace=sd->freeClusterCount()) > MIN_SPACE)) return 1;
    return 0;
}

uint16_t newFolder(int d)
{ static int d0=-1;
  if(d==d0) return 0; // same day
  d0=d;
  return 1;
}

int16_t newDirectory(char *dirName)
{   
    datetime_t t;
    rtc_get_datetime(&t);

    if(HourDir)
    {
      if(newFolder(t.hour))
      {   
          sprintf(dirName, "/%s_%s_%s_%06x_%04d%02d%02d/%02d/", 
              b_string,k_string,n_string,(unsigned int)SerNum, t.year,t.month,t.day,t.hour);
          //
          Serial.println(); Serial.print(": "); Serial.print(dirName); 
          return 1;   // have new directory
      }
    }
    else
    {
      if(newFolder(t.day))
      {   
          sprintf(dirName, "/%s_%s_%s_%06x_%04d%02d%02d/", 
              b_string,k_string,n_string,(unsigned int)SerNum, t.year,t.month,t.day);
          //
          Serial.println(); Serial.print(": "); Serial.print(dirName);
          return 1;   // have new directory
      }
    }
    return 0;       // keep old directory
}

int16_t newFileName(char *fileName)
{
    datetime_t t;
    rtc_get_datetime(&t);
    if(proc==0)
      sprintf(fileName, "%s%04d%02d%02d_%02d%02d%02d.wav", FilePrefix, t.year,t.month,t.day,t.hour,t.min,t.sec);
    else
      sprintf(fileName, "%s%04d%02d%02d_%02d%02d%02d.bin", FilePrefix, t.year,t.month,t.day,t.hour,t.min,t.sec);
    //
    Serial.println(); Serial.print(": "); Serial.print(fileName);
    Serial.print(" "); Serial.print(micros());

    return 1;
}

static char dirName[80];
static char fileName[80];
static int32_t fileHeader[128];
uint32_t nbuf;
void do_hibernate(void);

/**************** main data filing routine ************************/
int16_t storeData(int16_t status)
{
    if(status==CLOSED) // file closed: should open
    {   //if(!checkDiskSpace()) {return STOPPED;}
        //
        if(newDirectory(dirName))
        {   if(!sd->exists(dirName) && !sd->mkdir(dirName)) {Serial.println("Error mkdir"); return STOPPED;}         
            if(!sd->chdir(dirName)) {Serial.println("Error chdir"); return STOPPED;}
        }
        
        if(newFileName(fileName))
        {   
            file = sd->open(fileName, FILE_WRITE); 
            if(file) 
            { status = OPENED; 
              //msc_activate(false);
            }
            else 
            {   Serial.println("Failing open file");
                //msc_activate(true);
                return STOPPED; 
            }
        } 
        else
        {   //msc_activate(true);
            return STOPPED; // if file open fails: don't do anything
        }
    }
    //
    if(status==OPENED) // file is open: write first record (header)
    {   nbuf=0;
        char *hdr=0;
        if(proc==0)
        { 
          hdr = wavHeaderInit(fsamp, NCHAN_ACQ, NBITS, SerNum);
        }  
        else
        {
          makeHeader(fileHeader);
          hdr=(char *)fileHeader;
        }
        int nd;
        if((nd=file.write((const uint8_t*)hdr,512)) < 512) 
        { status = DOCLOSE;
        }
        else status=RUNNING;
    }
    //
    if(status==RUNNING) // file is open and header written: store data records
    {   uint32_t nd;
        if((nd=file.write((const uint8_t*)diskBuffer,4*nout_dat)) < 4*nout_dat) 
        { Serial.print(">"); 
          Serial.print(nd); 
          Serial.print(" "); 
          Serial.println(status); 
          status=DOCLOSE; 
        }
        else
          nbuf++;
        //
        disk_count++;
        if((nbuf % 1000)==0 ) file.flush();
    }    

    // following is done independent of data availability
    if((status==DOCLOSE) || (status==DOHIBERNATE) || (status==MUSTSTOP)) // should close file or stop acquisition
    {   // first close file
        if(file)
        {   if(proc==0)
            {
              char *hdr = wavHeaderUpdate(nout_dat*4);
              wavHeaderWrite(hdr);
            }
            file.close();
        }

        if(status==DOHIBERNATE)
        {   // shutdown acq board
          filing_exit();
          adc_exit();
            //adcReset();
            //acqPower(LOW);
            digitalWrite(LED,LOW);
            do_hibernate();
        }
        else if(status==DOCLOSE)
        {
          status=CLOSED;
        }
        else if(status==MUSTSTOP)
        { //msc_activate(true);
          status=STOPPED;
          filing_exit();
          adc_exit();
          digitalWrite(LED,LOW);
        }
    }
    return status;
}
//
volatile int32_t logBuffer[8];
int16_t saveData(int16_t status)
{
    if(status==STOPPED) 
    { pullData((uint32_t*)diskBuffer);
      for(int ii=0;ii<8;ii++) logBuffer[ii]=diskBuffer[ii];
    }

    if(status<CLOSED) return status; // we are stopped: don't do anything

    status=checkEndOfFile(status);

    if(getDataCount() >= NDBL)
    { 
      if(proc==0)
      { 
        for(int ii=0; ii<NDBL; ii++)
        { while(queue_isBusy()){continue;} //wait if acq writes to queue
          while(!pullData((uint32_t *)&diskBuffer[ii*NBUF_ACQ])) delay(1);
        }

        for(int ii=0;ii<8;ii++) logBuffer[ii]=diskBuffer[ii];

        nout_dat=MAX_DISK_BUFFER;
        if(NBITS==24)
        { // wav mode; store only top 24 bits of each word

          int jj=0;
          uint8_t * inpptr=(uint8_t *) diskBuffer;
          uint8_t * outptr=(uint8_t *) diskBuffer;
          for(int ii=0; ii<MAX_DISK_BUFFER;ii++)
          {
            outptr[3*ii]=inpptr[4*ii];
            outptr[3*ii+1]=inpptr[4*ii+1];
            outptr[3*ii+2]=inpptr[4*ii+2];
          }
          nout_dat=(MAX_DISK_BUFFER/4)*3;
        }
        else if(NBITS==16)
        { // wav mode; store only top 16 bits of each word
          int16_t * inpptr=(int16_t *) diskBuffer;
          int16_t * outptr=(int16_t *) diskBuffer;
          for(int ii=0; ii<MAX_DISK_BUFFER;ii++)
          {
            outptr[ii]=inpptr[2*ii];
          }
          nout_dat=(MAX_DISK_BUFFER/4)*2;
        }
      }
      else
      { // compressed mode; store all 32 bits
        for(int ii=0; ii<NDBL; ii++)
        { while(queue_isBusy()){continue;} //wait if acq writes to queue
          while(!pullData((uint32_t *)&diskBuffer[ii*NBUF_ACQ])) delay(1);
        }
        for(int ii=0;ii<8;ii++) logBuffer[ii]=diskBuffer[ii];
      }
      //
      if(haveStore)
      { digitalWrite(LED,HIGH);
        status=storeData(status);
        digitalWrite(LED,LOW);
      }
    }

    return status;
}

  /*********************** hibernate ******************************/
  /*  hibernating is shutting down the power snvs mode
      only RTC continuoes to run (if there is a 3V battery or power)
      hipernation is controlled by t_rep (sec), t_1,t_2,t_3, t_4 (h)

      for t_rep > t_on,  system will hibernate until next multiple of t_rep 

      t_1 to t_4 describe 2 acquirition windows (unit hour)
      acquisition happens from t_1 to t_2 and t_3 to t_4 
      (0 <= t_1 <= t_2 <= t_3 <=t_4 <=24)
      24 hour aquisition is ensured by t_1=0, t_2=12, t_3=12, t_4=24

      if actual day < d_0+D_REF, system will hibernate until d_0; D_REF= 20000 (or 4th October 2024) 

      if d_rep > d_on, system will hibernate after d_on until next multiple of d_rep
      wakeup time is estimated by estAlarmTime
  */
  uint32_t estAlarmTime(uint32_t secs)
  {   // estimate the wakup-time in seconds 
      // input: actual time in s
      // output: next wakup time in s
      // wakeup is in absolute seconds
      // 
      // secs is actual time in s
      uint32_t dd = secs/(24*3600);       // full days so far
      uint32_t hh =(secs%(24*3600))/3600; // full hours into day

      uint32_t d_x = (d_0+D_REF);

      // wake-up at midnight of start date
      if(0) // deactivate this function (comment to activate)
      if(dd<(d_x)) 
      { // we are too early
        secs=(d_x)*(24*3600);
        return secs;
      }
      //
      if(d_rep> d_on)
      {  // check if day is good for acqisition
        if(dd % d_rep >=d_on)
        {
          secs = ((dd/d_rep)+1)*d_rep*(24*3600);  
          return secs;
        }
      }
      //
      if(((hh>=h_1) && (hh<h_2)) || ((hh>=h_3) && (hh<h_4)) )
      { // are we between recording periods during acquisition day
        if(t_rep>t_on)
        { // normal hibernation for duty cycling 
          secs = ((secs/t_rep)+1)*t_rep;
          return secs;
        }
      }
      //
      if (hh<h_1)                // from mid-night to h_1 
      {
        secs = (dd*24+ h_1)*3600;     // next time is h1
      }
      else if ((hh>=h_2) && (hh<h_3)) // between the two recording periods
      {
        secs = (dd*24+h_3)*3600;      // next time is h3
      }
      else if (hh>=h_4) // after the second recording period (goes into next day)
      {
        dd++;
        secs = (dd*24+h_1)*3600;  // next time is next day at h_1
      }
      //
      // return start or actual time in seconds
      return secs;
  }


#if 0
  #include "Adafruit_TinyUSB.h"

  // USB Mass Storage object
  Adafruit_USBD_MSC usb_msc;

  bool fs_changed;

  int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize);
  int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize);
  void msc_flush_cb (void);

  void msc_activate(bool ready)
  {
      //usb_msc.setUnitReady(ready);
  }
  void msc_setup(void)
  {
    return;
    // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
    usb_msc.setID("Adafruit", "SD Card", "1.0");

    // Set read write callback
    usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

    // Still initialize MSC but tell usb stack that MSC is not ready to read/write
    // If we don't initialize, board will be enumerated as CDC only
    usb_msc.setUnitReady(false);
    usb_msc.begin();

    // If already enumerated, additional class driver begin() e.g msc, hid, midi won't take effect until re-enumeration
    if (TinyUSBDevice.mounted()) {
      TinyUSBDevice.detach();
      delay(10);
      TinyUSBDevice.attach();
    }
  }

  void msc_init()
  {
    #if SD_FAT_VERSION >= 20000
    uint32_t block_count = sd->card()->sectorCount();
  #else
    uint32_t block_count = sd->card()->cardSize();
  #endif
    // Set disk size, SD block size is always 512
    usb_msc.setCapacity(block_count, 512);

    // MSC is ready for read/write
    usb_msc.setUnitReady(true);

  }
  // Callback invoked when WRITE10 command is completed (status received and accepted by host).
  // used to flush any pending cache.
  void msc_flush_cb (void) {
  #if SD_FAT_VERSION >= 20000
    sd->card()->syncDevice();
  #else
    sd->card()->syncBlocks();
  #endif

    // clear file system's cache to force refresh
    Serial.println("msc_flush");
    sd->end();

    fs_changed = true;
  }
#endif

  // Callback invoked when received READ10 command.
  // Copy disk's data to buffer (up to bufsize) and
  // return number of copied bytes (must be multiple of block size)
  int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize) 
  {
    bool rc;

  #if SD_FAT_VERSION >= 20000
    rc = sd->card()->readSectors(lba, (uint8_t*) buffer, bufsize/512);
  #else
    rc = sd->card()->readBlocks(lba, (uint8_t*) buffer, bufsize/512);
  #endif

    return rc ? bufsize : -1;
  }

  // Callback invoked when received WRITE10 command.
  // Process data in buffer to disk's storage and 
  // return number of written bytes (must be multiple of block size)
  int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize) 
  {
    bool rc;

  #if SD_FAT_VERSION >= 20000
    rc = sd->card()->writeSectors(lba, buffer, bufsize/512);
  #else
    rc = sd->card()->writeBlocks(lba, buffer, bufsize/512);
  #endif

    return rc ? bufsize : -1;
  }

#if 0
  uint32_t msc_status=0;
  //void __USBInstallMassStorage() {}
  #define min(x,y) (x<y?x:y)
  extern "C" void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) 
  {
      (void) lun;

    const char *vid =  "WMXZ";
    const char *pid =  "Mass Storage";
    const char *rev =  "1.0";

    msc_status=1;
    memcpy(vendor_id, vid, min(strlen(vid), 8));
    memcpy(product_id, pid, min(strlen(pid), 16));
    memcpy(product_rev, rev, min(strlen(rev), 4));
  }

  extern "C" bool tud_msc_test_unit_ready_cb(uint8_t lun) 
  {
      (void) lun;
      msc_status=2;
      return false; // disable MSC as it does not yet work 
    return haveStore? true: false;
  }

  extern "C" void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size) 
  {
      (void) lun;
      *block_count=0;
      *block_size=0;
      msc_status=3;
    if(!haveStore) return;
    *block_count = 1024;//sdSectorCount;
    *block_size = 512;
  }

  extern "C" int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) 
  {
      (void) lun;
      (void) offset;
    //
      msc_status=4;
    return msc_write_cb(lba, (uint8_t*) buffer, bufsize/512);
  }

  extern "C" int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) 
  {
      (void) lun;
      (void) offset;
    //
      msc_status=5;
    return msc_write_cb(lba, (uint8_t*) buffer, bufsize/512);
  }

  extern "C" int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize) 
  {
      (void) lun;
      (void) buffer;
      (void) bufsize;
      msc_status=6;
    if(!haveStore) return -1;
    for(int ii=0;ii<8;ii++){Serial.print(scsi_cmd[ii]); Serial.print(' ');} Serial.println(); 
    return 0;
  }
#endif