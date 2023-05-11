/* microPAM 
 * Copyright (c) 2023, Walter Zimmer
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

#include <SdFat.h>

#include "mConfig.h"
#include "mQueue.h"
#include "mAcq.h"
#include "mRTC.h"
#include "mFiling.h"

volatile uint32_t t_acq=600;
volatile uint32_t t_on=600;
volatile uint32_t t_off=0;

#if SDFAT_FILE_TYPE != 3
 #error "SDFAT_FILE_TYPE != 3: edit SdFatConfig.h"
#endif

#if defined(TARGET_RP2040)
  const int _MOSI = 3;
  const int _MISO = 4;
  const int _CS   = 5;
  const int _SCK  = 6;

  #if ENABLE_DEDICATED_SPI !=1
  #error "ENABLE_DEDICATED_SPI !=1: edit SdFatConfig.h"
  #endif

  // Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
  #define SD_CONFIG SdSpiConfig(_CS, DEDICATED_SPI, SD_SCK_MHZ(25))

  extern "C" void flash_get_unique_id(uint8_t *p);
  uint32_t UniqueID[2];

#elif defined(__IMXRT1062__)
  #define SD_CONFIG SdioConfig(FIFO_SDIO)
#endif

SdFs sd;
FsFile file;

static HdrStruct wav_hdr;

volatile int haveStore=0;
uint32_t diskSpace=0;
uint32_t diskSize=0;

#define NDBL 8
#define MAX_DISK_BUFFER (NDBL*NBUF_ACQ)
static uint32_t diskBuffer[MAX_DISK_BUFFER];
uint32_t disk_count=0;

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

int16_t filing_init(void)
{
  #if defined(TARGET_RP2040)
    SPI.setRX(_MISO);
    SPI.setTX(_MOSI);
    SPI.setSCK(_SCK);
    SPI.setCS(_CS);

#    FsDateTime::callback = dateTime;

    flash_get_unique_id((uint8_t *) UniqueID);
    SerNum=UniqueID[1];
  #else
    SerNum = HW_OCOTP_MAC0 & 0xFFFFFF;
  #endif

  FsDateTime::callback = dateTime;

  for(int ii=0; ii<5;ii++)
  {
    if (sd.begin(SD_CONFIG)) 
    { Serial.println("card initialized.");
      haveStore=1;
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
}

char * timeStamp(void)
{
  static char date_time[80];
  
  datetime_t t;
  rtc_get_datetime(&t);

  sprintf(date_time,"%04d%02d%02d_%02d%02d%02d",t.year,t.month,t.day,t.hour,t.min,t.sec);
  return date_time;
}

char * headerInit(int32_t fsamp, int32_t nchan, int32_t nbits, int serNum)
{
  int nbytes=nbits/8;

  #define MAGIC "WMXZ"

  strcpy(wav_hdr.rId,"RIFF");
  strcpy(wav_hdr.wId,"WAVE");
  strcpy(wav_hdr.fId,"fmt ");
  strcpy(wav_hdr.dId,"data");
  strcpy(wav_hdr.iId,"info");

  wav_hdr.rLen=512-2*4; // will be updated at closing
  wav_hdr.fLen=0x10;
  wav_hdr.iLen = 512 - 13*4;
  wav_hdr.dLen = 0; // will be updated at closing

  wav_hdr.nFormatTag=1;
  wav_hdr.nChannels=1;
  wav_hdr.nSamplesPerSec=fsamp;
  wav_hdr.nAvgBytesPerSec=fsamp*nbytes;
  wav_hdr.nBlockAlign=nchan*nbytes;
  wav_hdr.nBitsPerSamples=nbits;

  strcpy(&wav_hdr.info[0], MAGIC);
  strcpy(&wav_hdr.info[4], timeStamp());
  sprintf(&wav_hdr.info[20]," %8x",serNum);
  strcpy(&wav_hdr.info[30]," end");

  return (char *)&wav_hdr;
}

char * headerUpdate(int32_t nbytes)
{
  wav_hdr.dLen = nbytes;
  wav_hdr.rLen += wav_hdr.dLen;
  return (char *)&wav_hdr;
}

void writeHeader(char * wav_hdr) 
{ 
  uint32_t fpos;
  fpos = file.curPosition();
  Serial.printf(" fpos=%d ",fpos);
  file.seek(0);
  file.write(wav_hdr,512);
  file.seek(fpos);
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
  *(int16_t*)(wheader+22)=nchan;// numChannels 
  *(int32_t*)(wheader+24)= fsamp; // sample rate 
  *(int32_t*)(wheader+28)= fsamp*nbytes; // byte rate
  *(int16_t*)(wheader+32)=nchan*nbytes; // block align
  *(int16_t*)(wheader+34)=nbits; // bits per sample 
  *(int32_t*)(wheader+40)=nsamp*nchan*nbytes; 
  *(int32_t*)(wheader+4)=36+nsamp*nchan*nbytes; 

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
    #define MAGIC "WMXZ"
    datetime_t t;
    rtc_get_datetime(&t);

    sprintf((char *)header,"%s%04d%02d%02d_%02d%02d%02d",
            MAGIC,t.year,t.month,t.day,t.hour,t.min,t.sec);

    header[5] = 20;          // SW version
    header[6] = SerNum;      // serial number
    header[7] = fsamp;
    header[8] = NCH;
    header[9] = t_acq;
    header[10] = t_on;
    header[11] = t_off;
    header[12] = proc;
    header[13] = shift;

    header[127]=0x55555555;
    return 1;
}

int16_t checkEndOfFile(int16_t status)
{ static volatile uint32_t tx_=0;
  uint32_t tx;
  tx=rtc_get();
  tx = tx % t_acq;
  if((status>OPENED) && (tx_>0) && (tx < tx_)) status=DOCLOSE;
  tx_=tx;
  return status;  
}

uint16_t checkDiskSpace(void)
{   if((diskSize>0) && ((diskSpace=sd.freeClusterCount()) > MIN_SPACE)) return 1;
    return 0;
}

uint16_t newHour(int h)
{ static int ho=0;
  if(h==ho) return 0;
  ho=h;
  return 1;
}

int16_t newDirectory(char *dirName)
{   
    datetime_t t;
    rtc_get_datetime(&t);

    if(newHour(t.hour))
    {   
        sprintf(dirName, "/%s%06x_%04d%02d%02d/%02d/", 
            DirPrefix,(unsigned int)SerNum, t.year,t.month,t.day,t.hour);
        //
        Serial.println(); Serial.print(": "); Serial.print(dirName);
        return 1;   // have new directory
    }
    return 0;       // keep old directory

}

int16_t newFileName(char *fileName)
{
    datetime_t t;
    rtc_get_datetime(&t);
    if(proc==0)
      sprintf(fileName, "%s_%02d%02d%02d.wav", FilePrefix, t.hour,t.min,t.sec);
    else
      sprintf(fileName, "%s_%02d%02d%02d.bin", FilePrefix, t.hour,t.min,t.sec);
    //
    Serial.println(); Serial.print(": "); Serial.print(fileName);
    return 1;
}

static char dirName[80];
static char fileName[80];
static int32_t fileHeader[128];
uint32_t nbuf;

/**************** main data filing routine ************************/
int16_t storeData(int16_t status)
{
    if(status==CLOSED) // file closed: should open
    {   //doTransactions(true);
        //if(!checkDiskSpace()) {return STOPPED;}
        //
        if(newDirectory(dirName))
        {   if(!sd.exists(dirName) && !sd.mkdir(dirName)) {Serial.println("Error mkdir"); return STOPPED;}         
            if(!sd.chdir(dirName)) {Serial.println("Error chdir"); return STOPPED;}
        }
        
        if(newFileName(fileName))
        {   
            file = sd.open(fileName, FILE_WRITE); 
            if(file) 
            { status = OPENED; 
            }
            else 
            {   Serial.println("Failing open file");
                return STOPPED; 
            }
        } 
        else
        {  return STOPPED; // if file open fails: don't do anything
        }
    }
    //
    if(status==OPENED) // file is open: write first record (header)
    {   nbuf=0;
        char *hdr=0;
        if(proc==0)
        { int nchan=NBUF_ACQ;
          int nbits = 32;

          hdr = headerInit(fsamp, nchan, nbits, SerNum);
        }  
        else
        {
          makeHeader(fileHeader);
          hdr=(char *)fileHeader;
        }
        if(file.write(hdr,512) < 512) 
        { status = DOCLOSE;
        } 
        else status=RUNNING;
    }
    //
    if(status==RUNNING) // file is open, header written: store data records
    {   uint32_t nd;
        if((nd=file.write((const void *)diskBuffer,4*MAX_DISK_BUFFER)) < 4*MAX_DISK_BUFFER) 
        { Serial.print(">"); Serial.print(nd); Serial.print(" "); Serial.println(status); status=DOCLOSE; }
        else
          nbuf++;
        disk_count++;
        if((nbuf % 1000)==0 ) file.flush();
    }    

    // following is done independent of data availability
    if((status==DOCLOSE) || (status==MUSTSTOP)) // should close file or stop acquisition
    {   if(file)
        {   if(proc==0)
            {
              char *hdr = headerUpdate(nbuf*MAX_DISK_BUFFER*4);
              writeHeader(hdr);

            }
            file.close();
        }
        status= (status==DOCLOSE)? CLOSED : STOPPED;
    }
    return status;
}
//
volatile uint32_t logBuffer[8];
int16_t saveData(int16_t status)
{
    if(status==STOPPED) 
    { 
      pullData(diskBuffer);
      for(int ii=0;ii<8;ii++) logBuffer[ii]=diskBuffer[ii];
      digitalWriteFast(13,HIGH);
    }
    else
      digitalWriteFast(13,LOW);

    if(status<CLOSED) return status; // we are stopped: don't do anything

    status=checkEndOfFile(status);

    if(getDataCount()>=NDBL)
    { 
      digitalWriteFast(13,HIGH);
      for(int ii=0; ii<NDBL; ii++) pullData(&diskBuffer[ii*NBUF_ACQ]);
      for(int ii=0;ii<8;ii++) logBuffer[ii]=diskBuffer[ii];
      if(haveStore)
        status=storeData(status);
    }
    else
      digitalWriteFast(13,LOW);

    return status;
}
