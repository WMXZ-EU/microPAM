#include "SdFat.h"

#include "global.h"
#include "rp2040.h"
#include "RTC.h"

uint32_t t_on = T_ON;     // seconds
uint32_t t_acq = T_ACQ;   // minutes 
uint32_t t_rep = T_REP;   // minutes (for continuous recording set t_rep < t_acq)

// microSD card
#if SDFAT_FILE_TYPE != 3
 #error "SDFAT_FILE_TYPE != 3: edit SdFatConfig.h"
#endif

// definitions
#if defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER)
  // for SPI
  #define SPI_SCK   18
  #define SPI_MOSI  19
  #define SPI_MISO  20
  #define SPI_CS    23


  void spi_init()
  { pinMode(SPI_CS, OUTPUT);
    digitalWrite(SPI_CS,HIGH);
    //
    SPI1.setCS(SPI_CS);
    SPI1.setRX(SPI_MISO);
    SPI1.setTX(SPI_MOSI);
    SPI1.setSCK(SPI_SCK);
  }
  // Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
  #define SD_CONFIG SdSpiConfig(SPI_CS, SHARED_SPI, SD_SCK_MHZ(50),(SpiPort_t *) &SPI1)
#endif

SdFs sd;  // defined in storage_configure
FsFile file;

// Call back for file timestamps.  Only called for file create and sync(). needed by SDFat
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) 
{
    datetime_t t;
    rtcGetDatetime(&t);    

    *date = FS_DATE(t.year,t.month,t.day);
    *time = FS_TIME(t.hour,t.min,t.sec);
    *ms10 = 0;
}

// wav file header
char wav_header[512];
void prep_header(int num_channels, int sampleRate, int bitsPerSample)
{
  memcpy(wav_header,"RIFF",4);
  *(uint32_t *) &wav_header[4]= 512-2*4;

  memcpy(wav_header+8, "WAVE",4);
  memcpy(wav_header+12,"fmt ",4);
  *(uint32_t *) &wav_header[16]= 16;  // length of fmd
  *(uint16_t *) &wav_header[20]= 1;   // PCM

  *(uint16_t *) &wav_header[22]= num_channels;
  *(uint32_t *) &wav_header[24]= sampleRate;
  *(uint32_t *) &wav_header[28]= (sampleRate * num_channels * bitsPerSample/8);
  *(uint16_t *) &wav_header[32]= (num_channels * bitsPerSample/8);
  *(uint16_t *) &wav_header[34]= bitsPerSample;

  memcpy(wav_header+36,"JUNK",4);
  *(uint32_t *) &wav_header[40]= (512-13*4);
  memcpy(wav_header+504,"data",4);
  *(uint32_t *) &wav_header[508]= (0);
}

void update_header(uint32_t nbytes)
{
  *(uint32_t *) &wav_header[4]= nbytes+ 512-2*4;
  *(uint32_t *) &wav_header[508]= nbytes;
}


uint16_t SD_init(void)
{
  getUID();
  //
  spi_init();
  int jj;
  for(jj=0;jj<5;jj++) if (sd.begin(SD_CONFIG)) break; else delay(1000);
  if(jj==5)
  {
    Serial.printf("SD Storage %d failed or missing",SPI_CS);  Serial.println();
    return 0;
  }
  else
  {
    uint64_t totalSize = sd.clusterCount();
    uint64_t freeSize  = sd.freeClusterCount();
    uint32_t clusterSize = sd.bytesPerCluster();
    Serial.printf("Storage %d ",SPI_CS); 
    Serial.print("; total clusters: "); Serial.print(totalSize); 
    Serial.print(" free clusters: "); Serial.print(freeSize);
    Serial.print(" clustersize: "); Serial.print(clusterSize/1024); Serial.println(" kByte");
  }
  FsDateTime::callback = dateTime;
  //
  // signal write acivity by flashing LED
  pinMode(LED_BUILTIN, OUTPUT);

  // prepare wav header (const content)
  prep_header(NCH, FSAMP, MBIT);

  return 1;
}

void SD_stop(void)
{
//https://github.com/greiman/SdFat/issues/401
  sd.card()->syncDevice();
}

// Filing
uint32_t num_bytes_written=0;
char date_str[20];
char time_str[20];
uint32_t old_day=32;
uint32_t old_hour=24;
uint32_t old_time = 0;

extern uint32_t loop_count;
extern uint32_t data_count;

status_t logger(int32_t * buffer,status_t status)
{
  if(status==CLOSED)
  { // open new file
    datetime_t t;
    rtcGetDatetime(&t);
    //
    sprintf(date_str,"%04d%02d%02d",t.year,t.month,t.day);
    sprintf(time_str,"%02d%02d%02d",t.hour,t.min,t.sec);
    //
    if(t.day != old_day)
    { char dirName[40];
      sprintf(dirName,"/%s_%s",uid_str,date_str);
      if(!sd.exists(dirName))
      { sd.mkdir(dirName);
      }
      sd.chdir(dirName);
      old_day=t.day;
    }
    //
    if(t.hour != old_hour)
    { char dirName[40];
      sprintf(dirName,"%02d",t.hour);
      if(!sd.exists(dirName))
      { sd.mkdir(dirName);
      }
      sd.chdir(dirName);        
      old_hour = t.hour;
    }
    //
    char fileName[80];
    sprintf(fileName,"%s_%s_%s.wav",uid_str,date_str,time_str);
    file=sd.open(fileName, FILE_WRITE);
    if(!file)
    { status=STOPPED; 
      return status;
    }
    //
    Serial.print(fileName); Serial.print("; ");
    file.seekSet(512);
    num_bytes_written=0;
    status=RECORDING;
  }
  //
  if((status==RECORDING) || (status==MUST_STOP))
  { // write to disk
    digitalWrite(LED_BUILTIN, HIGH);
    num_bytes_written += file.write(buffer,NBUF_I2S*4);
    digitalWrite(LED_BUILTIN, LOW);
    
    // check to close file
    uint32_t tt = rtc_get();
    uint32_t tmp_time=(tt % t_on );
    if((tmp_time < old_time) || (status == MUST_STOP))
    {
      // create header for WAV file and write to SD card
      update_header(num_bytes_written);

      uint64_t fpos;
      fpos = file.curPosition();
      //Serial.printf(" fpos=%d ",fpos);
      file.seekSet(0);
      file.write((const uint8_t*)wav_header,512);
      file.seekSet(fpos);

      file.close();
      //
      uint32_t num_samples = num_bytes_written / (4 * NCH);
      if(Serial)
      {
        Serial.printf("\t%d %d %d \t%x\n", num_samples, data_count, loop_count, buffer[0]);
        data_count = 0;
        loop_count = 0;
      }
      //
      // check for stopping or hibernation
      if(status == MUST_STOP)
      { Serial.print(" stopped ");
        status = STOPPED;
      }
      else
      {
        status = CLOSED;
        //
        if(t_rep>t_acq)                      // if foreseen  check for hibernation
        { uint32_t ttm=tt/60;
          //Serial.printf("%d %d %d %d\n",t_acq,t_rep,ttm,(ttm % t_rep));

          uint32_t dt2 = (ttm % t_rep);
          if(dt2>=t_acq) 
          {
            uint32_t alarm=((ttm/t_rep)+1)*t_rep*60;
            hibernate_until(alarm);
          }
        }
      }
    }
    old_time = tmp_time;
  }
  return status;
}
