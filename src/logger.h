
#include <TimeLib.h>
#include <SD.h>
#include <SdFat.h>


enum STATUS
{
    STOPPED=-1,
    CLOSED=0,
    OPENED=1,
    RUNNING=2,
    DOCLOSE=3,
    MUST_STOP=4
};

int32_t day_   = 0;
int16_t newDay(void)
{   uint32_t tx=rtc_get();
    int32_t d_ = (int32_t)tx/SECS_PER_DAY; // use days since 1970 as measure
    if(day_== d_) return 0;
    day_ = d_;
    return 1;
}

int32_t hour_ = 0;
int16_t newHour(void)
{   uint32_t tx=rtc_get();
    int32_t h_ = (int32_t) tx/SECS_PER_HOUR;  // use hours since 1970 as measures
    if(hour_== h_) return 0;
    hour_ = h_;
    return 1;
}

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
  uint32_t getTeensySerial(void) 
  {
    uint32_t num;
    num = HW_OCOTP_MAC0 & 0xFFFFFF;
    return num;
  }
#else
  uint32_t getTeensySerial(void)  { return 0;}
#endif

#define MIN_SPACE 1000
//SDClass *msd;
File file=NULL; 

const char *DirPrefix = "D";
const char *FilePrefix = "F";
uint32_t disk_count;

#define NBUF 10
char diskBuffer[NBUF*2*128];

static SDClass *checkDiskSpace(void)
{
    if(SD.sdfs.freeClusterCount() < MIN_SPACE) return &SD; else return 0;
}

static int16_t newDirectory(char *dirName)
{ if(newHour())
    {   uint32_t SerNum=getTeensySerial();
        tmElements_t tm;
        breakTime(rtc_get(), tm);
        sprintf(dirName, "/%s%06x_%04d%02d%02d/%02d/", 
                      DirPrefix,(unsigned int)SerNum,
                              tmYearToCalendar(tm.Year),tm.Month, tm.Day, tm.Hour);
        //
        Serial.print("\n"); Serial.print(dirName);
        return 1;
    }
    return 0;

}
static int16_t newFileName(char *fileName)
{
    tmElements_t tm;
    breakTime(rtc_get(), tm);
    sprintf(fileName, "%s_%02d%02d%02d.bin", FilePrefix, tm.Hour, tm.Minute, tm.Second);
    //
    Serial.print("\n"); Serial.print(": ");Serial.print(fileName);
    return 1;
}

static int16_t makeHeader(char *header)
{
    /**
     * @brief Make file header
     * @param header is pointer to header
     * 
     */
    #define MAGIC "WMXZ"
    tmElements_t tm;
    breakTime(rtc_get(), tm);

    int nd=sprintf(header,"%s%04d%02d%02d_%02d%02d%02d",
            MAGIC,tmYearToCalendar(tm.Year),tm.Month,tm.Day,tm.Hour,tm.Minute,tm.Second);
    char *ptr = header+(nd+1);

    int32_t *iptr = (int32_t *) ptr;
    // to be filled in

    uint32_t *uptr = (uint32_t*) header;
    uptr[127] = 0x55555555;
    //
    return 1;
}

int16_t saveData(int16_t status)
{   static char dirName[80];
    static char fileName[80];
    static char header[512];

    if(status<CLOSED) return status; // we are stopped: don't do anything

    if(queue1.available() <NBUF) return status; // must wait for data

    for(int ii=0; ii<NBUF; ii++)
    {
      memcpy(diskBuffer+ii*256, queue1.readBuffer(), 256);
      queue1.freeBuffer();
    }
    return status;
    do
    {   disk_count++;
        if(status==CLOSED) // file closed: should open
        {   //if(!(msd=checkDiskSpace())) return MUST_STOP;
            //
            if(newDirectory(dirName)) 
            {   if( !SD.sdfs.exists(dirName) && 
                    !SD.sdfs.mkdir(dirName) && 
                    !SD.sdfs.chdir(dirName)) { Serial.println("Error");status= MUST_STOP; while(1);}
            }
            //
            if(newFileName(fileName))
            {   
                file = SD.open(fileName, FILE_WRITE_BEGIN); 
                if(file) 
                    status = OPENED; 
                else 
                {   Serial.println("Failing open file");
                    status= MUST_STOP; 
                    break;
                }
            } 
            else
            { Serial.println("Failing new file");
               status= MUST_STOP; // if file open fails: don't do anything
               break;
            }
        }
        //
        if(status==OPENED) // file is open: write first record (header)
        {   makeHeader(header);
            if(file.write((const void*)header,512) < 512) status= MUST_STOP; else status=2;
        }
        //
        if(status>=RUNNING) // file is open, header written: store data records
        {   int32_t nbuf=sizeof(diskBuffer);
            if(file.write((const void *)diskBuffer, nbuf) < nbuf) status= MUST_STOP;
        }
    } while(0);
    
    // following is done independent of data availability
    if(status==DOCLOSE) // should close file
    {
        // writes are done, so enable again transaction activations
        file.flush();
        file.close();
        status = CLOSED;
    }
    if(status==MUST_STOP) // should close file and stop
    {   
        file.flush();
        file.close();
        status = STOPPED;
    }
    return status;
}
