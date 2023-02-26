#ifndef _LOGGER_H
#define _LOGGER_H

#include <TimeLib.h>
#include <SD.h>
#include <SdFat.h>
    /**
     * @brief Circular Buffer definitions
     * 
     */
    #define MAXBUF 100
    #define NBUF_ACQ (NCHAN_ACQ*AUDIO_BLOCK_SAMPLES)
    uint32_t data_buffer[MAXBUF*NBUF_ACQ];
    uint32_t to_buffer[MAXBUF];
    uint32_t t1_buffer[MAXBUF];

    /**
     * @brief Data storage class
     * 
     */
    class Data
    {
        public:
            Data(uint32_t * data, uint32_t *to, uint32_t *t1) 
            { /**
             * @brief Constructor
             * @param data is pointer to data store
             * @param to is pointer to rtc time
             * @param t1 is pointer to micros
             * 
             */
                data_buffer=data; to_buffer=to; t1_buffer=t1; front_=0; rear_= MAXBUF;
            }

            uint16_t push(uint32_t * src)
            { 
                /** 
                 * @brief push data to storage
                 * @param src is pointer to data block
                 */
                uint16_t f =front_ ;
                if(f == rear_) return 0;

                uint32_t *ptr= data_buffer+f*NBUF_ACQ;
                memcpy(ptr,src,NBUF_ACQ*4);

                if(++f==MAXBUF) f=0;
                front_ = f;
                
                return 1;
            }
            uint16_t push(uint32_t to, uint32_t t1, uint32_t * src)
            { 
                /** 
                 * @brief push data to storage
                 * @param src is pointer to data block
                 */
                uint16_t f =front_ ;
                if(f == rear_) return 0;

                uint32_t *ptr= data_buffer+f*NBUF_ACQ;
                memcpy(ptr,src,NBUF_ACQ*4);
                to_buffer[f]=to;
                t1_buffer[f]=t1;

                if(++f==MAXBUF) f=0;
                front_ = f;
                
                return 1;
            }

            uint16_t pull(uint32_t * dst, uint32_t ndbl)
            {   
                /** 
                 * @brief pull data from storage
                 * @param dst is pointer to data blocks
                 * @param ndbl is number of data blocks
                 */
                uint16_t r = rear_ +1;
                if(r >= MAXBUF) r=0;
                if(r/ndbl == front_/ndbl) return 0;
                
                uint32_t *ptr= data_buffer + r*NBUF_ACQ;
                memcpy(dst,ptr,ndbl*NBUF_ACQ*4);

                r+=(ndbl-1);
                rear_ = r;
                return 1;
            }
            uint16_t pull(uint32_t * dst)
            {   
                /** 
                 * @brief pull data from storage
                 * @param dst is pointer to data blocks
                 */
                uint16_t r = rear_ +1;
                if(r >= MAXBUF) r=0;
                if(r == front_) return 0;
                
                uint32_t *ptr= data_buffer + r*NBUF_ACQ;
                memcpy(dst,ptr,NBUF_ACQ*4);

                rear_ = r;
                return 1;
            }
            uint16_t pull(uint32_t *to, uint32_t *t1, uint32_t * dst)
            {   
                /** 
                 * @brief pull data from storage
                 * @param dst is pointer to data blocks
                 */
                uint16_t r = rear_ +1;
                if(r >= MAXBUF) r=0;
                if(r == front_) return 0;
                
                uint32_t *ptr= data_buffer + r*NBUF_ACQ;
                memcpy(dst,ptr,NBUF_ACQ*4);
                *to=to_buffer[r];
                *t1=t1_buffer[r];

                rear_ = r;
                return 1;
            }

            uint16_t getCount () 
            {  
                /**
                 * @brief get number of data blocks in storage
                 * 
                 */
                if(front_ > rear_) return (front_ - rear_); 
                return (front_+ MAXBUF -rear_); 
            }

    private:    
        uint16_t front_, rear_;
        uint32_t *data_buffer;
        uint32_t *to_buffer, *t1_buffer;
    };

    Data rawData(data_buffer,to_buffer,t1_buffer);

    uint16_t getCount () { return rawData.getCount(); }
    uint16_t pushData(uint32_t * src){ return rawData.push(src);}
    uint16_t pushData(uint32_t to, uint32_t t1, uint32_t * src){ return rawData.push(to,t1,src);}

    uint16_t pullData(uint32_t * dst) {return rawData.pull(dst);}
    uint16_t pullData(uint32_t * dst, uint32_t ndbl) {return rawData.pull(dst,ndbl);}
    uint16_t pullData(uint32_t *to, uint32_t *t1, uint32_t * src){ return rawData.pull(to,t1,src);}



enum STATUS
{
    STOPPED=-1,
    CLOSED=0,
    OPENED=1,
    RUNNING=2,
    DOCLOSE=3,
    MUST_STOP=4
};


#define MIN_SPACE 1000
File file=NULL; 

const char *DirPrefix = "D";
const char *FilePrefix = "F";
uint32_t disk_count;

#define NBLK 10
uint32_t diskBuffer[NBLK*NBUF_ACQ];

extern uint32_t SerNum;
extern int32_t fsamp;
extern int32_t t_acq,t_on,t_off;
extern int32_t nch;


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

static int16_t checkDiskSpace(void)
{
    return SD.sdfs.freeClusterCount() < MIN_SPACE;
}


static int16_t newDirectory(char *dirName)
{ if(newHour())
    {   SerNum=getTeensySerial();
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
    //float *fptr = (float *) ptr;
    
    // to be filled in
    iptr[0] = 1;                    // SW version
    iptr[1] = (int32_t)SerNum;      // serial number
    iptr[2] = fsamp;
    iptr[3] = nch;
    iptr[4] = t_acq;
    iptr[5] = t_on;
    iptr[6] = t_off;

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

    return status;

    //fetch data from circular buffer
    if(pullData(diskBuffer,NBLK)) do
    {   disk_count++;
        if(status==CLOSED) // file closed: should open
        {   if(!checkDiskSpace()) return MUST_STOP;
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
            if(file.write((const void *)diskBuffer, nbuf) < (size_t) nbuf) status= MUST_STOP;
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
#endif
