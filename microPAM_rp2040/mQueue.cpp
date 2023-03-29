#include <stdint.h>
#include <string.h>

#include "mQueue.h"

#if 1
  static uint32_t data_buffer[MAXBUF][NBUF_ACQ];
  static int head=0;
  static int tail=0;
  
  bool full(void)  { return (tail+1)%MAXBUF == head; }
  bool empty(void) { return head==tail; }
  uint16_t getDataCount(void) { int num = tail-head; return num<0 ? num+MAXBUF : num; }
  
  
  uint16_t pushData(uint32_t *data)
  {
    if ( full() ) return 0;
    memcpy(data_buffer[tail],data,4*NBUF_ACQ);
    tail = (tail+1)%MAXBUF;
    return 1; // signal success.
  }
  
  uint16_t pullData(uint32_t *data)
  {
    if ( empty() ) return 0;
    memcpy(data,data_buffer[head],4*NBUF_ACQ);
    head = (head+1)%MAXBUF;
    return 1;
  }

#else 

static uint32_t data_buffer[MAXBUF*NBUF_ACQ];

    /**
     * @brief Data storage class
     * 
     */
    class Data
    {
        public:
            Data(uint32_t * data) 
            { /**
             * @brief Constructor
             * @param data is pointer to data store
             * 
             */
                data_buffer=data; front_=0; rear_= MAXBUF;
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
    };

    Data rawData(data_buffer);

    uint16_t getDataCount () { return rawData.getCount(); }
    uint16_t pushData(uint32_t * src){ return rawData.push(src);}

    uint16_t pullData(uint32_t * dst) {return rawData.pull(dst);}
    uint16_t pullData(uint32_t * dst, uint32_t ndbl) {return rawData.pull(dst,ndbl);}
#endif