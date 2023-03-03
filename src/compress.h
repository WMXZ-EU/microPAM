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
 
#ifndef _COMPRESS_H
#define _COMPRESS_H

#include "Arduino.h"
#include "AudioStream.h"

class AudioCompress: public AudioStream
{
public:
  AudioCompress() : AudioStream(1, inputQueueArray) { }
  virtual void update(void);
  uint16_t pushData(uint16_t * src);
  int compress(void *inp);
private:
  audio_block_t *inputQueueArray[1];
};

void AudioCompress::update(void)
{ int ret=0;
  audio_block_t *block_in;
  block_in=receiveReadOnly(0);
  if(block_in) 
  {
    ret=compress(block_in->data);
    release(block_in);
    (void) ret;
  }
}

static uint16_t tempData[NSAMP];
static uint16_t outData[NSAMP];
static uint16_t dout[NSAMP];

int16_t *tempDatai=(int16_t*) tempData;

uint32_t proc_stat[16];

uint16_t AudioCompress::pushData(uint16_t * src)
{ audio_block_t *outputBlock;
  outputBlock=allocate();
  if(outputBlock)
  {
    memcpy(outputBlock->data,src,2*NSAMP);
    transmit(outputBlock);
    release(outputBlock);
    return 1;
  }
  return 0;
}

int AudioCompress::compress(void *inp)
{   
  int ret=1;
  uint32_t to = rtc_get();
  uint32_t t1 = micros();

    static int nout=0;

    int16_t *din = (int16_t *) inp;
    //
    // copy reference (first sample of all channels)
  for (int  ii = 0; ii < 1; ii++) tempDatai[ii] = din[ii];
        
  //differentiate (equiv 6 dB/Octave HP filter)
  for (int  ii = 1; ii < NSAMP; ii++) tempDatai[ii] = (din[ii] - din[ii - 1]);

  // find maximum in filtered data
  int32_t mx = 0;
  for (int ii = 1; ii < NSAMP; ii++)
  {
    int16_t dd =  tempDatai[ii];
    if(dd<0)  dd = -dd;
    if(dd>mx) mx = dd;
  }

  // estimate mask (allow only 'nice' values)
    int nb;
    for(nb=2; nb<16; nb++) if(mx < (1<<(nb-1))) break;
    // compression factor (16/nb)

    proc_stat[nb-1]++;

  // mask data (all but first sample) (mask needed for negative numbers)
  uint32_t msk = (1 << nb) - 1;
  for (int ii = 1; ii < NSAMP; ii++) { tempData[ii] &= (uint16_t)msk; }

  // pack all data
  int ncmp = (NSAMP*nb) / 16L;
  int ndat = 12+1 + ncmp;

    // clean data store
  for (int ii = 0; ii < NSAMP; ii++) outData[ii]=0;

  // prepare header
  uint32_t *iptr=(uint32_t *) outData;
  *iptr++ = 0xA5A5A5A5;
  *iptr++ = nb | shift<<16;
  *iptr++ = to;
  *iptr++ = t1;
  *iptr++ = 1;
  *iptr++ = 1+ncmp; // number of data after header
  //
  int kk=12;
  outData[kk++] = tempData[0]; tempData[0] = 0;

  // pack data
    // 
    int nx = 16;
    for (int ii = 0; ii < NSAMP; ii ++)
    {   nx -= nb;
        if(nx > 0)
        {   outData[kk] |= (tempData[ii] << nx);
        }
        else if(nx==0) 
        {   outData[kk++] |= tempData[ii];
            nx=16;
        } 
        else    // nx is < 0
        {   outData[kk++] |= (tempData[ii] >> (-nx));
            nx += 16;
            outData[kk] = (tempData[ii] << nx);
        }
    }
       
    // store actual data
    if ((nout + ndat) <= NSAMP)
    { // all data fit in current block
        for (int ii = 0; ii < ndat; ii++) dout[nout++] = outData[ii];
        //
    }
    else if ((nout + 12) > NSAMP) //avoid partial header (special case)
    {
        while(nout<NSAMP) dout[nout++] = 0; // fill rest of block with zero
        // store data
        if(!pushData(dout)) ret = 0;
        //
        // store data in next block
        nout=0;
        for (int ii=0; ii < ndat; ii++) dout[nout++] = outData[ii];
    }
    else
    { // data crosses two blocks
        int ii=0;
        int nr = NSAMP-nout;  //remaining data
        uint32_t *iptr = (uint32_t *) outData;
        iptr[5] = (iptr[5]<<16) | (nr-6);  //orig remaining data | actual remaining data after header 

        while (nout < NSAMP ) dout[nout++] = outData[ii++];
        // store data
        if(!pushData(dout)) ret = 0;
        //
        // store rest in next block
        nr=ndat-ii;
        // add blockHeader continuation
        iptr[5]=(iptr[5] & 0xffff0000) | nr; //orig remaining data | actual remaining data after header

        for(nout=0;nout<12;nout++) dout[nout] = outData[nout];
        while (ii < ndat) dout[nout++] = outData[ii++];
    }
    return ret;
}

#endif
