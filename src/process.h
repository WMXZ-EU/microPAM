#ifndef _PROCESS_H
#define _PROCESS_H

#include "logger.h"

uint16_t pushData(uint32_t * src);
uint16_t pushData(uint32_t to, uint32_t t1, uint32_t * src);

int compress(void *inp);

extern int32_t proc;
extern int32_t shift;

uint32_t acq_miss;

void process(void *src)
{   // dispatch processing 
    // is called from acq interrupt
    // so must be real time

  	uint32_t to = rtc_get();
	uint32_t t1 = micros();

    //
    switch(proc)
    {
        case 0: 
            if(!pushData(to,t1,(uint32_t *)src)) acq_miss++;
            break;
        case 1:
            if(!compress(src)) acq_miss++;
            break;
        default:
            break;
    }
}

/********************************** Integer compression *****************************************/
static uint32_t tempData[NBUF_ACQ];
static uint32_t outData[NBUF_ACQ];
static uint32_t dout[NBUF_ACQ];

static int32_t *tempDatai=(int32_t*) tempData;

uint32_t proc_stat[24];

int compress(void *inp)
{   
    int ret=1;
	uint32_t to = rtc_get();
	uint32_t t1 = micros();

    static int nout=0;

    int32_t *din = (int32_t *) inp;
    //
    // copy reference (first sample of all channels)
	for (int  ii = 0; ii < NCHAN_ACQ; ii++) 
        tempDatai[ii] = din[ii];
        
	//differentiate (equiv 6 dB/Octave HP filter)
	for (int  ii = NCHAN_ACQ; ii < NBUF_ACQ; ii++) 
        tempDatai[ii] = (din[ii] - din[ii - NCHAN_ACQ]);

	// find maximum in filtered data
	int32_t mx = 0;
	for (int ii = NCHAN_ACQ; ii < NBUF_ACQ; ii++)
	{
		int32_t dd =  tempDatai[ii];
		if(dd<0)  dd = -dd;
		if(dd>mx) mx = dd;
	}

	// estimate mask (allow only 'nice' values)
    int nb;
    for(nb=2; nb<24; nb++) if(mx < (1<<(nb-1))) break;
    // compression factor (32/nb)

    #if TEST > 0
        if(nb<24) nb=24;
    #endif

    proc_stat[nb-1]++;

	// mask data (all but first sample) (mask needed for negative numbers)
	uint32_t msk = (1 << nb) - 1;
	for (int ii = NCHAN_ACQ; ii < NBUF_ACQ; ii++) {	tempData[ii] &= msk; }

	// pack all data
	int ncmp = (NBUF_ACQ*nb) / 32L;
	int ndat = 12 + ncmp;

    // clean data store
	for (int ii = 0; ii < NBUF_ACQ; ii++) outData[ii]=0;

	// prepare header
	int kk = 0;
	outData[kk++] = 0xA5A5A5A5;
	outData[kk++] = nb | shift<<16;
	outData[kk++] = to;
	outData[kk++] = t1;
	outData[kk++] = NCHAN_ACQ;
	outData[kk++] = ncmp+NCHAN_ACQ; // number of data after header
	//
	outData[kk++] = tempData[0]; tempData[0] = 0;
	outData[kk++] = tempData[1]; tempData[1] = 0;
	outData[kk++] = tempData[2]; tempData[2] = 0;
	outData[kk++] = tempData[3]; tempData[3] = 0;
    #if NCHAN_ACQ>4
        outData[kk++] = tempData[4]; tempData[4] = 0;
        outData[kk++] = tempData[5]; tempData[5] = 0;
    #endif
    #if NCHAN_ACQ>6
        outData[kk++] = tempData[6]; tempData[6] = 0;
        outData[kk++] = tempData[7]; tempData[7] = 0;
    #endif

	// pack data
    // 
    int nx = 32;
    for (int ii = 0; ii < NBUF_ACQ; ii ++)
    {   nx -= nb;
        if(nx > 0)
        {   outData[kk] |= (tempData[ii] << nx);
        }
        else if(nx==0) 
        {   outData[kk++] |= tempData[ii];
            nx=32;
        } 
        else    // nx is < 0
        {   outData[kk++] |= (tempData[ii] >> (-nx));
            nx += 32;
            outData[kk] = (tempData[ii] << nx);
        }
    }
       
    // store actual data
    if ((nout + ndat) <= NBUF_ACQ)
    {	// all data fit in current block
        for (int ii = 0; ii < ndat; ii++) dout[nout++] = outData[ii];
        //
    }
    else if ((nout + 6) > NBUF_ACQ) //avoid partial header (special case)
    {
        while(nout<NBUF_ACQ) dout[nout++] = 0; // fill rest of block with zero
        // store data
        if(!pushData(dout)) ret = 0;
        //
        // store data in next block
        nout=0;
        for (int ii=0; ii < ndat; ii++) dout[nout++] = outData[ii];
    }
    else
    {	// data crosses two blocks
        int ii=0;
        int nr = NBUF_ACQ-nout;  //remaining data
        outData[5] = (outData[5]<<16) | (nr-6);  //orig remaining data | actual remaining data after header 

        while (nout < NBUF_ACQ ) dout[nout++] = outData[ii++];
        // store data
        if(!pushData(dout)) ret = 0;
        //
        // store rest in next block
        nr=ndat-ii;
        // add blockHeader continuation
        dout[0]=outData[0];
        dout[1]=outData[1];
        dout[2]=outData[2];
        dout[3]=outData[3];
        dout[4]=outData[4];
        dout[5]=(outData[5] & 0xffff0000) | nr; //orig remaining data | actual remaining data after header

        nout=6;
        while (ii < ndat) dout[nout++] = outData[ii++];
    }
    return ret;
}
#endif
