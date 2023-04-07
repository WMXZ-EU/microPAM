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

#include "Arduino.h"

#include "mConfig.h"
#include "mQueue.h"
#include "mCompress.h"
#include "mAcq.h"

#ifndef NBUF_ACQ        // should be defined in config.h
  #define NBUF_ACQ 128
#endif

#ifndef NBUF_I2S        // should be defined in config.h
  #define NBUF_I2S (2*NBUF_ACQ) //for stereo I2S
#endif

uint32_t procCount=0;
uint32_t procMiss=0;
int32_t acqBuffer[NBUF_ACQ]; 

int32_t fsamp=FSAMP;
int16_t shift=SHIFT;
int16_t proc=PROC_MODE;
static void process(int32_t * buffer);

/*======================================================================================*/
#if defined(TARGET_RP2040)

  pin_size_t _pinDOUT =0;
  pin_size_t _pinBCLK =1;

  int _freq=fsamp;
  int _bps =MBIT;
  int off=0;

  PIOProgram *_i2s;
  PIO _pio;
  int _sm;

  #include "hardware/pio.h"

  #define pio_i2s_in_wrap_target 0
  #define pio_i2s_in_wrap 7

  static const uint16_t pio_i2s_in_program_instructions[] = {
              //     .wrap_target
      0xa022, //  0: mov    x, y            side 0     
      0x4801, //  1: in     pins, 1         side 1     
      0x0041, //  2: jmp    x--, 1          side 0     
      0x5801, //  3: in     pins, 1         side 3     
      0xb022, //  4: mov    x, y            side 2     
      0x5801, //  5: in     pins, 1         side 3     
      0x1045, //  6: jmp    x--, 5          side 2     
      0x4801, //  7: in     pins, 1         side 1     
              //     .wrap
  };

  static const struct pio_program pio_i2s_in_program = {
      .instructions = pio_i2s_in_program_instructions,
      .length = 8,
      .origin = -1,
  };

  static inline pio_sm_config pio_i2s_in_program_get_default_config(uint offset) {
      pio_sm_config c = pio_get_default_sm_config();
      sm_config_set_wrap(&c, offset + pio_i2s_in_wrap_target, offset + pio_i2s_in_wrap);
      sm_config_set_sideset(&c, 2, false, false);
      return c;
  }

  static inline void pio_i2s_in_program_init(PIO pio, uint sm, uint offset, uint data_pin, uint clock_pin_base, uint bits) {
      pio_gpio_init(pio, data_pin);
      pio_gpio_init(pio, clock_pin_base);
      pio_gpio_init(pio, clock_pin_base + 1);
      pio_sm_config sm_config = pio_i2s_in_program_get_default_config(offset);
      sm_config_set_in_pins(&sm_config, data_pin);
      sm_config_set_sideset_pins(&sm_config, clock_pin_base);
      sm_config_set_in_shift(&sm_config, false, true, (bits <= 16) ? 2 * bits : bits);
      sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_RX);
      pio_sm_init(pio, sm, offset, &sm_config);
      uint pin_mask = 3u << clock_pin_base;
      pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);
      pio_sm_set_pins(pio, sm, 0); // clear pins
      pio_sm_exec(pio, sm, pio_encode_set(pio_y, bits - 2));
  }

  void i2s_setup(void)
  {
    float bitClk = _freq * _bps * 2.0 /* channels */ * 2.0 /* edges per clock */;

    _i2s = new PIOProgram( &pio_i2s_in_program);
    _i2s->prepare(&_pio, &_sm, &off);

    pio_i2s_in_program_init(_pio, _sm, off, _pinDOUT, _pinBCLK, _bps);
    pio_sm_set_clkdiv(_pio, _sm, (float)clock_get_hz(clk_sys) / bitClk);
    pio_sm_set_enabled(_pio, _sm, true);
  }
  /***************************************************************************/
  #include "hardware/dma.h"

  int _channelDMA[2];
  int32_t i2s_buffer[2][NBUF_I2S];

  int _wordsPerBuffer=NBUF_I2S;

  static void dma_irq(void);

  void dma_setup(void)
  {
      for (auto ii = 0; ii < 2; ii++) _channelDMA[ii] = dma_claim_unused_channel(true);

      int dreq = pio_get_dreq(_pio, _sm, false);
      volatile void *pioFIFOAddr =(volatile void*)&_pio->rxf[_sm];

      for (auto ii = 0; ii < 2; ii++) 
      {  
        dma_channel_config c = dma_channel_get_default_config(_channelDMA[ii]);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32); // 16b/32b transfers into PIO FIFO
        channel_config_set_read_increment(&c, false); // Reading same FIFO address
        channel_config_set_write_increment(&c, true); // Writing to incrememting buffers

        channel_config_set_dreq(&c, dreq); // Wait for the PIO TX FIFO specified
        channel_config_set_chain_to(&c, _channelDMA[ii ^ 1]); // Start other channel when done
        channel_config_set_irq_quiet(&c, false); // Need IRQs

        dma_channel_configure(_channelDMA[ii], &c, i2s_buffer[ii], pioFIFOAddr, _wordsPerBuffer , false);

        dma_channel_set_irq0_enabled(_channelDMA[ii], true);
      }

      irq_add_shared_handler(DMA_IRQ_0, dma_irq, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
      irq_set_enabled(DMA_IRQ_0, true);

      dma_channel_start(_channelDMA[0]);
  }

  static void dma_irq(void)
  {
    for(int ii=0; ii<2; ii++)
    if(dma_channel_get_irq0_status(_channelDMA[ii]))
    { //Serial.print(ii);
      dma_channel_acknowledge_irq0(_channelDMA[ii]);

      process(i2s_buffer[ii]);

      dma_channel_set_write_addr(_channelDMA[ii], i2s_buffer[ii], false);
      dma_channel_set_trans_count(_channelDMA[ii], _wordsPerBuffer, false);
    }
  }


/*======================================================================================*/
#elif defined(__IMXRT1062__)

  void set_audioClock(int nfact, int32_t nmult, uint32_t ndiv) // sets PLL4
  {
    CCM_ANALOG_PLL_AUDIO = CCM_ANALOG_PLL_AUDIO_BYPASS | CCM_ANALOG_PLL_AUDIO_ENABLE
            | CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(2) // 2: 1/4; 1: 1/2; 0: 1/1
            | CCM_ANALOG_PLL_AUDIO_DIV_SELECT(nfact);

    CCM_ANALOG_PLL_AUDIO_NUM   = nmult & CCM_ANALOG_PLL_AUDIO_NUM_MASK;
    CCM_ANALOG_PLL_AUDIO_DENOM = ndiv & CCM_ANALOG_PLL_AUDIO_DENOM_MASK;
    
    CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_POWERDOWN;//Switch on PLL
    while (!(CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_LOCK)) {}; //Wait for pll-lock
    
    const int div_post_pll = 1; // other values: 2,4
    CCM_ANALOG_MISC2 &= ~(CCM_ANALOG_MISC2_DIV_MSB | CCM_ANALOG_MISC2_DIV_LSB);
    if(div_post_pll>1) CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_LSB;
    if(div_post_pll>3) CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_MSB;
    
    CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_BYPASS;//Disable Bypass
  }

  void setAudioFrequency(int fs)
  {
      // PLL between 27*24 = 648MHz und 54*24=1296MHz
    int n1 = 4; //SAI prescaler 4 => (n1*n2) = multiple of 4
    int n2 = 1 + (24000000 * 27) / (fs * 256 * n1);

    double C = ((double)fs * 256 * n1 * n2) / 24000000;
    int c0 = C;
    int c2 = 10000;
    int c1 = C * c2 - (c0 * c2);
    set_audioClock(c0, c1, c2);

      // clear SAI1_CLK register locations
    CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI1_CLK_SEL_MASK))
        | CCM_CSCMR1_SAI1_CLK_SEL(2); // &0x03 // (0,1,2): PLL3PFD0, PLL5, PLL4
    CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK))
        | CCM_CS1CDR_SAI1_CLK_PRED(n1-1) // &0x07
        | CCM_CS1CDR_SAI1_CLK_PODF(n2-1); // &0x3f
    // Select MCLK
    IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1
      & ~(IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL_MASK))
      | (IOMUXC_GPR_GPR1_SAI1_MCLK_DIR | IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL(0));

  }

  void i2s_setup(void)
  {
    CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);

    // if receiver is enabled, do nothing
    if (I2S1_RCSR & I2S_RCSR_RE) return;
  //PLL:
    int fs = fsamp;
  
    setAudioFrequency(fs);

    CORE_PIN23_CONFIG = 3;  //1:MCLK
    CORE_PIN21_CONFIG = 3;  //1:RX_BCLK
    CORE_PIN20_CONFIG = 3;  //1:RX_SYNC

  	CORE_PIN8_CONFIG  = 3;  //1:RX_DATA0
  	IOMUXC_SAI1_RX_DATA0_SELECT_INPUT = 2;

    I2S1_RMR = 0;
    //I2S1_RCSR = (1<<25); //Reset
    I2S1_RCR1 = I2S_RCR1_RFW(1);
    I2S1_RCR2 = I2S_RCR2_SYNC(0) | I2S_RCR2_BCP  
              | (I2S_RCR2_BCD | I2S_RCR2_DIV((1)) | I2S_RCR2_MSEL(1));
    I2S1_RCR3 = I2S_RCR3_RCE;
    I2S1_RCR4 = I2S_RCR4_FRSZ((2-1)) | I2S_RCR4_SYWD((MBIT-1)) | I2S_RCR4_MF
          | I2S_RCR4_FSE | I2S_RCR4_FSP | I2S_RCR4_FSD;
    I2S1_RCR5 = I2S_RCR5_WNW((MBIT-1)) | I2S_RCR5_W0W((MBIT-1)) | I2S_RCR5_FBT((MBIT-1));

    I2S1_RCSR = I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE | I2S_RCSR_FR;

  }

  /***************************************************************************/
  #include "DMAChannel.h"

  static DMAChannel dma;
  uint32_t i2s_buffer[2*NBUF_I2S];
  static void acq_isr(void);

  void dma_setup(void)
  {
    dma.begin(true); // Allocate the DMA channel first

    dma.TCD->SADDR = (void *)((uint32_t)&I2S1_RDR0);
    dma.TCD->SOFF = 0;
    dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
    dma.TCD->NBYTES_MLNO = (MBIT/8);
    dma.TCD->SLAST = 0;
    dma.TCD->DADDR = i2s_buffer;
    dma.TCD->DOFF = (MBIT/8);
    dma.TCD->CITER_ELINKNO = 2*NBUF_I2S;
    dma.TCD->DLASTSGA = -sizeof(i2s_buffer);
    dma.TCD->BITER_ELINKNO = dma.TCD->CITER_ELINKNO;
    dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;

    dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_RX);

    dma.attachInterrupt(acq_isr, 0x60);	
    dma.enable();
  }

  #if defined(AUDIO_INTERFACE)
    #include "mAudioIF.h"
  #endif

  static void acq_isr(void)
  {
    uint32_t daddr;
    int32_t *src;
  
    daddr = (uint32_t)(dma.TCD->DADDR);

    dma.clearInterrupt();
  
    if (daddr < (uint32_t) &i2s_buffer[NBUF_I2S]) 
    {
      // DMA is receiving to the first half of the buffer
      // need to remove data from the second half
      src = (int32_t *)&i2s_buffer[NBUF_I2S];
    }
    else
    {
    // DMA is receiving to the second half of the buffer
    // need to remove data from the first half
      src = (int32_t *)&i2s_buffer[0];
    }

    #if NCH==1
      for(int ii=0; ii<NBUF_ACQ; ii++) src[2*ii+ICH]  = (src[2*ii+ICH]-BIAS) >> shift;
    #else
      for(int ii=0; ii<NBUF_I2S; ii++) src[ii]  = (src[ii]-BIAS) >> shift;
    #endif

    process(src);

    #if defined(AUDIO_INTERFACE)
      putAudio(src);
    #endif
  }

#endif

int32_t acqbias=0;
/***************************************************************************/
static void process(int32_t * buffer)
{ procCount++;

  for(int ii=0; ii<NBUF_ACQ; ii++) acqBuffer[ii]= buffer[2*ii+ICH];
  
  float tmp=0.0f;
  for(int ii=0; ii<NBUF_ACQ; ii++) tmp +=(float)acqBuffer[ii]/(float)NBUF_ACQ;
  acqbias=(int32_t) tmp;

  if(proc==0)
  {
    if(!pushData((uint32_t *)acqBuffer)) procMiss++;
  }
  else if(proc==1)
  {
   if(!compress((void *)acqBuffer)) procMiss++;
  }
}
