#include "Arduino.h"
#include "global.h"

#if defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_ADALOGGER)
#include "rp2040.h"


/************************************** I2S *******************************************************/
    // for ADALogger
    #define I2S_DOUT  11
    #define I2S_BCLK  9
    #define I2S_FSYNC (I2S_BCLK+1) // must always be this way


  pin_size_t _pinDOUT =I2S_DOUT;
  pin_size_t _pinBCLK =I2S_BCLK;

  int _bps =MBIT;
  int off=0;

  PIOProgram *_i2s;
  static PIO _pio;
  static int _sm;

  #include "hardware/pio.h"

#if FSAMP<64000
// ------------ //
// pio_i2s_in48 //
// ------------ //

#define pio_i2s_in48_wrap_target 0
#define pio_i2s_in48_wrap 9

static const uint16_t pio_i2s_in48_program_instructions[] = {
            //     .wrap_target
    0xa042, //  0: nop                    side 0     
    0xa842, //  1: nop                    side 1     
    0xe03d, //  2: set    x, 29           side 0     
    0x4801, //  3: in     pins, 1         side 1     
    0x0043, //  4: jmp    x--, 3          side 0     
    0x4862, //  5: in     null, 2         side 1     
    0xf03e, //  6: set    x, 30           side 2     
    0xb842, //  7: nop                    side 3     
    0x1047, //  8: jmp    x--, 7          side 2     
    0xb842, //  9: nop                    side 3     
            //     .wrap
};

static const struct pio_program pio_i2s_in_program = {
    .instructions = pio_i2s_in48_program_instructions,
    .length = 10,
    .origin = -1,
};

static inline pio_sm_config pio_i2s_in_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + pio_i2s_in48_wrap_target, offset + pio_i2s_in48_wrap);
    sm_config_set_sideset(&c, 2, false, false);
    return c;
}
#else
// ------------ //
// pio_i2s_in96 //
// ------------ //

#define pio_i2s_in96_wrap_target 0
#define pio_i2s_in96_wrap 11

static const uint16_t pio_i2s_in96_program_instructions[] = {
            //     .wrap_target
    0xa042, //  0: nop                    side 0     
    0xa842, //  1: nop                    side 1     
    0xa042, //  2: nop                    side 0     
    0xa842, //  3: nop                    side 1     
    0xe03c, //  4: set    x, 28           side 0     
    0x4801, //  5: in     pins, 1         side 1     
    0x0045, //  6: jmp    x--, 5          side 0     
    0x4863, //  7: in     null, 3         side 1     
    0xf03e, //  8: set    x, 30           side 2     
    0xb842, //  9: nop                    side 3     
    0x1049, // 10: jmp    x--, 9          side 2     
    0xb842, // 11: nop                    side 3     
            //     .wrap
};

static const struct pio_program pio_i2s_in_program = {
    .instructions = pio_i2s_in96_program_instructions,
    .length = 12,
    .origin = -1,
};

static inline pio_sm_config pio_i2s_in_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + pio_i2s_in96_wrap_target, offset + pio_i2s_in96_wrap);
    sm_config_set_sideset(&c, 2, false, false);
    return c;
}
#endif

static inline void pio_i2s_in_program_init(PIO pio, uint sm, uint offset, uint data_pin, uint clock_pin_base, uint bits) 
{
    pio_gpio_init(pio, data_pin);
    pio_gpio_init(pio, clock_pin_base);
    pio_gpio_init(pio, clock_pin_base + 1);
    //

    pio_sm_config sm_config;
   	sm_config = pio_i2s_in_program_get_default_config(offset);
    //
    sm_config_set_in_pins(&sm_config, data_pin);
    sm_config_set_sideset_pins(&sm_config, clock_pin_base);
    sm_config_set_in_shift(&sm_config, false, true, (bits <= 16) ? 2 * bits : bits);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_RX);
    //
    pio_sm_init(pio, sm, offset, &sm_config);
    //
    uint pin_mask = 3u << clock_pin_base;
    pio_sm_set_pindirs_with_mask(pio, sm, pin_mask, pin_mask);
    pio_sm_set_pins(pio, sm, 0); // clear pins
    //
    //pio_sm_exec(pio, sm, pio_encode_set(pio_y, bits - 2));
}

  void i2s_start(void) { pio_sm_set_enabled(_pio, _sm, true); }
  void i2s_stop(void)  { pio_sm_set_enabled(_pio, _sm, false); }
  
  void acqModifyFrequency(uint32_t fsamp)
  { 
    float bitClk = fsamp * _bps * 2.0 /* SAI channels */ * 2.0 /* edges per clock */;
    pio_sm_set_enabled(_pio, _sm, false);
    pio_sm_set_clkdiv(_pio, _sm, (float)clock_get_hz(clk_sys) / bitClk);
    pio_sm_set_enabled(_pio, _sm, true);
  }

  void i2s_setup(void)
  {
    _i2s = new PIOProgram( &pio_i2s_in_program);
    //
    _i2s->prepare(&_pio, &_sm, &off);

    pio_i2s_in_program_init(_pio, _sm, off, _pinDOUT, _pinBCLK, _bps);

    acqModifyFrequency(FSAMP); // Will also start I2S
  }

  static int32_t * last_buffer=0;
  static uint32_t have_last_buffer=0;
  int32_t * is2_last_read(void)
  { if(have_last_buffer) 
    { have_last_buffer=0;
      return last_buffer;
    }
    return 0;
  }

  int _channelDMA[2];
  static int32_t i2s_buffer[2][NBUF_I2S];

  int _wordsPerBuffer=NBUF_I2S;

  static void __not_in_flash_func(dma_irq)(void);

  void dma_setup(void)
  {
      for (auto ii = 0; ii < 2; ii++) _channelDMA[ii] = dma_claim_unused_channel(true);

      int dreq;
      dreq = pio_get_dreq(_pio, _sm, false);
      volatile void *pioFIFOAddr =(volatile void*)&_pio->rxf[_sm];

      for (auto ii = 0; ii < 2; ii++) 
      {  
        dma_channel_config c;
        c = dma_channel_get_default_config(_channelDMA[ii]);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32); // 16b/32b transfers into PIO FIFO
        channel_config_set_read_increment(&c, false); // Reading same FIFO address
        channel_config_set_write_increment(&c, true); // Writing to incrememting buffers

        channel_config_set_dreq(&c, dreq); // Wait for the PIO TX FIFO specified
        channel_config_set_chain_to(&c, _channelDMA[ii ^ 1]); // Start other channel when done
        channel_config_set_irq_quiet(&c, false); // Need IRQs

        dma_channel_configure(_channelDMA[ii], &c, i2s_buffer[ii], pioFIFOAddr, _wordsPerBuffer , false);

        dma_channel_set_irq0_enabled(_channelDMA[ii], true);
      }

      //irq_add_shared_handler(DMA_IRQ_0, dma_irq, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
      irq_add_shared_handler(DMA_IRQ_0, dma_irq, 0x60); // 0x80 is DEFAULT
      irq_set_enabled(DMA_IRQ_0, true);

      dma_channel_start(_channelDMA[0]);
  }

  static void __not_in_flash_func(dma_irq)(void)
  { static int32_t val=0;
    for(int ii=0; ii<2; ii++)
    if(dma_channel_get_irq0_status(_channelDMA[ii]))
    { //Serial.print(ii);
      dma_channel_acknowledge_irq0(_channelDMA[ii]);

      last_buffer=i2s_buffer[ii];
      have_last_buffer=1;

      dma_channel_set_write_addr(_channelDMA[ii], last_buffer, false);
      dma_channel_set_trans_count(_channelDMA[ii], _wordsPerBuffer, false);
      return;
    }
  }

/************************************** Hibernate *******************************************************/
  #include "RTC.h"
  #define XRTC_INT_PIN A2

/*
  #include "tusb.h"
  void USB_stop(void)
  {
    tud_disconnect();
    delay(100);
  }
*/

  extern void SD_stop(void);

  void stopSystem(void)
  {
    //
    SD_stop();
    //adc_exit();
    //
    //stopUSB();
  }

  #include "pico/stdlib.h"

  #include "pico.h"

  #include "pico/runtime_init.h"
  #include "hardware/pll.h"
  #include "hardware/regs/clocks.h"
  #include "hardware/clocks.h"
  #include "hardware/xosc.h"
  #include "hardware/structs/rosc.h"

  inline static void rosc_clear_bad_write(void) {
      hw_clear_bits(&rosc_hw->status, ROSC_STATUS_BADWRITE_BITS);
  }
  inline static bool rosc_write_okay(void) {
      return !(rosc_hw->status & ROSC_STATUS_BADWRITE_BITS);
  }
  inline static void rosc_write(io_rw_32 *addr, uint32_t value) {
      rosc_clear_bad_write();
      assert(rosc_write_okay());
      *addr = value;
      assert(rosc_write_okay());
  };

  void rosc_disable(void) {
      uint32_t tmp = rosc_hw->ctrl;
      tmp &= (~ROSC_CTRL_ENABLE_BITS);
      tmp |= (ROSC_CTRL_ENABLE_VALUE_DISABLE << ROSC_CTRL_ENABLE_LSB);
      rosc_write(&rosc_hw->ctrl, tmp);
      // Wait for stable to go away
      while(rosc_hw->status & ROSC_STATUS_STABLE_BITS);
  }

  void rosc_enable(void) {
      //Re-enable the rosc
      rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);

      //Wait for it to become stable once restarted
      while (!(rosc_hw->status & ROSC_STATUS_STABLE_BITS));
  }

  // use only xtal oscillator (12 MHz clock)
  void sleep_run_from_xosc(void) 
  {
      uint src_hz;
      uint clk_ref_src;
      src_hz = XOSC_HZ;
      clk_ref_src = CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC;

      // CLK_REF = XOSC
      clock_configure(clk_ref,
                      clk_ref_src,
                      0, // No aux mux
                      src_hz,
                      src_hz);

      // CLK SYS = CLK_REF
      clock_configure(clk_sys,
                      CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF,
                      0, // Using glitchless mux
                      src_hz,
                      src_hz);

      // CLK ADC = 0MHz
      clock_stop(clk_adc);
      clock_stop(clk_usb);
      clock_stop(clk_rtc);

          // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
      clock_configure(clk_peri,
                      0,
                      CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                      src_hz,
                      src_hz);

      pll_deinit(pll_sys);
      pll_deinit(pll_usb);

      // Can disable rosc
      rosc_disable();
  }

  void sleep_goto_dormant_until_pin(uint32_t gpio_pin) 
  {
      uint32_t event = 0;
      event = IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_LEVEL_LOW_BITS;

      gpio_init(gpio_pin);
      gpio_set_input_enabled(gpio_pin, true);
      gpio_set_dormant_irq_enabled(gpio_pin, event, true);

      xosc_dormant();
      // Execution stops here until woken up

      // Clear the irq so we can go back to dormant mode again if we want
      gpio_acknowledge_irq(gpio_pin, event);
      gpio_set_input_enabled(gpio_pin, false);
  }

  // To be called after waking up from sleep/dormant mode to restore system clocks properly
  void sleep_power_up(void)
  {
      // Re-enable the ring oscillator, which will essentially kickstart the proc
      rosc_enable();

      // Reset the sleep enable register so peripherals and other hardware can be used
      clocks_hw->sleep_en0 |= ~(0u);
      clocks_hw->sleep_en1 |= ~(0u);

      // Restore all clocks
      clocks_init();
  }

  void reboot(void){ rp2040.restart(); }

  void reset()
  {
    #define AIRCR_Register (*((volatile uint32_t*)(PPB_BASE + 0x0ED0C)))
    AIRCR_Register = 0x5FA0004;    
    //{ *(uint32_t *)0xE000ED0C =  0x5FA0004;}    // from Teensy
  }

/*
  void powerDown(void) 
  { stopSystem();
    sleep_run_from_xosc();
    clock_stop(clk_rtc);
    while (1) asm("wfi"); // will not return
  }
*/

  void goDormant(void) 
  {
    // 'switch-off' all I/O pins
    for(int p=0;p<30;p++)
    { pinMode(p, INPUT); // best performance!
      gpio_set_input_enabled(p, false); // disable input gate
    }
    // set-up RTC-wakeup pin
    pinMode(XRTC_INT_PIN,INPUT_PULLUP);
    gpio_set_input_enabled(XRTC_INT_PIN, true); // enable input gate
    sleep_run_from_xosc();
    clock_stop(clk_rtc);
    sleep_goto_dormant_until_pin(XRTC_INT_PIN);
    //
    // will resume action here
    sleep_power_up();
    delay(100);

    // simply restart program to facilitate setup
    reboot();
  }

/*
static uint32_t alarm=0;
uint32_t estAlarmTime(uint32_t secs) { return (secs<alarm)? alarm: secs; } // will be called from do_hibernate

  void do_hibernate(void) 
  { Serial.println("do hibernate"); Serial.flush();

    hibernate_init();

    //set alarm
    uint32_t secso = rtc_get();
    Serial.print(secso); Serial.print(" -> ");
    uint32_t secs = estAlarmTime(secso);
    Serial.println(secs);  Serial.flush();

    delay(100);
    if(secs>secso) hibernate_now(secs);
  }

  void hibernate_init(void)
  {
    stopSystem();
  }
*/

  void hibernate_until(uint32_t secs)
  { stopSystem();
    XRTCsetAlarm(secs);
    delay(100);
    goDormant();
  }

#include "pico/unique_id.h"
char uid_str[8]; 
void getUID(void)
{
  pico_unique_board_id_t id;
  pico_get_unique_board_id(&id);
  Serial.print("unique_board_id: ");
  for (int len = 0; len < 8; len++) 
  { Serial.print(id.id[len],HEX); Serial.print(' '); 
  } 
  Serial.println();  
  sprintf(uid_str,"%02X%02X%02X",id.id[2],id.id[1],id.id[0]);
}
/************************************ RTC **********************************************/

/************************************ NEO Pixel ****************************************/
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel neo_pixel(1, PIN_NEOPIXEL, NEO_GRB);

void neo_pixel_init()
{
  neo_pixel.begin();
  neo_pixel.clear();
}

void neo_pixel_show(uint16_t r, uint16_t g, uint16_t b)
{
  neo_pixel.setPixelColor(0, neo_pixel.Color(r, g, b));
  neo_pixel.show();
}

#endif