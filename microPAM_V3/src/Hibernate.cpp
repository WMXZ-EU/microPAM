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
  * File: hibernate.cpp
 */
#include "Arduino.h"

#include "Config.h"
#include "Hibernate.h"
#include "RTC.h"

#if defined(ARDUINO_ARCH_RP2040)
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
#endif


uint32_t estAlarmTime(uint32_t secs); // in Filing
void stopSystem(void); // in main ino


#if defined(ARDUINO_ARCH_RP2040)

  void reboot(void){ rp2040.restart(); }

  void reset()
  {
    #define AIRCR_Register (*((volatile uint32_t*)(PPB_BASE + 0x0ED0C)))
    AIRCR_Register = 0x5FA0004;    
    //{ *(uint32_t *)0xE000ED0C =  0x5FA0004;}    // from Teensy
  }

  void powerDown(void) 
  { stopSystem();
    sleep_run_from_xosc();
    clock_stop(clk_rtc);
    while (1) asm("wfi"); // will not return
  }

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

  void hibernate_init(void)
  { //stop processes 
    stopSystem();
  }

  void hibernate_now(uint32_t secs)
  {
      XRTCsetAlarm(secs);
      delay(100);
      goDormant();
  }

#elif defined(__IMXRT1062__)
//  #include "core_pins.h"

  #define SNVS_LPCR_LPTA_EN_MASK    (0x2U)

  static uint32_t snvs_tmp;   // to save control register

  void powerDown(void)
  {
    SNVS_LPCR |= (1 << 6); // turn off power
    while (1) asm("wfi");      
  }

  void hibernate_init(void)
  {
    stopSystem();
    //for(int ii=2;ii<5;ii++) { pinMode(ii,OUTPUT); digitalWrite(ii,LOW); }

    snvs_tmp = SNVS_LPCR;
    SNVS_LPSR |= 1;
    asm volatile("DSB");

    // disable alarm
    SNVS_LPCR &= ~SNVS_LPCR_LPTA_EN_MASK;
    while (SNVS_LPCR & SNVS_LPCR_LPTA_EN_MASK);

    // clear alarm value
    SNVS_LPTAR = 0;
    while (SNVS_LPTAR != 0);
  }

  void hibernate_now(uint32_t secs)
  {
      __disable_irq();
      SNVS_LPTAR = secs;
      while (SNVS_LPTAR != secs);
      // restore control register and set alarm
      SNVS_LPCR = snvs_tmp | SNVS_LPCR_LPTA_EN_MASK; 
      while (!(SNVS_LPCR & SNVS_LPCR_LPTA_EN_MASK));
      __enable_irq();
      //
      powerDown(); 
  }
#endif

#if defined(ARDUINO_ARCH_RP2040)
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
#else
FLASHMEM
void do_hibernate(void)
{
    uint32_t tmp = SNVS_LPCR; // save control register

    SNVS_LPSR |= 1;

    // disable alarm
    SNVS_LPCR &= ~SNVS_LPCR_LPTA_EN_MASK;
    while (SNVS_LPCR & SNVS_LPCR_LPTA_EN_MASK);

    __disable_irq();

    //get Time:
    uint32_t lsb, msb;
    do {
      msb = SNVS_LPSRTCMR;
      lsb = SNVS_LPSRTCLR;
    } while ( (SNVS_LPSRTCLR != lsb) | (SNVS_LPSRTCMR != msb) );
    uint32_t secso = (msb << 17) | (lsb >> 15);

    uint32_t secs = estAlarmTime(secso);
    //set alarm
    //secs += nsec;
    SNVS_LPTAR = secs;
    while (SNVS_LPTAR != secs);

    // restore control register and set alarm
    SNVS_LPCR = tmp | SNVS_LPCR_LPTA_EN_MASK; 
    while (!(SNVS_LPCR & SNVS_LPCR_LPTA_EN_MASK));

//    NVIC_CLEAR_PENDING(IRQ_SNVS_ONOFF);
//    attachInterruptVector(IRQ_SNVS_ONOFF, &call_back);
//    NVIC_SET_PRIORITY(IRQ_SNVS_ONOFF, 255); //lowest priority
//    asm volatile ("dsb"); //make sure to write before interrupt-enable
//    NVIC_ENABLE_IRQ(IRQ_SNVS_ONOFF);
    __enable_irq();
  
    SNVS_LPCR |= (1 << 6); // turn off power
    while (1) asm("wfi");  
}
#endif