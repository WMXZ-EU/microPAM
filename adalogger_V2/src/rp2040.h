/* microPAM 
 * Copyright (c) 2023/2024/2025, Walter Zimmer
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
#ifndef RP2040_H
#define RP2040_H

  void usb_stop(void);
  void reboot(void);
  void sleep_power_up(void);

  // I2S
  void i2s_setup(void);
  void dma_setup(void);
  void acqModifyFrequency(uint32_t fsamp);

  int32_t * is2_last_read(void) ;
  extern uint32_t missed_acq;
  
  void reset_missed_list(void);
  uint32_t * get_missed_list(void);

  // Filing
  void do_hibernate(void) ;
  void hibernate_init(void) ;
  void hibernate_until(uint32_t secs) ;

  extern char uid_str[]; 
  void getUID(void); 

  // NeoPixel
  void neo_pixel_init();
  void neo_pixel_show(uint16_t r, uint16_t g, uint16_t b);
#endif