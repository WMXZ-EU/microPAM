#ifndef RP2040_H
#define RP2040_H

  // I2S
  void i2s_setup(void);
  void dma_setup(void);

  int32_t * is2_last_read(void) ;

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