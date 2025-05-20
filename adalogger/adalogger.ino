#include <stdio.h>
#include <string.h>

#include "src/global.h"
#include "src/rp2040.h"
#include "src/RTC.h"
#include "src/Menu.h"
#include "src/Filing.h"

//-----------------------------------
// implementation
//-----------------------------------

status_t status=STOPPED;
uint32_t loop_count=0;
uint32_t data_count=0;

uint16_t have_disk=0;
void setup() {
  // put your setup code here, to run once:

  neo_pixel_init();
  // reduce MCU clock
  if(!set_sys_clock_khz(48'000, true)) neo_pixel_show(255, 0, 0);

  parameterInit();

  neo_pixel_show(10, 0, 0);

  while(millis()<(WAIT*1000)) if(Serial) { Serial.print(millis());break;}
  if(Serial) Serial.println("\nAdalogger");
  
  neo_pixel_show(0, 0, 0);

  for(int p=0;p<30;p++) // disable GIPOs
  { pinMode(p, INPUT); 
    gpio_set_input_enabled(p, false); 
  }

  parameterPrint();

  rtc_setup();

  i2s_setup();
  dma_setup();
  have_disk=SD_init();
  if(have_disk) status=CLOSED;
}

void loop() {
  // put your main code here, to run repeatedly:
  status = menu(status);
  //
  int32_t *buffer = is2_last_read();
  if(have_disk && buffer)
  {
    if(status != STOPPED)
    {
      status=logger(buffer,status);
      data_count++;
    }
  }
  loop_count++;
  asm("wfi");
}
