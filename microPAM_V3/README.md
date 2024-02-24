# microPAM_V3
 
 This directory contains the source code for both the Teensy T4.1 and the KB2040/RP2040 based microPAM project that 
 can be found at https://www.micropam.com .
 
 The kb2040/rp2040 code uses both cores with the acquisition running on the second core. 
 This is to ensure that optional data compression has time to run.

 Direct link to hex file (for Teensy 4.1): 
 - https://github.com/WMXZ-EU/microPAM/blob/main/microPAM_V3/build/teensy.avr.teensy41/microPAM_V3.ino.hex

 The actual default setting (as of 16-06-2023) are
 - 1 min wav files (no compression)
 - 48 kHz sampling 
 - 24 bit wav file
 - TLV320ADC6140 
 - Store data in //day/hour/file structure 

 The V3 verstion contains additionally an ADC interface to TLV320ADC
 