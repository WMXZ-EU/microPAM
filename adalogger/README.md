# adalogger
 This directory contains the source code for RP2040 based Adalogger project that 
 can be found at https://www.micropam.com .

 The menu is minimized to start/end acquisition, correct external RTC and to print actual acquisition parameters. It is expected that the user adapts the acquisition parameters in config.h and recompiles the program.

## Menu
 The menu commands are 
 - s: start acquisition
 - e: end acquisition
 - c: check and change RTC
 - p: print system and acquisition parameters 
 - !a xx: set squisition (file length) to xx seconds
 - !o x: set on time to x minutes (sic!)
 - !r x: set repetition rate to x minutes (sic!)
 - !w x: set eeprom mode to x (0: not to be used, 1: to be  used on reboot)
 
 ## Configuration
 The program configuration is controlled by config.h
 
## UF2 file
 Direct link to uf2 file: 
 - https://github.com/WMXZ-EU/microPAM/blob/main/adalogger/build/rp2040.rp2040.adafruit_feather_adalogger/adalogger.ino.uf2
 
 