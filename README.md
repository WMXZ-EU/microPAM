# microPAM
 
 This repository contains the source code for the microPAM project that can be found at https://www.micropam.com 
 
 There are three versions available
## microPAM_T4
This is a Teensy T4.1 only version that uses Teensy specific code. In particular it contains USB-Audio interface to PC.

This code is for quick testing of Teensy systems. Consider to use the microPAM_V2 version.

## microPAM_V2
This version contains code that runs both on RP2040 and Teensy T4.1 based systems. So far, it does not contain USB-Audio interface to PC.

However, this is the prefered code as for terrestrial applications

## microPAM_V3
This version contains code that aims to be used for the "microPAM-mare" implementation.
This code that is still under development

## adalogger
This code is dedicated to pr2040 based Adalogger with ICS43434 microphone. The menu is minimized to start/end acquisition, correct external RTC and to print actual acquisition parameters. It is expected that the user adapts the acquisition parameters in config.h and recompiles the program.

** See more detail in folders Readme file **
