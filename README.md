Reflow Oven Controller For MSP430G2553 Launchpad
========
+ Version: 1.2
+ Last Update: 05-15-2014
+ Initial Code Author: Kristen Villemez
+ Hardware Design: Juan Chong
+ Website: http://www.juanjchong.com/
+ Last Updated By: Juan Chong

Overview
========
This code is used to control a reflow oven. It allows for the option to change
solder types (lead vs. lead-free) before the reflow process is started.
Definitions for both profiles are included in this code. This code is designed to
work with the Reflow Oven Controller BoosterPack designed by Juan Chong. It uses
the MAX31855 thermocouple amplifier breakout board and library, and an on-board LCD 
screen for status display. A PID controller library is used to control a solid state
relay which drives the heater elements in a toaster oven.

Basic Use
=========
When the controller is first started, it will display the splash screen and prompt the
user to select a solder type - lead (Pb) or lead-free (NoPb). The default is lead solder.
If the user wishes to run the lead-free profile, press the Solder Select
button on the BoosterPack/Shield. Once you are ready to start the reflow process, press the
Start Reflow button and wait for the system to run through the reflow profile.

Attributions
========
+ Lim Phang Moh - http://www.rocketscream.com/
 Author of MAX31855 library and original Arduino code from which this program is based 
+ Brett Beauregard - http://www.brettbeauregard.com/
 Author of Arduino PID Library used in this program
+ Bill Earl - http://www.adafruit.com/
 Author of Sous Vide, an excellent self-tuning PID controller example

DISCLAIMER!!!
========
HIGH VOLTAGES AND CURRENTS ARE DANGEROUS! PLEASE UNDERSTAND THE RISKS OF WORKING 
WITH DANGEROUS HARDWARE BEFORE YOU BEGIN! USE COMMON SENSE WHEN WORKING
WITH THIS PROJECT. USE OF THIS HARDWARE AND SOFTWARE IS AT YOUR OWN RISK, AND
WE ARE NOT RESPONSIBLE OR LIABLE FOR ANY DAMAGE TO YOU OR YOUR SURROUNDINGS THAT
MAY OCCUR OUT OF USE OF THIS AND RELATED MATERIALS!

THIS SOFTWARE IS MADE AVAILABLE "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER
EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION ANY IMPLIED WARRANTIES OF
CONDITION, UNINTERRUPTED USE, MERCHANTABILITY, FITNESS FOR A PARTICULAR USE, OR
NON-INFRINGEMENT.

Licenses
=========
This hardware and software is released under:
Creative Commons Share Alike v3.0 License
  >>http://creativecommons.org/licenses/by-sa/3.0/
  
You are free to use this code and/or modify it. All we ask is an attribution, 
including supporting libraries and their respective authors used in this
software. If you would like to use this software and hardware for commercial
purposes, please contact the author using the website listed above.

Required Libraries
===================
- Arduino PID Library: (Energia or Arduino Compatible)
  >> https://github.com/b3rttb/Arduino-PID-Library
- MAX31855 Library (Energia or Arduino Compatible)
  >> https://github.com/rocketscream/MAX31855
- TwoMsTimer Library (Energia Compatible - Only Use If Programming An MSP430)
  >> https://github.com/freemansoft/build-monitor-devices/tree/master/ti_launchpad_rgb
- Timer1 Library (Arduino Compatible - Only Use If Programming An Arduino)
  >> http://playground.arduino.cc/code/timer1
- LiquidCrystal Library (Included By Default In Both Development Environments)

Revision History
========
1.0 - Initial release

1.1 - Fix bug where oven does not go into ERROR state when thermocouple communication is lost

1.2 - Optimized ISR tasks and LCD updating to avoid timing and memory issues.
