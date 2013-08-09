Reflow Oven Controller For MSP430G2553 Launchpad
========
- Version: 1.0
- Date: 08-08-2013
- Code Author: Kristen Villemez
- Hardware Design: Juan Chong
- Website: http://www.juanjchong.com/

Overview
========
This code is used to control a reflow oven. It allows for the option to change
solder types (lead vs. lead-free) before each time the reflow process is started.
Definitions for both profiles are included in this code. This code is designed to
work with the Reflow Oven Controller BoosterPack designed by Juan Chong. It uses
the MAX31855 thermocouple chip and library and an on-board LCD screen for status
display on the fly. A PID controller library is used to control the system.

Attributions
========
+ Lim Phang Moh - http://www.rocketscream.com - Original code off which this program is based
+ Brett Beauregard - http://www.brettbeauregard.com - Author of Arduino PID Library used in this program
+ Limor Fried - http://www.adafruit.com - Author of MAX31855 Library

DISCLAIMER!!!
========
HIGH VOLTAGES ARE DANGEROUS! PLEASE UNDERSTAND THE RISKS YOU ARE DEALING WITH
WHEN DEALING WITH THIS HARDWARE BEFORE YOU BEGIN! USE COMMON SENSE WHEN WORKING
WITH THIS PROJECT. USE OF THIS HARDWARE AND SOFTWARE IS AT YOUR OWN RISK, AND
WE ARE NOT RESPONSIBLE OR LIABLE FOR ANY DAMAGE TO YOU OR YOUR SURROUNDINGS THAT
MAY OCCUR OUT OF USE OF THIS AND RELATED MATERIALS!
THERE IS NO GUARANTEE OF FUNCTION OF THIS CODE IN YOUR PARTICULAR APPLICATION.
NO WARRANTY ON THIS SOFTWARE IS GUARANTEED NOR IMPLIED.

Licenses
=========
This hardware and software is released under...

Creative Commons Share Alike v3.0 License
http://creativecommons.org/licenses/by-sa/3.0/

You are free to use this code and/or modify it. All we ask is an attribution,
including supporting libraries and their respective authors used in this
software.

Required Libraries
===================
- Arduino PID Library: (works as is in Energia)
  >> https://github.com/b3rttb/Arduino-PID-Library
- MAX31855 Library (works as is in Energia)
  >> https://github.com/rocketscream/MAX31855
- FlexiTimer2 Library (works as is in Energia)
  >> http://www.pjrc.com/teensy/td_libs_MsTimer2.html
- LiquidCrystal Library (included in Energia)

Revision History
========
1.0 - Initial release
