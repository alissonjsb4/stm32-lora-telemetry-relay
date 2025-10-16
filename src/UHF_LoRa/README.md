# RS41HUP (Ham Use Project) - Project Horus Fork
Firmware for RS41 for HAM use.

It is possible to recycle RS41-SGP sondes for amateur radio use without any electrical changes! You just have to build a new firmware (this one) and apply it via a cheap programmer, the "ST-Linkv2" (or alternatives). The modified sonde can now transmit on a user-defined frequency in the 70cm band, with Habhub-compatible RTTY and (better performing) 4FSK telemetry!

Released under GPL v2.

# Important info
* This is a fork from  github.com/darksidelemm/RS41HUP  (not longer supported)
* Major adds:
  * Horus V2 protocoll (32 bytes) 
  * GPS-Watchdog: reboots RS41 if GPS gets lost longer then timeout as defined
  * GPS-TX Sync if Fix is available every minute
  * Second frequency if defined. changes every TX intervall between both
  * MORSECODE: Transmit additional data like Alt, QRA Locator, Sat-Count, Volt, CPU-Temp - or NO GPS if jamming
  * GPS-TX Sync now with offset after the TX Intervall.  e.g...
	* if delay 60s and offset 10s: tx on 00:00:10 / 00:01:10 / 00:02:10 ...
	* if delay 30s and offset 10s: tx on 00:00:10 / 00:00:40 / 00:01:10 ...
	* if delay 15s and offset 10s: tx on 00:00:10 / 00:00:25 / 00:00:40 ...
  * APRS_1200 Baud added.  Ratio can be defined how often to send a packet
    * Meaning of packet codings:
	* Sample: 11:35 DF7PN-7/WIDE1-1>APZQAP>UI,?,F0 (1198 baud): !4953.14N/00804.40EO/A=000846/P43S7T6V6 RS41 Balloon
	* /A=000846 is altitude in feet. Thumb rule: divide by 3
	* /P43S7T6V6  is P=PacketNbr, S=SatCount, T=internalTemperature, V=Volts (div by 100) here 6 = 0.06 Volt
	* ==> do not forget to add the new files to your coIDE project file
		* aprs.h; aprs.cpp; QAPRS*
	* Extended accuracy of the coordinates with DAO 
		* DAO adds after the APRS_COMMENT a sequence of  !W??!  where the ?? are some more decimals of the position
		* see for more info: http://www.aprs.org/aprs12/datum.txt
		* config.h : #define APRS_DAO 1     DAO is active // #define APRS_DAO 0     no extension is added to comment - packets are 5 bytes shorter	
  * Custom_field_list:  provide your own Data in Horus-V2 spare bytes
	* The V2 Protocol provides some bytes for own use. There is an default list if you do nothing.
	* Normally the webbrowser shows in left panel: Temerature, External - Relative Humidity, External and Pressure, External
	* Some of us wanted to provide instead a constant : Flight_number and SondeType. 
	* See config.h file for more infos at the defines USERFLAG_A and _B how to use them.


* This RS41HUB is recomended for floating flights with battery. It needs less mA then the RS41ng Version. 
* If power does not matter, than have a look on RS41ng.
* RS41ng is created by https://github.com/mikaelnousiainen/RS41ng.
* An Version with some special needs will also be found on https://github.com/whallmann/RS41ng


Original Repository: https://github.com/Qyon/STM32_RTTY, though this fork is based on [DF8OE's version](https://github.com/df8oe/RS41HUP).

Modifications by Mark Jessop <vk5qi@rfhead.net> include:
* Compatability with existing Project Horus RTTY Formats.
* Removed APRS support - no 70cm APRS infrastructure in Australia, so not really useful to us.
* Addition of the 4FSK 'Horus Binary' high performance telemetry mode
  * A decoder for the 4FSK mode is available here: https://github.com/projecthorus/horusdemodlib
  * Information on the 4FSK mode's performance is available here: https://www.rowetel.com/?p=5906


# Compilation
## Linux / OSX: 
* Grab the GNU ARM Embedded toolchain from here: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
* Extract the files as described on the download page
* Use local GIT to clone the RS41HUP_V2 project
* Within the RS41HUP_V2 directory:
  * CMakeLists.txt is prepared for Ubuntu, maybe edit is nessesary
  * `cmake .`
  * `make`
* Flashing: (premisse you intalled openocd before)
  * connect the sonde to the ST-Link adapter and power it on
  * in the RS41HUP_V2 dir start ./openocd_flash.sh. This unlocks the RS41 memory and push the RS41HUP.ELF file with a verify

## Windows:
Wolf: used to edit and compile the CooCox CoIDE - this works fine and it also uploads to the STM32 Board with ST-Link dongle.
Flashing:  right near the compile button in the toolbar there is the download button to the hardware. Try it. Tooltip is your friend

Use:
https://www.softpedia.com/get/Programming/Coding-languages-Compilers/CooCox-CoIDE.shtml

And:
https://launchpad.net/gcc-arm-embedded/5.0/5-2016-q3-update/+download/gcc-arm-none-eabi-5_4-2016q3-20160926-win32.exe

# Programming
Either:
* Use the ST Micro ST-LINK utility (windows only it seems?), or
* [stlink](https://github.com/texane/stlink) under Linux/OSX (will need to be unlocked first), or
* [OpenOCD](http://openocd.org) on Linux / RaspberryPi (see openocd_rs41.cfg file for usage) or
* Use `flash.sh` with a [Black Magic Probe](https://1bitsquared.com/products/black-magic-probe). You will need to modify the path to the debugger's serial interface.

Refer to [this file](./docs/programming_header.md) for programming header pinouts.

# Configuration
Configuration settings are located in [config.h](./config.h). Modify as appropriate before compiling/programming.

# Community
Telegram Chat:  feel free to join us at https://t.me/horus_flights

#Changelog
 * 31.01.2024 - CONFIG.H and MAIN.C - use your own Custom_field_list
 * 17.01.2024 - APRS.cpp - DAO extension added for more accuracy of position reports
 * 14.01.2024 - CMakeLists.txt: corrected the TOOLCHAIN_DIR line 12 to work under Ubuntu
 * 10.01.2024 - APRS_1200 added again to the modes, Ratio can be defined, a different frequency can be given
 * 04.01.2024 - Sync with GPS: added to configure an offset for less TX collisions on multi flights on same frequency
 * 31.12.2023 - Checked and extend MORSECODE data 
 * 26.12.2023 - Added choice for 2nd TX frequency: if activated, changes between each tx intervall.
 * 26.12.2023 - Added GPS-Sync: TX starts every full minute if gps-fix is availble
 * 23.12.2023 - Added a GPS-Watchdog: set a GPS-timeout after this the cpu makes a restart (GPS-jamming)
 * 23.12.2023 - Added Horus V2 32-Byte Format. Set by compiler switch in config.h
 * 14.12.2016 - Reverse engineeded connections, initial hard work, resulting in working RTTY by SQ7FJB
 * 07.01.2017 - GPS now using proprietiary UBLOX protocol, more elastic code to set working frequency by SQ5RWU
 * 23.01.2017 - Test APRS code, small fixes in GPS code by SQ5RWU
 * 06.06.2017 - APRS code fix, some code cleanup
 * June 2017 - starting with Linux support, making configuration more flexible by DF8OE
 * March 2018 - Addition of 4FSK binary mode support by Mark VK5QI


