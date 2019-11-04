# Single Channel LoRaWAN Gateway

Version 6.0.1, October 20, 2019	  
Author: M. Westenberg (mw12554@hotmail.com)  
Copyright: M. Westenberg (mw12554@hotmail.com)  

All rights reserved. This program and the accompanying materials are made available under the terms 
of the MIT License which accompanies this distribution, and is available at
https://opensource.org/licenses/mit-license.php  
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

Maintained by Maarten Westenberg (mw12554@hotmail.com)

********************************************************************************    
********************************************************************************    
# Semtech SX1262 port
Ported to use with [Semtech SX1262](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1262) chips and PlatformIO support   
Author: Bernd Giesecke (https://github.com/beegee-tokyo)    

## See Release Notes for changes
********************************************************************************    
********************************************************************************    


# Release Notes

Adapted to use [Semtech SX1262](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1262) module and PlatformIO compatible (November 4th 2019)
- Port is work in progress    
- **_CAD and HOP is not supported for SX1262 chips_**    
- **_Downlink is not working for SX1262 chips_**    
- Compiles under PlatformIO. Needs an empty **\<foldername\>.ino** file to compile under ArduinoIDE   
- Display output is changed for better readability    
- Most definitions for the gateway have been moved from **`ESP-sc-gway.h`** to **`defines.h`**
- WiFi AP credentials are now setup in **`wLan.cpp`**
- If **`_LOCALSERVER`** is enabled the known node keys must be added in **`loraModem.cpp`**
- If **`_TRUSTED_NODES`** is enabled the known nodes IDs and names must be added in **`sensor.cpp`** 
- Depending on [SX126x-Arduino](https://github.com/beegee-tokyo/SX126x-Arduino) library

Make ready for Publishing (October 20, 2019)
- Edit all files and make ready for publising to Github
- Change the country/region setting of the Gateway (As a result you probably have to update most of the package).
- Upon connecting over WiFi, display the address for 4 seconds before starting the Gateway function.

New features in version 5.3.4 (March 25, 2019)
- Make use of the latest Arduino IDE available version 1.8.21
- Added new iteams to the statc function so that it now has all the Package Statistics. Fieds are such as: mesg_ttl, msg_ttl_0 etc. Removed reference to the old counters.
- Changed Json version in the IDE to ArduinoJSON 6 in _txRx.ino
- Larger message history for ESP32 based on free heap

New features in version 5.3.3 (August 25, 2018)
- Bug Fixing SPIFFS Format in GUI
- Included a confirm dialog in RESET, BOOT and STATISTICS buttons for user to confirm
- Repaired WlanConnect issues
- Improved Downlink function. Work for SF8-SF10. Does not work reliable for SF7, SF11, SF12
- Provided documentation button o top of page
- Added expert mode button in GUI (Wifi, System and Interrupt data is only shown in expert mode)
- bug fixes and documentation

New features in version 5.3.2 (July 07, 2018)
- Support for local decoding of sensor messages received by the gateway. 
	Use #define _LOCALSERVER 1, to enable this functionality. Also specify for each node that you want
	to inspect the messages from the NwkSKey and AppSKey in the sensor.h file.
	NOTE: the heap of the ESP32 is much larger than of the ESP8266. SO please be careful not to add too many 
		features to the "old" gateway modules
- As a result reworked the sensor functions and changes such as adding/changing DevAddrm NwkSKEY 
	and AppSKey parameters to several functions
- Several in-line documentaton enhancements and typos were fixed

New features in version 5.3.1 (June 30, 2018)
- Included support for T-Beam board including on board GPS sensor (_sensor.ino). #define GATEWAYNODE 1 will
	turn the gateway into a node as well. Remember to set the address etc in ESP-sc-gway.h.
- First version to explore possibilities of 433 MHz LoRa frequencies. 
	Included frequency setting in the ESP-sc-gway.h file
- Changes to the WiFi inplementation. The gateway does now store the SSID and password.
- 

New features in version 5.3.0 (June 20, 2018)
- Connect to both public and private routers


New features in version 5.2.1 (June 6, 2018)
- Repair the downlink functions
- Repair sersor functions
- Bufgixes

New features in version 5.2.0 (May 30, 2018)
- Enable support for ESP32 from TTGO, several code changes where ESP32 differs from ESP8266. 
OLED is supported but NOT tested. Some hardware specific reporting functions of the WebGUI do not work yet.
- Include new ESP32WebServer library for support of ESP32
- Made pin configuration definitions in Gateway.h file, and support in loraModem.h and .ino files.

New features in version 5.1.1 (May 17, 2018)
- The LOG button in the GUI now opens a txt .CSV file in the browser with loggin details.
- Improved debugging in WebGUI, not only based on debug level but also on part of the software we want to debug.
- Clean up of StateMachine
- Enable filesystem formatting from the GUI

New features in version 5.1.0 (May 03, 2018)
- Improved debuggin in WebGUI, not only based on debug level but also on part of the software we want to debug.
- Clean up of StateMachine

New features in version 5.0.9 (Apr 08, 2018)
- In statistics overview the option is added to specify names for known nodes (in ESP-sc-gateway.h file)
- Keep track of the amount of messages per channel (only 3 channels supported).
- Use the SPIFFS filesystem to provide log statistics of messages. Use the GUI and on the top select log. 
The log data is displayed in the USB Serial area (for the moment).
- Remove a lot of the debug==1 and debug==2 messages as they are not useful.

New features in version 5.0.8 (Mar 26, 2018)
- Simplified State machine and removed unnecessary code
- Changed the WiFi Disconnect code (bug in SDK). When WiFi.begin() is executed, 
  the previous accesspoint is not deleted. But the ESP8266 does also not connect 
  to it anymore. The workaround restarts the WiFi completely.
- Repair the bug causing the Channel setting to switch back to channel 0 when the RFM95 modem is reset 
  (after upstream message)
- Introduced the _utils.ino with Serial line utilities
- Documentation Changes
- Small bug fixes
- Removal of unused global variables

New features in version 5.0.7 (Feb 12, 2018)
- On low debug value (0) we show the time in the rx status message on 
- WlanConnect function updated

New features in version 5.0.7 (Feb 24, 2018)
- Changed WlaConnect function to not hang and give more debug info
- Made the change to display correct MAC address info

New features in version 5.0.6 (Feb 11, 2018)
- All timer functions that show lists on website etc are now based on now() en NTP, 
the realtime functions needed for LoRa messages are still based on micros() or millis()
- Change some USB debug messages
- Added to the documentation of the README.md

New Features in version 5.0.5 (Feb 2, 2018)
- Change timer functions to now() and secons instead of millis() as the latter one overflows once 
every 50 days.
- Add more debug information
- Simplified and enhanced the State Machine function

New features in version 5.0.4 (January 1, 2018)
- Cleanup of the State machine
- Separate file for oLED work, support for 1.3" SH3006 chips based oLED.
- Still not supported: Multi Frequency works, but with loss of #packages, 
  and some packages are recognizeg at the wrong frequency (but since they are so close that could happen).
- In-line documenattion cleaned up

New features in version 5.0.1 (November 18, 2017)
- Changed the state machine to run in user space only
- No Watchdog Resets anymore
- For each SF, percentage of such packages received of total packages
- OTAA and downlink work (again) although not always
- Nober of packages per hour displayed in webserver
- All Serial communication only when DUSB==1 is defined at compile time

New features in version 4.0.9 (August 11, 2017)

- This release contains updates for memory leaks in several Gateway files
- Also changes in OLED functions

New features in version 4.0.8 (August 05, 2017)

- This release updates for memory leaks in NTP routines (see ESP-sc-gway.h file for NTP_INTR
- OLED support contributed by Dorijan Morelj (based on Adreas Spies' release)

New features in version 4.0.7 (July 22, 2017)

- This release contains merely updates to memory leaks and patches to avoid chip resets etc.
- The webinterface allows the user to see more parameters and has buttons to set/reset these parameters.
- By setting debug >=2, the webinterface will display more information.
- The gateway allows OTA (Over the Air) update. Please have an Apple "Bonjour" somewhere on your network 
(included in iTunes) and you will see the network port in the "Port" section of your IDE.

New features in version 4.0.4 (June 24, 2017):

- Review of the _wwwServer.ino file. Repaired some of the bugs causing crashes of the webserver.
- Updated the README.md file with more cofniguration information

New features in version 4.0.3 (June 22, 2017):

- Added CMAC functions so that the sensor functions work as expected over TTN
- Webserver prints a page in chunks now, so that memory usage is lower and more heap is left over for variables
- Webserver does refresh every 60 seconds
- Implemented suggested change of M. for answer to PKT_PULL_RESP
- Updated README.md to correctly displa all headers
- Several small bug fixes

New features in version 4.0.0 (January 26, 2017)):

- Implement both CAD (Channel Activity Detection) and HOP functions (HOP being experimental)
- Message history visible in web interface
- Repaired the WWW server memory leak (due to String assignments)
- Still works on one interrupt line (GPIO15), or can be configured to work with 2 interrupt lines for dio0 and dio1
  for two or more interrupt lines (better performance for automatic SF setting?)
- Webserver with debug level 3 and level 4 (for interrupt testing).
  dynamic setting thorugh the web interface. Level 3 and 4 will show more info
  on sf, rssi, interrupt flags etc.
- Tested on Arduino IDE 1.18.0
- See http://things4u.github.io for documentation

New features in version 3.3.0 (January 1, 2017)):

- Redesign of the Webserver interface
- Use of the SPIFFS filesystem to store SSID, Frequency, Spreading Factor and Framecounter to survice reboots and resets of the ESP8266
- Possibility to set the Spreading Factor dynamically throug the web interface
- Possibility to set the Frequency in the web interface
- Reset the Framecounter in te webinterface

New features in version 3.2.2 (December 29, 2016)):

- Repair the situation where WIFIMANAGER was set to 0 in the ESP-sc-gway.h file. The sketch would not compile which is now repaired
- The compiler would issue a set of warnings related to the ssid and passw setting in the ESP-sc-gway.h file. Compiler was complaining (and it should) because char* were statically initialised and modified in the code.

New features in version 3.2.1 (December 20, 2016)):

- Repair the status messages to the server. All seconds, minutes, hours etc. are now reported in 2 digits. The year is reported in 4 digits.

New features in version 3.2.0 (December 08, 2016)):

- Several bugfixes

New features in version 3.1 (September 29, 2016)):

- In the ESP-sc-gway.h it is possible to set the gateway as sensor node as well. Just set the DevAddr and AppSKey in the _sensor.ino file and be able to forward any sensor or other values to the server as if they were coming from a LoRa node.
- If the #define _STRICT_1CH is set (to 1) then the system will be able to send downlink messages to LoRa nodes that are strict 1-channel devices (all frequencies but frequency 0 are disabled and Spreading Factor (SF) is fixed to one value).
- Code clean-up. The code has been made smaller in the area of loraWait() functions and where the radio is initiated for receiving of transmitting messages.
- Several small bug fixes
- Licensing, the license has been changed to MIT

New features in version 3.0 (September 27, 2016):

- WiFiManager support
- Limited SPIFF (filesystem) support for persistent data storage
- Added functions to webserver. Webserver port now 80 by default (!)

Other

- Supports ABP nodes (TeensyLC and Arduino Pro-mini)
- Supports OTAA functions on TeensyLC and Arduino Pro-Mini (not all of them) for SF7 and SF8.
- Supports SF7, SF8. SF7 is tested for downstream communication
- Listens on configurable frequency and spreading factor
- Send status updates to server (keepalive)
- PULL_DATA messages to server
- It can forward messages to two servers at the same time (and read from them as well)
- DNS support for server lookup
- NTP Support for time sync with internet time servers
- Webserver support (default port 8080)
- .h header file for configuration

Not (yet) supported:

- SF7BW250 modulation
- FSK modulation
- RX2 timeframe messages at frequency 869,525 MHz are not (yet) supported.
- SF9-SF12 downlink messaging available but needs more testing


# License

The source files of the gateway sketch in this repository is made available under the MIT
license. The libraries included in this repository are included for convenience only and all have their own license, and are not part of the ESP 1ch gateway code.
