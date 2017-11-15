# ADE7953-Wattmeter

Example library for operating the ADE7953 Single-Phase AC Line measurement IC over SPI interfacing 

Created by Umar Kazmi, Crystal Lai, and Michael Klopfer, Ph.D.

PCB v1 and v2 Board design by Damien Putignani

Version 3 PCB update by Yuxiang (Eric) Li	and Zihan (Bronco) Chen

University of California, Irvine - California Plug Load Research Center (CalPlug)

February 23, 2017

Copyright (C) The Regents of the University of California, 2017

Released into the public domain. This is an example C++ library for Arduino for operating the ADE7953 Single-Phase AC Line measurement IC over SPI and tested with the Arduino Uno and C++ Arduino-style compiler for the Espressif ESP8266. 

Installation
--------------------------------------------------------------------------------

To install this library, just place this entire folder as a subfolder in your
Arduino/lib/targets/libraries folder.
When installed, this library should look like:
Arduino/lib/targets/libraries/ADE7953              (this library's folder)
Arduino/lib/targets/libraries/ADE7953/ADE7953.cpp     (the library implementation file)
Arduino/lib/targets/libraries/ADE7953/ADE7953.h       (the library description file)
Arduino/lib/targets/libraries/ADE7953/examples     (the examples)
Arduino/lib/targets/libraries/ADE7953/readme.txt   (this file)

Building
--------------------------------------------------------------------------------

After this library is installed, you can start the Arduino application.
You may see a few warning messages as it's built.
To use this library in a sketch, go to the Sketch | Import Library menu and select Test. This will add a corresponding line to the top of your sketch: #include <ADE7953.h>
To stop using this library, delete that line from your sketch.

Background of Operation
--------------------------------------------------------------------------------

The ADE7953 uses registers for communication.  Depending on the type, this can be an 8 bit, 16 bit, 24 bit or 32 bit register.  Communication of over 8 bits uses sequential bytes using typical SPI byte-wise communication.  There are redundancies in how registers are accessed via different bit values for registers.  For addresses, the same command may be represented in multiple bit versions (eg a 24bit or 32 bit). When you call on a certain address, the corresponding value is returned with the value bit size corresponding to the 24 or 32 bit address called on. The responding value returns as the function called and of the same bit size.  Refer to the documentation!  For Debug, the Serial Monitor runs at 115200 baudrate when the reporting DEBUG functionality is activated.  Uncomment IFDEF statements in the library for library operation debug. 

Usage
--------------------------------------------------------------------------------

There are several layers of functionality in the library.  Functions are defined to permit direct communication via registers of different bit sizes.  One can use the functions in this library for direct communication using the send and receive capability of the register calls or use direct functions.  The direct functions are also included. In order to access the functions, you must write in the Arduino file myADE7953.getFunction(), Function being the function you want such as Vrms or IrmsA (the different names of functions can be found in the .h file), while myADE7953 calls on the library. 

Demo
--------------------------------------------------------------------------------

When connecting a 80-Watt Lightbulb with a 110V AC, 60HZ, source and using our demo code, the serial monitor reads out the following approximate measurements from the current sensor:

Vrms (V): 110

IrmsA (mA): 718

Apparent Power A (mW): 80000

Active Power A (mW): 80000

Reactive Power A (mW): 4100

Power Factor (div 100): 99.90

Active Energy A (hex): 5132 


Physical Demo Board
----------
This folder contains all the files required to construct an isolated ADE7953 wattmeter board. Please refer to the README inside the folder for more details on using the files.  The version 3 supercedes the version 2 board.  In the Version 2 revision, there is no tri-state on the MISO line on the external side of the chipset. Because of this, the board works properly when it is the only device on the SPI bus. If more devices are used on the same SPI bus, the CS line (inverted) must be used to drive a tristate to isolate the MISO line from this board.  

The Version 2 board has the neutral connected to the isolated ground.  In bench testing, this design worked well, but in general usage, several catostrophic board failures resulted (not that crazy, look at how the AD datasheet and AD demo boards are wired :) ).  Accordingly, a safer design was developed (version 3).  As previously stated, the version 2 of the board does not allow for multiple devices to be used on the SPI as the MISO line is held low when communication has ended (the isolator is always enabled, causing this).  In the updated version 3, a tri-state buffer allows proper SPI usage with multiple devices.

