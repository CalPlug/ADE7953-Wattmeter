# ADE7953-Wattmeter

Simple library for operating the ADE7953 Single-Phase AC Line measurement IC over SPI for Arduino Uno 
Created by Umar Kazmi, Crystal Lai, and Michael Klopfer, Ph.D.
February 23, 2017 
University of California, Irvine - California Plug Load Research Center (CalPlug)
Released into the public domain. This is an example C++ library for Arduino for operating the ADE7953 Single-Phase AC Line measurement IC over SPI for Arduino Uno 

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

The ADE7953 uses registers for communication.  Depending on the type, this can be an 8 bit, 16 bit, 24 bit or 32 bit register.  Communication of over 8 bits uses sequential bytes.  For addresses, the same command may be represented in multiple bit versions (eg a 24bit or 32 bit). When you call on a certain address, the corresponding value is returned with the value bit size corresponding to the 24 or 32 bit address called on. The responding value returns as the function called and of the same bit size. The Serial Monitor runs at 115200 baudrate. 

Usage
--------------------------------------------------------------------------------

There are several layers of functionality in the library.  Functions are defined to permit direct communication via registers of different bit sizes.  One can use the functions in this library for direct communication using the send and receive capability of the register calls or use direct functions.  The direct functions are also included. In order to access the functions, you must write in the Arduino file myADE7953.getFunction(), Function being the function you want such as Vrms or IrmsA (the different names of functions can be found in the .h file), while myADE7953 calls on the library. 

Demo
--------------------------------------------------------------------------------

When connecting a 80-Watt Lightbulb with a 110Voltage source and using our demo code, the serial monitor reads out the following approximate measurements from the current sensor:

Vrms (V): 110

IrmsA (mA): 718

Apparent Power A (mW): 80000

Active Power A (mW): 80000

Reactive Power A (mW): 4100

Power Factor A (x100): 99.90

Active Energy A (hex): 5132 


Documents
----------
This folder contains all the files required to construct an isolated ADE7953 wattmeter board. Please refer to the README inside the folder for more details on using the files.

NOTE: In the current board revision, there is no tri-state on the MISO line on the external side of the chipset. This must be added into the next board revision. Because of this, the board works properly when it is the only device on the SPI bus. If more devices are used on the same SPI bus, the CS line (inverted) must be used to drive a tristate to isolate the MISO line from this board!!!
