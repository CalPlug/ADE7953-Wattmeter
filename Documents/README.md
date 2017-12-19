Wattmeter_v2_gerber_files.zip
------------------------------
Upload this .zip file to https://www.smart-prototyping.com/PCB-Prototyping.html to order PCBs for populating.



wattemeterBOM.xlsx
-------------------
This spreadsheet is the bill of materials for populating the PCBs.



Wattmeter_v2.brd & Wattmeter_v2.sch
------------------------------------
Open them with EagleCAD and print them out, and have them nearby while populating the PCBs to know what goes where.



Construction of Wattmeter.pdf
-----------------------------
Shows what a finished Wattmeter looks like, and has some important tips on populating the PCBs.  

NOTE: There is no tri-state on the MISO line in board Version 2.  This is added into the next board revision (Board version 3.  Because of this, the board works properly when it is the only device on the SPI bus.  If more devices are used on the same SPI bus, the CS line (inverted) must be used to drive a tristate to isolate the MISO line from this board!!! 

IMG_20170828_063213658.jpg
-----------------------------
Image of the blank and populated PCB of the wattmeter
