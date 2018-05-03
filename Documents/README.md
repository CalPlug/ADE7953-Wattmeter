Note:

The boards (versions 2-4) will take 3.3 to 5V as an input DC power in per our tests.  More efficient operation is at 3.3V and what we typically use.  Preference to use 3.3V logic voltages but 5V tolerant - but output logic voltage will match supply!  Be Careful!
Mind the isolation capability of the isolator - avoid ground loops and large differences between grounds.  On V2 be careful as to the L/N orrientation for the AC input.  A second resistor divider network on V3 provides more protestion.  V4 uses a multiplexer in place of a set of networks to allow Ground Fault (GF) detection as part of a potential GFI system.

File Manifest: 

Wattmeter_v2_gerber_files.zip
------------------------------
Upload this .zip file to https://www.smart-prototyping.com/PCB-Prototyping.html to order PCBs for populating.


wattemeterBOM.xlsx (in Wattmeter_V2 Folder)
-------------------
This spreadsheet is the bill of materials for populating the PCBs.


Wattmeter_v2.brd & Wattmeter_v2.sch (in Wattmeter_V2 Folder)
------------------------------------
Open them with EagleCAD and print them out, and have them nearby while populating the PCBs to know what goes where.



Construction of Wattmeter.pdf and Stilting Components to a PCB_in_V2_MJKedit.docx and WattmeterCheckoutGuide
-----------------------------
Documents showing assembly and test procedures.
Shows what a finished Wattmeter looks like, and has some important tips on populating the PCBs.  

NOTE: There is no tri-state on the MISO line in board Version 2.  This is added into the next board revision (Board version 3.  Because of this, the board works properly when it is the only device on the SPI bus.  If more devices are used on the same SPI bus, the CS line (inverted) must be used to drive a tristate to isolate the MISO line from this board!!! 

PopulatedV2_Image.jpg
-----------------------------
Image of the blank and populated PCB of the wattmeter
