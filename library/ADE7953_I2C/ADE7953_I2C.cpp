/*
 ADE7953.cpp - Simple library for operating the ADE7953 Single-Phase AC Line measurement IC over SPI for Arduino Uno 
  Created by Umar Kazmi, Crystal Lai, and Michael Klopfer, Ph.D.
  January 23, 2017 - v6.2 (pre-release)
  May 21, 2018 - I2C version modified by Lifeng Liang
  University of California, Irvine - California Plug Load Research Center (CalPlug)
  Released into the public domain.
*/

#include "Arduino.h"
#include <Wire.h>
#include "ADE7953_I2C.h"
//#define ADE7953_VERBOSE_DEBUG //This line turns on verbose debug via serial monitor (Normally off or //'ed).  Use sparingly and in a test program!  Turning this on can take a lot of memory!  This is non-specific and for all functions, beware, it's a lot of output!  Reported bytes are in HEX
int ADE_Address = 56;//I2C Address of ADE7953, or using equivalent value in hex: 0x38 




//******************************************************************************************



//*****************ADE7953 Register Value Constants*****************//
//The #define used to save space where functions are not invoked - list of full general commands for ADE7953, not all implemented

//8-bit Registers
#define SAGCYC_8 0x000 //SAGCYC, (R/W) Default: 0x00, Unsigned, Sag lines Cycle 
#define DISNOLOAD_8 0x001 //DISNOLOAD, (R/W) Default: 0x00, Unsigned, No-load detection disable* 
#define LCYCMODE_8 0x004 //LCYCMODE, (R/W) Default: 0x40, Unsigned, Line cycle accumulation mode configuration **
#define PGA_V_8 0x007 //PGA_V, (R/W) Default: 0x00, Unsigned, Voltage channel gain configuration (Bits[2:0])  
#define PGA_IA_8 0x008 //PGA_IA, (R/W) Default: 0x00, Unsigned, Current Channel A gain configuration (Bits[2:0])  
#define PGA_IB_8 0x009 //PGA_IB, (R/W) Default: 0x00, Unsigned, Current Channel B gain configuration (Bits[2:0]) 
#define WRITE_PROTECT_8 0x040 //WRITE_PROTECT, (R/W) Default: 0x00, Unsigned, Write protection bits (Bits[2:0]) 
#define LAST_OP_8 0x0FD //LAST_OP, (R/W) Default: 0x00, Unsigned, Contains the type (read or write) of the last successful communication (0x35 read 0xCA = write) 
#define LAST_RWDATA_8 0x0FF //LAST_RWDATA_8, (R/W) Default: 0x00, Unsigned, Contains the data from the last successful 8-bit register communication  
#define Version_8 0x702 //Version, (R/W) Default: N/A, Unsigned, Contains the silicon version number 
#define EX_REF_8 0x800 //EX_REF, (R/W) Default: 0x00, Unsigned, Reference input configuration:0 = internal 1 = external 

//*DISNOLOAD register

//**LCYCMODE register


//16-bit Registers
#define ZXTOUT_16 0x100 //ZXTOUT, (R/W) Default:0xFFFF, Unsigned,Zero-crossing timeout
#define LINECYC_16 0x101 //LINCYC, (R/W) Default:0x0000, Unsigned,Number of half line cycles for line cycle energy accumulation mode
#define CONFIG_16 0x102 //CONFIG, (R/W) Default:0x8004, Unsigned,Configuration register***
#define CF1DEN_16 0x103 //CF1DEN, (R/W) Default:0x003F, Unsigned,CF1 frequency divider denominator. When modifying this register, two sequential write operations must be performed to ensure that the write is successful. 
#define CF2DEN_16 0x104 //CF2DEN, (R/W) Default:0x003F, Unsigned,CF2 frequency divider denominator. When modifying this register, two sequential write operations must be performed to ensure that the write is successful. 
#define CFMODE_16 0x107 //CFMODE, (R/W) Default:0x0300, Unsigned, CF output selection */
#define PHCALA_16 0x108 //PHCALA, (R/W) Default:0x0000, Signed,Phase calibration register (Current Channel A). This register is in sign magnitude format. 
#define PHCALB_16 0x109 //PHCALB, (R/W) Default:0x0000, Signed,Phase calibration register (Current Channel B). This register is in sign magnitude format. 
#define PFA_16 0x10A //PFA, (R) Default:0x0000, Signed,Power factor (Current Channel A) 
#define PFB_16 0x10B //PFB, (R) Default:0x0000, Signed,Power factor (Current Channel B) 
#define ANGLE_A_16 0x10C //ANGLE_A, (R) Default:0x0000, Signed,Angle between the voltage input and the Current Channel A input 
#define ANGLE_B_16 0x10D //ANGLE_B, (R) Default:0x0000, Signed,Angle between the voltage input and the Current Channel B input 
#define Period_16 0x11E //Period, (R) Default:0x0000, Unsigned, Period register 
#define ALT_OUTPUT_16 0x110 //ALT_OUTPUT, (R/W) Default:0x0000, Unsigned,Alternative output functions**/
#define LAST_ADD_16 0x1FE //LAST_ADD, (R) Default:0x0000, Unsigned, Contains the address of the last successful communication 
#define LAST_RWDATA_16 0x1FF //LAST_RWDATA_16, (R) Default:0x0000, Unsigned,Contains the data from the last successful 16-bit register communication 
#define Reserved_16 0x120 //Reserved, (R/W) Default:0x0000, Unsigned,This register should be set to 30h to meet the performance specified in Table 1. To modify this register, it must be unlocked by setting Register Address 0xFE to 0xAD immediately prior. (16 bit)


//*** CONFIG register
//*/ CFMODE register
//**/ALT_OUTPUT register


//24-bit and 32-bit registers
#define SAGLVL_24 0x200 //SAGLVL, (R/W) Default: 0x000000, Unsigned, Sag Voltage Level (24 bit)
#define SAGLVL_32 0x300 //SAGLVL, (R/W) Default: 0x000000, Unsigned, Sag Voltage Level (32 bit)
#define ACCMODE_24 0x201 //ACCMODE, (R/W) Default:0x000000, Unsigned, Accumulation mode(24 bit)
#define ACCMODE_32 0x301 //ACCMODE, (R/W) Default: 0x000000, Unsigned, Accumulation mode(32 bit)
#define AP_NOLOAD_24 0x203 //AP_NOLOAD, (R/W) Default: 0x00E419, Unsigned,Active power no-load level(24 bit)
#define AP_NOLOAD_32 0x303 //AP_NOLOAD, (R/W) Default: 0x00E419, Unsigned,Active power no-load level(32 bit)
#define VAR_NOLOAD_24 0x204 //VAR_NOLOAD, (R/W) Default: 0x000000, Unsigned,Reactive power no-load level(24 bit)
#define VAR_NOLOAD_32 0x304 //VAR_NOLOAD, (R/W) Default: 0x000000, Unsigned,Reactive power no-load level(32 bit)
#define VA_NOLOAD_24 0x205 //VA_NOLOAD, (R/W) Default: 0x000000, Unsigned,Apparent power no-load level(24 bit)
#define VA_NOLOAD_32 0x305 //VA_NOLOAD, (R/W) Default: 0x000000, Unsigned,Apparent power no-load level(32 bit)
#define AVA_24 0x210 //AVA, (R) Default: 0x000000, Signed,Instantaneous apparent power (Current Channel A)(24 bit)
#define AVA_32 0x310 //AVA, (R) Default: 0x000000, Signed,Instantaneous apparent power (Current Channel A)(32 bit)
#define BVA_24 0x211 //BVA, (R) Default: 0x000000, Signed,Instantaneous apparent power (Current Channel B)(24 bit)
#define BVA_32 0x311 //BVA, (R) Default: 0x000000, Signed,Instantaneous apparent power (Current Channel B)(32 bit)
#define AWATT_24 0x212 //AWATT, (R) Default: 0x000000, Signed,Instantaneous active power (Current Channel A)(24 bit)
#define AWATT_32 0x312 //AWATT, (R) Default: 0x000000, Signed,Instantaneous active power (Current Channel A)(32 bit)
#define BWATT_24 0x213 //BWATT, (R) Default: 0x000000, Signed,Instantaneous active power (Current Channel B)(24 bit)
#define BWATT_32 0x313 //BWATT, (R) Default: 0x000000, Signed,Instantaneous active power (Current Channel B)(32 bit)
#define AVAR_24 0x214 //AVAR, (R) Default: 0x000000, Signed,Instantaneous reactive power (Current Channel A)(24 bit)
#define AVAR_32 0x314 //AVAR, (R) Default: 0x000000, Signed,Instantaneous reactive power (Current Channel A)(32 bit)
#define BVAR_24 0x215 //BVAR, (R) Default: 0x000000, Signed,Instantaneous reactive power (Current Channel B)(24 bit)
#define BVAR_32 0x315 //BVAR, (R) Default: 0x000000, Signed,Instantaneous reactive power (Current Channel B)(32 bit)
#define IA_24 0x216 //IA, (R) Default: 0x000000, Signed, Instantaneous current (Current Channel A)(24 bit)
#define IA_32 0x316 //IA, (R) Default: 0x000000, Signed,Instantaneous current (Current Channel A)(32 bit)
#define IB_24 0x217 //IB, (R) Default: 0x000000, Signed,Instantaneous current (Current Channel B)(24 bit)
#define IB_32 0x317 //IB, (R) Default: 0x000000, Signed,Instantaneous current (Current Channel B)(32 bit)
#define V_24 0x218 //V, (R) Default: 0x000000, Signed,Instantaneous voltage (voltage channel)(24 bit)
#define V_32 0x318 //V, (R) Default: 0x000000, Signed,Instantaneous voltage (voltage channel)(32 bit)
#define IRMSA_24 0x21A //IRMSA, (R) Default: 0x000000, Unsigned,IRMS register (Current Channel A)(24 bit)
#define IRMSA_32 0x31A //IRMSA, (R) Default: 0x000000, Unsigned,IRMS register (Current Channel A)(32 bit)
#define IRMSB_24 0x21B //IRMSB, (R) Default: 0x000000, Unsigned,IRMS register (Current Channel B)(24 bit)
#define IRMSB_32 0x31B //IRMSB, (R) Default: 0x000000, Unsigned,IRMS register (Current Channel B)(32 bit)
#define VRMS_24 0x21C //VRMS, (R) Default: 0x000000, Unsigned, VRMS register (24 bit)
#define VRMS_32 0x31C //VRMS, (R) Default: 0x000000, Unsigned, VRMS register (32 bit)

#define AENERGYA_24 0x21E //AENERYGA, (R) Default: 0x000000, Signed,Active energy (Current Channel A) (24 bit)
#define AENERGYA_32 0x31E //AENERYGA, (R) Default: 0x000000, Signed,Active energy (Current Channel A)(32 bit)
#define AENERGYB_24 0x21F //AENERYGB, (R) Default: 0x000000, Signed,Active energy (Current Channel B)(24 bit)
#define AENERGYB_32 0x31F //AENERYGB, (R) Default: 0x000000, Signed,Active energy (Current Channel B)(32 bit)
#define RENERGYA_24 0x220 //RENERGYA, (R) Default: 0x000000, Signed,Reactive energy (Current Channel A) (24 bit)
#define RENERGYA_32 0x320 //RENERGYA, (R) Default: 0x000000, Signed,Reactive energy (Current Channel A)(32 bit)
#define RENERGYB_24 0x221 //RENERGYB, (R) Default: 0x000000, Signed,Reactive energy (Current Channel B) (24 bit)
#define RENERGYB_32 0x321 //RENERGYB, (R) Default: 0x000000, Signed,Reactive energy (Current Channel B)(32 bit)
#define APENERGYA_24 0x222 //APENERGYA, (R) Default: 0x000000, Signed,Apparent energy (Current Channel A) (24 bit)
#define APENERGYA_32 0x322 //APENERGYA, (R) Default: 0x000000, Signed,Apparent energy (Current Channel A)(32 bit)
#define APENERGYB_24 0x223 //APENERGYB, (R) Default: 0x000000, Signed,Apparent energy (Current Channel B)(24 bit)
#define APENERGYB_32 0x323 //APENERGYB, (R) Default: 0x000000, Signed,Apparent energy (Current Channel B)(32 bit)
#define OVLVL_24 0x224 //OVLVL, (R/W) Default: 0xFFFFFF, Unsigned, Overvoltage level(24 bit)
#define OVLVL_32 0x324 //OVLVL, (R/W) Default: 0xFFFFFF, Unsigned,Overvoltage level(32 bit)

#define OILVL_24 0x225 //OILVL, (R/W) Default: 0xFFFFFF,Unsigned, Overcurrent level (24 bit)
#define OILVL_32 0x325 //OILVL, (R/W) Default: 0xFFFFFF, Unsigned,Overcurrent level (32 bit)

#define VPEAK_24 0x226 //VPEAK, (R) Default: 0x000000, Unsigned, Voltage channel peak(24 bit)
#define VPEAK_32 0x326 //VPEAK, (R) Default: 0x000000, Unsigned,Voltage channel peak(32 bit)
#define RSTVPEAK_24 0x227 //RSTVPEAK, (R) Default: 0x000000, Unsigned,Read voltage peak with reset (24 bit)
#define RSTVPEAK_32 0x327 //RSTVPEAK, (R) Default: 0x000000, Unsigned,Read voltage peak with reset(32 bit)
#define IAPEAK_24 0x228 //IAPEAK, (R) Default: 0x000000, Unsigned,Current Channel A peak(24 bit)
#define IAPEAK_32 0x328 //IAPEAK, (R) Default: 0x000000, Unsigned,Current Channel A peak(32 bit)
#define RSTIAPEAK_24 0x229 //RSTIAPEAK, (R) Default: 0x000000, Unsigned, Read Current Channel A peak with reset(24 bit)
#define RSTIAPEAK_32 0x329 //RSTIAPEAK, (R) Default: 0x000000, Unsigned,Read Current Channel A peak with reset(32 bit)
#define IBPEAK_24 0x22A //IBPEAK, (R) Default: 0x000000, Unsigned, Current Channel B peak(24 bit)
#define IBPEAK_32 0x32A //IBPEAK, (R) Default: 0x000000, Unsigned,Current Channel B peak(32 bit)
#define RSTIBPEAK_24 0x22B //RSTIBPEAK, (R) Default: 0x000000, Unsigned, Read Current Channel B peak with reset(24 bit)
#define RSTIBPEAK_32 0x32B //RSTIBPEAK, (R) Default: 0x000000, Unsigned,Read Current Channel B peak with reset(32 bit)
#define IRQENA_24 0x22C //IRQENA, (R/W) Default: 0x100000, Unsigned,Interrupt enable (Current Channel A (24 bit)
#define IRQENA_32 0x32C //IRQENA, (R/W) Default: 0x100000, Unsigned,Interrupt enable (Current Channel A(32 bit)
#define IRQSTATA_24 0x22D //IRQSTATA, (R) Default: 0x000000, Unsigned, Interrupt status (Current Channel A(24 bit)
#define IRQSTATA_32 0x32D //IRQSTATA, (R) Default: 0x000000, Unsigned,Interrupt status (Current Channel A(32 bit)
#define RSTIRQSTATA_24 0x22E //RSTIRQSTATA, (R) Default: 0x000000, Unsigned, Reset interrupt status (Current Channel A) (24 bit)
#define RSTIRQSTATA_32 0x32E //RSTIRQSTATA, (R) Default: 0x000000, Unsigned,Reset interrupt status (Current Channel A)(32 bit)
#define IRQENB_24 0x22F //IRQENB, (R/W) Default: 0x000000, Unsigned,Interrupt enable (Current Channel B (24 bit)
#define IRQENB_32 0x32F //IRQENB, (R/W) Default: 0x000000, Unsigned,Interrupt enable (Current Channel B (32 bit)
#define IRQSTATB_24 0x230 //IRQSTATB, (R) Default: 0x000000, Unsigned, Interrupt status (Current Channel B(24 bit)
#define IRQSTATB_32 0x330 //IRQSTATB, (R) Default: 0x000000, Unsigned,Interrupt status (Current Channel B(32 bit)
#define RSTIRQSTATB_24 0x231 //RSTIRQSTATB, (R) Default: 0x000000, Unsigned,Reset interrupt status (Current Channel B) (24 bit)
#define RSTIRQSTATB_32 0x331 //RSTIRQSTATB, (R) Default: 0x000000, Unsigned, Reset interrupt status (Current Channel B)(32 bit)
#define CRC_24 0x000 //CRC, (R) Default: 0x000000, Unsigned, Checksum(24 bit)
#define CRC_32 0x37F //CRC, (R) Default: 0xFFFFFF, Unsigned,Checksum(32 bit)
#define AIGAIN_24 0x280 //AIGAIN, (R/W) Default: 0x400000, Unsigned, Current channel gain (Current Channel A)(24 bit)
#define AIGAIN_32 0x380 //AIGAIN, (R/W) Default: 0x400000, Unsigned,Current channel gain (Current Channel A)(32 bit)
#define AVGAIN_24 0x281 //AVGAIN, (R/W) Default: 0x400000, Unsigned, Voltage channel gain(24 bit)
#define AVGAIN_32 0x381 //AVGAIN, (R/W) Default: 0x400000, Unsigned,Voltage channel gain(32 bit)
#define AWGAIN_24 0x282 //AWGAIN, (R/W) Default: 0x400000, Unsigned,Active power gain (Current Channel A)(24 bit)
#define AWGAIN_32 0x382 //AWGAIN, (R/W) Default: 0x400000, Unsigned,Active power gain (Current Channel A)(32 bit)
#define AVARGAIN_24 0x283 //AVARGAIN, (R/W) Default: 0x400000, Unsigned, Reactive power gain (Current Channel A)(24 bit)
#define AVARGAIN_32 0x383 //AVARGAIN, (R/W) Default: 0x400000, Unsigned, Reactive power gain (Current Channel A)(32 bit)
#define AVAGAIN_24 0x284 //AVAGAIN, (R/W) Default: 0x400000, Unsigned, Apparent power gain (Current Channel A) (24 bit)
#define AVAGAIN_32 0x384 //AVAGAIN, (R/W) Default: 0x400000, Unsigned,Apparent power gain (Current Channel A)(32 bit)
#define Reserved_24 0x285 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified (24 bit)
#define Reserved_32 0x385 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified(32 bit)


#define AIRMSOS_24 0x286 //AIRMSOS, (R/W) Default: 0x000000, Signed,IRMS offset (Current Channel A) (24 bit)
#define AIRMSOS_32 0x386 //AIRMSOS, (R/W) Default: 0x000000, Signed,IRMS offset (Current Channel A)(32 bit)
#define Reserved1_24 0x287 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified (24 bit)
#define Reserved1_32 0x387 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified(32 bit)

#define VRMSOS_24 0x288 //VRMSOS, (R/W) Default: 0x000000, Signed, VRMS offset(24 bit)
#define VRMSOS_32 0x388 //VRMSOS, (R/W) Default: 0x000000, Signed,VRMS offset(32 bit)
#define AWATTOS_24 0x289 //AWATTOS, (R/W) Default: 0x000000, Signed, Active power offset correction (Current Channel A)(24 bit)
#define AWATTOS_32 0x389 //AWATTOS, (R/W) Default: 0x000000, Signed,Active power offset correction (Current Channel A)(32 bit)
#define AVAROS_24 0x28A //AVAROS, (R/W) Default: 0x000000, Signed, Reactive power offset correction (Current Channel A)(24 bit)
#define AVAROS_32 0x38A //AVAROS, (R/W) Default: 0x000000, Signed, Reactive power offset correction (Current Channel A)(32 bit)
#define AVAOS_24 0x28B //AVAOS, (R/W) Default: 0x000000, Signed, Apparent power offset correction (Current Channel A(24 bit)
#define AVAOS_32 0x38B //AVAOS, (R/W) Default: 0x000000, Signed,Apparent power offset correction (Current Channel A(32 bit)
#define BIGAIN_24 0x28C //BIGAIN, (R/W) Default: 0x400000, Unsigned,Current channel gain (Current Channel B) (24 bit)
#define BIGAIN_32 0x38C //BIGAIN, (R/W) Default: 0x400000, Unsigned,Current channel gain (Current Channel B)(32 bit)
#define BVGAIN_24 0x28D //BVGAIN, (R/W) Default: 0x400000, Unsigned, This register should not be modified(24 bit)
#define BVGAIN_32 0x38D //BVGAIN, (R/W) Default: 0x400000, Unsigned,This register should not be modified(32 bit)
#define BWGAIN_24 0x28E //BWGAIN, (R/W) Default: 0x400000, Unsigned, Active power gain (Current Channel B)(24 bit)
#define BWGAIN_32 0x38E //BWGAIN, (R/W) Default: 0x400000, Unsigned,Active power gain (Current Channel B)(32 bit)
#define BVARGAIN_24 0x28F //BVARGAIN, (R/W) Default: 0x400000, Unsigned, Reactive power gain (Current Channel B)(24 bit)
#define BVARGAIN_32 0x38F //BVARGAIN, (R/W) Default: 0x400000, Unsigned,Reactive power gain (Current Channel B)(32 bit)
#define BVAGAIN_24 0x290 //BVAGAIN, (R/W) Default: 0x400000, Unsigned, Apparent power gain (Current Channel B)(24 bit)
#define BVAGAIN_32 0x390 //BVAGAIN, (R/W) Default: 0x400000, Unsigned,Apparent power gain (Current Channel B)(32 bit)

#define Reserved2_24 0x291 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified (24 bit)
#define Reserved2_32 0x391 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified(32 bit)

#define BIRMSOS_24 0x292 //BIRMSOS, (R/W) Default: 0x000000, Unsigned, IRMS offset (Current Channel B)(24 bit)
#define BIRMSOS_32 0x392 //BIRMSOS, (R/W) Default: 0x000000, Unsigned,IRMS offset (Current Channel B)(32 bit)

#define Reserved3_24 0x293 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified (24 bit)
#define Reserved3_32 0x393 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified(32 bit)
#define Reserved4_24 0x294 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified (24 bit)
#define Reserved4_32 0x394 //Reserved, (R/W) Default: 0x000000, Signed,This register should not be modified(32 bit)

#define BWATTOS_24 0x295 //BWATTOS, (R/W) Default: 0x000000, Unsigned, Active power offset correction (Current Channel B)(24 bit)
#define BWATTOS_32 0x395 //BWATTOS, (R/W) Default: 0x000000, Unsigned,Active power offset correction (Current Channel B)(32 bit)
#define BVAROS_24 0x296 //BVAROS, (R/W) Default: 0x000000, Unsigned,Reactive power offset correction (Current Channel B)(24 bit)
#define BVAROS_32 0x396 //BVAROS, (R/W) Default: 0x000000, Unsigned,Reactive power offset correction (Current Channel B)(32 bit)
#define BVAOS_24 0x297 //BVAOS, (R/W) Default: 0x000000, Unsigned, Apparent power offset correction (Current Channel B)(24 bit)
#define BVAOS_32 0x397 //BVAOS, (R/W) Default: 0x000000, Unsigned,Apparent power offset correction (Current Channel B)(32 bit)
#define LAST_RWDATA_24 0x2FF //LAST_RWDATA, (R) Default: 0x000000, Unsigned, Contains the data from the last successful 24-bit/32-bit register communication(24 bit)
#define LAST_RWDATA_32 0x3FF //LAST_RWDATA, (R) Default: 0x000000, Unsigned, Contains the data from the last successful 24-bit/32-bit register communication(32 bit)

//*******************************************************************************************


//****************User Program Functions*****************


uint8_t ADE7953::getVersion(){
  return i2cAlgorithm8_read(functionBitVal(Version_8,1), functionBitVal(Version_8,0));  //An example of the address lookup - the spiAlgorithm8_read((functionBitVal(addr,1), functionBitVal(addr,1)) would return the eqivenet to spiAlgorithm8_read(0x07,0x02) when working properly
}

float ADE7953::getPowerFactorA(){  
	int16_t value=0;  
	value=i2cAlgorithm16_read((functionBitVal(PFA_16,1)),(functionBitVal(PFA_16,0))); 
	float decimal = decimalize(value, 327.67, 0);
return abs(decimal);
  }     

int16_t ADE7953::getPhaseCalibA(){  
	int16_t value=0;  
	value=i2cAlgorithm16_read((functionBitVal(PHCALA_16,1)),(functionBitVal(PHCALA_16,0))); 
return value;
  }   

float ADE7953::getPeriod(){  
	uint16_t value=0;  
	value=i2cAlgorithm16_read((functionBitVal(Period_16,1)),(functionBitVal(Period_16,0))); 
	float decimal = decimalize(value, 1, 0);
return decimal;
  }

unsigned long ADE7953::getAPNOLOAD(){  //use signed long for signed registers, and unsigned long for unsigned registers
	unsigned long value=0;  //use signed long for signed registers, and unsigned long for unsigned registers
	value=i2cAlgorithm32_read((functionBitVal(AP_NOLOAD_32,1)),(functionBitVal(AP_NOLOAD_32,0))); //Call MSB and LSB from the register constant (template for how all functions should be called)
return value;
  }
  
long ADE7953::getInstVoltage(){  
	long value=0;  
	value=i2cAlgorithm32_read((functionBitVal(V_32,1)),(functionBitVal(V_32,0)));
return value;
  }
  
float ADE7953::getVrms(){  
	unsigned long value=0;  
	value=i2cAlgorithm32_read((functionBitVal(VRMS_32,1)),(functionBitVal(VRMS_32,0)));
	float decimal = decimalize(value, 19090, 0);
return decimal;
  }  
  
long ADE7953::getInstCurrentA(){  
	long value=0;  
	value=i2cAlgorithm32_read((functionBitVal(IA_32,1)),(functionBitVal(IA_32,0))); 
return value;
  }
  
float ADE7953::getIrmsA(){  
	unsigned long value=0;  
	value=i2cAlgorithm32_read((functionBitVal(IRMSA_32,1)),(functionBitVal(IRMSA_32,0))); 
	float decimal = decimalize(value, 1327, 0);
return decimal;
  }
  
unsigned long ADE7953::getVpeak(){  
	unsigned long value=0;  
	value=i2cAlgorithm32_read((functionBitVal(VPEAK_32,1)),(functionBitVal(VPEAK_32,0))); 
return value;
  }

unsigned long ADE7953::getIpeakA(){  
	unsigned long value=0;  
	value=i2cAlgorithm32_read((functionBitVal(IAPEAK_32,1)),(functionBitVal(IAPEAK_32,0))); 
return value;
  }

long ADE7953::getActiveEnergyA(){  
	long value=0; 
	value=i2cAlgorithm32_read((functionBitVal(AENERGYA_32,1)),(functionBitVal(AENERGYA_32,0))); 
return value;
  }
  
long ADE7953::getReactiveEnergyA(){  
	long value=0;  
	value=i2cAlgorithm32_read((functionBitVal(RENERGYA_32,1)),(functionBitVal(RENERGYA_32,0))); 
return value;
  }

long ADE7953::getApparentEnergyA(){ 
	long value=0;  
	value=i2cAlgorithm32_read((functionBitVal(APENERGYA_32,1)),(functionBitVal(APENERGYA_32,0))); 
return value;
  }
    
float ADE7953::getInstApparentPowerA(){  
	long value=0;  
	value=i2cAlgorithm32_read((functionBitVal(AVA_32,1)),(functionBitVal(AVA_32,0))); 
	float decimal = decimalize(value, 1.502, 0);
return abs(decimal);
  }
  
float ADE7953::getInstActivePowerA(){  
	long value=0;  
	value=i2cAlgorithm32_read((functionBitVal(AWATT_32,1)),(functionBitVal(AWATT_32,0))); 
	float decimal = decimalize(value, 1.502, 0);
return abs(decimal);
  }
  
float ADE7953::getInstReactivePowerA(){  
	long value=0;  
	value=i2cAlgorithm32_read((functionBitVal(AVAR_32,1)),(functionBitVal(AVAR_32,0))); 
	float decimal = decimalize(value, 1.502, 0);
return decimal;
  }
  
//*******************************************************


//****************ADE 7953 Library Control Functions**************************************

//****************Object Definition*****************
ADE7953::ADE7953(int CLK, int CS)
{
  _CLK=CLK;
  _CS=CS;
  }
//**************************************************

//****************Initialization********************
void ADE7953::initialize(){
    
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("ADE7953:initialize function started \n"); 
  #endif
  
  Wire.begin();
  delay(50);
  pinMode(_CS, OUTPUT);  
  pinMode(_CLK, OUTPUT); 
  digitalWrite(_CS, HIGH);// set CS & CLK pin HIGH for autodection of ADE7953 in I2C communication mode
  digitalWrite(_CLK, HIGH);

  delay(50);
  
  //LOCKING THE COMMUNICATION INTERFACE
  Wire.beginTransmission(ADE_Address);
  Wire.write(0x01);//Address 0x102, MSB first
  Wire.write(0x02);
  Wire.write(0x20);//COMM_LOCK is bit 15, default is 1, need to be set to 0 to lock the communication interface. Keep the other bits in its default setup value.
  Wire.write(0x00);
  Wire.endTransmission();
  delayMicroseconds(5);//Bus-free time minimum 4.7us
  
  
  
  Wire.beginTransmission(ADE_Address);
  Wire.write(0x00);	//Pass in MSB of register 0x00FE first.
  Wire.write(0xFE);	//Pass in LSB of register 0x00FE next.
  Wire.write(0x00);
  Wire.write(0xAD);
  Wire.endTransmission();
  delayMicroseconds(5);//Bus-free time minimum 4.7us
  
  
  Wire.beginTransmission(ADE_Address);
  Wire.write(0x01); //Pass in MSB of register 0x0120 first.
  Wire.write(0x20);	//Pass in LSB of register 0x0120 next.
  Wire.write(0x00); //Pass in MSB of 0x0030 first to write to 0x0120.
  Wire.write(0x30); //Pass in LSB of 0x0030 next to write to 0x0120.
  Wire.endTransmission(); 
  delayMicroseconds(5);//Bus-free time minimum 4.7us
  
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print(" ADE7953:initialize function completed"); 
  #endif
  
  //Calibrations
  //i2cAlgorithm16_write((functionBitVal(PHCALA_16,1)),(functionBitVal(PHCALA_16,0)),0x00,0x00);
  //delay(100);
  i2cAlgorithm32_write((functionBitVal(AP_NOLOAD_32,1)),(functionBitVal(AP_NOLOAD_32,0)),0x00,0x00,0x00,0x01); //Check for ensuring read and write operations are okay
  delay(100);
  i2cAlgorithm8_write((functionBitVal(LCYCMODE_8,1)),(functionBitVal(LCYCMODE_8,0)),0b01111111); //Enable line cycle accumulation mode for all energies and channels
  delay(100);
  i2cAlgorithm16_write((functionBitVal(LINECYC_16,1)),(functionBitVal(LINECYC_16,0)),0x00,0x78); //Sets number of half line cycle accumulations to 120
  delay(100);

}
//**************************************************

byte ADE7953::functionBitVal(int addr, uint8_t byteVal){
//Returns as integer an address of a specified byte - basically a byte controlled shift register with "byteVal" controlling the byte that is read and returned
  int x = ((addr >> (8*byteVal)) & 0xff);
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::functionBitVal function details: ");
   Serial.print("\nAddress input (dec): ");  
   Serial.print(addr, DEC);
   Serial.print("\n Byte requested (dec): ");  
   Serial.print(byteVal, DEC);
   Serial.print("\n Returned Value (dec): ");
   Serial.print(x, DEC);  
   Serial.print(" Returned Value (HEX): ");
   Serial.print(x, HEX); 
   Serial.print("\n ADE7953::functionBitVal function completed "); 
  #endif
  
  return x;
}


uint8_t ADE7953::i2cAlgorithm8_read(byte MSB, byte LSB) { //This is the algorithm that reads from a register in the ADE7953. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\n ADE7953::i2cAlgorithm8_read function started "); 
  #endif
  uint8_t readval_unsigned = 0;  //This variable is the unsigned integer value to compile read bytes into (if needed)
  byte one;
  
  Wire.beginTransmission(ADE_Address);
  Wire.write(MSB);
  Wire.write(LSB); 
  Wire.endTransmission(0);
  
  Wire.requestFrom(ADE_Address,1);	//Request 1 Byte from the specified address
  if(Wire.available() >= 1){	//Wait for response
  one = Wire.read();
  }
  //Wire.endTransmission();//prevent bus being taken from other devices
  
    #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::i2cAlgorithm8_read function details: ");
   Serial.print("\nAddress Byte 1(MSB)[HEX]: ");  
   Serial.print(MSB, HEX);
   Serial.print("\n Address Byte 2(LSB)[HEX]: ");  
   Serial.print(LSB, HEX);
   Serial.print("\n Returned bytes (1 for 8-bit return form): ");
   Serial.print(one, HEX);
   Serial.print("\n ADE7953::i2cAlgorithm8_read function completed "); 
  #endif
  
  //Post-read packing and bitshifting operation
    readval_unsigned = one;  //Process MSB (nothing much to see here for only one 8 bit value)
  
	return readval_unsigned;  //uint8_t versus long because it is only an 8 bit value, function returns uint8_t.
 } 

uint16_t ADE7953::i2cAlgorithm16_read(byte MSB, byte LSB) { //This is the algorithm that reads from a register in the ADE7953. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::i2cAlgorithm16_read function started "); 
  #endif
  uint16_t readval_unsigned = 0;  //This variable is the unsigned integer value to compile read bytes into (if needed)
  byte one;
  byte two;
  
  Wire.beginTransmission(ADE_Address);
  Wire.write(MSB);//Pass in MSB first 
  Wire.write(LSB); 
  Wire.endTransmission(0);//prevent bus being taken from other devices
  Wire.requestFrom(ADE_Address,2);	//Request 2 Byte from the specified address
  if (2 <= Wire.available()) { // if two bytes were received
  one = Wire.read();//read MSB 
  two = Wire.read();//read LSB
  }
  
  
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::i2cAlgorithm16_read function details: ");
   Serial.print("\nAddress Byte 1(MSB)[HEX]: ");  
   Serial.print(MSB, HEX);
   Serial.print("\n Address Byte 2(LSB)[HEX]: ");  
   Serial.print(LSB, HEX);
   Serial.print("\n Returned bytes (1(MSB) and 2) [HEX]: ");
   Serial.print(one, HEX);
   Serial.print(" ");
   Serial.print(two, HEX);
   Serial.print("\n ADE7953::i2cAlgorithm16_read function completed "); 
  #endif
   
   //Post-read packing and bitshifting operation
   //readval_unsigned = (((uint32_t) one << 8) + ((uint32_t) two));  //(Alternate bitshift algorithm)
   
   readval_unsigned = (one << 8);  //Process MSB  (Alternate bitshift algorithm)
   readval_unsigned = readval_unsigned + two;  //Process LSB
			   
			return readval_unsigned;	
    }

uint32_t ADE7953::i2cAlgorithm24_read(byte MSB, byte LSB) { //This is the algorithm that reads from a register in the ADE7953. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::i2cAlgorithm24_read function started "); 
  #endif 
  //long readval_signed=0;
  uint32_t readval_unsigned = 0;  //This variable is the unsigned integer value to compile read bytes into (if needed)
  byte one;
  byte two;
  byte three;

  Wire.beginTransmission(ADE_Address);
  Wire.write(MSB);//Pass in MSB of register first
  Wire.write(LSB); 
  Wire.endTransmission(0);//prevent bus being taken from other devices
  Wire.requestFrom(ADE_Address,3);	//Request 3 Byte from the specified address
  if (3 <= Wire.available()) { // if three bytes were received
  one = Wire.read();//read MSB 
  two = Wire.read();
  three = Wire.read();//read LSB
  }
  
  
 #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::i2cAlgorithm24_read function details: ");
   Serial.print("\nAddress Byte 1(MSB)[HEX]: ");  
   Serial.print(MSB, HEX);
   Serial.print(" Address Byte 2(LSB)[HEX]: ");  
   Serial.print(LSB, HEX);
   Serial.print("\n Returned bytes (1(MSB) to 3)[HEX]: ");
   Serial.print(one, HEX);
   Serial.print(" ");
   Serial.print(two, HEX);
   Serial.print(" ");
   Serial.print(three, HEX);
   Serial.print("\n ADE7953::i2cAlgorithm24_read function completed "); 
  #endif

  //Post-read packing and bitshifting operation
  readval_unsigned = (((uint32_t) one << 16)+ ((uint32_t) two << 8) + ((uint32_t) three)); //(Alternative shift algorithm)
   

   
 // readval_unsigned =  ((one << 16) & 0x00FF0000);  //process MSB  //(Alternative shift algorithm)
 // readval_unsigned = readval_unsigned + ((two << 8) & 0X0000FF00);
 // readval_unsigned = readval_unsigned + (three & 0X000000FF);  //Process LSB

			return readval_unsigned;
  }
  
uint32_t ADE7953::i2cAlgorithm32_read(byte MSB, byte LSB) { //This is the algorithm that reads from a 32 bit register in the ADE7953. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.  Caution, some register elements contain information that is only 24 bit with padding on the MSB
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::i2cAlgorithm32_read function started "); 
  #endif 
  uint32_t readval_unsigned = 0;  //This variable is the unsigned integer value to compile read bytes into (if needed)
  byte one;
  byte two;
  byte three;
  byte four;

  Wire.beginTransmission(ADE_Address);
  Wire.write(MSB);
  Wire.write(LSB); 
  Wire.endTransmission(0);//prevent bus being taken from other devices
  Wire.requestFrom(ADE_Address,4);	//Request 4 Byte from the specified address
  if (4 <= Wire.available()) {	//Wait for response
  one = Wire.read();//read MSB 
  two = Wire.read();
  three = Wire.read();
  four = Wire.read();//read LSB
  }
 
  
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::i2cAlgorithm32_read function details: ");
   //Serial.print("Address Byte 1(MSB)[HEX]: ");  
   //Serial.print(MSB, BIN);
   //Serial.print(" Address Byte 2(LSB)[HEX]: ");  
   //Serial.print(LSB, BIN);
   Serial.print("\n Returned bytes (1(MSB) to 4)[HEX]: ");
   Serial.print(one, BIN);
   Serial.print(" ");
   Serial.print(two, BIN);
   Serial.print(" ");
   Serial.print(three, BIN);
   Serial.print(" ");
   Serial.print(four, BIN);
   Serial.print("\n ADE7953::i2cAlgorithm32_read function completed "); 
  #endif
  
  //Post-read packing and bitshifting operation
  readval_unsigned = (((uint32_t) one << 24)+ ((uint32_t) two << 16) + ((uint32_t) three << 8) + (uint32_t) four);
  
/*   readval_unsigned = (one << 24);  //Process MSB 
  //Serial.println(readval_unsigned, HEX);  
  readval_unsigned = readval_unsigned + (two << 16);  
  //Serial.println(readval_unsigned, HEX);
  readval_unsigned = readval_unsigned + (three << 8);  
  //Serial.println(readval_unsigned, HEX);
  readval_unsigned = (readval_unsigned + (four));  //Process LSB
  Serial.println(readval_unsigned, BIN);  */

  return readval_unsigned;
}


void ADE7953::i2cAlgorithm32_write(byte MSB, byte LSB, byte onemsb, byte two, byte three, byte fourlsb) { //This is the algorithm that writes to a register in the ADE7953. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\n i2cAlgorithm32_write function started "); 
  #endif 
 
  Wire.beginTransmission(ADE_Address);
  Wire.write(MSB);
  Wire.write(LSB); 
  Wire.write(onemsb);
  Wire.write(two);
  Wire.write(three);
  Wire.write(fourlsb);
  Wire.endTransmission();//release bus for other devices

  
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::i2cAlgorithm32_read function details: ");
   Serial.print("\nAddress Byte 1(MSB)[HEX]: ");  
   Serial.print(MSB, HEX);
   Serial.print("\n Address Byte 2(LSB)[HEX]: ");  
   Serial.print(LSB, HEX);
   Serial.print("\n Written bytes (1(MSB) to 4)[HEX]: "); //MSB to LSB order
   Serial.print(onemsb, HEX);
   Serial.print(" ");
   Serial.print(two, HEX);
   Serial.print(" ");
   Serial.print(three, HEX);
   Serial.print(" ");
   Serial.print(fourlsb, HEX);
   Serial.print("\n i2cAlgorithm32_write function completed "); 
  #endif
}
  
void ADE7953::i2cAlgorithm24_write(byte MSB, byte LSB, byte onemsb, byte two, byte threelsb) { //This is the algorithm that writes to a register in the ADE7953. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\n i2cAlgorithm24_write function started "); 
  #endif

  Wire.beginTransmission(ADE_Address);
  Wire.write(MSB);
  Wire.write(LSB); 
  Wire.write(onemsb);
  Wire.write(two);
  Wire.write(threelsb);
  Wire.endTransmission();//release bus for other devices
  
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::i2cAlgorithm24_read function details: ");
   Serial.print("\nAddress Byte 1(MSB)[HEX]: ");  
   Serial.print(MSB, HEX);
   Serial.print("\n Address Byte 2(LSB)[HEX]: ");  
   Serial.print(LSB, HEX);
   Serial.print("\n Written bytes (1(MSB) to 3)[HEX]: ");  //MSB to LSB order
   Serial.print(onemsb, HEX);
   Serial.print(" ");
   Serial.print(two, HEX);
   Serial.print(" ");
   Serial.print(threelsb, HEX);
   Serial.print("\n i2cAlgorithm24_write function completed "); 
  #endif
  }
  
void ADE7953::i2cAlgorithm16_write(byte MSB, byte LSB, byte onemsb, byte twolsb) { //This is the algorithm that writes to a register in the ADE7953. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\n i2cAlgorithm16_write function started "); 
  #endif

  Wire.beginTransmission(ADE_Address);
  Wire.write(MSB);
  Wire.write(LSB); 
  Wire.write(onemsb);
  Wire.write(twolsb);
  Wire.endTransmission();//release bus for other devices
  
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::i2cAlgorithm16_read function details: ");
   Serial.print("\nAddress Byte 1(MSB)[HEX]: ");  
   Serial.print(MSB, HEX);
   Serial.print("\n Address Byte 2(LSB)[HEX]: ");  
   Serial.print(LSB, HEX);
   Serial.print("\n Written bytes (1(MSB) to 2)[HEX]: ");  //MSB to LSB order
   Serial.print(onemsb, HEX);
   Serial.print(" ");
   Serial.print(twolsb, HEX);
   Serial.print("\n i2cAlgorithm16_write function completed "); 
  #endif
  }
  
void ADE7953::i2cAlgorithm8_write(byte MSB, byte LSB, byte onemsb) { //This is the algorithm that writes to a register in the ADE7953. The arguments are the MSB and LSB of the address of the register respectively. The values of the arguments are obtained from the list of functions above.
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\n i2cAlgorithm8_write function started "); 
  #endif

  Wire.beginTransmission(ADE_Address);
  Wire.write(MSB);
  Wire.write(LSB); 
  Wire.write(onemsb);
  Wire.endTransmission();//release bus for other devices
  
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::i2cAlgorithm8_read function details: ");
   Serial.print("\nAddress Byte 1(MSB)[HEX]: ");  
   Serial.print(MSB, HEX);
   Serial.print("\n Address Byte 2(LSB)[HEX]: ");   
   Serial.print(LSB, HEX);
   Serial.print("\n Written bytes (1 of 1)[HEX]: ");  //MSB to LSB order
   Serial.print(onemsb, HEX);
   Serial.print("\n i2cAlgorithm16_write function completed "); 
  #endif
  }
  
float ADE7953::decimalize(long input, float factor, float offset) //This function adds a decimal point to the input value and returns it as a float
{
  #ifdef ADE7953_VERBOSE_DEBUG
   Serial.print("\nADE7953::calibration function executed ");
  #endif
return ((float)input/factor)+offset;
}
