/*
 ADE7953.cpp - Simple library for operating the ADE7953 Single-Phase AC Line measurement IC over SPI for Arduino Uno 
  Created by Umar Kazmi, Crystal Lai, and Michael Klopfer, Ph.D.
  January 23, 2017 - v6.2 (pre-release)
  University of California, Irvine - California Plug Load Research Center (CalPlug)
  Released into the public domain.
*/

#ifndef ADE7953_I2C_h
#define ADE7953_I2C_h

#include "Arduino.h" //this includes the arduino library header. It makes all the Arduino functions available in this tab.
#include <Wire.h>

/* const unsigned int READ = 0b10000000;  //This value tells the ADE7953 that data is to be read from the requested register.
const unsigned int WRITE = 0b00000000; //This value tells the ADE7953 that data is to be written to the requested register.
const int SPI_freq = 1000000;//Communicate with the ADE7953 at 1 MHz frequency. */


class ADE7953 {
  public:
    ADE7953(int CLK, int CS);
    void initialize();
    uint8_t i2cAlgorithm8_read(byte MSB, byte LSB);
	uint16_t i2cAlgorithm16_read(byte MSB, byte LSB);
    uint32_t i2cAlgorithm24_read(byte MSB, byte LSB);
    uint32_t i2cAlgorithm32_read(byte MSB, byte LSB);
    
	
	uint8_t getVersion();
	float getPowerFactorA();
	float getPeriod();	
	int16_t getPhaseCalibA();
	unsigned long getAPNOLOAD();
    long getInstVoltage();
	float getVrms();
	long getInstCurrentA();
	float getIrmsA();
	unsigned long getVpeak();
	unsigned long getIpeakA();
	long getActiveEnergyA();
	long getReactiveEnergyA();
	long getApparentEnergyA();
	float getInstApparentPowerA();
	float getInstActivePowerA();
	float getInstReactivePowerA();
	

	byte functionBitVal(int addr, uint8_t byteVal);
	void i2cAlgorithm32_write(byte MSB, byte LSB, byte onemsb, byte two, byte three, byte fourlsb);
	void i2cAlgorithm24_write(byte MSB, byte LSB, byte onemsb, byte two, byte threelsb);
	void i2cAlgorithm16_write(byte MSB, byte LSB, byte onemsb, byte twolsb);
	void i2cAlgorithm8_write(byte MSB, byte LSB, byte onemsb);
	
	
	float decimalize(long input, float factor, float offset);
  
  private:
  	int _CLK;
	int _CS;
};

#endif
