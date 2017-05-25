/*
 ADE7953.cpp - Simple library for operating the ADE7953 Single-Phase AC Line measurement IC over SPI for Arduino Uno 
  Created by Umar Kazmi, Crystal Lai, and Michael Klopfer, Ph.D.
  January 23, 2017 - v6.2 (pre-release)
  University of California, Irvine - California Plug Load Research Center (CalPlug)
  Released into the public domain.
*/

#ifndef ADE7953_h
#define ADE7953_h

#include "Arduino.h" //this includes the arduino library header. It makes all the Arduino functions available in this tab.
#include <SPI.h>

const unsigned int READ = 0b10000000;  //This value tells the ADE7953 that data is to be read from the requested register.
const unsigned int WRITE = 0b00000000; //This value tells the ADE7953 that data is to be written to the requested register.
const int SPI_freq = 1000000;//Communicate with the ADE7953 at 1 MHz frequency.


class ADE7953 {
  public:
    ADE7953(int SS, int SPI_freq);
    void initialize();
    uint8_t spiAlgorithm8_read(byte MSB, byte LSB);
	uint16_t spiAlgorithm16_read(byte MSB, byte LSB);
    uint32_t spiAlgorithm24_read(byte MSB, byte LSB);
    uint32_t spiAlgorithm32_read(byte MSB, byte LSB);
    
	
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
	void spiAlgorithm32_write(byte MSB, byte LSB, byte onemsb, byte two, byte three, byte fourlsb);
	void spiAlgorithm24_write(byte MSB, byte LSB, byte onemsb, byte two, byte threelsb);
	void spiAlgorithm16_write(byte MSB, byte LSB, byte onemsb, byte twolsb);
	void spiAlgorithm8_write(byte MSB, byte LSB, byte onemsb);
	
	
	float decimalize(long input, float factor, float offset);
  
  private:
  	int _SS;
    int _SPI_freq;
};

#endif
