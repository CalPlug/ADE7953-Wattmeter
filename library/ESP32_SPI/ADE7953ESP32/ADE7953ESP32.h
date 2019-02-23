/*
 ADE7953ESP32.h - Simple library for operating the ADE7953 Single-Phase AC Line measurement IC over SPI for ESP32 Arduino Core
  Created by Umar Kazmi, Crystal Lai, and Michael Klopfer, Ph.D.
  Modified for ESP32 operation by Luis Contreras, 2018 with polish updates by M.J. Klopfer
  University of California, Irvine - California Plug Load Research Center (CalPlug)
  Released into the public domain.    Feb 22, 2019 - v1.0 (first final release)
*/

#ifndef ADE7953ESP32_h
#define ADE7953ESP32_h

#include "Arduino.h" //this includes the arduino library header. It makes all the Arduino functions available in this tab.
#include "esp32-hal-spi.h"

const unsigned int READ = 0b10000000;  //This value tells the ADE7953 that data is to be read from the requested register.
const unsigned int WRITE = 0b00000000; //This value tells the ADE7953 that data is to be written to the requested register.
const int SPI_freq = 1000000;//Communicate with the ADE7953 at 1 MHz frequency, this is the default value


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
	float getPowerFactorB();
	float getPeriod();	
	int16_t getPhaseCalibA();
	int16_t getPhaseCalibB();
	unsigned long getAPNOLOAD();
    long getInstVoltage();
	float getVrms();
	long getInstCurrentA();
	long getInstCurrentB();
	float getIrmsA();
	float getIrmsB();
	unsigned long getVpeak();
	unsigned long getIpeakA();
	unsigned long getIpeakB();
	long getActiveEnergyA();
	long getActiveEnergyB();
	long getReactiveEnergyA();
	long getReactiveEnergyB();
	long getApparentEnergyA();
	long getApparentEnergyB();
	float getInstApparentPowerA();
	float getInstApparentPowerB();
	float getInstActivePowerA();
	float getInstActivePowerB();
	float getInstReactivePowerA();
	float getInstReactivePowerB();
	

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
