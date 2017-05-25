// Basic Test Demonstration for ADE7953 to read and report values (ADE7953_TEST)
//California Plug Load Research Center - 2017


#include <ADE7953.h>
#include <SPI.h>

//Define ADE7953 object with hardware parameters specified
#define local_SPI_freq 1000000  //Set SPI_Freq at 1MHz (#define, (no = or ;) helps to save memory)
#define local_SS 10  //Set the SS pin for SPI communication as pin 10  (#define, (no = or ;) helps to save memory)
ADE7953 myADE7953(local_SS, local_SPI_freq); // Call the ADE7953 Object with hardware parameters specified, the "local" lets us use the same parameters for examples in this program as what is assigned to the ADE7953 object

void setup() {
  Serial.begin(115200);
  delay(200);
  SPI.begin();
  delay(200);
  myADE7953.initialize();   //The ADE7953 must be initialized once in setup.
}

//int count;

void loop() {
  
  long apnoload, activeEnergyA;
  float vRMS, iRMSA, powerFactorA, apparentPowerA, reactivePowerA, activePowerA;

  apnoload = myADE7953.getAPNOLOAD();
  Serial.print("APNOLOAD (hex): ");
  Serial.println(apnoload, HEX);
  delay(200); 

  vRMS = myADE7953.getVrms();  
  Serial.print("Vrms (V): ");
  Serial.println(vRMS);
  delay(200);

  iRMSA = myADE7953.getIrmsA();  
  Serial.print("IrmsA (mA): ");
  Serial.println(iRMSA);
  delay(200);

  apparentPowerA = myADE7953.getInstApparentPowerA();  
  Serial.print("Apparent Power A (mW): ");
  Serial.println(apparentPowerA);
  delay(200);

  activePowerA = myADE7953.getInstActivePowerA();  
  Serial.print("Active Power A (mW): ");
  Serial.println(activePowerA);
  delay(200);

  reactivePowerA = myADE7953.getInstReactivePowerA();  
  Serial.print("Rective Power A (mW): ");
  Serial.println(reactivePowerA);
  delay(200);

  powerFactorA = myADE7953.getPowerFactorA();  
  Serial.print("Power Factor A (x100): ");
  Serial.println(powerFactorA);
  delay(200);

  activeEnergyA = myADE7953.getActiveEnergyA();  
  Serial.print("Active Energy A (hex): ");
  Serial.println(activeEnergyA);
  delay(200);

  Serial.println();


//  if(count>5){
//    while(1);
//  }
//  count++;
}
