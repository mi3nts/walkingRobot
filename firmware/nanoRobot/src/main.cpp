#include "Arduino.h"
// #include "Seeed_BME280.h"
// #include "MutichannelGasSensor.h"

//  Light Sensors
#include "SI114X.h"
#include "TMG39931Mints.h"

#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>

#include "Adafruit_AS726x.h"
#include "Adafruit_TSL2591.h"
#include <SparkFun_VEML6075_Arduino_Library.h>

#include "jobsMints.h"
#include "devicesMints.h"



#define CS 10


// TMG39931 I2C address is 0x39(57)
bool TMG39931Online;
TMG39931Mints TMG(0x39);

bool SI114XOnline;
SI114X SI = SI114X();


//create the object
bool AS7262Online;
Adafruit_AS726x ams;

bool    TSL2591Online;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

bool VEML6075Online;
VEML6075 veml ;

uint8_t groveLuminancePin = A0;
uint8_t groveLightPin = A1;
uint8_t groveUVPin = A2;


OPCN3NanoMints opc = OPCN3NanoMints(CS);
bool  OPCN3Online;

bool SCD30Online;
SCD30 scd;

bool MGS001Online;

bool BME280Online;
BME280 bme280; // I2C

uint16_t sensingPeriod = 1000;
uint16_t initPeriod = 1500;

unsigned long startTime;

void setup() {

  delay(initPeriod);
  initializeSerialMints();


//  Light Sensors
  delay(initPeriod);
  AS7262Online = initializeAS7262Mints();

  delay(initPeriod);
  TSL2591Online      = initializeTSL2591Mints();

  delay(initPeriod);
  VEML6075Online      = initializeVEML6075Mints();

  delay(initPeriod);
  TMG39931Online      = initializeTMG39931Mints();

  delay(initPeriod);
  SI114XOnline      = initializeSI114XMints();


  delay(initPeriod);
  BME280Online = initializeBME280Mints();
  //
  delay(initPeriod);
  MGS001Online =  initializeMGS001Mints();

  delay(initPeriod);
  SCD30Online = initializeSCD30Mints();

  delay(initPeriod);
  OPCN3Online =  initializeOPCN3Mints();



}


// the loop routine runs over and over again forever:
void loop() {


      startTime  = millis();


  delay(sensingPeriod);
   if(AS7262Online)
   {
     readAS7262Mints();
   }
   delay(sensingPeriod);

   if(TSL2591Online)
       {
         readTSL2591Mints();
       }

   delay(sensingPeriod);

   if(VEML6075Online)
       {
         readVEML6075Mints();
       }
     delay(sensingPeriod);

   if(TMG39931Online)
       {
         readTMG39931Mints();
       }
     delay(sensingPeriod);

  delay(sensingPeriod);
     readGL001Mints(groveLightPin);


  delay(sensingPeriod);
     readGUV001Mints(groveUVPin);


  delay(sensingPeriod);
     readAPDS9002Mints(groveLuminancePin);


    delay(sensingPeriod);
    if(BME280Online)
    {
      readBME280Mints();
    }

    delay(sensingPeriod);
    if(MGS001Online)
    {
      readMGS001Mints();
    }
    // //
    delay(sensingPeriod);
    if(SCD30Online)
    {
      readSCD30Mints();
    }

    delay(sensingPeriod);
    if(OPCN3Online)
    {
      readOPCN3Mints();
    }

    delayMints(millis() - startTime,20000);

}
