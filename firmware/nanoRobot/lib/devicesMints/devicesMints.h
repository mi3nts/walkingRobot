#ifndef DEVICES_MINTS_H
#define DEVICES_MINTS_H
//
#include <Arduino.h>
#include "OPCN3NanoMints.h"
#include "MutichannelGasSensor.h"

#include "Adafruit_Sensor.h"
#include "Seeed_BME280.h"
#include "SparkFun_SCD30_Arduino_Library.h"

#include "Adafruit_AS726x.h"
#include "Adafruit_TSL2591.h"
#include <SparkFun_VEML6075_Arduino_Library.h>
#include "TMG39931Mints.h"
#include "SI114X.h"
#include "jobsMints.h"


// // For GUV001
void readGUV001Mints(uint8_t pinIn);


// // For GL001
void readGL001Mints(uint8_t pinIn);

// // For SI114X
extern SI114X SI;
bool initializeSI114XMints();
void readSI114XMints();

// // For TMG39931
extern TMG39931Mints TMG;
bool initializeTMG39931Mints();
void readTMG39931Mints();

// // For AS726x
extern Adafruit_AS726x ams;
bool initializeAS7262Mints();
void readAS7262Mints();

// // For VEML6075
extern VEML6075 veml;
bool initializeVEML6075Mints();
void readVEML6075Mints();
//
//
extern Adafruit_TSL2591 tsl;
bool initializeTSL2591Mints();
void readTSL2591Mints();
// Taken from:https://github.com/adafruit/Adafruit_TSL2591_Library/blob/master/examples/tsl2591/tsl2591.ino
void displayTSL2591DetailsMints(void);
void configureTSL2591Mints(void);
void advancedReadTSL2591Mnits(void);
// Ends functions taken from:https://github.com/adafruit/Adafruit_TSL2591_Library/blob/master/examples/tsl2591/tsl2591.ino


float readAPDS9002Mints(uint8_t analogpin);
float FmultiMapAPDS9002(float val, float * _in, float * _out, uint8_t size);
    // Taken from: https://github.com/SeeedDocument/Seeed-WiKi/blob/master/docs/Grove-Luminance_Sensor.md


extern BME280 bme280;
bool initializeBME280Mints();
void readBME280Mints();
//
bool initializeMGS001Mints();
void readMGS001Mints();
//

extern SCD30 scd;
bool initializeSCD30Mints();
void readSCD30Mints();

// //
extern OPCN3NanoMints opc;
bool initializeOPCN3Mints();
void readOPCN3Mints();

















#endif
