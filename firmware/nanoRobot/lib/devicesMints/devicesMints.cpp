
#include "devicesMints.h"

// BME280  ---------------------------------------
//
bool initializeBME280Mints(){
      if (bme280.init()) {
        Serial.println("BME280 Initiated");
        delay(1);
        return true;
      }
      else
      {
      Serial.println("BME280 not found");
      delay(1);

      return false;
      }

}

void readBME280Mints(){

  float temperature    = bme280.getTemperature();
  float pressure       = bme280.getPressure();
  float humidity       = bme280.getHumidity();
  float altitude       = bme280.calcAltitude(pressure);

  String readings[4] = { String(temperature,2) , String(pressure,2), String(humidity,2) , String(altitude,2) };
  sensorPrintMints("BME280",readings,4);
}

//
//
// // // MGS001  ---------------------------------------

bool initializeMGS001Mints(){

  gas.begin(0x04);//the default I2C address of the slave is 0x04
  gas.powerOn();
  Serial.println("MGS001 Initiated");
  Serial.print("MGS001 Firmware Version = ");
  Serial.println(gas.getVersion());
  delay(1);

return true;
}

void readMGS001Mints(){

  float nh3     = gas.measure_NH3();
  float co      = gas.measure_CO();
  float no2     = gas.measure_NO2();
  float c3h8    = gas.measure_C3H8();
  float c4h10   = gas.measure_C4H10();
  float ch4     = gas.measure_CH4();
  float h2      = gas.measure_H2();
  float c2h5oh  = gas.measure_C2H5OH();


  String readings[8] = { String(nh3,2) , String(co,2), String(no2,2) , String(c3h8,2), String(c4h10,2) , String(ch4,2), String(h2,2) , String(c2h5oh,2) };
  sensorPrintMints("MGS001",readings,8);
}




// SCD30 ---------------------------------------
bool initializeSCD30Mints(){
  if (scd.begin()) {
    Serial.println("SCD30 Initiated");
    delay(1);
    return true;
  }else{
    Serial.println("SCD30 not found");
    delay(1);
    return false;
  }

  delay(2000);
}

void readSCD30Mints(){

  uint16_t co2         = scd.getCO2();
  uint16_t temperature = scd.getTemperature();
  uint16_t humidity    = scd.getHumidity();

  String readings[3] = { String(co2), String(temperature) , String(humidity) };
  sensorPrintMints("SCD30",readings,3);

}






bool initializeOPCN3Mints(){

  return opc.initialize();

}

void readOPCN3Mints(){

  struct histogramData  allData= opc.readHistogramData();

  String readings[43] =  { \
                          String(allData.valid), \
                          String(allData.binCount0), \
                          String(allData.binCount1), \
                          String(allData.binCount2), \
                          String(allData.binCount3), \
                          String(allData.binCount4), \
                          String(allData.binCount5), \
                          String(allData.binCount6), \
                          String(allData.binCount7), \
                          String(allData.binCount8), \
                          String(allData.binCount9), \
                          String(allData.binCount10), \
                          String(allData.binCount11), \
                          String(allData.binCount12), \
                          String(allData.binCount13), \
                          String(allData.binCount14), \
                          String(allData.binCount15), \
                          String(allData.binCount16), \
                          String(allData.binCount17), \
                          String(allData.binCount18), \
                          String(allData.binCount19), \
                          String(allData.binCount20), \
                          String(allData.binCount21), \
                          String(allData.binCount22), \
                          String(allData.binCount23), \
                          String(allData.bin1TimeToCross), \
                          String(allData.bin3TimeToCross), \
                          String(allData.bin5TimeToCross), \
                          String(allData.bin7TimeToCross), \
                          String(allData.samplingPeriod), \
                          String(allData.sampleFlowRate), \
                          String(allData.temperature), \
                          String(allData.humidity), \
                          String(allData.pm1,2), \
                          String(allData.pm2_5,2), \
                          String(allData.pm10,2),\
                          String(allData.rejectCountGlitch), \
                          String(allData.rejectCountLongTOF), \
                          String(allData.rejectCountRatio), \
                          String(allData.rejectCountOutOfRange), \
                          String(allData.fanRevCount), \
                          String(allData.laserStatus), \
                          String(allData.checkSum) \
                        };
  sensorPrintMints("OPCN3",readings,43);
}



void readGL001Mints(uint8_t pinIn){
  
  String readings[1] = { \
        String(analogRead(pinIn)),\
     };

  delay(2);
  sensorPrintMints("GL001",readings,1);

}


void readGUV001Mints(uint8_t pinIn){

    int sensorValue;
    long  sum=0;
    for(int i=0;i<1024;i++)// accumulate readings for 1024 times
    {
        sensorValue=analogRead(pinIn);
        sum=sensorValue+sum;
        delay(2);
    }

    String readings[1] = { \
        String(sensorValue),\
     };
  sensorPrintMints("GUV001",readings,1);

}


//
//  SI114X ---------------------------------------
bool initializeSI114XMints()
{
if (SI.Begin())
    {
      Serial.println("SI114X Initiated");
      delay(1);
      return true;
    }else
    {
      Serial.println("SI114X Not found");
      delay(1);
      return false;
    }
}


void readSI114XMints(){

   String readings[6] = { \
        String(SI.ReadVisible()),\
        String(SI.ReadIR()),\
        String((float)SI.ReadUV()/100),\
        String(SI.ReadProximity(0X26)),\
        String(SI.ReadProximity(0X28)),\
        String(SI.ReadProximity(0X2A))\
     };
  sensorPrintMints("SI114X",readings,6);

}


//
// TMG39931 ---------------------------------------
bool initializeTMG39931Mints()
{
  TMG.begin();
  return true;
}


void readTMG39931Mints(){

     struct sensorData  tmgData= TMG.readData();
     
    String readings[5] = { \
        String(tmgData.infraRed),\
        String(tmgData.red),\
        String(tmgData.green),\
        String(tmgData.blue),\
        String(tmgData.proximity),\
     };
  sensorPrintMints("TMG3993",readings,5);

}

//
//
// AS7262 ---------------------------------------
bool initializeAS7262Mints()
{
    if (ams.begin())
    {
      Serial.println("AS7262 Initiated");
      delay(1);
      return true;
    }else
    {
      Serial.println("AS7262 Not found");
      delay(1);
      return false;
    }
}


void readAS7262Mints(){

    uint8_t temperature = ams.readTemperature();

    //ams.drvOn(); //uncomment this if you want to use the driver LED for readings
    ams.startMeasurement(); //begin a measurement

    //wait till data is available
    bool rdy = false;
    while(!rdy){
      delay(5);
      rdy = ams.dataReady();
    }

    //read the values!
    uint16_t AS7262Values[AS726x_NUM_CHANNELS];
    float    AS7262Calibrated[AS726x_NUM_CHANNELS];

    ams.readRawValues(AS7262Values);
    ams.readCalibratedValues(AS7262Calibrated);

  String readings[13] = { String(temperature),\
       String(AS7262Values[AS726x_VIOLET]),\
       String(AS7262Values[AS726x_BLUE]),\
       String(AS7262Values[AS726x_GREEN]),\
       String(AS7262Values[AS726x_YELLOW]),\
       String(AS7262Values[AS726x_ORANGE]),\
       String(AS7262Values[AS726x_RED]),\
       String(AS7262Calibrated[AS726x_VIOLET]),\
       String(AS7262Calibrated[AS726x_BLUE]),\
       String(AS7262Calibrated[AS726x_GREEN]),\
       String(AS7262Calibrated[AS726x_YELLOW]),\
       String(AS7262Calibrated[AS726x_ORANGE]),\
       String(AS7262Calibrated[AS726x_RED])\
     };
  sensorPrintMints("AS7262",readings,13);

}

//     veml.begin(VEML6070_4_T);
//     Serial.println("VEML6070 Initiated");
//     delay(1);
//     return true;
//
// }
//
// void readVEML6070Mints(){
//
//   float UVLightLevel = veml.readUV();
//
//   String readings[1] = { String(UVLightLevel,2) };
//   sensorPrintMints("VEML6070",readings,1);
//
// }


// VEML6075 ---------------------------------------
bool initializeVEML6075Mints()
{
    if (veml.begin())
    {
      // cannot set to 800
        veml.setIntegrationTime(VEML6075::IT_800MS);

        // Set the high dynamic mode
        veml.setHighDynamic(VEML6075::DYNAMIC_HIGH);
          // Get the mode


      Serial.println("VEML6075 Initiated");

      delay(1);
      return true;
    }else
    {
      Serial.println("VEML6075 Not found");
      delay(1);
      return false;
    }
}



void readVEML6075Mints(){

  String readings[7] = {   String(veml.rawUva()),
                           String(veml.rawUvb()),
                           String(veml.visibleCompensation()),
                           String(veml.irCompensation()),
                           String(veml.uva()),
                           String(veml.uvb()),
                           String(veml.index())
                         };
  sensorPrintMints("VEML6075",readings,7);

}

// TSL2591 ---------------------------------------
bool initializeTSL2591Mints()
{
  if (tsl.begin())
  {
    delay(10);
    displayTSL2591DetailsMints();
    delay(10);
    configureTSL2591Mints();
    Serial.println("TSL2591 Initiated");
    delay(1);
    return true;
  }else
  {
    Serial.println("TSL2591 not found");
    delay(1);
    return false;
  }
}

void readTSL2591Mints(){

  uint32_t luminosity      = tsl.getFullLuminosity();
  uint16_t ir              = luminosity >> 16;
  uint16_t full            = luminosity & 0xFFFF;
  uint16_t visible         = full - ir;
  float    lux             = tsl.calculateLux(full, ir);

  String readings[5] = { String(luminosity) ,  String(ir) , String(full) , String(visible) , String(lux,6) };
  sensorPrintMints("TSL2591",readings,5);

}


/**************************************************************************/
/*
    Taken from:
    https://github.com/adafruit/Adafruit_TSL2591_Library/blob/master/examples/tsl2591/tsl2591.ino
*/
/**************************************************************************/

void displayTSL2591DetailsMints(void)
{
    sensor_t sensor;
    tsl.getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
    Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
    Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));
    Serial.println(F("------------------------------------"));
    Serial.println(F(""));
    delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void configureTSL2591Mints(void)
{


    // You can change the gain on the fly, to adapt to brighter/dimmer light situations
    //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
    tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
    //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

    // Changing the integration time gives you a longer time over which to sense light
    // longer timelines are slower, but are good in very low light situtations!
    //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
    // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

    /* Display the gain and integration time for reference sake */
    Serial.println(F("------------------------------------"));
    Serial.print  (F("Gain:         "));
    tsl2591Gain_t gain = tsl.getGain();
    switch(gain)
    {
      case TSL2591_GAIN_LOW:
        Serial.println(F("1x (Low)"));
        break;
      case TSL2591_GAIN_MED:
        Serial.println(F("25x (Medium)"));
        break;
      case TSL2591_GAIN_HIGH:
        Serial.println(F("428x (High)"));
        break;
      case TSL2591_GAIN_MAX:
        Serial.println(F("9876x (Max)"));
        break;
    }
    Serial.print  (F("Timing:       "));
    Serial.print((tsl.getTiming() + 1) * 100, DEC);
    Serial.println(F(" ms"));
    Serial.println(F("------------------------------------"));
    Serial.println(F(""));
}



void advancedReadTSL2591Mnits(void)
{
    // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
    // That way you can do whatever math and comparisons you want!
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir, full;
    ir = lum >> 16;
    full = lum & 0xFFFF;
    Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
    Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
    Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
    Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
    Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
}


/**************************************************************************/
/*
    Ends functions taken from:
    https://github.com/adafruit/Adafruit_TSL2591_Library/blob/master/examples/tsl2591/tsl2591.ino
*/
/**************************************************************************/




// APDS9002 ---------------------------------------

float readAPDS9002Mints(uint8_t analogpin)
{

    float VoutArray[] =  {0.00191633333,  0.00565133333,   0.01916333333, 0.06967166666, 0.25331666666, 0.88945, 2.2815,   3.178,  3.83333333333};
    float  LuxArray[] =  { 1.0108,     3.1201,  9.8051,   27.43,   69.545,   232.67,  645.11,   73.52,  1000};
    uint8_t raw = analogRead(analogpin);
    float voltage = raw  * (5.0 / 1023.0);
    float luminance  = FmultiMapAPDS9002(voltage, VoutArray, LuxArray, 9);
    String readings[3] = { String(luminance) ,String(voltage) , String(raw) };
    sensorPrintMints("APDS9002",readings,3);

}


/**************************************************************************/
/*
    Taken from:
    https://github.com/SeeedDocument/Seeed-WiKi/blob/master/docs/Grove-Luminance_Sensor.md
*/
/**************************************************************************/

float FmultiMapAPDS9002(float val, float * _in, float * _out, uint8_t size)
{
    // take care the value is within range
    // val = constrain(val, _in[0], _in[size-1]);
    if (val <= _in[0]) return _out[0];
    if (val >= _in[size-1]) return _out[size-1];

    // search right interval
    uint8_t pos = 1;  // _in[0] allready tested
    while(val > _in[pos]) pos++;

    // this will handle all exact "points" in the _in array
    if (val == _in[pos]) return _out[pos];

    // interpolate in the right segment for the rest
    return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}


