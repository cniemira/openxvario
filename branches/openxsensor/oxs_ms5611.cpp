#include "Arduino.h"
#include "OXS_MS5611.h"
#include "HardwareSerial.h"

OXS_MS5611::OXS_MS5611(uint8_t addr, HardwareSerial &print, uint16_t kalman_r) {
  // constructor
  _addr=addr;
  varioData.paramKalman_r=kalman_r;  // sensor noise
  varioData.paramKalman_q=0.5; // 0.05process noise
  printer = &print; //operate on the address of print
  printer->begin(115200);
  printer->print("Vario Sensor:MS5611 I2C Addr=");
  printer->println(addr,HEX);
}
float metersPerMbar=0;
// **************** Setup the MS5611 sensor *********************
void OXS_MS5611::setup()
{
  varioData.available=false;
  // printer->println("Setup Sensor Start.");
  Wire.begin();
  SendCommand(0x1e);
  delay(100);
  for (byte i = 1; i <=6; i++)
  {
    unsigned int low, high;
    SendCommand(0xa0 + i * 2);
    Wire.beginTransmission(_addr);  
    Wire.requestFrom((uint8_t)_addr, (uint8_t)2);
    if(Wire.available()!=2) {
#ifdef DEBUG
      printer->println("Error: calibration data not available");
#endif
    }
    high = Wire.read();
    low = Wire.read();
    _calibrationData[i] = high<<8 | low;
#ifdef DEBUG

    printer->print("calibration data #");
    printer->print(i);
    printer->print(" = ");
    printer->println( _calibrationData[i] ); 
#endif
  }
  // Prefill buffers.
  printer->print("Prefilling vario buffers...");
  unsigned long prefillMillis=millis();
#ifndef DEBUG  
  while(millis()<prefillMillis+10000)readSensor();
  printer->println("done.");
#endif

// Test Code
  metersPerMbar=calcAltitude(rawPressure-100)-calcAltitude(rawPressure);
  printer->print("metersPerMbar=");
  printer->println(metersPerMbar,DEC);

  
// End of test Code


  resetValues();
}
/****************************************************************/
/* getData - Read data from I2C bus                             */
/****************************************************************/
/*long OXS_MS5611::getData(byte command, byte del)
 {
 long result = 0;
 SendCommand(command);
 delay(del);
 SendCommand(0x00);
 Wire.beginTransmission(_addr);  
 Wire.requestFrom(_addr,(uint8_t) 3);
 
 #ifdef DEBUG
 if(Wire.available()!=3)printer->println("Error: raw data not available");
 #endif
 
 for (int i = 0; i <= 2; i++)    result = (result<<8) | Wire.read(); 
 return result;
 }
 */


/****************************************************************/
/* SendCommand - Send a command to the I2C Bus               */
/****************************************************************/
/* Send a command to the MS5611 */
void OXS_MS5611::SendCommand(byte command)
{
  Wire.beginTransmission(_addr);
  if (!Wire.write(command)){
#ifdef DEBUG

    printer->println("Error: write()");
#endif 
  }
  if (Wire.endTransmission()) 
  {
#ifdef DEBUG
    printer->print("Error when sending command: ");
    printer->println(command, HEX);
#endif
  }
}

/****************************************************************/
/* readSensor - Read pressure + temperature from the MS5611    */
/****************************************************************/
int32_t D1;
int64_t D2, dT, P;
float TEMP;
int64_t OFF, SENS;

byte SensorState=0;
unsigned long lastmicros;
void OXS_MS5611::readSensor()
{
  if (SensorState==0) // ========================== Request Pressure
  {
    SendCommand(0x48);
    lastmicros=micros();    
    SensorState+=1;
  }
  else if (SensorState==1) // ========================== get Pressure value
  {
    if (micros()-lastmicros>=9000){
      long result = 0;
      SendCommand(0x00);
      Wire.beginTransmission(_addr);  
      Wire.requestFrom(_addr,(uint8_t) 3);
#ifdef DEBUG
      if(Wire.available()!=3)printer->println("Error: raw data not available");
#endif
      for (int i = 0; i <= 2; i++)    result = (result<<8) | Wire.read(); 
      D1=result;
      SendCommand(0x50); // request Temperature
      lastmicros=micros(); 
      SensorState+=1;
    }   
  }
  else if (SensorState==2) // =========================  Get Temperature Value
  {
    if (micros()-lastmicros>=1000){
      long result = 0;
      SendCommand(0x00);
      Wire.beginTransmission(_addr);  
      Wire.requestFrom(_addr,(uint8_t) 3);
#ifdef DEBUG
      if(Wire.available()!=3)printer->println("Error: raw data not available");
#endif
      for (int i = 0; i <= 2; i++)    result = (result<<8) | Wire.read(); 
      D2=result;
      lastmicros=micros(); 
      // Do Conversion
      dT = D2 - ((long)_calibrationData[5] << 8);
      TEMP = (2000 + (((int64_t)dT * (int64_t)_calibrationData[6]) >> 23)) / (float)100;
      varioData.temperature=(int)(TEMP*10);
      if (_calcTemperature==0)_calcTemperature=varioData.temperature;

      OFF  = (((int64_t)_calibrationData[2]) << 16) + ((_calibrationData[4] * dT) >> 7);
      SENS = (((int64_t)_calibrationData[1]) << 15) + ((_calibrationData[3] * dT) >> 8);
      P= ((((D1 * SENS) >> 21) - OFF) >> (15-EXTRA_PRECISION)) / ((1<<EXTRA_PRECISION) );

      //rawPressure=P + PressureCalibrationOffset;
      rawPressure=P*100 ;
      if (P>0)varioData.available=true;
      else varioData.available=false;
      kalman_update(rawPressure);
      calcAltitude(varioData.pressure);
     
      SensorState=0;
    }   
  }
}

void OXS_MS5611::kalman_update(int64_t measurement)
{
  static float x=rawPressure; //value
  static float p=200; //estimation error covariance
  static float k=0; //kalman gain
  measurement=measurement*100;
  // update the prediction value
  p = p + varioData.paramKalman_q;

  // update based on measurement
  k = p / (p + varioData.paramKalman_r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;

#ifdef KALMANDUMP
  abweichung=measurement -x;
  // output to processing.
  printer.print(rawalt);  
  printer->print(",");
  printer->print(x);  
  printer->print(",");
  printer->println(abweichung);
#endif
  varioData.pressure=x/10000;
  SaveClimbRate(x);
}
/*
void OXS_MS5611::kalman_update(float measurement)
 {
 static float x=rawPressure; //value
 static float p=200; //estimation error covariance
 static float k=0; //kalman gain
 measurement=measurement*100;
 // update the prediction value
 p = p + varioData.paramKalman_q;
 
 // update based on measurement
 k = p / (p + varioData.paramKalman_r);
 x = x + k * (measurement - x);
 p = (1 - k) * p;
 
 #ifdef KALMANDUMP
 abweichung=measurement -x;
 // output to processing.
 printer.print(rawalt);  printer->print(",");
 printer->print(x);  printer->print(",");
 printer->println(abweichung);
 #endif
 varioData.pressure=x/100;
 }
 */
/****************************************************************/
/* getAltitude - convert mbar and temperature into an altitude  */
/*               value                                          */
/****************************************************************/
float OXS_MS5611::calcAltitude(float pressure) {
  float   p; // Pressure
  //int32_t t1=varioData.temperature/10;
  int32_t t1=_calcTemperature/10; // only use the first measured temperature for calculation
//  p=(float)varioData.pressure/100;
  p=(float) pressure/100;
  //printer->print(" result=");     
  //printer->print(p,DEC);

  const float sea_press = 1013.25;
  p=(((pow((sea_press / p), 1/5.257) - 1.0) * ( (t1/100) + 273.15)) / 0.0065 *100);

  //SaveClimbRate(p);

  varioData.absoluteAlt=(int32_t)p;
  varioData.relativeAlt=varioData.absoluteAlt- varioData.altOffset;
  if(varioData.maxAbsAlt<varioData.absoluteAlt)varioData.maxAbsAlt=varioData.absoluteAlt;
  if(varioData.minAbsAlt>varioData.absoluteAlt)varioData.minAbsAlt=varioData.absoluteAlt;
  if(varioData.maxRelAlt<varioData.relativeAlt)varioData.maxRelAlt=varioData.relativeAlt;
  if(varioData.minRelAlt>varioData.relativeAlt)varioData.minRelAlt=varioData.relativeAlt;
  return p;
}
/*void OXS_MS5611::calcAltitude() {
 float   result;
 //int32_t t1=varioData.temperature/10;
 int32_t t1=_calcTemperature/10; // only use the first measured temperature for calculation
 result=(float)varioData.pressure/100;
 const float sea_press = 1013.25;
 result=(((pow((sea_press / result), 1/5.257) - 1.0) * ( (t1/100) + 273.15)) / 0.0065 *100);
 SaveClimbRate(result);
 
 //_absoluteAlt=result;
 //_relativeAlt=_absoluteAlt- varioData.altOffset;
 
 varioData.absoluteAlt=(int32_t)result;
 varioData.relativeAlt=varioData.absoluteAlt- varioData.altOffset;
 if(varioData.maxAbsAlt<varioData.absoluteAlt)varioData.maxAbsAlt=varioData.absoluteAlt;
 if(varioData.minAbsAlt>varioData.absoluteAlt)varioData.minAbsAlt=varioData.absoluteAlt;
 if(varioData.maxRelAlt<varioData.relativeAlt)varioData.maxRelAlt=varioData.relativeAlt;
 if(varioData.minRelAlt>varioData.relativeAlt)varioData.minRelAlt=varioData.relativeAlt;
 }*/
/*******************************************************************************/
/* SaveClimbRate => calculate the current climbRate                            */
/* GetClimbRate = > Retrieve the average climbRate from the buffer             */
/*******************************************************************************/
#define CR_BUFFER_SIZE 20

//float climbRateBuffer[CR_BUFFER_SIZE];
float crsum=0;
int crNumValues=0;
void OXS_MS5611::SaveClimbRate(float pressure)
{
  static float lastPressure=pressure;
  static unsigned long Start=micros();
  static byte cnt =0;
  static float relheight=0;
  
  crsum+=(lastPressure-pressure); // sum up the pressure dufferences
  crNumValues+=1;
   
 
  if(crNumValues==CR_BUFFER_SIZE){
    float dbgtmp1=crsum;
     crsum=crsum *(metersPerMbar/1000000); // convert to altitude change in ??
     float CurrentClimbRate=(float) crsum *   (   (float)1000000     /  ( (float)micros() - Start ) ); // convert to m/s

    varioData.climbRate=CurrentClimbRate*100; // export value as m/s
    relheight+=crsum;
        
    //printer->println("=============== CurrentClimbRate !!!");
    printer->print(" (relheight)=");     
    printer->print(relheight*100,DEC); 
    printer->print("cm,(");    
    printer->print(crsum); 
    printer->print("cm)");     
    printer->print(" time=");     
    printer->print(micros()-Start);
    printer->print(" CurrentClimbRate=");  
    printer->println(CurrentClimbRate);
    crNumValues=0;
    Start=micros();
    crsum=0;
    if (varioData.maxClimbRate<varioData.climbRate)varioData.maxClimbRate=varioData.climbRate;
    if (varioData.minClimbRate>varioData.climbRate)varioData.minClimbRate=varioData.climbRate;
 
  }
  lastPressure=pressure;
}
/*
void OXS_MS5611::SaveClimbRate(float pressure)
{
  static float lastPressure=pressure;
  static unsigned long Start=micros();
  static byte cnt =0;
  float CurrentClimbRate=(float) ((lastPressure-pressure)*(metersPerMbar/1000000))*((float)1000000/((float)micros()-Start));
  Start=micros();    
  crsum+=CurrentClimbRate*100;// convert to cm

  crNumValues+=1;
   
  printer->print(" (lastPressure-pressure)=");     
  printer->print((lastPressure-pressure),DEC); 
  printer->print(" diff=");     
  printer->print((lastPressure-pressure)*(metersPerMbar/1000000),DEC);
  printer->print(" CurrentClimbRate=");  
  printer->println(CurrentClimbRate);
 
  if(crNumValues==CR_BUFFER_SIZE){
    varioData.climbRate=crsum/CR_BUFFER_SIZE;
    crsum=varioData.climbRate;
    crNumValues=0;
     printer->println("=============== CurrentClimbRate !!!");
    if (varioData.maxClimbRate<varioData.climbRate)varioData.maxClimbRate=varioData.climbRate;
    if (varioData.minClimbRate>varioData.climbRate)varioData.minClimbRate=varioData.climbRate;
  }
  lastPressure=pressure;
}*/



/*
void OXS_MS5611::SaveClimbRateOld(float alti){
 static unsigned long Start=micros();
 static float lastAlti=alti;
 static float MaxDiff=-1000;
 static float MinDiff=1000;
 static float sumAlti=0;
 static byte cnt=0;
 if(MaxDiff < (alti-lastAlti))MaxDiff=alti-lastAlti;
 if(MinDiff > (alti-lastAlti))MinDiff=alti-lastAlti;
 sumAlti+=(alti-lastAlti);
 //  printer->print("timecalc=");    printer->print(timecalc);
 // printer->print(" alti=");    printer->print(alti);
 // printer->print(" lastAlti=");    printer->print(lastAlti);
 // printer->print(" alti-lastAlti=");    printer->print(alti-lastAlti);
 // float CurrentClimbRate=(float)(alti-lastAlti)*((float)1000000/(float)timecalc);
 // printer->print("CurrentClimbRate=");    printer->println(CurrentClimbRate);
 
 cnt+=1;
 if (cnt==ClimbRateQueueLength){
 cnt=0;
 
 // Do not use the t2o top values for the calculation
 //sumAlti-=MaxDiff;
 //sumAlti-=MinDiff;
 MaxDiff=0;
 MinDiff=0;
 //printer->print("RESULT==");      //printer->println(varioData.climbRate);
 //printer->print("sumAlti==");     //printer->println(sumAlti);
 //printer->print("Micros==");      //printer->println(  (float)micros()-Start);
 float CurrentClimbRate=(float)(sumAlti)*((float)1000000/((float)micros()-Start));
 //printer->print("CurrentClimbRate==");  /printer->println(CurrentClimbRate);
 varioData.climbRate=CurrentClimbRate;
 if (varioData.maxClimbRate<varioData.climbRate)varioData.maxClimbRate=varioData.climbRate;
 if (varioData.minClimbRate>varioData.climbRate)varioData.minClimbRate=varioData.climbRate;
 #ifdef DEBUG
 printer->print("RESULT==");    
 printer->println(varioData.climbRate);
 #endif
 Start=micros();
 sumAlti=0;
 }
 lastAlti=alti;
 
 }*/
void OXS_MS5611::resetValues(){
  varioData.altOffset=varioData.absoluteAlt;
  varioData.maxRelAlt=0;
  varioData.minRelAlt=0;
  varioData.maxAbsAlt=varioData.absoluteAlt;
  varioData.minAbsAlt=varioData.absoluteAlt;
  varioData.minClimbRate=varioData.climbRate;
  varioData.maxClimbRate=varioData.climbRate;
}


