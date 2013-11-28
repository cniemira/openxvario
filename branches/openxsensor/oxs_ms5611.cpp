#include "Arduino.h"
#include "oxs_ms5611.h"
#include "HardwareSerial.h"

OXS_MS5611::OXS_MS5611(uint8_t addr, HardwareSerial &print, uint16_t kalman_r) {
  // constructor
  _addr=addr;
  varioData.paramKalman_r=kalman_r;  // sensor noise
  varioData.paramKalman_q=0.5; // 0.05process noise
#ifdef DEBUG  
  printer = &print; //operate on the address of print
  printer->begin(115200);
  printer->print("Vario Sensor:MS5611 I2C Addr=");
  printer->println(addr,HEX);
#endif

}
float metersPerMbar=0;
// **************** Setup the MS5611 sensor *********************
void OXS_MS5611::setup()
{
  varioData.available=false;
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
  // calculate the initial Meters per Millibar value used in the climb rate calculation
  unsigned long prefillMillis=millis();
    while(millis()<prefillMillis+500)readSensor();
  metersPerMbar=calcAltitude(rawPressure-100)-calcAltitude(rawPressure);
#ifdef DEBUG  
  printer->print("metersPerMbar=");
  printer->println(metersPerMbar,DEC);
#endif
#ifdef DEBUG  
  printer->print("Prefilling vario buffers...");
#endif

  

#ifdef DEBUG  
  printer->println("done.");
#endif
  
  resetValues();
}

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
byte recalcCnt=0;
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
      varioData.absoluteAlt=(int32_t)calcAltitude(varioData.pressure);
      varioData.relativeAlt=varioData.absoluteAlt- varioData.altOffset;
      if(varioData.maxAbsAlt<varioData.absoluteAlt)varioData.maxAbsAlt=varioData.absoluteAlt;
      if(varioData.minAbsAlt>varioData.absoluteAlt)varioData.minAbsAlt=varioData.absoluteAlt;
      if(varioData.maxRelAlt<varioData.relativeAlt)varioData.maxRelAlt=varioData.relativeAlt;
      if(varioData.minRelAlt>varioData.relativeAlt)varioData.minRelAlt=varioData.relativeAlt;
      recalcCnt+=1;
      // recalculate the  Meters per Millibar value every 100 values (used in the climb rate calculation)
      if (recalcCnt==100){
        metersPerMbar=calcAltitude(rawPressure-100)-calcAltitude(rawPressure);
        recalcCnt=0;
      }
      SensorState=0;
    }   
  }
}

void OXS_MS5611::kalman_update(int64_t measurement)
{
  static float x=rawPressure*100; //value
  static float p=19.56; //estimation error covariance
  static float k=0; //kalman gain
  measurement=measurement*100;
  // update the prediction value
  p = p + varioData.paramKalman_q;

  // update based on measurement
  k = p / (p + varioData.paramKalman_r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;
  
  varioData.pressure=x/10000;
  varioData.paramKalman_k=p;
  SaveClimbRate(x);
}
/****************************************************************/
/* getAltitude - convert mbar and temperature into an altitude  */
/*               value                                          */
/****************************************************************/
float OXS_MS5611::calcAltitude(float pressure) {
  float   p; // Pressure
  int32_t t1=_calcTemperature/10; // only use the first measured temperature for calculation
  p=(float) pressure/100;
  const float sea_press = 1013.25;
  p=(((pow((sea_press / p), 1/5.257) - 1.0) * ( (t1/100) + 273.15)) / 0.0065 *100);

  return p;
}

/*******************************************************************************/
/* SaveClimbRate => calculate the current climbRate                            */
/* GetClimbRate = > Retrieve the average climbRate from the buffer             */
/*******************************************************************************/
#define CR_BUFFER_SIZE 20

float crsum=0;
int crNumValues=0;
void OXS_MS5611::SaveClimbRate(float pressure)
{
  static float lastPressure=pressure;
  static unsigned long Start=micros();
  crsum+=(lastPressure-pressure); // sum up the pressure dufferences
  crNumValues+=1;
  if(crNumValues==CR_BUFFER_SIZE){
    crsum=crsum *(metersPerMbar/1000000); // convert to altitude change in ??
    float CurrentClimbRate=(float) crsum *   (   (float)1000000     /  ( (float)micros() - Start ) ); // convert to m/s
    varioData.climbRate=CurrentClimbRate*100; // export value as m/s
    crNumValues=0;
    Start=micros();
    crsum=0;
    if (varioData.maxClimbRate<varioData.climbRate)varioData.maxClimbRate=varioData.climbRate;
    if (varioData.minClimbRate>varioData.climbRate)varioData.minClimbRate=varioData.climbRate;
  }
  lastPressure=pressure;
}

void OXS_MS5611::resetValues(){
  varioData.altOffset=varioData.absoluteAlt;
  varioData.maxRelAlt=0;
  varioData.minRelAlt=0;
  varioData.maxAbsAlt=varioData.absoluteAlt;
  varioData.minAbsAlt=varioData.absoluteAlt;
  varioData.minClimbRate=varioData.climbRate;
  varioData.maxClimbRate=varioData.climbRate;
}




