#include "Arduino.h"
#include "OXS_MS5611.h"
#include "HardwareSerial.h"

OXS_MS5611::OXS_MS5611(uint8_t addr, HardwareSerial &print) {
	// constructor
	_addr=addr;
	varioData.paramKalman_r=200;  // sensor noise
	varioData.paramKalman_q=0.05; // process noise
    printer = &print; //operate on the address of print
	printer->begin(115200);
	printer->print("Vario Sensor:MS5611 I2C Addr=");printer->println(_addr,HEX);
}

// **************** Setup the MS5611 sensor *********************
void OXS_MS5611::setup()
{
  // Serial.println("Setup Sensor Start.");
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
	   printer->println("Error: calibration data not available");
    }
    high = Wire.read();
    low = Wire.read();
	_calibrationData[i] = high<<8 | low;
    printer->print("calibration data #");
    printer->print(i);
    printer->print(" = ");
    printer->println( _calibrationData[i] ); 

  }
  // Prefill buffers.
  printer->print("Prefilling Vario Buffers...");

  for(int i = 1 ;i<=200;i++)readSensor();
  printer->println("done.");
  
  resetValues();
}
/****************************************************************/
/* getData - Read data from I2C bus                             */
/****************************************************************/
long OXS_MS5611::getData(byte command, byte del)
{
  long result = 0;
  SendCommand(command);
  delay(del);
  SendCommand(0x00);
  Wire.beginTransmission(_addr);  
  Wire.requestFrom(_addr,(uint8_t) 3);
  
#ifdef DEBUG
  if(Wire.available()!=3)Serial.println("Error: raw data not available");
#endif

  for (int i = 0; i <= 2; i++)    result = (result<<8) | Wire.read(); 
  return result;
}


/****************************************************************/
/* SendCommand - Send a command to the I2C Bus               */
/****************************************************************/
/* Send a command to the MS5611 */
void OXS_MS5611::SendCommand(byte command)
{
  Wire.beginTransmission(_addr);
  if (!Wire.write(command)){
      printer->println("Error: write()");
  }
  if (Wire.endTransmission()) 
  {
    printer->print("Error when sending command: ");
    printer->println(command, HEX);
  }
}
  
/****************************************************************/
/* readSensor - Read pressure + temperature from the MS5611    */
/****************************************************************/
float OXS_MS5611::readSensor()
{
  int32_t D1;
  int64_t D2, dT, P;
  float TEMP;
  int64_t OFF, SENS;

  D1 = getData(0x48, 9); //OSR=4096 0.012 mbar precsision
  D2 = getData(0x50, 1);

  dT = D2 - ((long)_calibrationData[5] << 8);
  TEMP = (2000 + (((int64_t)dT * (int64_t)_calibrationData[6]) >> 23)) / (float)100;
  varioData.temperature=(int)(TEMP*10);
/*
  OFF = ((unsigned long)_calibrationData[2] << 16) + (((int64_t)_calibrationData[4] * dT) >> 7);
  SENS = ((unsigned long)_calibrationData[1] << 15) + (((int64_t)_calibrationData[3] * dT) >> 8);
  P = (((D1 * SENS) >> 21) - OFF) >> 15;
*/  
  
  //test
  OFF  = (((int64_t)_calibrationData[2]) << 16) + ((_calibrationData[4] * dT) >> 7);
  SENS = (((int64_t)_calibrationData[1]) << 15) + ((_calibrationData[3] * dT) >> 8);
  P= ((((D1 * SENS) >> 21) - OFF) >> (15-EXTRA_PRECISION)) / ((1<<EXTRA_PRECISION) );
  
  //rawPressure=P + PressureCalibrationOffset;
  rawPressure=P ;
  
  kalman_update(rawPressure);
  calcAltitude();
  return rawPressure;
  
}

void OXS_MS5611::kalman_update(float measurement)
{
  static float x=rawPressure; //value
  static float p=100; //estimation error covariance
  static float k=0; //kalman gain
  
  // update the prediction value
  p = p + varioData.paramKalman_q;

  // update based on measurement
  k = p / (p + varioData.paramKalman_r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;
  
#ifdef KALMANDUMP
  abweichung=measurement -x;
  // output to processing.
  Serial.print(rawalt);Serial.print(",");Serial.print(x);Serial.print(",");Serial.println(abweichung);
#endif
  varioData.pressure=x;
}
/****************************************************************/
/* getAltitude - convert mbar and temperature into an altitude  */
/*               value                                          */
/****************************************************************/
void OXS_MS5611::calcAltitude() {
  float   result;
  int32_t t1=varioData.temperature/100;
  result=(float)varioData.pressure/100;
  const float sea_press = 1013.25;
  result=(((pow((sea_press / result), 1/5.257) - 1.0) * ( (t1) + 273.15)) / 0.0065 *100);
  SaveClimbRate(result);
  _absoluteAlt=result;
  _relativeAlt=_absoluteAlt- varioData.altOffset;
   
  varioData.absoluteAlt=(int32_t)_absoluteAlt;
  varioData.relativeAlt=(int32_t)_relativeAlt;
  if(varioData.maxAbsAlt<varioData.absoluteAlt)varioData.maxAbsAlt=varioData.absoluteAlt;
  if(varioData.minAbsAlt>varioData.absoluteAlt)varioData.minAbsAlt=varioData.absoluteAlt;
}
/*******************************************************************************/
/* SaveClimbRate => calculate the current climbRate                            */
/* GetClimbRate = > Retrieve the average climbRate from the buffer             */
/*******************************************************************************/
void OXS_MS5611::SaveClimbRate(float alti){
  long now=micros();
  static long lastMicrosVerticalSpeed=micros();
  long timecalc=now-lastMicrosVerticalSpeed; // the time passed since last CR Calculation
  static float lastAlti=alti;
  static byte cnt=0;
  lastMicrosVerticalSpeed=now;
  float CurrentClimbRate=(float)(alti-lastAlti)*((float)1000000/(float)timecalc);
  _climbRateQueue[cnt]=CurrentClimbRate; // store the current ClimbRate
  cnt+=1;
  if (cnt ==ClimbRateQueueLength)cnt=0;
  lastAlti=alti;
  calcClimbRate();
}
/* calc the average climbRate */
void OXS_MS5611::calcClimbRate(){
  float myClimbRate=0;
  for(int i=0;i<ClimbRateQueueLength;){
    myClimbRate+=_climbRateQueue[i];
    i++;
  }
  myClimbRate/=ClimbRateQueueLength;
  
  // rounding
  //myClimbRate=((int32_t) (myClimbRate/2)) *2;
  varioData.climbRate=(int32_t) myClimbRate;
  if (varioData.maxClimbRate<varioData.climbRate)varioData.maxClimbRate=varioData.climbRate;
  if (varioData.minClimbRate>varioData.climbRate)varioData.minClimbRate=varioData.climbRate;
}
void OXS_MS5611::resetValues(){
	varioData.altOffset=varioData.absoluteAlt;
	varioData.maxAbsAlt=varioData.absoluteAlt;
	varioData.minAbsAlt=varioData.absoluteAlt;
	varioData.minClimbRate=varioData.climbRate;
	varioData.maxClimbRate=varioData.climbRate;
}