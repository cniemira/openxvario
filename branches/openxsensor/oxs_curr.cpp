#include "Arduino.h"
#include "oxs_curr.h"
#include "HardwareSerial.h"
OXS_CURRENT::OXS_CURRENT(uint8_t pinCurrent, HardwareSerial &print) {
  // constructor
  printer = &print; //operate on the address of print
  _microsLastCurrent=0;
  _pinCurrent=pinCurrent;
  pinMode(pinCurrent,INPUT); 
  currentData.available=false;
#ifdef DEBUG  
  printer->print("Current sensor on pin:");
  printer->println(pinCurrent);
#endif
}

// **************** Setup the Current sensor *********************
void OXS_CURRENT::setupMinMaxA( int32_t milliAmps0V,int32_t milliAmps5V)
{
  currentData.milliAmps0V=milliAmps0V;
  currentData.milliAmps5V=milliAmps5V;

  currentData.milliVoltPerAmpere=5000/  ((milliAmps5V-milliAmps0V)/1000);
  currentData.idleMilliVolts=abs((int32_t)milliAmps0V)*(int32_t)currentData.milliVoltPerAmpere/1000;

#ifdef DEBUG  
  debugSendSetup();
#endif
  prefillBuffer();
  resetValues();
}
void OXS_CURRENT::setupIdleMvA( uint16_t idleMilliVolts,int16_t milliVoltPerAmpere)
{
  currentData.milliVoltPerAmpere=milliVoltPerAmpere;
  currentData.idleMilliVolts=idleMilliVolts;
  currentData.milliAmps0V=(int32_t)idleMilliVolts*1000/(int32_t)milliVoltPerAmpere*-1 ;
  currentData.milliAmps5V=(1000*(5000-(int32_t)idleMilliVolts))/(int32_t)milliVoltPerAmpere;

  debugSendSetup();
  prefillBuffer();
  resetValues();
}
void OXS_CURRENT::prefillBuffer()
{
  // Prefill buffers.
#ifdef DEBUG
  printer->print("Prefilling current buffers...");
#endif
  for (int i=0;i<CURRENT_BUFFER_LENGTH;i++){
    delay(10);
    readSensor(5000);
  }
#ifdef DEBUG
  printer->println("done.");
#endif
  resetValues();

}
/****************************************************************/
/* debugSendSetup - Send the current setup data via debug..     */
/****************************************************************/
void OXS_CURRENT::debugSendSetup()
{
  printer->println("Current sensor setup:");
  printer->print("Pin=");
  printer->println(_pinCurrent);
  printer->print("idleMv=");
  printer->println(currentData.idleMilliVolts);
  printer->print("mV/A=");
  printer->println(currentData.milliVoltPerAmpere);
  printer->print("mA@0V=");
  printer->println(currentData.milliAmps0V);
  printer->print("mA@5V=");
  printer->println(currentData.milliAmps5V);
}

void OXS_CURRENT::readSensor(uint16_t vRef)
{
  static unsigned long UpdateMs=0;
  int vcc=vRef;
  uint16_t value=uint16_t((float)analogRead(_pinCurrent)*(float)(vcc/1023.0));
  currentData.available=true;
  SaveCurrent(map(value,0,vcc,currentData.milliAmps0V,currentData.milliAmps5V)); // save the current measurements...

  // calculate the consumed milliAmps every <n> ms
  if (millis()>(UpdateMs+100)){
    UpdateMs=millis();
    uint16_t value=uint16_t((float)analogRead(_pinCurrent)*(float)(vcc/1023.0));
    currentData.available=true;
    SaveCurrent(map(value,0,vcc,currentData.milliAmps0V,currentData.milliAmps5V)); // save the current measurements...

    if (_microsLastCurrent>0){  // internal mAh calculation

      unsigned long micrPassed=micros()-_microsLastCurrent;
      _microsLastCurrent=micros();
      float tmp1=(60*60);
      currentData.consumedMilliAmps+=( (currentData.milliAmps/tmp1)*micrPassed)/1000000;
    }
    _microsLastCurrent=micros();
  }
}

/****************************************************************/
/* SaveCurrent - save a new Current value to the buffer       */
/* Rotating Buffer for calculating the Average Current         */
/****************************************************************/
void OXS_CURRENT::SaveCurrent(long current){
  static int cnt =0;
  static long sum =0;
  sum+=current;
  cnt++;
  if(cnt==CURRENT_BUFFER_LENGTH){
    currentData.milliAmps=sum/CURRENT_BUFFER_LENGTH;
    sum=0;
    cnt=0;
    if(currentData.minMilliAmps>currentData.milliAmps)currentData.minMilliAmps=currentData.milliAmps;
    if(currentData.maxMilliAmps<currentData.milliAmps)currentData.maxMilliAmps=currentData.milliAmps;
  }
}

void OXS_CURRENT::resetValues(){
  currentData.consumedMilliAmps=0;
  currentData.maxMilliAmps=currentData.milliAmps;
  currentData.minMilliAmps=currentData.milliAmps;
}




