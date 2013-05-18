#include "Arduino.h"
#include "oxs_curr.h"
#include "HardwareSerial.h"
long _currentValues[CURRENT_BUFFER_LENGTH +1];  // Averaging buffer for the measured current values
    
OXS_CURRENT::OXS_CURRENT(uint8_t pinCurrent, HardwareSerial &print) {
  // constructor
  printer = &print; //operate on the address of print
  _microsLastCurrent=0;
  _pinCurrent=pinCurrent;
  pinMode(pinCurrent,INPUT); 
  currentData.available=false;
  printer->begin(115200);
  printer->print("Current sensor on pin:");
  printer->println(pinCurrent);
}

// **************** Setup the Current sensor *********************
void OXS_CURRENT::setupMinMaxA( int32_t milliAmps0V,int32_t milliAmps5V)
{
  currentData.milliAmps0V=milliAmps0V;
  currentData.milliAmps5V=milliAmps5V;
 
  currentData.milliVoltPerAmpere=5000/  ((milliAmps5V-milliAmps0V)/1000);
  currentData.idleMilliVolts=abs((int32_t)milliAmps0V)*(int32_t)currentData.milliVoltPerAmpere/1000;

  debugSendSetup();
  resetValues();
}
void OXS_CURRENT::setupIdleMvA( uint16_t idleMilliVolts,int16_t milliVoltPerAmpere)
{
  currentData.milliVoltPerAmpere=milliVoltPerAmpere;
  currentData.idleMilliVolts=idleMilliVolts;
  currentData.milliAmps0V=(int32_t)idleMilliVolts*1000/(int32_t)milliVoltPerAmpere*-1 ;
  currentData.milliAmps5V=(1000*(5000-(int32_t)idleMilliVolts))/(int32_t)milliVoltPerAmpere;

  //=(int32_t)idleMilliVolts/(int32_t)milliVoltPerAmpere*1000*-1
  
  debugSendSetup();
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

void OXS_CURRENT::readSensor()
{
    static unsigned long UpdateMs=0;
    uint16_t value=uint16_t((float)analogRead(_pinCurrent)*(float)(readVccMv()/1023.0));
    currentData.available=true;
    
    /*printer->print(" !!value=");    printer->print(value);
    printer->print(" !!current=");    printer->println(map(value,0,readVccMv(),currentData.milliAmps0V,currentData.milliAmps5V));
    printer->print(" !!mA=");    printer->println(getAverageCurrent());
*/    SaveCurrent(map(value,0,readVccMv(),currentData.milliAmps0V,currentData.milliAmps5V)); // save the current measurements...
    if (millis()>UpdateMs+1000){
      UpdateMs=millis();
      if (_microsLastCurrent>0){  // internal mAh calculation
        unsigned long micrPassed=micros()-_microsLastCurrent;
        _microsLastCurrent=micros();
        Serial.print(" MicrosPassed="); Serial.print(micrPassed);
        float tmp1=(60*60);
        printer->print(" A/SEC=");  printer->print((currentData.milliAmps/tmp1),DEC);
        printer->print(" mAh+=");  printer->print( ( (currentData.milliAmps/tmp1)*micrPassed)/1000000,DEC);
        currentData.consumedMilliAmps+=( (currentData.milliAmps/tmp1)*micrPassed)/1000000;
      }
   
    _microsLastCurrent=micros();
    currentData.milliAmps=getAverageCurrent();
    }
}



/****************************************************************/
/* SaveCurrent - save a new Current value to the buffer       */
/* Rotating Buffer for calculating the Average Current         */
/****************************************************************/
void OXS_CURRENT::SaveCurrent(long current){
  static int cnt =0;
  _currentValues[cnt++]=current;
  if(cnt>CURRENT_BUFFER_LENGTH){
      cnt=0;
  }
}
/****************************************************************/
/* getAverageCurrent - calculate average Current based on all    */
/* entries in the Current buffer                               */
/****************************************************************/
long OXS_CURRENT::getAverageCurrent()
{
  long result=0;
  for (int i=0;i<CURRENT_BUFFER_LENGTH;i++) result +=_currentValues[i];
  //return ((result/CurrentValuesCount)/10)*10;
  return result/CURRENT_BUFFER_LENGTH;
}

void OXS_CURRENT::resetValues(){
  currentData.consumedMilliAmps=0;
  currentData.maxMilliAmps=currentData.milliAmps;
  currentData.minMilliAmps=currentData.milliAmps;
}

/* readVcc - Read the real internal supply voltageof the arduino 

 Improving Accuracy
 While the large tolerance of the internal 1.1 volt reference greatly limits the accuracy 
 of this measurement, for individual projects we can compensate for greater accuracy. 
 To do so, simply measure your Vcc with a voltmeter and with our readVcc() function. 
 Then, replace the constant 1125300L with a new constant:
 
 scale_constant = internal1.1Ref * 1023 * 1000
 
 where
 
 internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
 
 This calibrated value will be good for the AVR chip measured only, and may be subject to
 temperature variation. Feel free to experiment with your own measurements. 
 */
/* 
 Meine Versorgungsspannung per Multimeter =4,87V 
 1,1*4870 /5115 =1,047311827957
 */
//const long scaleConst = 1071.4 * 1000 ; // internalRef * 1023 * 1000;
const long scaleConst = 1125.300 * 1000 ; // internalRef * 1023 * 1000;
int OXS_CURRENT::readVccMv() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result = scaleConst / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (int)result; // Vcc in millivolts
}


