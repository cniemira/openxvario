#include "Arduino.h"
#include "oxs_arduino.h"
#include "HardwareSerial.h"
#include "oxs_config.h"

OXS_ARDUINO::OXS_ARDUINO(HardwareSerial &print) 
{
  printer = &print; //operate on the address of print   
}

// **************** Setup the Current sensor *********************
void OXS_ARDUINO::setupDivider( int16_t DividerAnalogPin, uint32_t ohmToGnd,uint32_t OhmToBat )
{
  if (DividerAnalogPin >=0)
  {
    _pinDivider=DividerAnalogPin;
    _resistorFactor = (float)ohmToGnd/((float)OhmToBat + (float)ohmToGnd);
#ifdef DEBUG
    printer->print("Divider ohmToGnd=");
    printer->println( ohmToGnd);
    printer->print("Divider OhmToBat=");
    printer->println( OhmToBat);
    printer->print("Divider _resistorFactor=");
    printer->println( _resistorFactor);
    printer->print("Divider _Multiplicator=");
    printer->println(1/ _resistorFactor);
    printer->print("Maximum allowed Voltage allowed on Voltage Divider=");
    printer->println(5/ _resistorFactor);
#endif
  }
  prefillBuffer();
  readSensor();
  resetValues();
}
void OXS_ARDUINO::prefillBuffer()
{
  // Prefill buffers.
#ifdef DEBUG
  printer->print("Prefilling arduino buffers...");
#endif
  for (int i=0;i<VREF_BUFFER_LENGTH*2;i++){
     readSensor();
  }
#ifdef DEBUG
  printer->println("done.");
#endif
  resetValues();

}
// R2= Resistor from analog pin to GND
// R1= Resistor from analog pin to to voltage to be measured
// Val= Measured Voltage on Analog Pin
// Voltage=Val/(R2/(R1+R2)
// Maximum voltage that is allowed (theoretical) on the voltage divider is 5/R1/(R+R2);

void OXS_ARDUINO::readSensor()
{
  static uint32_t lastMs=millis();
  arduinoData.loopTimeMilliSeconds=millis()-lastMs;
  lastMs=millis();
  SaveVRef((uint16_t)readVccMv());
  if (_pinDivider >=0)
  {
    int val = analogRead(_pinDivider); // read the value from the sensor
    SaveDividerVoltage(((float)val *((float)arduinoData.vrefMilliVolts/(float)1024)/(float)_resistorFactor));
  }
  arduinoData.available=true;
}

void OXS_ARDUINO::resetValues()
{
  arduinoData.maxVrefMilliVolts=arduinoData.vrefMilliVolts;
  arduinoData.minVrefMilliVolts=arduinoData.vrefMilliVolts;       // in mV
  arduinoData.minDividerMilliVolts=arduinoData.dividerMilliVolts; // in mV
  arduinoData.maxDividerMilliVolts=arduinoData.dividerMilliVolts;  // in mV
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
uint16_t OXS_ARDUINO::readVccMv() {
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

  delay(1); // Wait for Vref to settle // Not reqruited for pro mini with atmega328
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  uint16_t result = (high<<8) | low;

  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result = scaleConst / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (uint16_t)result; // Vcc in millivolts
}
/****************************************************/
/* Savexyz - save a new  value to the buffer        */
/* Buffer for calculating the Averages     */
/****************************************************/
void OXS_ARDUINO::SaveVRef(uint16_t value)
{
  static byte cnt =0;
  static uint32_t sum=0;
  sum+=value;
  cnt++;
  if(cnt==VREF_BUFFER_LENGTH){
    arduinoData.vrefMilliVolts=sum/VREF_BUFFER_LENGTH;
    if(arduinoData.minVrefMilliVolts>arduinoData.vrefMilliVolts)arduinoData.minVrefMilliVolts=arduinoData.vrefMilliVolts;
    if(arduinoData.maxVrefMilliVolts<arduinoData.vrefMilliVolts)arduinoData.maxVrefMilliVolts=arduinoData.vrefMilliVolts;
    sum=0;
    cnt=0;
  }
}
void OXS_ARDUINO::SaveDividerVoltage(uint16_t value)
{
  static byte cnt =0;
  static uint32_t sum=0;
  sum+=value;

  cnt++;
  if(cnt==DIVIDER_BUFFER_LENGTH){
    arduinoData.dividerMilliVolts=sum/DIVIDER_BUFFER_LENGTH ;

#ifdef VOLTAGE_DIVIDER_CALIBRATION_OFFSET_MV
    if ((int32_t)arduinoData.dividerMilliVolts >(int32_t)VOLTAGE_DIVIDER_CALIBRATION_OFFSET_MV)arduinoData.dividerMilliVolts+=VOLTAGE_DIVIDER_CALIBRATION_OFFSET_MV;
    else arduinoData.dividerMilliVolts=0;
#endif
    if(arduinoData.minDividerMilliVolts>arduinoData.dividerMilliVolts)arduinoData.minDividerMilliVolts=arduinoData.dividerMilliVolts;
    if(arduinoData.maxDividerMilliVolts<arduinoData.dividerMilliVolts)arduinoData.maxDividerMilliVolts=arduinoData.dividerMilliVolts; 
    cnt=0;
    sum=0;
    cnt=0;
  }
}








