#ifndef OXS_ARDUINO_h
#define OXS_ARDUINO_h

#include "Arduino.h"

struct ARDUINODATA {
  bool available;                   // true if data is available
  uint16_t vrefMilliVolts;          // in mV the internal measured voltage Reference
  uint16_t dividerMilliVolts;
  
  uint16_t maxVrefMilliVolts;       // in mV
  uint16_t minVrefMilliVolts;       // in mV
  uint16_t minDividerMilliVolts;    // in mV
  uint16_t maxDividerMilliVolts;    // in mV
  uint16_t loopTimeMilliSeconds;    // loop time in ms
};
#define VREF_BUFFER_LENGTH 150
#define DIVIDER_BUFFER_LENGTH 150
class OXS_ARDUINO {
  public:
#ifdef DEBUG  
    OXS_ARDUINO(HardwareSerial &print);
#else
//    OXS_ARDUINO(HardwareSerial &print);
    OXS_ARDUINO( uint8_t x );
#endif
    ARDUINODATA arduinoData ;
	void setupDivider( int16_t DividerAnalogPin, uint32_t ohmToGnd,uint32_t OhmToBat );
	void readSensor();
	void resetValues();
    
  private:
#ifdef DEBUG  
     HardwareSerial* printer;
#endif
     byte _pinDivider;              // The analog Input PIN for the voltage Divider
		 uint32_t vrefSum ;
     uint16_t readVccMv();
		 uint32_t ArduinoSum ;
     float _resistorFactor;
     void SaveVRef(uint16_t value);
     void SaveDividerVoltage(uint16_t value);
     void prefillBuffer();

};

#endif // OXS_ARDUINO_h


