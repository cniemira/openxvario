#ifndef OXS_ARDUINO_h
#define OXS_ARDUINO_h

#include "arduino.h"

struct ARDUINODATA {
  bool available;                   // true if data is available
  uint32_t vrefMilliVolts;          // in mV the internal measured voltage Reference
  uint32_t dividerMilliVolts;
  
  uint32_t maxVrefMilliVolts;       // in mV
  uint32_t minVrefMilliVolts;       // in mV
  uint32_t minDividerMilliVolts;    // in mV
  uint32_t maxDividerMilliVolts;    // in mV
  uint16_t loopTimeMilliSeconds;    // loop time in ms
};
#define VREF_BUFFER_LENGTH 150
#define DIVIDER_BUFFER_LENGTH 150
class OXS_ARDUINO {
  public:
    OXS_ARDUINO(HardwareSerial &print);
    ARDUINODATA arduinoData ;
	void setupDivider( int16_t DividerAnalogPin, uint32_t ohmToGnd,uint32_t OhmToBat );
	void readSensor();
	void resetValues();
    
  private:
     HardwareSerial* printer;
     byte _pinDivider;              // The analog Input PIN for the voltage Divider
     uint16_t readVccMv();
     float _resistorFactor;
     void SaveVRef(uint16_t value);
     void SaveDividerVoltage(uint16_t value);

};

#endif // OXS_ARDUINO_h
