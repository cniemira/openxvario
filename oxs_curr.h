#ifndef OXS_CURRENT_h
#define OXS_CURRENT_h

#include "Arduino.h"

struct CURRENTDATA {
  int32_t milliAmps;          // in mA
  int32_t consumedMilliAmps;  // in mA
  
  // config values should be set via method calls
  int32_t maxMilliAmps;       // in mA
  int32_t minMilliAmps;       // in mA
  uint16_t idleMilliVolts;       // in mV
  int16_t milliVoltPerAmpere;// in mV
  int32_t MilliAmps0V;       // in mA theoretical value for calculation. do not need to match allowed sensor range
  int32_t MilliAmps5V;       // in mA theoretical value for calculation. do not need to match allowed sensor range
};

class OXS_CURRENT {
  public:
    OXS_CURRENT(uint8_t addr, HardwareSerial &print);
    CURRENTDATA currentData ;
	void setupMinMaxA( int32_t MilliAmps0V,int32_t MilliAmps5V);
	void setupIdleMvA( uint16_t idleMilliVolts,int16_t milliVoltPerAmpere);
	void readSensor();
	void resetValues();
    
  private:
     HardwareSerial* printer;
     byte _pinCurrent;
     void debugSendSetup();
};

#endif // OXS_CURRENT_h
