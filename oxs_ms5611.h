#ifndef OXS_MS5611_h
#define OXS_MS5611_h

#include "Arduino.h"
#include <Wire.h>

#define PressureCalibrationOffset 0
#define EXTRA_PRECISION 5
#define ClimbRateQueueLength 20// averaging buffer for the climbrate
struct VARIODATA {
  bool available;          // true if data is available
  float pressure;          // in 1/100 mBar
  int16_t temperature;     // in 1/10 Celsius

  int32_t absoluteAlt;     // in cm
  int32_t relativeAlt;	   // in cm
  int32_t maxRelAlt;       // in cm
  int32_t minRelAlt;       // in cm
  int32_t maxAbsAlt;       // in cm
  int32_t minAbsAlt;       // in cm
  int32_t climbRate;       // in cm
  int32_t minClimbRate;    // in cm
  int32_t maxClimbRate;    // in cm

  int32_t altOffset;       // in cm    
  uint16_t paramKalman_r;  // 50..1000  sensor noise
  double paramKalman_q;    // 0.05      process noise covariance
  float paramKalman_k;
};
class OXS_MS5611 {
public:
  OXS_MS5611(uint8_t addr, HardwareSerial &print, uint16_t kalman_r);
  VARIODATA varioData ;
  int64_t rawPressure; // in 1/100 mBar
  void setup();
  void  readSensor();

  void resetValues();

private:
  uint8_t _addr;
  unsigned int _calibrationData[7]; // The factory calibration data of the ms5611
  void SendCommand(byte command);
  long getData(byte command, byte del);
  HardwareSerial* printer;
 
 //void kalman_update(float measurement);
 void kalman_update(int64_t measurement);

  //float _climbRateQueue[ClimbRateQueueLength+1 ];
  void SaveClimbRate(float alti);
  //void SaveClimbRateOld(float alti);
  float calcAltitude(float pressure);
  //void calcClimbRate();
  int16_t _calcTemperature;     // in 1/10 Celsius only used for alt calculation. 
 };

#endif // OXS_MS5611



