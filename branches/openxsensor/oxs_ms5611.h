#ifndef OXS_MS5611_h
#define OXS_MS5611_h

#include "Arduino.h"
#include <Wire.h>

#define PressureCalibrationOffset 0
#define EXTRA_PRECISION 5
#define ClimbRateQueueLength 20 // averaging buffer for the climbrate
struct VARIODATA {
  bool available;          // true if data is available
  float pressure;          // in 1/100 mBar
  int16_t temperature;     // in 1/10 Celsius
  
  int32_t absoluteAlt;     // in cm
  int32_t relativeAlt;	   // in cm
  int32_t maxAbsAlt;       // in cm
  int32_t minAbsAlt;       // in cm
  int32_t climbRate;       // in cm
  int32_t minClimbRate;    // in cm
  int32_t maxClimbRate;    // in cm
  
  int32_t altOffset;       // in cm    
  uint16_t paramKalman_r;  // 50..1000  sensor noise
  double paramKalman_q;    // 0.05      process noise covariance
};
class OXS_MS5611 {
  public:
    OXS_MS5611(uint8_t addr, HardwareSerial &print);
    VARIODATA varioData ;
	float rawPressure; // in 1/100 mBar

	/* float pressure; 
	int32_t absoluteAlt;
	int32_t relativeAlt;
	int32_t maxAbsAlt;
	int32_t minAbsAlt;
	int16_t temperature;     // in 1/10 Celsius
	uint16_t paramKalman_r;             // 50..1000
	double paramKalman_q; //process noise covariance
    */
	
	void setup();
	float readSensor();
	
	void resetValues();
    
  private:
    uint8_t _addr;
    unsigned int _calibrationData[7]; // The factory calibration data of the ms5611
	void SendCommand(byte command);
    long getData(byte command, byte del);
    HardwareSerial* printer;
	void kalman_update(float measurement);
	float _climbRateQueue[ClimbRateQueueLength ];
	void SaveClimbRate(float alti);
    void calcAltitude();
	void calcClimbRate();
	//float _altOffset;
	float _absoluteAlt;
	float _relativeAlt;
	};

#endif // OXS_MS5611
