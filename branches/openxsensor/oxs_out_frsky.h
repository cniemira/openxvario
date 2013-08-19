#ifndef OXS_OUT_FRSKY_h
#define OXS_OUT_FRSKY_h

#include <../../../../libraries/SoftwareSerial/SoftwareSerial.h>
#include "Arduino.h"
#include "oxs_config.h"
#include "OXS_MS5611.h" // we need the variodata struct
#include "OXS_curr.h" // we need the currentdata struct
#include "OXS_arduino.h" // we need the arduinodata struct

#define INTERVAL_FRAME1 100
#define INTERVAL_FRAME2 1000

#define FRSKY_USERDATA_GPS_ALT_B    0x01
#define FRSKY_USERDATA_TEMP1        0x02
#define FRSKY_USERDATA_RPM          0x03
#define FRSKY_USERDATA_FUEL         0x04
#define FRSKY_USERDATA_TEMP2        0x05
#define FRSKY_USERDATA_CELL_VOLT    0x06

#define FRSKY_USERDATA_GPS_ALT_A    0x09
#define FRSKY_USERDATA_BARO_ALT_B   0x10
#define FRSKY_USERDATA_GPS_SPEED_B  0x11
#define FRSKY_USERDATA_GPS_LONG_B   0x12
#define FRSKY_USERDATA_GPS_LAT_B    0x13
#define FRSKY_USERDATA_GPS_CURSE_B  0x14
#define FRSKY_USERDATA_GPS_DM       0x15
#define FRSKY_USERDATA_GPS_YEAR     0x16
#define FRSKY_USERDATA_GPS_HM       0x17
#define FRSKY_USERDATA_GPS_SEC      0x18
#define FRSKY_USERDATA_GPS_SPEED_A  0x19
#define FRSKY_USERDATA_GPS_LONG_A   0x1A
#define FRSKY_USERDATA_GPS_LAT_A    0x1B
#define FRSKY_USERDATA_GPS_CURSE_A  0x1C

#define FRSKY_USERDATA_BARO_ALT_A   0x21
#define FRSKY_USERDATA_GPS_LONG_EW  0x22
#define FRSKY_USERDATA_GPS_LAT_EW   0x23
#define FRSKY_USERDATA_ACC_X        0x24
#define FRSKY_USERDATA_ACC_Y        0x25
#define FRSKY_USERDATA_ACC_Z        0x26

#define FRSKY_USERDATA_CURRENT      0x28

#define FRSKY_USERDATA_VERT_SPEED   0x30 // open9x Vario Mode Only
#define FRSKY_USERDATA_VFAS_NEW     0x39 // open9x Vario Mode Only

#define FRSKY_USERDATA_VOLTAGE_B    0x3A
#define FRSKY_USERDATA_VOLTAGE_A    0x3B


class OXS_OUT_FRSKY {
  public:
    OXS_OUT_FRSKY(uint8_t pinTx,HardwareSerial &print);
    VARIODATA* varioData ;
    CURRENTDATA* currentData ;
    ARDUINODATA* arduinoData ;
    void setup();
    
#if defined(FRSKY_SPORT)
    bool timeToSend();
#endif

    void sendData();
    
  private:
    uint8_t _pinTx;
    HardwareSerial* printer;
    void SendFrame1A();
    void SendFrame1B();
    void SendFrame2();
    void SendValue(uint8_t ID, uint16_t Value);
    void SendCellVoltage(uint8_t cellID, uint16_t voltage);
    void SendGPSDist(uint16_t dist);
    void SendTemperature1(int16_t tempc);
    void SendTemperature2(int16_t tempc);
    void SendRPM(uint16_t rpm) ;
    void SendAlt(long altcm);
    void SendGPSAlt(long altcm);
    void SendFuel(uint16_t fuel);
    void SendCurrentMilliAmps(int32_t milliamps);
    SoftwareSerial _mySerial;

#if defined(FRSKY_SPORT)
    short crc;
    uint8_t counter;
    void sendByte(uint8_t byte);
    void sendValue(uint16_t id, uint32_t value);
    void sendCrc();
#endif
    
};

#endif // OXS_OUT_FRSKY_h




