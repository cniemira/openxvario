
#include <../../../../libraries/SoftwareSerial/SoftwareSerial.h>
#include "Arduino.h"

#include "OXS_OUT_FRSKY.h"
#include "HardwareSerial.h"
#include "oxs_config.h"


OXS_OUT_FRSKY::OXS_OUT_FRSKY(uint8_t pinTx,HardwareSerial &print) : 
_mySerial(SoftwareSerial(0,pinTx,true))
{
  // constructor
  _pinTx=pinTx;
  printer = &print; //operate on the address of print
}
// Software Serial is used including the Inverted Signal option ( the "true" in the line below ) 
// Pin PIN_SerialTX has to be connected to rx pin of the receiver
// we do not need the RX so we set it to 0

// **************** Setup the FRSky OutputLib *********************
void OXS_OUT_FRSKY::setup()
{

  _mySerial=SoftwareSerial(0, _pinTx,true); // RX, TX
  _mySerial.begin(9600);
  printer->begin(115200);
  printer->print("FRSky Output Module: TX Pin=");
  printer->println(_pinTx);
  printer->println("FRSky Output Module: Setup!");
}

/****************************************************************/
/* sendData - Output data to the FrSky Receiver/Transceiver     */
/****************************************************************/
void OXS_OUT_FRSKY::sendData()
{
  static unsigned long lastMsFrame1=0;
  static unsigned long lastMsFrame2=0;

  if ( (millis()-lastMsFrame1) > INTERVAL_FRAME1  ) {
    lastMsFrame1=millis();
    SendFrame1();

  }
  if ( (millis()-lastMsFrame2) > INTERVAL_FRAME2  ) {
    lastMsFrame2=millis();
    SendFrame2();
  }
}
// Send Frame 1 via serial
void OXS_OUT_FRSKY::SendFrame1(){
  printer->print("FRSky output module: SendFrame1:");
  if (varioData.available){ //========================================================================== Vario Data
    printer->print("Sending vario data ");
    SendAlt(varioData.absoluteAlt);    
    SendValue(FRSKY_USERDATA_VERT_SPEED,(int16_t)varioData.climbRate); // ClimbRate in open9x Vario mode
    // ********************************* The DIST Field
#ifdef SEND_AltAsDIST     // send alt as adjusted to precision in dist field
    if (varioData.absoluteAlt- SEND_AltAsDIST <= 32768) SendGPSDist(uint16_t(varioData.absoluteAlt- SEND_AltAsDIST ));
    else if (varioData.absoluteAlt- SEND_AltAsDIST < 327680) SendGPSDist(uint16_t((varioData.absoluteAlt-SEND_AltAsDIST)/(long)10));
    // If altitude gets higher than 327,68m alt/10 will be transmitted till 3276,8m then alt/100
    else SendGPSDist(uint16_t((varioData.absoluteAlt-SEND_AltAsDIST)/(long)100));
#endif 
#ifdef SEND_PressureAsDIST
    SendGPSDist(uint16_t(varioData.pressure/10));
#endif
#ifdef SEND_SensitivityAsDist
    SendGPSDist(uint16_t(varioData.paramKalman_r));
#endif
    // ********************************* The RPM Field   
#ifdef SEND_AltAsRPM     
    SendRPM(varioData.absoluteAlt);
#endif
#ifdef SEND_PressureAsRPM
    SendRPM(uint16_t(varioData.pressure/10));
#endif
#ifdef SEND_SensitivityAsRPM
    SendRPM(uint16_t(varioData.paramKalman_r));
#endif
    // ********************************* The Temp 1 FIeld
#ifdef SEND_TEMP_T1      
      SendTemperature1(varioData.temperature); 
#endif
#ifdef SEND_PressureAsT1 // pressure in T1 Field
      SendTemperature1(varioData.pressure-SEND_PressureAsT1*10); //pressure in T1 Field
#endif

    // ********************************* The Temp 2 FIeld
#ifdef SEND_TEMP_T2      
    SendTemperature2(varioData.temperature); 
#endif
#ifdef SEND_SensitivityAsT2 // Kalman Param R in Temp2
    SendTemperature2(uint16_t(varioData.paramKalman_r)*10); 
#endif
#ifdef SEND_PressureAsT2 // pressure in T2 Field
    SendTemperature2(varioData.pressure-SEND_PressureAsT2*10); //pressure in T2 Field
#endif
  }else {
    printer->print("No vario data Available!");
  }

  _mySerial.write(0x5E); // End of Frame 1!
  printer->print("absoluteAlt=");    
  printer->println( ( ((float)(int32_t)(varioData.absoluteAlt))) /100);
}
// Send Frame 2 via serial
void OXS_OUT_FRSKY::SendFrame2(){
  printer->print("FRSky Output Module: SendFrame2!");
  printer->print("mBar=");    
  printer->println( ( ((float)(int32_t)(varioData.pressure))) /100);
}

/**********************************************************/
/* SendValue => send a value as frsky sensor hub data     */
/**********************************************************/
void OXS_OUT_FRSKY::SendValue(uint8_t ID, uint16_t Value) {
  uint8_t tmp1 = Value & 0x00ff;
  uint8_t tmp2 = (Value & 0xff00)>>8;
  _mySerial.write(0x5E);  
  _mySerial.write(ID);
  if(tmp1 == 0x5E) { 
    _mySerial.write(0x5D);    
    _mySerial.write(0x3E);  
  } 
  else if(tmp1 == 0x5D) {    
    _mySerial.write(0x5D);    
    _mySerial.write(0x3D);  
  } 
  else {    
    _mySerial.write(tmp1);  
  }
  if(tmp2 == 0x5E) {    
    _mySerial.write(0x5D);    
    _mySerial.write(0x3E);  
  } 
  else if(tmp2 == 0x5D) {    
    _mySerial.write(0x5D);    
    _mySerial.write(0x3D);  
  } 
  else {    
    _mySerial.write(tmp2);  
  }
  // mySerial.write(0x5E);
}

/**********************************************************/
/* SendCellVoltage => send a cell voltage                 */
/**********************************************************/
void OXS_OUT_FRSKY::SendCellVoltage(uint8_t cellID, uint16_t voltage) {
  voltage /= 2;
  uint8_t v1 = (voltage & 0x0f00)>>8 | (cellID<<4 & 0xf0);
  uint8_t v2 = (voltage & 0x00ff);
  uint16_t Value = (v1 & 0x00ff) | (v2<<8);
  SendValue(FRSKY_USERDATA_CELL_VOLT, Value);
}
/**********************************/
/* SendGPSDist => send 0..32768   */
/**********************************/
void OXS_OUT_FRSKY::SendGPSDist(uint16_t dist) {// ==> Field "Dist" in open9x
  SendValue(0x3C,uint16_t(dist)); //>> DIST
}
void OXS_OUT_FRSKY::SendTemperature1(int16_t tempc) {
  SendValue(FRSKY_USERDATA_TEMP1, tempc/10);
}
void OXS_OUT_FRSKY::SendTemperature2(int16_t tempc) {
  SendValue(FRSKY_USERDATA_TEMP2, tempc/10);
}
/*************************************/
/* SendRPM => Send Rounds Per Minute */
/*************************************/
void OXS_OUT_FRSKY::SendRPM(uint16_t rpm) {
  byte blades=2;
  rpm = uint16_t((float)rpm/(60/blades));  
  SendValue(FRSKY_USERDATA_RPM, rpm);
}

/**********************************/
/* SendAlt => Send ALtitude in cm */
/**********************************/
void OXS_OUT_FRSKY::SendAlt(long altcm)
{
  uint16_t Centimeter =  uint16_t(abs(altcm)%100);
  long Meter;
  if (altcm >0){
    Meter = (altcm-(long)Centimeter);
  }
  else{
    Meter = -1*(abs(altcm)+(long)Centimeter);
  }
  Meter=Meter/100;


  SendValue(FRSKY_USERDATA_BARO_ALT_B, (int16_t)Meter);
  SendValue(FRSKY_USERDATA_BARO_ALT_A, Centimeter);
}
/****************************************************************/
/* SendGPSAlt - send the a value to the GPS altitude field      */
/****************************************************************/
void OXS_OUT_FRSKY::SendGPSAlt(long altcm)
{
  uint16_t Centimeter =  uint16_t(altcm%100);
  int16_t Meter = int16_t((altcm-Centimeter)/100);

  SendValue(FRSKY_USERDATA_GPS_ALT_B, Meter);
  SendValue(FRSKY_USERDATA_GPS_ALT_A, Centimeter);
}


