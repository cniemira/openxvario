
#include "oxs_config.h"
#include "Arduino.h"

#include "OXS_OUT_FRSKY.h"
#include "HardwareSerial.h"
#include "Aserial.h"

#if defined(FRSKY_SPORT)
struct t_sportData MyData[2] ;
#endif

OXS_OUT_FRSKY::OXS_OUT_FRSKY(uint8_t pinTx,HardwareSerial &print)
{
  printer = &print; //operate on the address of print
}

// **************** Setup the FRSky OutputLib *********************
void OXS_OUT_FRSKY::setup()
{

#if defined(FRSKY_SPORT)
	MyData[0].next = &MyData[1] ;
	initSportUart( &MyData[0] ) ;
#else
	initHubUart() ;
#endif
	
#ifdef DEBUG
  printer->begin(115200);
  printer->print("FRSky Output Module: TX Pin=");
  printer->println(_pinTx);
  printer->println("FRSky Output Module: Setup!");
#endif
#if defined(FORCE_ABSOLUTE_ALT) && !defined(FRSKY_SPORT)
  SendAlt(1);  // send initial height
  SendValue(0x00,int16_t(1)); //>> overwrite alt offset in open 9x in order to start with display of absolute altitude... 
  SendValue(0x30,(int16_t)(varioData->absoluteAlt/100)); //>> overwrite min alt in open 9x
  SendValue(0x31,(int16_t)(varioData->absoluteAlt/100)); //>> overwrite min alt in open 9x
  sendHubByte(0x5E) ; // End of Frame 1!
#endif
}

#if defined(FRSKY_SPORT)

#define ALT_ID       0x0100
#define VARIO_ID     0x0110

void OXS_OUT_FRSKY::sendData()
{
	static uint8_t counter ;
  switch ( counter)
	{
    case 0 :  
			setNewData( &MyData[1], VARIO_ID, varioData->climbRate ) ;
      break;
     
    case 1 :
			setNewData( &MyData[0], ALT_ID, varioData->absoluteAlt ) ;
      break;
  }
  counter = (counter + 1) & 1 ;
}

#else
/****************************************************************/
/* sendData - Output data to the FrSky Receiver/Transceiver     */
/****************************************************************/
void OXS_OUT_FRSKY::sendData()
{
  static unsigned long lastMsFrame1=0;
  static unsigned long lastMsFrame2=0;

  if ( (millis()-lastMsFrame1) > INTERVAL_FRAME1  ) {
    static byte SwitchFrameVariant=0;
    lastMsFrame1=millis();
    if (SwitchFrameVariant==0)SendFrame1A();
    if (SwitchFrameVariant==1)SendFrame1B();
    SwitchFrameVariant++;
    if(SwitchFrameVariant==2)SwitchFrameVariant=0 ;
  }
  if ( (millis()-lastMsFrame2) > INTERVAL_FRAME2  ) {
    lastMsFrame2=millis();
    SendFrame2();
  }
}
//======================================================================================================Send Frame 1A via serial
void OXS_OUT_FRSKY::SendFrame1A(){
#ifdef DEBUG
  printer->print("FRSky output module: SendFrame1:");
#endif
  if (varioData!=NULL){
    if (varioData->available){ //========================================================================== Vario Data
#ifdef DEBUG
      printer->print("Sending vario data ");
#endif
      SendAlt(varioData->absoluteAlt);    

      // Attempt to work around the annoying 10cm double beep
      //SendValue(FRSKY_USERDATA_VERT_SPEED,(int16_t)varioData->climbRate); // ClimbRate in open9x Vario mode
      if (varioData->climbRate==10) SendValue(FRSKY_USERDATA_VERT_SPEED,(int16_t)9); // ClimbRate in open9x Vario mode
      else if (varioData->climbRate==-10) SendValue(FRSKY_USERDATA_VERT_SPEED,(int16_t)-9);
      else SendValue(FRSKY_USERDATA_VERT_SPEED,(int16_t)varioData->climbRate); // ClimbRate in open9x Vario mode

        // ********************************* The Temp 1 FIeld
#ifdef SEND_TEMP_T1      
      SendTemperature1(varioData->temperature/10); 
#endif

#ifdef SEND_PressureAsT1 // pressure in T1 Field
      SendTemperature1((varioData->pressure-SEND_PressureAsT1)/10); //pressure in T1 Field
#endif

      // ********************************* The Temp 2 FIeld
#ifdef SEND_TEMP_T2      
      SendTemperature2(varioData->temperature); 
#endif
#ifdef SEND_SensitivityAsT2 // Kalman Param R in Temp2
      SendTemperature2(uint16_t(varioData->paramKalman_r)); 
#endif
#ifdef SEND_Sensitivity2AsT2 // Kalman Param R in Temp2
      SendTemperature2(uint16_t(varioData->paramKalman_r2)); 
#endif
#ifdef SEND_PressureAsT2 // pressure in T2 Field
      SendTemperature2((varioData->pressure) /10 -SEND_PressureAsT2); //pressure in T2 Field reduced by offset
#endif
    }
    else {
#ifdef DEBUG
      printer->print("No vario data Available!");
#endif
    }// if (varioData->available){ 
  } //(varioData==NULL)
  if (currentData!=NULL){
    if (varioData->available){ //========================================================================== Current Data
#ifdef DEBUG
      printer->print("Sending current data:");      
      printer->println(currentData->milliAmps);
#endif
      SendCurrentMilliAmps(currentData->milliAmps);    
#ifdef SEND_MilliampsAsT1      
      SendTemperature1(currentData->milliAmps); 
#endif
#ifdef SEND_MilliampsAsT2      
      SendTemperature2(currentData->milliAmps); 
#endif
    } // if (currentData->available)
  } // if(currentData==NULL)
  // =====================================================================================================  Arduino Data
  if (arduinoData->available)
  {
#ifdef PIN_VOLTAGE_DIVIDER
    SendValue(FRSKY_USERDATA_VFAS_NEW,(int16_t)arduinoData->dividerMilliVolts/100); 
#else
    SendValue(FRSKY_USERDATA_VFAS_NEW,(int16_t)arduinoData->vrefMilliVolts/100); 
#endif
#ifdef SEND_LoopTimeAsT2      
    SendTemperature2(arduinoData->loopTimeMilliSeconds); 
#endif



  }
  sendHubByte(0x5E) ; // End of Frame 1!
}
//============================================================================================================== Send Frame 1B via serial
void OXS_OUT_FRSKY::SendFrame1B(){
#ifdef DEBUG
  printer->print("FRSky output module: SendFrame1:");
#endif
  if (varioData!=NULL){
    if (varioData->available){ //========================================================================== Vario Data
#ifdef DEBUG
      printer->print("Sending vario data B ");
#endif
      // ********************************* The DIST Field
#ifdef SEND_AltAsDIST     // send alt as adjusted to precision in dist field
      if (varioData->absoluteAlt- SEND_AltAsDIST <= 32768) SendGPSDist(uint16_t(varioData->absoluteAlt- SEND_AltAsDIST ));
      else if (varioData->absoluteAlt- SEND_AltAsDIST < 327680) SendGPSDist(uint16_t((varioData->absoluteAlt-SEND_AltAsDIST)/(long)10));
      // If altitude gets higher than 327,68m alt/10 will be transmitted till 3276,8m then alt/100
      else SendGPSDist(uint16_t((varioData->absoluteAlt-SEND_AltAsDIST)/(long)100));
#endif 
#ifdef SEND_PressureAsDIST
      SendGPSDist(uint16_t(varioData->pressure/10));
#endif
#ifdef SEND_SensitivityAsDist
      SendGPSDist(uint16_t(varioData->paramKalman_r));
#endif

      // ********************************** The Fuel Field
#ifdef SEND_SensitivityAsFuel
      SendFuel(uint16_t(varioData->paramKalman_r));
#endif
#ifdef SEND_PressureAsFuel
      SendFuel(uint16_t(varioData->pressure/100));
#endif
      // ********************************* The RPM Field   
#ifdef SEND_AltAsRPM     
      SendRPM(varioData->absoluteAlt);
#endif
#ifdef SEND_PressureAsRPM
      SendRPM(uint16_t(varioData->pressure/10));
#endif
#ifdef SEND_SensitivityAsRPM
      SendRPM(uint16_t(varioData->paramKalman_r));
#endif


      //******************************************* Min Max Alt
#ifdef SEND_MIN_MAX_ALT
      SendValue(0x31,int16_t(varioData->minRelAlt/100));   //minAltitude OK!
      SendValue(0x32,int16_t(varioData->maxRelAlt/100));   //maxAltitude OK!
      /*SendValue(0x33,uint16_t(5678));    //maxRPM OK
       SendValue(0x34,uint16_t(111));   //T1+ OK
       SendValue(0x35,int16_t(222));    //T2+ OK!
       SendValue(0x36,int16_t(1000));    //maxGpsSpeed; id ok, but 1000= 1840?? 
       SendValue(0x37,int16_t(7000));    //Dst+ OK!
       SendValue(0x39,int16_t(5000));    //FAS OK, documented!
       */
#endif
    }
    else {
#ifdef DEBUG
      printer->print("No vario data Available!");
#endif
    }// if (varioData->available){ 
  } //(varioData==NULL)
  if (currentData!=NULL){
    if (varioData->available){ //========================================================================== Current Data
#ifdef SEND_mAhAsDist
      SendGPSDist(uint16_t(currentData->consumedMilliAmps));
#endif
#ifdef SEND_mAhAsFuel
      SendFuel(uint16_t(currentData->consumedMilliAmps));
#endif
#ifdef SEND_mAhPercentageAsFuel
      SendFuel( uint16_t(  constrain(map(currentData->consumedMilliAmps,SEND_mAhPercentageAsFuel,0,0,100) ,0,100  ) ) );
#endif
#ifdef SEND_MAX_CURRENT
      SendValue(0x38,int16_t(currentData->maxMilliAmps));    //Cur+ OK!
#endif
    } // if (currentData->available)
  } // if(currentData==NULL)
#ifdef SEND_VRefAsFuel
  SendFuel( uint16_t(arduinoData->vrefMilliVolts) );
#endif
#ifdef SEND_DividerMilliVoltsAsFuel
  SendFuel( uint16_t(arduinoData->dividerMilliVolts) );
#endif
#ifdef SEND_VRefAsDist
  SendGPSDist( uint16_t(arduinoData->vrefMilliVolts) );
#endif
#ifdef SEND_DividerVoltageAsDist
  SendGPSDist( uint16_t(arduinoData->dividerMilliVolts) );
#endif
  sendHubByte(0x5E) ; // End of Frame 1!
}


// Send Frame 2 via serial
void OXS_OUT_FRSKY::SendFrame2(){
#ifdef DEBUG
  printer->print("FRSky Output Module: SendFrame2!");
  printer->print("mBar=");    
  printer->println( ( ((float)(int32_t)(varioData->pressure))) /100);
#endif
}

/**********************************************************/
/* SendValue => send a value as frsky sensor hub data     */
/**********************************************************/
void OXS_OUT_FRSKY::SendValue(uint8_t ID, uint16_t Value) {
  uint8_t tmp1 = Value & 0x00ff;
  uint8_t tmp2 = (Value & 0xff00)>>8;
  sendHubByte(0x5E) ;
  sendHubByte(ID);
  if(tmp1 == 0x5E) { 
    sendHubByte(0x5D);    
    sendHubByte(0x3E);  
  } 
  else if(tmp1 == 0x5D) {    
    sendHubByte(0x5D);    
    sendHubByte(0x3D);  
  } 
  else {    
    sendHubByte(tmp1);  
  }
  if(tmp2 == 0x5E) {    
    sendHubByte(0x5D);    
    sendHubByte(0x3E);  
  } 
  else if(tmp2 == 0x5D) {    
    sendHubByte(0x5D);    
    sendHubByte(0x3D);  
  } 
  else {    
    sendHubByte(tmp2);  
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

/************************************************************/
/* SendTemperature1/2 =>  tempc in 1/100th of degree celsius */
/* Display Range in openTX: -3276,8..32768=-3276,8C..+3276,C */
/************************************************************/
void OXS_OUT_FRSKY::SendTemperature1(int16_t tempc) {
  SendValue(FRSKY_USERDATA_TEMP1, tempc);
}
void OXS_OUT_FRSKY::SendTemperature2(int16_t tempc) {
  SendValue(FRSKY_USERDATA_TEMP2, tempc);
}
/*************************************/
/* SendRPM => Send Rounds Per Minute */
/*************************************/
void OXS_OUT_FRSKY::SendRPM(uint16_t rpm) {
  byte blades=2;
  rpm = uint16_t((float)rpm/(60/blades));  
  SendValue(FRSKY_USERDATA_RPM, rpm);
}

/*************************************/
/* SendFuel => SendFuel Level        */
/*************************************/
void OXS_OUT_FRSKY::SendFuel(uint16_t fuel) {
  SendValue(FRSKY_USERDATA_FUEL, fuel);
}

/**********************************/
/* SendAlt => Send ALtitude in cm */
/**********************************/
void OXS_OUT_FRSKY::SendAlt(long altcm)
{
  uint16_t Centimeter =  uint16_t(abs(altcm)%100);
  long Meter;
#ifdef FORCE_ABSOLUTE_ALT && !defined(FRSKY_SPORT)
  altcm-=1;
#endif

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
/* parameter: altitude in cm between -3276800 and 3276799       */
/* cm will not be displayed                                     */
/****************************************************************/
void OXS_OUT_FRSKY::SendGPSAlt(long altcm)
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

  // need to send a gps fix before openTX displays this field....
  SendValue(FRSKY_USERDATA_GPS_LONG_A, 1);
  SendValue(FRSKY_USERDATA_GPS_LONG_B, 1);
  SendValue(FRSKY_USERDATA_GPS_LAT_A, 1);
  SendValue(FRSKY_USERDATA_GPS_LAT_B, 1);
  // now send the data
  SendValue(FRSKY_USERDATA_GPS_ALT_B, Meter);
  SendValue(FRSKY_USERDATA_GPS_ALT_A, Centimeter);
}
/***********************************************/
/* SendCurrentMilliAmps => Send Current        */
/* current will be transmitted as 1/10th of A  */
/* Range: -32768..32767 => disp -327,6..3276,7 */
/***********************************************/
void OXS_OUT_FRSKY::SendCurrentMilliAmps(int32_t milliamps) 
{
#ifdef ForceAbsolutCurrent
  milliamps=abs(milliamps);
#endif 
  SendValue(FRSKY_USERDATA_CURRENT, (uint16_t)(milliamps/100));
}
#endif




