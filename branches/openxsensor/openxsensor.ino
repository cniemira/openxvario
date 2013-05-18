#include <Wire.h>
#include "oxs_config.h"
#include "oxs_current.h"
#include "oxs_ms5611.h"
#include "oxs_out_frsky.h"
#include <SoftwareSerial.h>

/***************************************************************************************/
/* Hardware settings=>Here you can change which pins will be used for which connection */
/*                    and the I2C address of the connected MS5611 module               */
/*                    Normaly you do not have to change any of these                   */
/***************************************************************************************/
#define PIN_SerialTX        4  // 4  the pin to transmit the serial data to the frsky telemetry enabled receiver
#define PIN_ClimbLed        13 // 13 the led used to indicate lift ( led lights up above +0.10m/s)
#define PIN_AnalogClimbRate 3  // 3 the pin used to optionally write the data to the frsky a1 or a2 pin (could be 3,5,6,9,10,11)
#define PIN_PPM 2              // default: 2 the pin to read the PPM Signal on coming from the receiver.           
                               // you can uncomment this line if you want to completly disable the remote control functionality
#define PIN_CurrentSensor   2  // the Analog pin the optional current Sensor is connected to 

#define DEBUG

OXS_MS5611 oxs_MS5611(I2CAdd,Serial);
OXS_CURRENT oxs_Current(PIN_CurrentSensor,Serial);
OXS_OUT_FRSKY oxs_OutFrsky(PIN_SerialTX,Serial);

void setup(){
  #ifdef DEBUG
    Serial.begin(115200);
  #endif
  
  oxs_MS5611.setup();
  //oxs_Current.setupMinMaxA(-13514,13514);
  oxs_Current.setupIdleMvA( 2500,185);
  oxs_OutFrsky.setup();

}



void loop(){

  static int loopcnt=0;
  static unsigned long LastOutputMs=millis();
  
  // Input Modules go Here if they should be read as fast as possible
  oxs_MS5611.readSensor();
  oxs_Current.readSensor();
  
  loopcnt+=1;
  if (millis()>LastOutputMs+1){ // invoke output routines every 50ms if there's something new to do.
    //Serial.print("loopMs=");  Serial.print(  (millis()-LastOutputMs) / loopcnt);Serial.print("  ;");
    loopcnt=0; LastOutputMs=millis();  
    
    // OutputModule go here
    #ifdef DEBUG
      //OutputToSerial(oxs_MS5611.varioData);
    #endif
    oxs_OutFrsky.varioData=oxs_MS5611.varioData;
    oxs_OutFrsky.sendData();
  }
}

void OutputToSerial(VARIODATA &varioData,CURRENTDATA &currentDData){
    Serial.print("mBar=");    Serial.print( ( ((float)(int32_t)(varioData.pressure))) /100);
    Serial.print(";relAlt=");  Serial.print( ( (float)varioData.relativeAlt)/100);
    
    Serial.print("  ;absAlt=");  Serial.print( ( (float)varioData.absoluteAlt)/100);
    Serial.print("(");  Serial.print( ( (float)varioData.minAbsAlt)/100);
    Serial.print("..");  Serial.print( ( (float)varioData.maxAbsAlt)/100);
    Serial.print(")");
    
    Serial.print("  ;Vspd=");    Serial.print( ( (float)varioData.climbRate)/100);
    Serial.print("(");    Serial.print( ( (float)varioData.minClimbRate)/100);
    Serial.print("..");    Serial.print( ( (float)varioData.maxClimbRate)/100);
    Serial.print(")");
   
    Serial.print("  ;Temperature=");    Serial.print((float)varioData.temperature/10);
    Serial.println();
    
    Serial.print("A=");    Serial.print( ( ((float)(int32_t)(currentDData.milliAmps))) /1000);
    Serial.print("mAh=");  Serial.print( currentDData.consumedMilliAmps);
    Serial.println();
    
 /*    int32_t milliAmps;          // in mA
  int32_t maxMilliAmps;       // in mA
  int32_t minMilliAmps;       // in mA
  int32_t consumedMilliAmps;  // in mA
  
  // config values should be set via method calls
  uint16_t idleMilliVolts;       // in mV
  int16_t milliVoltPerAmpere;// in mV
  int32_t MilliAmps0V;       // in mA theoretical value for calculation. do not need to match allowed sensor range
  int32_t MilliAmps5V;       // in mA theoretical value for calculation. do not need to match allowed sensor range
*/
}
