#include <Wire.h>
#include "oxs_config.h"
#include "oxs_ms5611.h"
#include "oxs_curr.h"
#include "oxs_out_frsky.h"
#include <SoftwareSerial.h>

#ifdef SAVE_TO_EEPROM
#include <EEPROM.h>
#include "EEPROMAnything.h"
#endif

#define DEBUG

#define VARIO

// Create instances of the used classes
#ifdef VARIO
OXS_MS5611 oxs_MS5611(I2CAdd,Serial);
#endif
#ifdef PIN_CurrentSensor
OXS_CURRENT oxs_Current(PIN_CurrentSensor,Serial);
#endif
OXS_OUT_FRSKY oxs_OutFrsky(PIN_SerialTX,Serial);

//************************************************************************************************** Setup()
void setup(){
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("openXsensor starting..");
  Serial.print("freeRam=");  
  Serial.println(freeRam());
#endif 

#ifdef PIN_PushButton  
  pinMode(PIN_PushButton, INPUT_PULLUP);
#endif

  pinMode(PIN_LED, OUTPUT); // The signal LED

  // Invoke all setup methods 
#ifdef VARIO
  oxs_MS5611.setup();
#endif
#ifdef PIN_CurrentSensor
  oxs_Current.setupIdleMvA( IdleMillivolts,MillivoltsPerAmpere);
  //  oxs_Current.setupMinMaxA(-13469,13559);
#endif

  oxs_OutFrsky.setup();
#ifdef SAVE_TO_EEPROM
  LoadFromEEProm();
#endif
}


//************************************************************************************************** Loop()
void loop(){
  static int loopcnt=0;
  static unsigned long LastOutputMs=millis();
  loopcnt+=1;

  // Read all the sensors / Inputs
#ifdef VARIO
  oxs_MS5611.readSensor();
#endif
#ifdef PIN_CurrentSensor
  oxs_Current.readSensor();
#endif

#ifdef PIN_PushButton
  // Check if a button has been pressed
  checkButton();
#endif

  if (millis()>LastOutputMs+100){ // invoke output routines every 50ms if there's something new to do.
#ifdef DEBUG
    Serial.print("loopMs=");  
    Serial.print(  (millis()-LastOutputMs) / loopcnt);
    Serial.print("  ;");
    Serial.print("freeRam=");  
    Serial.println(freeRam());
#endif

    loopcnt=0; 
    LastOutputMs=millis();  

    // OutputModule go here
#ifdef DEBUG
    OutputToSerial(oxs_MS5611.varioData,oxs_Current.currentData);
#endif
#ifdef VARIO
    oxs_OutFrsky.varioData=&oxs_MS5611.varioData;
#endif
#ifdef PIN_CurrentSensor
    oxs_OutFrsky.currentData=&oxs_Current.currentData;
#endif
    oxs_OutFrsky.sendData();
  }

#ifdef SAVE_TO_EEPROM
  static unsigned long LastEEPromMs=millis();
  if (millis()>LastEEPromMs+10000){ // Save Persistant Data To EEProm every 10 seconds
    LastEEPromMs=millis();
    SaveToEEProm();
  }
#endif
}

#ifdef PIN_PushButton
void checkButton()
{
  static int lastSensorVal=HIGH;
  static unsigned long buttonDownMs;
  //read the pushbutton value into a variable
  int sensorVal = digitalRead(PIN_PushButton);
  if (sensorVal == HIGH) {
    // button not pressed released
    digitalWrite(PIN_LED, LOW);
  }
  else {
    //button is currently being pressed down
    unsigned long buttonPressDuration=millis()-buttonDownMs;

    if( (buttonPressDuration>1000) and (buttonPressDuration<1200) )
      digitalWrite(PIN_LED, LOW); // Blink after 1 second

    else if( (buttonPressDuration>3000) and (buttonPressDuration<3200) )
      digitalWrite(PIN_LED, LOW); // Blink 1 after 3 second
    else if( (buttonPressDuration>3300) and (buttonPressDuration<3400) )
      digitalWrite(PIN_LED, LOW); // Blink 2 after 3 second

    else if( (buttonPressDuration>10000) and (buttonPressDuration<10200) )
      digitalWrite(PIN_LED, LOW); // Blink after 10 second
    else if( (buttonPressDuration>10300) and (buttonPressDuration<10400) )
      digitalWrite(PIN_LED, LOW); // Blink after 10 second
    else if( (buttonPressDuration>10500) and (buttonPressDuration<10600) )
      digitalWrite(PIN_LED, LOW); // Blink after 10 second
    else digitalWrite(PIN_LED, HIGH);
  }
  if( (lastSensorVal==HIGH) && (sensorVal==LOW))
  { // Button has been pressed down
    buttonDownMs=millis();
  }
  if( (lastSensorVal==LOW) && (sensorVal==HIGH))
  { // Button has been released
    unsigned long buttonPressDuration=millis()-buttonDownMs;
    //Serial.print("Button Released after ms:");
    //Serial.println(buttonPressDuration);
    // Do Something after certain times the button has been pressed for....
    if( (buttonPressDuration>=100) and (buttonPressDuration<3000) )
      Reset1SecButtonPress();
    else if( (buttonPressDuration>=3000) and (buttonPressDuration<5000) )
      Reset3SecButtonPress();
    //else if( (buttonPressDuration>=5000) and (buttonPressDuration<10000) )
    else if( (buttonPressDuration>=10000) and (buttonPressDuration<15000) )
      Reset10SecButtonPress();
  }

  lastSensorVal=sensorVal;
}
//ToDo: implement different reset actiuons on button press
void Reset1SecButtonPress()
{
#ifdef DEBUG
  Serial.println("================================================== Reset 0.1-3 sec");
#endif

#ifdef PIN_CurrentSensor
  oxs_Current.currentData.maxMilliAmps=oxs_Current.currentData.milliAmps;
  oxs_Current.currentData.minMilliAmps=oxs_Current.currentData.milliAmps;
#endif
#ifdef VARIO
  oxs_MS5611.varioData.maxAbsAlt=oxs_MS5611.varioData.absoluteAlt;
  oxs_MS5611.varioData.minClimbRate=oxs_MS5611.varioData.climbRate;
  oxs_MS5611.varioData.maxClimbRate=oxs_MS5611.varioData.climbRate;
#endif
}
void Reset3SecButtonPress()
{
#ifdef DEBUG
  Serial.println("================================================== Reset 3-5 sec");
#endif
  oxs_Current.currentData.consumedMilliAmps=0;

}

void Reset10SecButtonPress()
{
#ifdef DEBUG
  Serial.println("Reset 5-10");
#endif
}

#endif 

#ifdef DEBUG
void OutputToSerial(VARIODATA &varioData,CURRENTDATA &currentDData){
#ifdef VARIO
  Serial.print("mBar=");    
  Serial.print( ( ((float)(int32_t)(varioData.pressure))) /100);
  Serial.print(";relAlt=");  
  Serial.print( ( (float)varioData.relativeAlt)/100);

  Serial.print("  ;absAlt=");  
  Serial.print( ( (float)varioData.absoluteAlt)/100);
  Serial.print("(");  
  Serial.print( ( (float)varioData.minAbsAlt)/100);
  Serial.print("..");  
  Serial.print( ( (float)varioData.maxAbsAlt)/100);
  Serial.print(")");

  Serial.print("  ;Vspd=");    
  Serial.print( ( (float)varioData.climbRate)/100);
  Serial.print("(");    
  Serial.print( ( (float)varioData.minClimbRate)/100);
  Serial.print("..");    
  Serial.print( ( (float)varioData.maxClimbRate)/100);
  Serial.print(")");

  Serial.print("  ;Temp=");    
  Serial.print((float)varioData.temperature/10);
#endif
#ifdef PIN_CurrentSensor
  Serial.print("  ;mA=");    
  Serial.print( ( ((float)(int32_t)(currentDData.milliAmps))) );
  Serial.print("(");    
  Serial.print(  ( ((float)(int32_t)(currentDData.minMilliAmps))) );
  Serial.print("..");    
  Serial.print( ( ((float)(int32_t)(currentDData.maxMilliAmps))) );
  Serial.print(")");
  Serial.print("  ;mAh=");  
  Serial.print( currentDData.consumedMilliAmps);
#endif
  Serial.println();
}
#endif
#ifdef SAVE_TO_EEPROM
/****************************************/
/* SaveToEEProm => save persistant Data */
/****************************************/
void SaveToEEProm(){
  int adr=0;
  // Store the last known value from the ppm signal to the eeprom
  //eepromIntWrite(0,111); // just for testing purposes...
#ifdef PIN_CurrentSensor
  adr+=EEPROM_writeAnything(adr, oxs_Current.currentData.consumedMilliAmps);
  Serial.print(" Saved To EEProm!!!mAh=");    
  Serial.print( oxs_Current.currentData.consumedMilliAmps);

  adr+=EEPROM_writeAnything(adr, oxs_Current.currentData.maxMilliAmps);
  Serial.print(" Saved To EEProm!!!maxMilliAmps=");    
  Serial.print( oxs_Current.currentData.maxMilliAmps);

  adr+=EEPROM_writeAnything(adr, oxs_Current.currentData.minMilliAmps);
  Serial.print(" Saved To EEProm!!!minMilliAmps=");    
  Serial.print( oxs_Current.currentData.minMilliAmps);
#endif

#ifdef VARIO
  adr+=EEPROM_writeAnything(adr, oxs_MS5611.varioData.maxAbsAlt);
  Serial.print(" Saved To EEProm!!!maxAbsAlt=");    
  Serial.print( oxs_MS5611.varioData.maxAbsAlt);

  adr+=EEPROM_writeAnything(adr, oxs_MS5611.varioData.minAbsAlt);
  Serial.print(" Saved To EEProm!!!minAbsAlt=");    
  Serial.print( oxs_MS5611.varioData.minAbsAlt);

  adr+=EEPROM_writeAnything(adr, oxs_MS5611.varioData.minClimbRate);
  Serial.print(" Saved To EEProm!!!minClimbRate=");    
  Serial.print( oxs_MS5611.varioData.minClimbRate);

  adr+=EEPROM_writeAnything(adr, oxs_MS5611.varioData.maxClimbRate);
  Serial.print(" Saved To EEProm!!!maxClimbRate=");    
  Serial.print( oxs_MS5611.varioData.maxClimbRate);
#endif  
  /*adr+=EEPROM_writeAnything(adr, oxs_MS5611.varioData.paramKalman_r);
   Serial.print(" Saved To EEProm!!!paramKalman_r=");    Serial.print( oxs_MS5611.varioData.paramKalman_r);
   */

  //delay (500);
}

/******************************************/
/* LoadFromEEProm => load persistant Data */
/******************************************/
void LoadFromEEProm(){
  int adr=0;

  // Store the last known value from the ppm signal to the eeprom
  //eepromIntWrite(0,111); // just for testing purposes...
  Serial.println("-------------- Restored from EEProm: ---------------");
#ifdef PIN_CurrentSensor
  adr+=EEPROM_readAnything(adr, oxs_Current.currentData.consumedMilliAmps);
  Serial.print(" mAh=");    
  Serial.print( oxs_Current.currentData.consumedMilliAmps);

  adr+=EEPROM_readAnything(adr, oxs_Current.currentData.maxMilliAmps);
  Serial.print(" maxMilliAmps=");    
  Serial.print( oxs_Current.currentData.maxMilliAmps);

  adr+=EEPROM_readAnything(adr, oxs_Current.currentData.minMilliAmps);
  Serial.print(" minMilliAmps=");    
  Serial.print( oxs_Current.currentData.minMilliAmps);
#endif
#ifdef VARIO
  adr+=EEPROM_readAnything(adr, oxs_MS5611.varioData.maxAbsAlt);
  Serial.print(" maxAbsAlt=");    
  Serial.print( oxs_MS5611.varioData.maxAbsAlt);

  adr+=EEPROM_readAnything(adr, oxs_MS5611.varioData.minAbsAlt);
  Serial.print(" minAbsAlt=");    
  Serial.print( oxs_MS5611.varioData.minAbsAlt);

  adr+=EEPROM_readAnything(adr, oxs_MS5611.varioData.minClimbRate);
  Serial.print(" minClimbRate=");    
  Serial.print( oxs_MS5611.varioData.minClimbRate);

  adr+=EEPROM_readAnything(adr, oxs_MS5611.varioData.maxClimbRate);
  Serial.print(" maxClimbRate=");    
  Serial.print( oxs_MS5611.varioData.maxClimbRate);

  /*  adr+=EEPROM_readAnything(adr, oxs_MS5611.varioData.paramKalman_r);
   Serial.print(" paramKalman_r=");    Serial.print( oxs_MS5611.varioData.paramKalman_r);
   */
#endif
}

/*
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
 
 
 int32_t milliAmps;          // in mA
 float consumedMilliAmps;  // in mA
 
 // config values should be set via method calls
 int32_t maxMilliAmps;       // in mA
 int32_t minMilliAmps;       // in mA
 uint16_t idleMilliVolts;       // in mV
 int16_t milliVoltPerAmpere;// in mV
 int32_t milliAmps0V;       // in mA theoretical value for calculation. do not need to match allowed sensor range
 int32_t milliAmps5V;       // in mA theoretical value for calculation. do not need to match allowed sensor range
 
 */
#endif //saveToEeprom

/***********************************************/
/* freeRam => Cook coffee and vaccuum the flat */
/***********************************************/
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



