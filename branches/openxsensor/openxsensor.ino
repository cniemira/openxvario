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

// Create instances of the used classes
OXS_MS5611 oxs_MS5611(I2CAdd,Serial);
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
  oxs_MS5611.setup();
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
  oxs_MS5611.readSensor();
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
    oxs_OutFrsky.varioData=&oxs_MS5611.varioData;
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
    if( (buttonPressDuration>=1000) and (buttonPressDuration<3000) )
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
  Serial.println("Reset 1-3 sec");
#endif
}
void Reset3SecButtonPress()
{
#ifdef DEBUG
  Serial.println("Reset 3-5 sec");
#endif
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
  // Store the last known value from the ppm signal to the eeprom
  //eepromIntWrite(0,111); // just for testing purposes...

  EEPROM_writeAnything(0, oxs_Current.currentData.consumedMilliAmps);
  Serial.print(" Saved To EEProm!!!mAh=");  
  Serial.print( oxs_Current.currentData.consumedMilliAmps);
  //delay (500);
}

/******************************************/
/* LoadFromEEProm => load persistant Data */
/******************************************/
void LoadFromEEProm(){
  // Store the last known value from the ppm signal to the eeprom
  //eepromIntWrite(0,111); // just for testing purposes...
  EEPROM_readAnything(0, oxs_Current.currentData.consumedMilliAmps);
  Serial.print(" Restored from EEProm!!!mAh=");  
  Serial.print( oxs_Current.currentData.consumedMilliAmps);
  delay (5000);
}

/*
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


