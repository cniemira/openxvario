#include <Wire.h>
#include "oxs_config.h"
#include "oxs_arduino.h"
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
OXS_MS5611 oxs_MS5611(I2CAdd,Serial,KALMAN_R);
#endif


#ifdef PIN_CurrentSensor
OXS_CURRENT oxs_Current(PIN_CurrentSensor,Serial);
#endif
OXS_OUT_FRSKY oxs_OutFrsky(PIN_SerialTX,Serial);
OXS_ARDUINO oxs_Arduino(Serial);
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
#ifdef PIN_VOLTAGE_DIVIDER
  oxs_Arduino.setupDivider(PIN_VOLTAGE_DIVIDER, RESISTOR_ANALOG_TO_GND,RESISTOR_ANALOG_TO_BATTERY);
#else
  oxs_Arduino.setupDivider(-1, 33000,56000);
#endif
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
  //delay(3);
  // Read all the sensors / Inputs
  oxs_Arduino.readSensor();
#ifdef VARIO
  oxs_MS5611.readSensor();
#endif
#ifdef PIN_CurrentSensor
  oxs_Current.readSensor(oxs_Arduino.arduinoData.vrefMilliVolts);
#endif


#ifdef PIN_PushButton
  // Check if a button has been pressed
  checkButton();
#endif


  if (millis()>LastOutputMs+100){ // invoke output routines every 50ms if there's something new to do.
#ifdef DEBUG
    /*Serial.print("loopMs=");      
    Serial.print(  float((millis()-LastOutputMs)) / loopcnt);    
    Serial.print(" ;");
    Serial.print("freeRam=");      
    Serial.println(freeRam());*/
#endif

    loopcnt=0; 
    LastOutputMs=millis();  

    // OutputModule go here
#ifdef DEBUG
    //OutputToSerial();
#endif
#ifdef VARIO
    oxs_OutFrsky.varioData=&oxs_MS5611.varioData;
#endif
#ifdef PIN_CurrentSensor
    oxs_OutFrsky.currentData=&oxs_Current.currentData;
#endif
    oxs_OutFrsky.arduinoData=&oxs_Arduino.arduinoData;
    oxs_OutFrsky.sendData();


    // PPM Processing
    // Read the ppm Signal from receiver
#ifdef PIN_PPM 
#ifdef PPM_ProgrammingMode 
    if (ppmProgMode)
#endif
#ifdef PPM_AllwaysUsed 
      if (true)
#endif
        ProcessPPMSignal();
#endif

  }//if (millis()>LastOutputMs+100)
#ifdef SAVE_TO_EEPROM
  static unsigned long LastEEPromMs=millis();
  if (millis()>LastEEPromMs+1000){ // Save Persistant Data To EEProm every 10 seconds
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
  oxs_MS5611.varioData.minAbsAlt=oxs_MS5611.varioData.absoluteAlt;
  oxs_MS5611.varioData.maxRelAlt=oxs_MS5611.varioData.relativeAlt;
  oxs_MS5611.varioData.minRelAlt=oxs_MS5611.varioData.relativeAlt;
  oxs_MS5611.varioData.minClimbRate=oxs_MS5611.varioData.climbRate;
  oxs_MS5611.varioData.maxClimbRate=oxs_MS5611.varioData.climbRate;
#endif
  oxs_Arduino.resetValues();
}
void Reset3SecButtonPress()
{
#ifdef DEBUG
  Serial.println("================================================== Reset 3-5 sec");
#endif
#ifdef PIN_CurrentSensor
  oxs_Current.currentData.consumedMilliAmps=0;
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
void OutputToSerial(){
#define DEBUGMINMAX
#ifdef FALSE
  Serial.print("vRef=");    
  Serial.print( (float)oxs_Arduino.arduinoData.vrefMilliVolts /1000);
#ifdef DEBUGMINMAX
  Serial.print("(");  
  Serial.print( ( (float)oxs_Arduino.arduinoData.minVrefMilliVolts)/1000);
  Serial.print("..");  
  Serial.print( ( (float)oxs_Arduino.arduinoData.maxVrefMilliVolts)/1000);
  Serial.print(")");
#endif
#endif

  Serial.print(" ;divV=");    
  Serial.print((float)oxs_Arduino.arduinoData.dividerMilliVolts /1000);
#ifdef DEBUGMINMAX
  Serial.print("(");  
  Serial.print( ( (float)oxs_Arduino.arduinoData.minDividerMilliVolts)/1000);
  Serial.print("..");  
  Serial.print( ( (float)oxs_Arduino.arduinoData.maxDividerMilliVolts)/1000);
  Serial.print(")");
#endif

#ifdef VARIO
  Serial.print(" ;mBar=");    
  Serial.print( ( ((float)(int32_t)(oxs_MS5611.varioData.pressure))) /100);
  Serial.print(";relAlt=");  
  Serial.print( ( (float)oxs_MS5611.varioData.relativeAlt)/100);
#ifdef DEBUGMINMAX
  Serial.print("(");  
  Serial.print( ( (float)oxs_MS5611.varioData.minRelAlt)/100);
  Serial.print("..");  
  Serial.print( ( (float)oxs_MS5611.varioData.maxRelAlt)/100);
  Serial.print(")");
#endif
  Serial.print(" ;absAlt=");  
  Serial.print( ( (float)oxs_MS5611.varioData.absoluteAlt)/100);
#ifdef DEBUGMINMAX
  Serial.print("(");  
  Serial.print( ( (float)oxs_MS5611.varioData.minAbsAlt)/100);
  Serial.print("..");  
  Serial.print( ( (float)oxs_MS5611.varioData.maxAbsAlt)/100);
  Serial.print(")");
#endif 

  Serial.print(" ;Vspd=");    
  Serial.print( ( (float)oxs_MS5611.varioData.climbRate)/100);
#ifdef DEBUGMINMAX
  Serial.print("(");    
  Serial.print( ( (float)oxs_MS5611.varioData.minClimbRate)/100);
  Serial.print("..");    
  Serial.print( ( (float)oxs_MS5611.varioData.maxClimbRate)/100);
  Serial.print(")");
#endif
#ifdef FALSE
  Serial.print(" ;Temp=");    
  Serial.print((float)oxs_MS5611.varioData.temperature/10);
#endif
#endif
#ifdef PIN_CurrentSensor
  Serial.print(" ;mA=");    
  Serial.print( ( ((float)(int32_t)(oxs_Current.currentData.milliAmps))) );
  Serial.print("(");    
#ifdef DEBUGMINMAX
  Serial.print(  ( ((float)(int32_t)(oxs_Current.currentData.minMilliAmps))) );
  Serial.print("..");    
  Serial.print( ( ((float)(int32_t)(oxs_Current.currentData.maxMilliAmps))) );
  Serial.print(")");
#endif
  Serial.print(" ;mAh=");  
  Serial.print( oxs_Current.currentData.consumedMilliAmps);
#endif
  Serial.println();
}
#endif
#ifdef SAVE_TO_EEPROM
/****************************************/
/* SaveToEEProm => save persistant Data */
/****************************************/
void SaveToEEProm(){
  static byte state=0;
  static int adr=0;
  Serial.print("++++++++++++ SAving to EEProm:");    
  Serial.println(state);    
  switch (state){
  case 0:
#ifdef PIN_CurrentSensor
    adr+=EEPROM_writeAnything(adr, oxs_Current.currentData.consumedMilliAmps);
#endif
  case 1:
#ifdef PIN_CurrentSensor
    adr+=EEPROM_writeAnything(adr, oxs_Current.currentData.maxMilliAmps);
#endif
  case 2:
#ifdef PIN_CurrentSensor
    adr+=EEPROM_writeAnything(adr, oxs_Current.currentData.minMilliAmps);
#endif
#ifdef VARIO
  case 3:

    adr+=EEPROM_writeAnything(adr, oxs_MS5611.varioData.maxRelAlt);

  case 4:

    adr+=EEPROM_writeAnything(adr, oxs_MS5611.varioData.minRelAlt);

  case 5:

    adr+=EEPROM_writeAnything(adr, oxs_MS5611.varioData.maxAbsAlt);

  case 6:

    adr+=EEPROM_writeAnything(adr, oxs_MS5611.varioData.minAbsAlt);

  case 7:

    adr+=EEPROM_writeAnything(adr, oxs_MS5611.varioData.maxClimbRate);

  case 8:

    adr+=EEPROM_writeAnything(adr, oxs_MS5611.varioData.minClimbRate);
#endif
  }

  state++;
  if(state >8){
    state=0;
    adr=0;
  }
}

/******************************************/
/* LoadFromEEProm => load persistant Data */
/******************************************/
void LoadFromEEProm(){
  int adr=0;

  // Store the last known value from the ppm signal to the eeprom
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
  adr+=EEPROM_readAnything(adr, oxs_MS5611.varioData.maxRelAlt);
  Serial.print(" maxRelAlt=");    
  Serial.print( oxs_MS5611.varioData.maxRelAlt);

  adr+=EEPROM_readAnything(adr, oxs_MS5611.varioData.minRelAlt);
  Serial.print(" minRelAlt=");    
  Serial.print( oxs_MS5611.varioData.minRelAlt);


  adr+=EEPROM_readAnything(adr, oxs_MS5611.varioData.maxAbsAlt);
  Serial.print(" maxAbsAlt=");    
  Serial.print( oxs_MS5611.varioData.maxAbsAlt);

  adr+=EEPROM_readAnything(adr, oxs_MS5611.varioData.minAbsAlt);
  Serial.print(" minAbsAlt=");    
  Serial.print( oxs_MS5611.varioData.minAbsAlt);

  adr+=EEPROM_readAnything(adr, oxs_MS5611.varioData.maxClimbRate);
  Serial.print(" minClimbRate=");    
  Serial.print( oxs_MS5611.varioData.minClimbRate);

  adr+=EEPROM_readAnything(adr, oxs_MS5611.varioData.minClimbRate);
  Serial.print(" maxClimbRate=");    
  Serial.print( oxs_MS5611.varioData.maxClimbRate);
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
/**********************************************************/
/* ProcessPPMSignal => read PPM signal from receiver and  */
/*   use its value to adjust sensitivity                  */
/**********************************************************/
#ifdef PIN_PPM

void ProcessPPMSignal(){
  static boolean SignalPresent= false;
  unsigned long ppm= ReadPPM();
  Serial.println(ppm);
#ifdef PPM_ProgrammingMode 
  static unsigned int ppm_min=980;
  static unsigned int ppm_max=2000;
#else
  static unsigned int ppm_min=PPM_Range_min;
  static unsigned int ppm_max=PPM_Range_max;
#endif

  if (ppm>0){
    SignalPresent=true;// Signal is currently present!
#ifdef PPM_ProgrammingMode 
    if (ppm<ppm_min)ppm_min=ppm;
    if (ppm>ppm_max)ppm_max=ppm;
#endif     
    //kalman_r=map(ppm, 981,1999,KALMAN_R_MIN,KALMAN_R_MAX);
#ifdef VARIO
    oxs_MS5611.varioData.paramKalman_r=map(ppm, ppm_min,ppm_max,KALMAN_R_MIN/10,KALMAN_R_MAX/10)*10; // map value and change stepping to 10
#endif
  }
}
#endif
/**********************************************************/
/* ReadPPM => read PPM signal from receiver               */
/*   pre-evaluate its value for validity                  */
/**********************************************************/

// ReadPPM - Read ppm signal and detect if there is no signal
#ifdef PIN_PPM
unsigned long ReadPPM(){
  unsigned long ppm= pulseIn(PIN_PPM, HIGH, 20000); // read the pulse length in micro seconds
#ifdef DEBUG
  //Serial.print("PPM=");Serial.println(ppm);
#endif
  if ((ppm>2500)or (ppm<500))ppm=0; // no signal!
  return ppm;
}
#endif


/***********************************************/
/* freeRam => Cook coffee and vaccuum the flat */
/***********************************************/
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}







