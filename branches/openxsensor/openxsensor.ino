//#include <Wire.h>
#include "oxs_config.h"
#include "oxs_arduino.h"
#include "oxs_ms5611.h"
#include "oxs_curr.h"
#include "oxs_out_frsky.h"

#ifdef SAVE_TO_EEPROM
#include <EEPROM.h>
#include "EEPROMAnything.h"
#endif

#include "Aserial.h"

extern unsigned long micros( void ) ;
extern unsigned long millis( void ) ;

// #define DEBUG
#define VARIO

// Create instances of the used classes
#ifdef VARIO
#ifdef DEBUG  
OXS_MS5611 oxs_MS5611(I2CAdd,Serial,KALMAN_R);
#else
OXS_MS5611 oxs_MS5611(I2CAdd,KALMAN_R);
#endif
#endif

#ifdef PIN_CurrentSensor
#ifdef DEBUG  
OXS_CURRENT oxs_Current(PIN_CurrentSensor,Serial);
#else
OXS_CURRENT oxs_Current(PIN_CurrentSensor);
#endif
#endif

#ifdef DEBUG  
OXS_OUT_FRSKY oxs_OutFrsky(PIN_SerialTX,Serial);
#else
OXS_OUT_FRSKY oxs_OutFrsky(PIN_SerialTX);
#endif

#ifdef DEBUG  
OXS_ARDUINO oxs_Arduino(Serial);
#else
OXS_ARDUINO oxs_Arduino(0);
#endif

#define FORCE_INDIRECT(ptr) __asm__ __volatile__ ("" : "=e" (ptr) : "0" (ptr))


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
#endif
  oxs_OutFrsky.setup();

#ifdef SAVE_TO_EEPROM
  LoadFromEEProm();
#endif

#ifdef PPM_INTERRUPT
	PORTD |= PPM_PIN_HEX ;	// Pullup resistor
	DDRD &= ~PPM_PIN_HEX ;	// Input
	EICRA |= PPM_INT_MASK ;		// Interrupt on rising edge
	EIFR = PPM_INT_BIT ;			// Clear interrupt flag
	EIMSK |= PPM_INT_BIT ;		// Enable interrupt
#endif

}

void readSensors()
{
  // Read all the sensors / Inputs
#ifdef PIN_CurrentSensor
  oxs_Arduino.readSensor();
#endif
#ifdef VARIO
  oxs_MS5611.readSensor();
#endif
#ifdef PIN_CurrentSensor
  oxs_Current.readSensor(oxs_Arduino.arduinoData.vrefMilliVolts);
#endif
}

#if defined(FRSKY_SPORT)
  #define TIME_TO_SEND() (1)
#else
  #define TIME_TO_SEND() (millis()>LastOutputMs+100)
#endif
//************************************************************************************************** Loop()
void loop(){
  static int loopcnt=0;
  static unsigned long LastOutputMs=millis();
  loopcnt+=1;
  
//  readSensors();
//extern uint32_t newMillis( void ) ;
//	millis() ;
  
#ifdef PIN_PushButton
  // Check if a button has been pressed
  checkButton();
#endif

  if (TIME_TO_SEND()) { // invoke output routines every 50ms if there's something new to do.
    loopcnt=0; 
#if defined(FRSKY_SPORT)
#ifdef MEASURE_RPM
	if ( millis() > LastOutputMs+2000 )
	{
		LastOutputMs += 2000 ;
extern uint8_t RpmSet ;
extern uint16_t Rpm ;
		if ( RpmSet == 0 )
		{
			Rpm = 0 ;			
		}
		RpmSet = 0 ;		 
	}
#endif // MEASURE_RPM
#else
    LastOutputMs=millis();  
#endif
    // OutputModule go here
#ifdef DEBUG
    OutputToSerial();
#endif

#ifdef VARIO
    oxs_OutFrsky.varioData=&oxs_MS5611.varioData;
#endif

#ifdef PIN_CurrentSensor
    oxs_OutFrsky.currentData=&oxs_Current.currentData;
#endif

    oxs_OutFrsky.arduinoData=&oxs_Arduino.arduinoData;

#if defined(FRSKY_SPORT)
    if (SportSync)
		{
			SportSync = 0 ;
      readSensors();
    }
    if (DataSent)
		{
	 		DataSent = 0 ;
      oxs_OutFrsky.sendData();
		}
#else
    oxs_OutFrsky.sendData();
#endif

    // PPM Processing
    // Read the ppm Signal from receiver
#ifdef PIN_PPM 
    if (true)
      ProcessPPMSignal();
#endif //PIN_PPM 

  }//if (millis()>LastOutputMs+100)
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
  static unsigned int buttonDownMs;
  //read the pushbutton value into a variable
  int sensorVal = digitalRead(PIN_PushButton);
  if (sensorVal == HIGH) {
    // button not pressed released
    digitalWrite(PIN_LED, LOW);
  }
  else {
    //button is currently being pressed down
    unsigned int buttonPressDuration=millis()-buttonDownMs;

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
    unsigned int buttonPressDuration=millis()-buttonDownMs;
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
	struct CURRENTDATA *cd = &oxs_Current.currentData ;
	FORCE_INDIRECT(cd) ;
  cd->maxMilliAmps=cd->milliAmps;
  cd->minMilliAmps=cd->milliAmps;
#endif
#ifdef VARIO
	struct VARIODATA *vd = &oxs_MS5611.varioData ;
	FORCE_INDIRECT(vd) ;
  vd->maxAbsAlt=vd->absoluteAlt;
  vd->minAbsAlt=vd->absoluteAlt;
  vd->maxRelAlt=vd->relativeAlt;
  vd->minRelAlt=vd->relativeAlt;
  vd->minClimbRate=vd->climbRate;
  vd->maxClimbRate=vd->climbRate;
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



#ifdef SAVE_TO_EEPROM
/****************************************/
/* SaveToEEProm => save persistant Data */
/****************************************/
void SaveToEEProm(){
  static byte state=0;
  static int adr=0;
#ifdef DEBUG
  Serial.print("++++++++++++ SAving to EEProm:");    
  Serial.println(state);    
#endif
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
#endif //VARIO
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
#endif //VARIO
}
#endif //saveToEeprom


/**********************************************************/
/* ProcessPPMSignal => read PPM signal from receiver and  */
/*   use its value to adjust sensitivity                  */
/**********************************************************/
#ifdef PIN_PPM
void ProcessPPMSignal(){
  static boolean SignalPresent= false;
  unsigned int ppm= ReadPPM();
#ifdef DEBUG
  Serial.print("ppm=");
  Serial.println(ppm);
#endif
  static unsigned int ppm_min=PPM_Range_min;
  static unsigned int ppm_max=PPM_Range_max;
  if (ppm>0){
    SignalPresent=true;// Signal is currently present!
#ifdef VARIO
    oxs_MS5611.varioData.paramKalman_r=map(ppm, ppm_min,ppm_max,KALMAN_R_MIN/10,KALMAN_R_MAX/10)*10; // map value and change stepping to 10
#endif
  }
}


/**********************************************************/
/* ReadPPM => read PPM signal from receiver               */
/*   pre-evaluate its value for validity                  */
/**********************************************************/

#ifdef PPM_INTERRUPT
uint16_t StartTime ;
uint16_t EndTime ;
uint8_t PulseTime ;		// A byte to avoid 

#if PIN_PPM == 2
ISR(INT0_vect, ISR_NOBLOCK)
#else
ISR(INT1_vect, ISR_NOBLOCK)
#endif
{
	cli() ;
	uint16_t time = TCNT1 ;	// Read timer 1
	sei() ;
	if ( EICRA & PPM_INT_EDGE )
	{
		StartTime = time ;
		EICRA &= ~PPM_INT_EDGE ;				// falling edge
	}
	else
	{
		EndTime = time ;		
		time -= StartTime ;
		time >>= 7 ;		// Nominal 125 to 250
		time -= 60 ;		// Nominal 65 to 190
		if ( time > 255 )
		{
			time = 255 ;			
		}
		PulseTime = time ;
		EICRA |= PPM_INT_EDGE ;				// Rising edge
	}
}

unsigned int ReadPPM()
{
	unsigned int ppm = PulseTime ;
	ppm += 60 ;
	ppm <<= 3 ;	// To microseconds
	if ((ppm>2500)or (ppm<500)) ppm=0 ; // no signal!
  
	return ppm ;
}

// Removed as will no longer work
//#else
//// ReadPPM - Read ppm signal and detect if there is no signal

//unsigned int ReadPPM(){
//  unsigned int ppm= pulseIn(PIN_PPM, HIGH, 20000); // read the pulse length in micro seconds
//#ifdef DEBUG
//  //Serial.print("PPM=");Serial.println(ppm);
//#endif
//  if ((ppm>2500)or (ppm<500))ppm=0; // no signal!
//  return ppm;
//}

#endif


#endif //PIN_PPM



#ifdef DEBUG

void OutputToSerial(){
#define DEBUGMINMAX
#define DEBUGVREF
#define DEBUGDIVIDER
#define DEBUGTEMP
  Serial.print(" K=");    
  Serial.print( oxs_MS5611.varioData.paramKalman_k,DEC);

#ifdef DEBUGVREF
  Serial.print("vRef=");    
  Serial.print( (float)oxs_Arduino.arduinoData.vrefMilliVolts /1000);
#ifdef DEBUGMINMAX
  Serial.print("(");  
  Serial.print( ( (float)oxs_Arduino.arduinoData.minVrefMilliVolts)/1000);
  Serial.print("..");  
  Serial.print( ( (float)oxs_Arduino.arduinoData.maxVrefMilliVolts)/1000);
  Serial.print(")");
#endif //DEBUGMINMAX
#endif //DEBUGVREF

#ifdef DEBUGDIVIDER
  Serial.print(" ;divV=");    
  Serial.print((float)oxs_Arduino.arduinoData.dividerMilliVolts /1000);
#ifdef DEBUGMINMAX
  Serial.print("(");  
  Serial.print( ( (float)oxs_Arduino.arduinoData.minDividerMilliVolts)/1000);
  Serial.print("..");  
  Serial.print( ( (float)oxs_Arduino.arduinoData.maxDividerMilliVolts)/1000);
  Serial.print(")");
#endif //DEBUGMINMAX
#endif //DEBUGDIVIDER

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
#endif //DEBUGMINMAX
  Serial.print(" ;absAlt=");  
  Serial.print( ( (float)oxs_MS5611.varioData.absoluteAlt)/100);
#ifdef DEBUGMINMAX
  Serial.print("(");  
  Serial.print( ( (float)oxs_MS5611.varioData.minAbsAlt)/100);
  Serial.print("..");  
  Serial.print( ( (float)oxs_MS5611.varioData.maxAbsAlt)/100);
  Serial.print(")");
#endif  //DEBUGMINMAX

  Serial.print(" ;Vspd=");    
  Serial.print( ( (float)oxs_MS5611.varioData.climbRate)/100);
#ifdef DEBUGMINMAX
  Serial.print("(");    
  Serial.print( ( (float)oxs_MS5611.varioData.minClimbRate)/100);
  Serial.print("..");    
  Serial.print( ( (float)oxs_MS5611.varioData.maxClimbRate)/100);
  Serial.print(")");
#endif //DEBUGMINMAX
#ifdef DEBUGTEMP
  Serial.print(" ;Temp=");    
  Serial.print((float)oxs_MS5611.varioData.temperature/10);
#endif //DEBUGTEMP
#endif // VARIO

#ifdef PIN_CurrentSensor
  Serial.print(" ;mA=");    
  Serial.print( ( ((float)(int32_t)(oxs_Current.currentData.milliAmps))) );
  Serial.print("(");    
#ifdef DEBUGMINMAX
  Serial.print(  ( ((float)(int32_t)(oxs_Current.currentData.minMilliAmps))) );
  Serial.print("..");    
  Serial.print( ( ((float)(int32_t)(oxs_Current.currentData.maxMilliAmps))) );
  Serial.print(")");
#endif //DEBUGMINMAX
  Serial.print(" ;mAh=");  
  Serial.print( oxs_Current.currentData.consumedMilliAmps);
#endif // PIN_CurrentSensor
  Serial.println();
}

/***********************************************/
/* freeRam => Cook coffee and vaccuum the flat */
/***********************************************/
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

#endif	// DEBUG


