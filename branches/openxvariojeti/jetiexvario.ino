// openxvario http://code.google.com/p/openxvario/
// started by R.Schloßhan 

//********************** Jeti-Duplex-EX Mod by H.Stoecklein *************************
//* JETIDUPLEX Version 9.3.2013   (based on openXvario version r123 24.2.2013)
//* Bei aktivem define JETIDUPLEX wird zusätzlich an der seriellen HW-Schnittstelle   
//* ein Jeti-Duplex EX-kompatibler 9600Bd-Datenstrom ausgesendet, der die digitalen
//* Sensorkanäle Spannung, Temperatur, Relativhöhe und Steigrate enthält, ausserdem
//* ein Classic-Jetibox-Textframe mit VSPEAK-kompatibler Ascii-Anzeige derselben Daten.
//* Jetibox-Tastaturabfrage / Menüsteuerung sind aktuell (noch) nicht unterstützt.
//*
//* HINWEISE:
//*  - HW-UART hängt via 100 Ohm an Jeti-EXT, kann nicht mehr für DEBUG genutzt werden
//*  - Als Library ist JETIBOX.INO erforderlich (unverändert), suche nach folgendem:
//*      Based on "Jeti Box Communication for Arduino Mini Pro 328 16MHz" by Alexander Buschek
//*      Modified and stripped down to minimal functions by sceak
//***********************************************************************************

#define JETIDUPLEX                              // Jeti Duplex EX-Protokoll
#define DUOLED                                  // DuoLED auf dem myVarioEX-Board

//* Sensor-IDs für Jeti Duplex EX
#define MANUFAC_ID1 0xB7			// Manufacturer
#define MANUFAC_ID2 0x55
#define DEVICE_ID1 0xA8  			// Device
#define DEVICE_ID2 0x55

/***************************************************************************************/
/* Remote controlling the sensitivity using a servo port on your receiver              */
/* ==> choose 0 to 1 option! You need to have the PIN_PPM defined as well              */
/***************************************************************************************/
//#define PPM_AllwaysUsed  // PPM Signal always be used to control the sensitivity
// Choose this if you want to use a dedicated channel just for 
// adjusting the sensitivity

/***************************************************************************************/
/* Other Configuration Options  => various different values to take influence on       */
/*                    various stuff. Normaly you do not have to change any of these    */
/***************************************************************************************/
// Choose if the led should be used for indicationg lift.
#define LED_ClimbBlink 10  // Blink when climbRate > 0.05cm. The numeric value can be changed to a different cm value

#define FORCE_ABSOLUTE_ALT // If defined, the height offset in open9x will be resetted upon startup, which results 
// in an absolute height display in open9x . (You can still change to a relative display 
// by pressing [MENU] in the telem.screens
// If not defined, open9x will use the first transmitted altitude as an internal offset, 
// which results in an initial height of 0m

/***************************************************************************************/
/* Which data to send in which telemetry field                                         */
/* ==> choose freely one value per Telemetry field                                     */
/***************************************************************************************/
//**************** the DIST field (GPS Distance) (choose only one)**********************/
#define SEND_AltAsDIST 0   // 0 Altitude in DIST the numeric value (in cm) is an offset that will
// be subtracted from the actual height for higher display precision.
// e.g: Actual height is 456,78 Meters ( DIsplay in DIst would be 456 7)
// if we subtract 300 display will be 156 78
// This is due to the fact that the highest precission in this field
//  will only be transmitted up to an altitude of 327.68 m
//#define SEND_SensitivityAsDist // sensitivity in DIST
//#define SEND_PressureAsDIST    // pressure in DIST field

//************************** the T1 (temperature 1) Field? (choose only one) **********/
#define SEND_TEMP_T1         // MS5611 temperature as Temp1
//#define SEND_PressureAsT1 9000 // pressure in 1/10th of mBar in T1 Field subtracted by
// the number in the define statment. 
// e.g. 950mbar => 9500 -offset of 9000 => 500 in display

//************************** the T2 (temperature 2) Field? (choose only one) **********/
//#define SEND_TEMP_T2    // MS5611 temperature as Temp2
#define SEND_SensitivityAsT2  // Kalman Param R in Temp2

//************************** the RPM Field? (choose only one) *************************/
//(unprecise field! resolution is in steps of 30RPM!)
//#define SEND_AltAsRPM      // Altitude in RPM ;-)
//#define SEND_SensitivityAsRPM  // Kalman Param R in RPM
//#define SEND_PressureAsRPM // pressure in RPM Field


/***************************************************************************************/
/* The standard data being send to the trasnmitter                                     */
/***************************************************************************************/
#define SEND_Alt        // Send alt in the altitude field
#define SEND_VERT_SPEED // Send vertical speed (climbrate) as id 0x38
// #define SEND_VREF    // Voltage Reference as cell0
#define SEND_VFAS_NEW   // Voltage as VFAS 

/***************************************************************************************/
/* Analog IN Pins  => 6 free optional input to be transfered in the cell 1 to cell 6   */
/*                    fields. uncomment any line to enable its transmission            */
/***************************************************************************************/
// Define a voltagepin if you want to transmit the value to open9x:
//#define PIN_VoltageCell1 0    //  Pin for measuring Voltage of cell 1 ( Analog In Pin! )
//#define PIN_VoltageCell2 1    //  Pin for measuring Voltage of cell 2 ( Analog In Pin! )
//#define PIN_VoltageCell3 2    //  Pin for measuring Voltage of cell 3 ( Analog In Pin! )
//#define PIN_VoltageCell4 3    //  Pin for measuring Voltage of cell 4 ( Analog In Pin! )
//#define PIN_VoltageCell5 6    //  Pin for measuring Voltage of cell 5 ( Analog In Pin! )
//#define PIN_VoltageCell6 7    //  Pin for measuring Voltage of cell 6 ( Analog In Pin! )

/***************************************************************************************/
/* Hardware settings=>Here you can change which pins will be used for which connection */
/*                    and the I2C address of the connected MS5611 module               */
/*                    Normaly you do not have to change any of these                   */
/***************************************************************************************/
#define PIN_SerialTX 4        // 4  the pin to transmit the serial data to the frsky telemetry enabled receiver
#define PIN_ClimbLed 13       // 13 the led used to indicate lift ( led lights up above +0.10m/s)
#define PIN_AnalogClimbRate 3 // 3 the pin used to optionally write the data to the frsky a1 or a2 pin (could be 3,5,6,9,10,11)
#define PIN_PPM 2             // default: 2 the pin to read the PPM Signal on coming from the receiver.           
// you can uncomment this line if you want to completly disable the remote control functionality

// **** Red / Green duoLED on myVarioEX-Board for Fall / Climb Indication
#ifdef DUOLED
#define PIN_redLED 8
#define PIN_greenLED 9
#endif

const int I2CAdd=0x77;        // 0x77 The I2C Address of the MS5611 breakout board 
// (normally 0x76 or 0x77 configured on the MS5611 module via a solder pin or fixed)

/***************************************************************************************/
/* sensitivity / data filtering / Kalman filter parameters                             */
/* The Kalman Filter is being used to remove noise from the MS5611 sensor data. It can */
/* be adjusted by changing the value Q (process Noise) and R (Sensor Noise).           */
/* Best pactise is to only change R, as both parametrers influence each other.         */
/* (less Q requires more R to compensate and vica versa)                               */
/***************************************************************************************/
#define KALMAN_Q 0.05 // 0.05 do not tamper with this value unless you are looking for adventure ;-)
#define KALMAN_R 180  // default:110   change this value if you want to adjust the default sensitivity!
// this will only be used if PIN_PPM is NOT defined
//  50 = fast but more errors (good for sensor testing...but quite a lot of noise)
// 100 = medium speed and less false indications (nice medium setting with a silence window (indoor)of -0.2 to +0.2)
// 200 = conservative setting ;-)
// 500 = still useable maybe for a slow flyer?
// 1000 and q=0.001 is it moving at all?? in stable airpressure you can measure 
//      the height of your furniture with this setting. but do not use it for flying ;-)
// .. everything inbetween in relation to these values.

/***************************************************************************************/
/* Optional analog output of vertical speed (to A1 or A2 on receiver                   */
/* In case you want to connect the vario to a receiver that only has A1/2  and no RS232*/
/* additional Hardware required! read the WiKi if you want to use this                 */
/***************************************************************************************/
//#define ANALOG_CLIMB_RATE   // If defined, the vertical speed will be written as 
// PWM Signal to the defined port
#define OutputClimbRateMin -3 // -3 this values eqaly 0V voltage to the receiver
#define OutputClimbRateMax  3 // 3  this values eqaly 3.2V voltage to the receiver

/*****************************************************************************************************/
/*****************************************************************************************************/
/*****************************************************************************************************/
//                No changes below this line unless you know what you are doing                       /
/*****************************************************************************************************/
/*****************************************************************************************************/
/*****************************************************************************************************/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <EEPROM.h> //Needed to access the eeprom read write functions

// #define DEBUG

// Software Serial is used including the Inverted Signal option ( the "true" in the line below ) 
// Pin PIN_SerialTX has to be connected to rx pin of the receiver
// we do not need the RX so we set it to 0
SoftwareSerial mySerial(0, PIN_SerialTX, true); // RX, TX


//#define KALMANPOTI  //if this is defined, 2x 4,7k potentiometer can be attached to A2 and A3 to directly 
// adjust the following values:
// q = process noise (send to transmitter as T2 (q multiplied by 1000)
// r = sensor noise (send to transmitter as DIST)
//#define KALMANDUMP  // for filter process visualization. Output the measured and esitmated pressure 
// via RS232. then this data can be visualized e.g. using processing.
// you probably do not need to define this

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

const int AverageValueCount=6; // was 6, the number of values that will be used to calculate a new average pressure
const int calcIntervallMS = 50; // was 25, the intervall for the calculation intervall (do not change!)
// the GetClimbRate() requires this to be a fixed value of 50ms currently
long avgPressure;

long pressureValues[AverageValueCount+1];
long pressureStart; // airpressure measured in setup() can be used to calculate total climb / relative altitude

unsigned int calibrationData[7]; // The factory calibration data of the ms5611
unsigned long millisStart,lastMillisCalc,lastMillisVerticalSpeed,lastMillisFrame1,lastMillisFrame2;
int Temp=0;
int myAlt=0;
float alt=0,rawalt=0;
int16_t Height ;
int16_t DeltaHeight ;
int16_t HeightStart=0 ;
uint16_t FrameCnt=0 ;
uint16_t Volt=0 ;
long climbRate=0;
long Lastclimb=0;
long Averageclimb=0;
int abweichung=0;

#ifdef JETIDUPLEX
//* Variablen für Jeti Duplex Classic und EX
char ex[32];                // EX Framebuffer
char JetiLine1[17];         // Classic Jetibox Zeile 1
char JetiLine2[17];         // und Zeile 2
uint16_t ExFrameCnt=0;      // Framezähler für EX
#endif // JETIDUPLEX

double kalman_q= KALMAN_Q; //process noise covariance
double kalman_r= KALMAN_R; //measurement noise covariance

boolean ledIsOn=false;
int ledcnt=0;
int countDownSeconds=65535;
int i=0;

/********************************************************************************** Setup() */
void setup()
{
#ifdef ANALOG_CLIMB_RATE
  analogWrite(PIN_AnalogClimbRate,255/5*1.6); // initialize the output pin 
#endif
#ifdef PIN_VoltageCell1
  pinMode(PIN_VoltageCell1,INPUT); 
#endif
#ifdef PIN_VoltageCell2   
  pinMode(PIN_VoltageCell2,INPUT); 
#endif
#ifdef PIN_VoltageCell3   
  pinMode(PIN_VoltageCell3,INPUT); 
#endif
#ifdef PIN_VoltageCell4   
  pinMode(PIN_VoltageCell4,INPUT); 
#endif
#ifdef PIN_VoltageCell5   
  pinMode(PIN_VoltageCell5,INPUT);
#endif
#ifdef PIN_VoltageCell6   
  pinMode(PIN_VoltageCell6,INPUT); 
#endif

  Wire.begin();

#ifdef KALMANDUMP define DEBUG
#endif

#ifdef DUOLED
  pinMode (PIN_greenLED, OUTPUT);
  pinMode (PIN_redLED, OUTPUT);
#endif

#ifdef JETIDUPLEX
  //------------ Setup Serial Port for Jetibox Output  HST ----------------
  JetiUartInit();
  delay (500);
  for (i=i; i<40; i++)    // 40 x 0,08 s lang Einschaltmeldung auf Classic Jetibox anzeigen
  {
    JetiSensor ("Jeti myEX Vario ","Ver. 09.03.2013 ",0,false);
    delay(80);
  }
#endif // JETIDUPLEX

  // Serial.begin(9600);

#ifdef KALMANDUMP undefine DEBUG
#endif
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);

#ifdef PIN_PPM   
  pinMode(PIN_PPM,INPUT); 
  digitalWrite(PIN_PPM,HIGH); // enable the internal pullup resistor
#endif

  // Setup the ms5611 Sensor and read the calibration Data
  setupSensor();

  // prefill the air pressure buffer
  for (int i = 0;i<AverageValueCount*2;i++){
    SavePressure(getPressure());
  }
  pressureStart= getAveragePress();
  alt=getAltitude(getPressure (),Temp);

#ifdef FORCE_ABSOLUTE_ALT
  SendAlt(1);  // send initial height
  SendValue(0x00,int16_t(1)); //>> overwrite alt offset in open 9x in order to start with display of absolute altitude... 
  SendValue(0x30,(int16_t)(alt/100)+2); //>> overwrite min alt in open 9x
  SendValue(0x31,(int16_t)(alt/100)-2); //>> overwrite min alt in open 9x
  mySerial.write(0x5E); // End of Frame 1!
#endif

  millisStart=millis();
}

/********************************************************************************** Loop () */
void loop()
{
  uint16_t Height ;
  long pressure;

  pressure=getPressure();
  SavePressure(pressure);// Fill the pressure buffer
  rawalt=getAltitude(pressure,Temp);
  alt=kalman_update(rawalt);

  // ------------------------------------------------------ Do the math every <n> ms
  if( (lastMillisCalc + calcIntervallMS) <=millis()) 
  {
    lastMillisCalc=millis(); 
    climbRate=GetClimbRate(alt);

#ifdef LED_ClimbBlink 
    if(climbRate >LED_ClimbBlink) ledOn();
    else ledOff();
#endif

#ifdef DUOLED
   if (climbRate > LED_ClimbBlink) 
      digitalWrite(PIN_greenLED, 0);
   else
      digitalWrite(PIN_greenLED, 1);
   
   if (climbRate < -LED_ClimbBlink) 
      digitalWrite(PIN_redLED, 0);
   else
      digitalWrite(PIN_redLED, 1);
#endif

  }

  /*  if( (lastMillisVerticalSpeed + 100) <=millis()) 
   {
   lastMillisVerticalSpeed=millis(); 
   #ifdef SEND_VERT_SPEED   
   SendValue(FRSKY_USERDATA_VERT_SPEED,(int16_t)climbRate); // ClimbRate in open9x Vario mode
   mySerial.write(0x5E); // End of Frame xx!
   #endif
   }
   */
  // ---------------------------------------------------- Jeti-Frame to send every 100ms 
#ifdef JETIDUPLEX
  if( (lastMillisFrame1 + 100) <=millis())         // Jeti alle 100 ms
#else
  if( (lastMillisFrame1 + 200) <=millis())         // FrSky alle 200 ms
#endif // JETIDUPLEX
  {
    lastMillisFrame1=millis(); 
    avgPressure=getAveragePress(); 

#ifdef DEBUG
    Serial.print(" Pressure:");    
    Serial.print(pressure,DEC);
    Serial.print(" AveragePressure:");    
    Serial.print(avgPressure,DEC);
    Serial.print(" Altitude:");      
    Serial.print(alt,DEC);
    Serial.print(" ClimbRate:");      
    Serial.print(climbRate,DEC);
    Serial.print(" K_R:");    
    Serial.print(kalman_r);
    Serial.print(" Temp:");    
    Serial.print(Temp,DEC);
    Serial.print(" VCC:"); 
    Serial.print(readVccMv());
    Serial.println();
#endif

#ifdef JETIDUPLEX
    // -----------------------------------------------------------
    // ------------------- Jetibox output -- HST -----------------
    // -----------------------------------------------------------
    Height = (uint16_t)alt/10;            // absolute Höhe errechnen
    if (FrameCnt < 50)                    // bei ersten Durchläufen gleitenden Mittelwert errechnen für relative Höhenangabe
    {
      FrameCnt++;
      HeightStart = (HeightStart + Height)/2 ;
    }

    DeltaHeight = Height - HeightStart;            // Höhe über Startplatz errechnen
    Averageclimb = (Lastclimb + climbRate) / 2 ;   // Mittelwert aus letzten 2 Steigraten

    // Die 2 Zeilen der Classic-Jetibox beschreiben
    // Unilog 1-Mimikri, damit auch Vspeak die Daten erkennt und als Vario piept...   
    strcpy (JetiLine1, "  0.00V     2.0m");        // Zeile 1: Spannung ist noch Dummy (Unilog-kompatibel)
    Dezi_Integer (JetiLine1, DeltaHeight, 9);

    strcpy (JetiLine2, "  0.00C      m/s");        // Zeile 2
    Dezi_Integer99 (JetiLine2, Temp, 1);
    Dezi_Integer99 (JetiLine2, (int16_t)Averageclimb, 8);

    // Jetzt ein Jeti EX-Datenpaket errechnen und gleich abschicken
    if (ExFrameCnt++ > 255)                                // Häufigkeit der EX Protokollheader (1..9: Header, sonst: EX-Datenpakete)
      ExFrameCnt = 1;                                      // gezielter Zähler-Überlauf
    SendEx (ExFrameCnt) ;                                  // Jeti EX-Frame an Jetibox verschicken 

    JetiSensor(JetiLine1, JetiLine2, 0, false);            // Jeti Classic Textframe direkt hinterher

    Lastclimb = climbRate ;                                // Aktuelle Climbrate für Mittelwertbildung merken
#endif // JETIDUPLEX

#ifdef SEND_Alt          
    SendAlt(alt);
#endif
#ifdef SEND_VERT_SPEED   
    SendValue(FRSKY_USERDATA_VERT_SPEED,(int16_t)climbRate); // ClimbRate in open9x Vario mode
#endif
#ifdef SEND_VREF 
    // internal voltage as cell 0 which is not optimal. somebody will probably have to add another id
    // for this or change the the way the vfas voltage gets displayed 
    SendCellVoltage(0,readVccMv()); 
#endif   
#ifdef SEND_VFAS_NEW
    // vcc as vfas (supposed to be working in an upcoming version of open9x)
    SendValue(FRSKY_USERDATA_VFAS_NEW,(int16_t)readVccMv()/100); 
#endif   
#ifdef PIN_VoltageCell1 
    SendCellVoltage(1,ReadVoltage(PIN_VoltageCell1));
#endif
#ifdef PIN_VoltageCell2
    SendCellVoltage(2,ReadVoltage(PIN_VoltageCell2));
#endif
#ifdef PIN_VoltageCell3 
    SendCellVoltage(3,ReadVoltage(PIN_VoltageCell3));
#endif
#ifdef PIN_VoltageCell4 
    SendCellVoltage(4,ReadVoltage(PIN_VoltageCell4));
#endif
#ifdef PIN_VoltageCell5 
    SendCellVoltage(5,ReadVoltage(PIN_VoltageCell5));
#endif

    // ********************************* The DIST Field
#ifdef SEND_AltAsDIST     // send alt as adjusted to precision in dist field
    if (alt- SEND_AltAsDIST <= 32768) SendGPSDist(uint16_t(alt- SEND_AltAsDIST ));
    else if (alt- SEND_AltAsDIST < 327680) SendGPSDist(uint16_t((alt-SEND_AltAsDIST)/(long)10));
    // If altitude gets higher than 327,68m alt/10 will be transmitted till 3276,8m then alt/100
    else SendGPSDist(uint16_t((alt-SEND_AltAsDIST)/(long)100));
#endif 
#ifdef SEND_PressureAsDIST
    SendGPSDist(uint16_t(avgPressure/10));
#endif
#ifdef SEND_SensitivityAsDist
    SendGPSDist(uint16_t(kalman_r));
#endif
    // ********************************* The Temp 1 FIeld
#ifdef SEND_TEMP_T1      
    SendTemperature1(Temp); //internal MS5611 voltage as temperature T1
#endif
#ifdef SEND_PressureAsT1 // pressure in T1 Field
    SendTemperature1(avgPressure-SEND_PressureAsT1*10); //pressure in T1 Field
#endif
    // ********************************* The Temp 2 FIeld
#ifdef SEND_TEMP_T2      
    SendTemperature2(Temp); //internal MS5611 voltage as temperature T1
#endif
#ifdef SEND_SensitivityAsT2 // Kalman Param R in Temp2
    SendTemperature2(uint16_t(kalman_r)*10); //internal MS5611 voltage as temperature T1
#endif

    // ********************************* The RPM Field   
#ifdef SEND_AltAsRPM     
    SendRPM(alt);
#endif
#ifdef SEND_PressureAsRPM
    SendRPM(uint16_t(avgPressure/10));
#endif
#ifdef SEND_SensitivityAsRPM
    SendRPM(uint16_t(kalman_r));
#endif

    // ******************************** Other send stuff...
#ifdef KALMANPOTI
    SendTemperature2(kalman_q*10000);
    SendGPSDist(uint16_t(kalman_r));
#endif

    // SendCurrent(133.5);    // example to send a current. 
    mySerial.write(0x5E); // End of Frame 1!

#ifdef ANALOG_CLIMB_RATE   
    SendAnalogClimbRate(climbRate); //Write the Clib/SinkRate to the output Pin
#endif

  }
  /*
  // Frame 2 to send every 1000ms (must be this intervall, otherwise the climb rate calculation in open9x will not work 
   if( (lastMillisFrame2 + 1000) <=millis()) 
   {
   float avgTemp=getAverageTemperature(0);
   SendGPSAlt(getAltitude(avgPressure,avgTemp));
   
   mySerial.write(0x5E); // End of Frame 1!
   lastMillisFrame2=millis(); 
   }
   */
}

#ifdef ANALOG_CLIMB_RATE
/**********************************************************/
/* SendAnalogClimbRate => output a voltage for vert.speed */
/**********************************************************/
void SendAnalogClimbRate(long cr)
{
  int outValue;
  outValue=map(cr,OutputClimbRateMin*100,OutputClimbRateMax*100,0,255/5*3.2);
  // Protect against too high values-.
  if(outValue>(255/5*3.2))outValue=255/5*3.2; // just in case we did something wrong... stay in the pin limits of max 3.3Volts
  analogWrite(PIN_AnalogClimbRate,(int)outValue);
}
#endif

/**********************************************************/
/* SaveToEEProm => save the kalman_r (sensirtivity) to the*/
/*   eeprom and inform the pilot                          */
/**********************************************************/
void SaveToEEProm(){
  // Store the last known value from the ppm signal to the eeprom
  eepromIntWrite(0,kalman_r); // just for testing purposes...
  // send a Signal to the user that the value has been stored
  // Blink LED: 1 Second on. 3time on for 200 ms, 1 second on
  SendGPSDist(uint16_t(9999));
  ledOn() ;
  delay(1000);
  SendGPSDist(uint16_t(kalman_r));
  ledOff();
  delay(200);
  ledOn() ;
  delay(200);
  ledOff();
  delay(200);
  ledOn() ;
  delay(200);
  ledOff();
  delay(200);
  ledOn() ;
  delay(200);
  ledOff();
  delay(200);
  SendGPSDist(uint16_t(9999));
  ledOn() ;
  delay(1000);
  ledOff();
}

/**********************************************************/
/* ReadPPM => read PPM signal from receiver               */
/*   pre-evaluate its value for validity                  */
/**********************************************************/

// ReadPPM - Read ppm signal and detect if there is no signal
unsigned long ReadPPM(){
  unsigned long ppm= pulseIn(PIN_PPM, HIGH, 20000); // read the pulse length in micro seconds
  // Serial.print("PPM=");Serial.println(ppm);
  if ((ppm>2500)or (ppm<500))ppm=0; // no signal!
  return ppm;
}

/**********************************************************/
/* eepromIntWrite => Write an unsigned int to eeprom      */
/**********************************************************/
void eepromIntWrite(int adr, unsigned int value){
  //Serial.print("Writing to eeprom:");Serial.println(value);
  byte lowB = ((value >> 0) & 0xFF);
  byte highB = ((value >> 8) & 0xFF);
  EEPROM.write(adr, lowB);
  EEPROM.write(adr+ 1, highB);
}

/**********************************************************/
/* eepromIntRead => Read unsigned int from the eeprom     */
/**********************************************************/
// Read unsigned integer from the eeprom
unsigned int eepromIntRead(int adr){
  byte lowB = EEPROM.read(adr);
  byte highB = EEPROM.read(adr + 1);
  //Serial.print("Read from eeprom:");Serial.println(lowB + (highB << 8));
  return lowB + (highB << 8);
}

/*******************************************************************************/
/* GetClimbRate => calculate the current climbRate                             */
/* has to be invoked every 50ms for a queue length of 5 with a multiplier of 4 */
/* or every 25 ms for a queue length of 5 with a multiplier of 8               */
/* or every 25 ms for a queue length of 8 with a multiplier of 5               */
/*******************************************************************************/
//const byte ClimbRateQueueLength=8;
const byte ClimbRateQueueLength=8;
float GetClimbRate(float alti){
  static float climbRateQueue[ClimbRateQueueLength];
  static float lastAlti=alti;
  static byte cnt=0;
  static float myClimbRate=0;
  myClimbRate -= climbRateQueue[cnt]; // overwrite the oldest value in the queue
  climbRateQueue[cnt]=alti-lastAlti; // store the current height difference
  cnt+=1;
  if (cnt ==ClimbRateQueueLength)cnt=0;
  myClimbRate += alti-lastAlti; // add the current height difference
  lastAlti=alti;
  return(myClimbRate*( (1000/calcIntervallMS)/ClimbRateQueueLength));  // if this func ist being called <N> times a second than the multiplier must be <N>/queuelen
  //  return(myClimbRate*5);  // if this func ist being called <N> times a second than the multiplier must be <N>/queuelen
}

/**********************************************************/
/* SendValue => send a value as frsky sensor hub data     */
/**********************************************************/
void SendValue(uint8_t ID, uint16_t Value) {
  uint8_t tmp1 = Value & 0x00ff;
  uint8_t tmp2 = (Value & 0xff00)>>8;
  mySerial.write(0x5E);  
  mySerial.write(ID);
  if(tmp1 == 0x5E) { 
    mySerial.write(0x5D);    
    mySerial.write(0x3E);  
  } 
  else if(tmp1 == 0x5D) {    
    mySerial.write(0x5D);    
    mySerial.write(0x3D);  
  } 
  else {    
    mySerial.write(tmp1);  
  }
  if(tmp2 == 0x5E) {    
    mySerial.write(0x5D);    
    mySerial.write(0x3E);  
  } 
  else if(tmp2 == 0x5D) {    
    mySerial.write(0x5D);    
    mySerial.write(0x3D);  
  } 
  else {    
    mySerial.write(tmp2);  
  }
  // mySerial.write(0x5E);
}
/**********************************************************/
/* ReadVoltage => read the voltage of an analog input pin */
/**********************************************************/
uint16_t ReadVoltage(int pin)
{
  // Convert the analog reading (which goes from 0 - 1023) to a millivolt value
  return uint16_t((float)analogRead(pin)*(float)(readVccMv()/1023.0));
}

/**********************************************************/
/* SendCellVoltage => send a cell voltage                 */
/**********************************************************/
void SendCellVoltage(uint8_t cellID, uint16_t voltage) {
  voltage /= 2;
  uint8_t v1 = (voltage & 0x0f00)>>8 | (cellID<<4 & 0xf0);
  uint8_t v2 = (voltage & 0x00ff);
  uint16_t Value = (v1 & 0x00ff) | (v2<<8);
  SendValue(FRSKY_USERDATA_CELL_VOLT, Value);
}
/**********************************/
/* SendGPSDist => send 0..32768   */
/**********************************/
void SendGPSDist(uint16_t dist) {// ==> Field "Dist" in open9x
  SendValue(0x3C,uint16_t(dist)); //>> DIST
}
void SendTemperature1(int16_t tempc) {
  SendValue(FRSKY_USERDATA_TEMP1, tempc/10);
}
void SendTemperature2(int16_t tempc) {
  SendValue(FRSKY_USERDATA_TEMP2, tempc/10);
}
/*************************************/
/* SendRPM => Send Rounds Per Minute */
/*************************************/
void SendRPM(uint16_t rpm) {
  byte blades=2;
  rpm = uint16_t((float)rpm/(60/blades));  
  SendValue(FRSKY_USERDATA_RPM, rpm);
}
/*************************************/
/* SendCurrent => Send Current       */
/*************************************/

void SendCurrent(float amp) {
  SendValue(FRSKY_USERDATA_CURRENT, (uint16_t)(amp*10));
}
/**********************************/
/* SendAlt => Send ALtitude in cm */
/**********************************/
void SendAlt(long altcm)
{
  // The initial altitude setting in open9x seems not to  work if we send 0m. it works fine though if we use 1m, so as a workaround we 
  // increase all alt values by 1.
  uint16_t Centimeter =  uint16_t(altcm%100);
  int16_t Meter = int16_t((altcm-(long)Centimeter)/(long)100);
#ifdef FORCE_ABSOLUTE_ALT
  Meter-=1; // To compensate for a Offset of 1 at the beginning
#endif
#ifdef XDEBUG
  Serial.print("Meter:");
  Serial.print(Meter);
  Serial.print(",");
  Serial.println(Centimeter);
#endif

  SendValue(FRSKY_USERDATA_BARO_ALT_B, Meter);
  SendValue(FRSKY_USERDATA_BARO_ALT_A, Centimeter);
}

void SendGPSAlt(long altcm)
{
  altcm+=100;// The initial altitude setting in open9x seems not to  work if we send 0m. 
  // it works fine though if we use 1m, so as a workaround we increase all alt values by 1.
  uint16_t Centimeter =  uint16_t(altcm%100);
  int16_t Meter = int16_t((altcm-Centimeter)/100);

#ifdef DEBUG
  Serial.print("GPSAlt! Meter:");
  Serial.print(Meter);
  Serial.print("Centimeter:");
  Serial.println(Centimeter);
#endif
  SendValue(FRSKY_USERDATA_GPS_ALT_B, Meter);
  SendValue(FRSKY_USERDATA_GPS_ALT_A, Centimeter);
}

/****************************************************************/
/* getAltitude - convert mbar and temperature into an altitude  */
/*               value                                          */
/****************************************************************/
float getAltitude(long press, int temp) {
  float   result;
  temp/=10;
  result=(float)press/100;
  const float sea_press = 1013.25;

  return (((pow((sea_press / result), 1/5.257) - 1.0) * ( (temp/100) + 273.15)) / 0.0065 *100);
}

/****************************************************************/
/* SavePressure - save a new pressure value to the buffer       */
/* Rotating Buffer for calculating the Average Pressure         */
/****************************************************************/
void SavePressure(long currentPressure){
  static int cnt =0;
  pressureValues[cnt++]=currentPressure;
  if(cnt>AverageValueCount){
    cnt=0;
  }
}
/****************************************************************/
/* getAveragePress - calculate average pressure based on all    */
/* entries in the pressure buffer                               */
/****************************************************************/
long getAveragePress()
{
  long result=0;
  for (int i=0;i<AverageValueCount;i++) result +=pressureValues[i];
  return result/AverageValueCount;
}

/****************************************************************/
/* getPressure - Read pressure + temperature from the MS5611    */
/****************************************************************/
long getPressure()
{
  long D1, D2, dT, P;
  float TEMP;
  int64_t OFF, SENS;

  //  D1 = getData(0x48, 10); //OSR=4096 0.012 mbar precsision
  D1 = getData(0x48, 9); //OSR=4096 0.012 mbar precsision
  D2 = getData(0x50, 1);

  dT = D2 - ((long)calibrationData[5] << 8);
  TEMP = (2000 + (((int64_t)dT * (int64_t)calibrationData[6]) >> 23)) / (float)100;
  Temp=(int)(TEMP*10);
  OFF = ((unsigned long)calibrationData[2] << 16) + (((int64_t)calibrationData[4] * dT) >> 7);
  SENS = ((unsigned long)calibrationData[1] << 15) + (((int64_t)calibrationData[3] * dT) >> 8);
  P = (((D1 * SENS) >> 21) - OFF) >> 15;
  return P;
}

/****************************************************************/
/* getData - Read data from I2C bus                             */
/****************************************************************/
long getData(byte command, byte del)
{
  long result = 0;
  twiSendCommand(I2CAdd, command);
  delay(del);
  twiSendCommand(I2CAdd, 0x00);
  Wire.requestFrom(I2CAdd, 3);
#ifdef DEBUG
  if(Wire.available()!=3)Serial.println("Error: raw data not available");
#endif

  for (int i = 0; i <= 2; i++)    result = (result<<8) | Wire.read(); 
  return result;
}

/****************************************************************/
/* setupSensor - and read the calibration Data                  */
/****************************************************************/
void setupSensor()
{
#ifdef DEBUG
  Serial.println("Setup Sensor Start.");
#endif
  twiSendCommand(I2CAdd, 0x1e);
  delay(100);
  for (byte i = 1; i <=6; i++)
  {
    unsigned int low, high;
    twiSendCommand(I2CAdd, 0xa0 + i * 2);
    Wire.requestFrom(I2CAdd, 2);
    if(Wire.available()!=2) {
#ifdef DEBUG
      Serial.println("Error: calibration data not available");
#endif
    }
    high = Wire.read();
    low = Wire.read();
    calibrationData[i] = high<<8 | low;
#ifdef DEBUG
    Serial.print("calibration data #");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println( calibrationData[i] ); 
#endif
  }

#ifdef DEBUG
  Serial.println("Setup Sensor Done.");
#endif
}

/****************************************************************/
/* twiSendCommand - Send a command to the I2C Bus               */
/****************************************************************/
/* Send a command to the MS5611 */
void twiSendCommand(byte address, byte command)
{
  Wire.beginTransmission(address);
  if (!Wire.write(command)){
#ifdef DEBUG
    Serial.println("Error: write()");
#endif
  }
  if (Wire.endTransmission()) 
  {
#ifdef DEBUG
    Serial.print("Error when sending command: ");
    Serial.println(command, HEX);
#endif
  }
}

void ledOn()
{
  digitalWrite(PIN_ClimbLed,1);
  ledIsOn=true;
}


void ledOff()
{
  digitalWrite(PIN_ClimbLed,0);
  ledIsOn=false;
}

/* readVcc - Read the real internal supply voltageof the arduino 
 
 Improving Accuracy
 While the large tolerance of the internal 1.1 volt reference greatly limits the accuracy 
 of this measurement, for individual projects we can compensate for greater accuracy. 
 To do so, simply measure your Vcc with a voltmeter and with our readVcc() function. 
 Then, replace the constant 1125300L with a new constant:
 
 scale_constant = internal1.1Ref * 1023 * 1000
 
 where
 
 internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
 
 This calibrated value will be good for the AVR chip measured only, and may be subject to
 temperature variation. Feel free to experiment with your own measurements. 
 */
/* 
 Meine Versorgungsspannung per Multimeter =4,87V 
 1,1*4870 /5115 =1,047311827957
 */
//const long scaleConst = 1071.4 * 1000 ; // internalRef * 1023 * 1000;
const long scaleConst = 1125.300 * 1000 ; // internalRef * 1023 * 1000;
int readVccMv() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result = scaleConst / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (int)result; // Vcc in millivolts
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
float kalman_update(float measurement)
{
  static int lcnt=0;
  static float x=alt; //value
  static float p=100; //estimation error covariance
  static float k=0; //kalman gain
#ifdef KALMANPOTI
  // Use 2 analog potis (4,7K) connected to A2 and A3 in order to adjust the 2 main parameters of this filter
  // outer pins of each pot to +5 and GND, wiper to A2,A3
  float pl=analogRead(2);
  int pr=analogRead(3);
  pl=mapFloat(pl,0,1023,0,0.1);  // range for q is 0-0.1
  pr=mapFloat(pr,0,1023,0,1000); // range for r is 0-1000
  kalman_q = pl; // the process noise q, 
  kalman_r = pr; // the sensor noise r, 
#endif

  // update the prediction value
  p = p + kalman_q;

  // update based on measurement
  k = p / (p + kalman_r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;

  /*
  abweichung=measurement -kalman_state.x;
   Serial.print("q=");Serial.print(q);  Serial.print(" r=");Serial.print(r);
   Serial.print(" p=");  Serial.print(p);  Serial.print(" X=");Serial.print(x);
   Serial.print(" k=");Serial.print(k);  Serial.print(" D=");Serial.print(abweichung );
   Serial.println();*/

#ifdef KALMANDUMP
  abweichung=measurement -x;
  // output to processing.
  Serial.print(rawalt);
  Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.println(abweichung);
#endif
  return x;
}

#ifdef JETIDUPLEX
//* 1/10 einer Dezimalzahl mit Dezimalpunkt in String ausgeben
void Dezipunkt_Zahl (char *buf, uint16_t Zahl, uint8_t Pos)
{
  if (Zahl > 9999) Zahl = 9999;			// Zahlen größer 999.9 begrenzen

  Zahl=Zahl-(abs(Zahl/10000)*10000);
  if (Zahl<1000) buf[Pos]=' ';
  else buf[Pos] = 48+(Zahl/1000);

  Zahl=Zahl-(abs(Zahl/1000)*1000);
  if ((Zahl<100)&(buf[Pos]==' ')) buf[Pos+1]=' ';
  else buf[Pos+1] = 48+(Zahl/100);

  Zahl=Zahl-(abs(Zahl/100)*100);
  if ((Zahl<10)&(buf[Pos+1]==' ')) buf[Pos+2]='0';
  else buf[Pos+2] = 48+(Zahl/10);

  buf[Pos+3] = '.';
  buf[Pos+4] = 48+Zahl-(abs(Zahl/10)*10);
}

//* 1/10 eines Words 2-stellig mit Dezimalpunkt in String ausgeben
void Dezipunkt_Zahl99 (char *buf, uint16_t Zahl, uint8_t Pos)
{
  if (Zahl > 999) Zahl = 999;			// Zahlen grÃ¶ÃŸer 99.9 begrenzen

  Zahl=Zahl-(abs(Zahl/1000)*1000);
  if ((Zahl<100)) buf[Pos]=' ';
  else buf[Pos]=48+(Zahl/100);

  Zahl=Zahl-(abs(Zahl/100)*100);
  if ((Zahl<10)&(buf[Pos]==' ')) buf[Pos+1]='0';
  else buf[Pos+1]=48+(Zahl/10);

  buf[Pos+2]='.';
  buf[Pos+3]=48+Zahl-(abs(Zahl/10)*10);
}

//* 1/10 eines Words mit Vorzeichen und Dezimalpunkt in String ausgeben
void Dezi_Integer (char *buf, uint16_t Zahl, uint8_t Pos)
{
  if (Zahl>32000) 			// negativer Wert im Zweier-Kompliment
  {
    buf[Pos]='-';
    Zahl = 0xFFFF - Zahl;	// in positiven Wert umrechnen
  }
  else
    buf[Pos]='+';
  Dezipunkt_Zahl (buf, Zahl, Pos+1) ;
}

//* 1/10 eines Words 2-stellig mit Vorzeichen und Dezimalpunkt in String ausgeben
void Dezi_Integer99 (char *buf, uint16_t Zahl, uint8_t Pos)
{
  if (Zahl>32000) 			// negativer Wert im Zweier-Kompliment
  {
    buf[Pos]='-';
    Zahl = 0xFFFF - Zahl;	// in positiven Wert umrechnen
  }
  else
    buf[Pos]='+';
  Dezipunkt_Zahl99 (buf, Zahl, Pos+1) ;
}

//* Jeti EX-Protokoll: Berechne 8-bit CRC polynomial X^8 + X^2 + X + 1
#define POLY 0x07
unsigned char update_crc (unsigned char crc, unsigned char crc_seed)
{
  unsigned char crc_u;
  unsigned char i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for (i=0; i<8; i++)
    crc_u = ( crc_u & 0x80 ) ? POLY ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

//* CRC8-Checksumme über EX-Frame berechnen, Originalroutine von Jeti
unsigned char jeti_crc8 (char *exbuf, unsigned char framelen)
{
  unsigned char crc = 0;
  unsigned char c;

  for(c=2; c<framelen; c++)
    crc = update_crc (exbuf[c], crc);
  return (crc);
}

//* EX-Datenframe zum Versand vorbereiten (Frameheader, Crypting, Länge, CRC)
void jeti_crypt (char *exbuf, unsigned char n)
{
  unsigned char i;
  uint8_t key;
  uint8_t cryptcode[4] = { 
    0x52,0x1C,0x6C,0x23   };

  // Statische Framedaten ergänzen
  exbuf[0] = 0x7E;					// EX-Frame Separator
  exbuf[1] = 0x2F;
  exbuf[2] |= n-2;					// Framelänge im Byte 2 ablegen
  exbuf[3] = MANUFAC_ID1; 
  exbuf[4] = MANUFAC_ID2;       // ID anlegen
  exbuf[5] = DEVICE_ID1;  
  exbuf[6] = DEVICE_ID2;

  key=(uint8_t)random()+1;			// Crypt Key errechnen 
  exbuf[7] = key;					// und ablegen

  exbuf[8]=exbuf[8] ^ key ^ 0x6D;
  if (!(exbuf[2] & 0x02))			// irgendwas an Byte 8 rumfummeln... ?	
    exbuf[8]^=0x3F;

  exbuf[9] ^= 32;					// Roberts Tipp (tnx!)

  for (i=9;i<n;i++)				// Frame ab Byte 9 verschlüsseln
  {
    exbuf[i]=exbuf[i] ^ key ^ (cryptcode[i%4] + ((i%2)?((i-8)&0xFC):0));
    if (key & 0x02)
      exbuf[i]^=0x3F;
  }
  exbuf[n] = jeti_crc8 (exbuf, n);	                // Checksumme berechnen und ablegen
}

//* Eine Zeichenkette ohne Endekennung in EX-Buffer ab Position "pos" einkopieren
uint8_t jeticpy_pos (char *s1, char *exbuf, uint8_t n)
{
  uint8_t i ;
  i = n ;
  do
  {
    exbuf[i] = *s1++ ;
    i++;
  } 
  while (*s1) ;
  return(i) ;							  // Länge des EX-Buffers zurückgeben
}

//* Eine Zeichenkette ohne Endekennung in Jeti-(EX)-Buffer ab Position "n" einkopieren
uint8_t jeticpy_n (char *s1, char *exbuf, uint8_t n)
{
  uint8_t i ;
  i = n ;
  do
  {
    exbuf[i] = *s1++ ;
    i++;
  } 
  while (*s1) ;
  return(i) ;							  // Länge des EX-Buffers zurückgeben
}

//* Signed Integer in Duplex-EX-Format "int14" wandeln (Kommaposition an vorletzter Stelle)
uint16_t uint14 (long value)                            
{
  if (value < 0)
    return ((value >> 8) & 0x1F) | 0xA0;       	// 6-Bit-HighByte (negativ, Komma an vorletzter Stelle) oberste 3 Bits löschen und NEG + Komma setzen
  else
    return (value >> 8) | 0x20;         	// 6-Bit-HighByte (Komma an vorletzter Stelle)
}

//* Eines von verschiedenen EX-Frames erzeugen und abschicken
//* Bei geradem x keine Infodaten, um mindestens jedes 2.Mal ein EX-Datenpaket zu senden
void SendEx (uint8_t x)
{
  uint8_t n=0;
  uint8_t i=0;
  unsigned char ack;

  switch (x)
  {
  case 0:                                                      // bei 0 gar kein EX-Paket schicken
    return;
  case 1:
    ex[2] = 0x00;						// 2Bit Type(0-3) 0x40=Data, 0x00=Text
    ex[8] = 0x00;						// Identifier (0 = Sensorname)
    ex[9] = (9<<3) | 1;				                // 5Bit der Beschreibung / 3Bit Länge der Einheit (SPC=Dummy)
    n = jeticpy_n ("myVarioEX ", ex, 10);
    break;
  case 3:
    ex[2] = 0x00;						// 2Bit Type(0-3) 0x40=Data, 0x00=Text
    ex[8] = 0x01;						// Identifier (1 = Kanalname)
    ex[9] = (8<<3) | 1;				                // 5Bit der Beschreibung / 3Bit Länge der Einheit
    n = jeticpy_n ("SpannungV", ex, 10);                        // ab Position 10 in den Buffer kopieren
    break;	    
  case 5:
    ex[2] = 0x00;						// 2Bit Type(0-3) 0x40=Data, 0x00=Text
    ex[8] = 0x02;						// Identifier (2 = Kanalname)
    ex[9] = (5<<3) | 1;				                // 5Bit der Beschreibung / 3Bit Länge der Einheit
    n = jeticpy_n ("Hoehem", ex, 10);                          // ab Position 10 in den Buffer kopieren
    break;
  case 7:
    ex[2] = 0x00;						// 2Bit Type(0-3) 0x40=Data, 0x00=Text
    ex[8] = 0x03;						// Identifier (3 = Kanalname)
    ex[9] = (10<<3) | 2;				        // 5Bit der Beschreibung / 3Bit Länge der Einheit
    n = jeticpy_n ("Temperatur\xB0\x43", ex, 10);               // ab Position 10 in den Buffer kopieren (Einheit "°C")
    break;
  case 9:
    ex[2] = 0x00;						// 2Bit Type(0-3) 0x40=Data, 0x00=Text
    ex[8] = 0x04;						// Identifier (4 = Kanalname)
    ex[9] = (9<<3) | 3 ;            			        // 5Bit der Beschreibung / 3Bit Länge der Einheit
    n = jeticpy_n ("Steigratem/s", ex, 10);                     // ab Position 10 in den Buffer kopieren
    break;

  default:
    ex[2] = 0x40;						// 2Bit Type(0-3) 0x40=Data, 0x00=Text
    n=8;						        // Position im Buffer

    //* Die aktuellen Datenwerte der einzelnen Kanäle verschicken
    ex[n++] = 0x11;					// Identifier = 1 (Kanal 1= Spannung) + Data Type 1 (int14_t)
    Volt = 126;						// Testwert 12.6V (Dummy, da aktuell noch keine A/D-Abfrage)
    ex[n++] = (Volt & 0xFF);			        // LowByte Spannung
    ex[n++] = ((Volt >> 8) & 0xFF) | 0x20 ;  	        // HighByte Spannung, 0x20 = Komma-Position an vorletzter Stelle

    ex[n++] = 0x21;					// Identifier = 2 (Kanal 2= Höhe) + Data Type 1 (int14_t)
    ex[n++] = DeltaHeight & 0xFF;              	  	// LowByte Höhe
    ex[n++] = uint14 (DeltaHeight) ;                 	// 6-Bit-HighByte Höhe (Komma an vorletzter Stelle)

    ex[n++] = 0x31;				  	// Identifier = 3 (Kanal 3= Temp) + Data Type 1 (int14_t)
    ex[n++] = Temp & 0xFF;		                // LowByte Temperatur
    ex[n++] = uint14 (Temp);                  	        // HighByte Temperatur (Komma-Pos an vorletzter Stelle)

    ex[n++] = 0x41;				        // Identifier = 4 (Kanal 4= Steigrate) + Data Type 1 (int14_t)
    ex[n++] = climbRate & 0xFF;        		        // LowByte Steigrate
    ex[n++] = uint14 (climbRate);                  	// HighByte Steigrate (Komma-Pos an vorletzter Stelle)
  }                                                     // in "n" bleibt die genutzte Paketlänge stehen

  jeti_crypt (ex, n) ;		       // EX-Frame ergänzen und verschlüsseln...

  ack = JetiGetChar();                 // Tastatur von Jetibox / DC-16-Sender abfragen, wird aber noch nicht ausgewertet
  UCSR0B &= ~(1<<RXEN0);               // disable RX (just to make sure)
  UCSR0B |= (1<<TXEN0);                // enable TX 

  JetiTransmitByte (0x7E, false);      // EX Frameheader verschicken
  for (i=1; i<=n; i++)                 // und EX Datenframe gleich hinterher
      JetiTransmitByte (ex[i], true);
}
#endif // JETIDUPLEX


