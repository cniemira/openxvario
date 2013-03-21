
// openxvario http://code.google.com/p/openxvario/
// started by R.Schloßhan 

/***************************************************************************************/
/* Activate FRSKY protocol alternatively to JETIDUPLEX just to save code, so           */
/* naked JETIDUPLEX code fits also into atmega168 controller instead of atmega328      */
/***************************************************************************************/
// #define FRSKY

// JETIDUPLEX mod by H.Stöcklein
#define JetiVersionStr "Ver. 21.03.2013 "

/***************************************************************************************/
/* Remote controlling the sensitivity using a servo port on your receiver              */
/* ==> choose 0 to 1 option! You need to have the PIN_PPM defined as well              */
/***************************************************************************************/
//#define PPM_AllwaysUsed  // PPM Signal always be used to control the sensitivity
                           // Choose this if you want to use a dedicated channel just for 
                           // adjusting the sensitivity

//#define PPM_ProgrammingMode // PPM Signal will only used if programming mode is initiated
                            // if this option is enabled, the vario will go into programming
                            // mode on startup if the ppm value is in a predefined range
                            // on powerup (e.g. rudder stick full left)
                            // the pulse length defining this "activation window"
                            // can be defined using the PPM_ProgrammingMode_minppm/maxppm values

/***************************************************************************************/
/* Other Configuration Options  => various different values to take influence on       */
/*                    various stuff. Normaly you do not have to change any of these    */
/***************************************************************************************/
// Choose if the led should be used for indicationg lift.
#define LED_ClimbBlink 10  // Blink when climbRate > 0.05cm. The numeric value can be changed to a different cm value

//#define FORCE_ABSOLUTE_ALT // If defined, the height offset in open9x will be resetted upon startup, which results 
                           // in an absolute height display in open9x . (You can still change to a relative display 
                           // by pressing [MENU] in the telem.screens
                           // If not defined, open9x will use the first transmitted altitude as an internal offset, 
                           // which results in an initial height of 0m

/********************** Jeti-Duplex-EX mod by H.Stoecklein *****************************/
/* JETIDUPLEX Version 21.3.2013   (based on openXvario version r131)                   */
/* By "define JETIDUPLEX" a DUPLEX EX compatible data stream is created at the HW UART */
/* with the channels Voltage, Temperature, Relative and absol. Altitude, Climbrate +   */
/* Jetibox textframe with the same data.                                               */
/* REMARK: */
/*  - HW-UART is connected via 100 Ohm to Jeti-EXT, so no more DEBUG available         */
/*  - Additional library required: JETIBOX.INO (unmodified), search for:               */
/*      Based on "Jeti Box Communication for Arduino Mini Pro 328 16MHz" by Alex...    */
/*      Modified and stripped down to minimal functions by sceak                       */
/***************************************************************************************/
#define JETIDUPLEX                  // Activate Jeti Duplex EX Protocol
#define DUOLED                      // Use DuoLED on myVarioEX-Board (Arduino Pins 8 and 9)
#define MYVARIOEX                   // Use Different Voltage Divider for Lipo on Custom Vario Board
#define CLIMB2DEC                   // Display climbrate with 2 decimals for sensor & filter testing

// Sensor-IDs for Jeti Duplex EX device identification
#define MANUFAC_ID1 0xB7	    // Manufacturer
#define MANUFAC_ID2 0x55
#define DEVICE_ID1 0xA8  	    // Device
#define DEVICE_ID2 0x55

/***************************************************************************************/
/* Which data to send in which telemetry field                                         */
/* ==> choose freely one value per Telemetry field                                     */
/***************************************************************************************/
#ifdef FRSKY

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
//#define SEND_SensitivityAsT2  // Kalman Param R in Temp2
#define SEND_PressureAsT2 9000 // pressure in 1/10th of mBar in T1 Field subtracted by
                               // the number in the define statment. 
                               // e.g. 950mbar => 9500 -offset of 9000 => 500 in display

//************************** the RPM Field? (choose only one) *************************/
//(unprecise field! resolution is in steps of 30RPM!)
//#define SEND_AltAsRPM      // Altitude in RPM ;-)
#define SEND_SensitivityAsRPM  // Kalman Param R in RPM
//#define SEND_PressureAsRPM // pressure in RPM Field


/***************************************************************************************/
/* The standard data being send to the trasnmitter                                     */
/***************************************************************************************/
#define SEND_Alt        // Send alt in the altitude field
#define SEND_VERT_SPEED // Send vertical speed (climbrate) as id 0x38
// #define SEND_VREF    // Voltage Reference as cell0
#define SEND_VFAS_NEW   // Voltage as VFAS 

#endif // FRSKY

/***************************************************************************************/
/* Analog IN Pins  => 6 free optional input to be transfered in the cell 1 to cell 6   */
/*                    fields. uncomment any line to enable its transmission            */
/*   JETIDUPLEX displays only the PIN_VoltageCell1 as "Spannung", others may be added  */
/***************************************************************************************/
// Define a voltagepin if you want to transmit the value to open9x:
#define PIN_VoltageCell1 0    //  Pin for measuring Voltage of cell 1 ( Analog In Pin! )
//#define PIN_VoltageCell2 1    //  Pin for measuring Voltage of cell 2 ( Analog In Pin! )
//#define PIN_VoltageCell3 2    //  Pin for measuring Voltage of cell 3 ( Analog In Pin! )
//#define PIN_VoltageCell4 3    //  Pin for measuring Voltage of cell 4 ( Analog In Pin! )
//#define PIN_VoltageCell5 6    //  Pin for measuring Voltage of cell 5 ( Analog In Pin! )
//#define PIN_VoltageCell6 7    //  Pin for measuring Voltage of cell 6 ( Analog In Pin! )

/***************************************************************************************/
/*   Red / Green duoLED on myVarioEX-Board for Fall / Climb Indication                 */
/*   to be used on Hennings Custom Vario Board instead of Arduino LED                  */
/***************************************************************************************/
#ifdef DUOLED
#define PIN_redLED 8
#define PIN_greenLED 9
#endif

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
const int I2CAdd=0x77;        // 0x77 The I2C Address of the MS5611 breakout board 
                              // (normally 0x76 or 0x77 configured on the MS5611 module 
                              // via a solder pin or fixed)

/***************************************************************************************/
/* sensitivity / data filtering / Kalman filter parameters                             */
/* The Kalman Filter is being used to remove noise from the MS5611 sensor data. It can */
/* be adjusted by changing the value Q (process Noise) and R (Sensor Noise).           */
/* Best pactise is to only change R, as both parametrers influence each other.         */
/* (less Q requires more R to compensate and vica versa)                               */
/***************************************************************************************/
#define KALMAN_Q 0.05 // 0.05 do not tamper with this value unless you are looking for adventure ;-)
#define KALMAN_R 300  // default:300   change this value if you want to adjust the default sensitivity!
                      // this will only be used if PIN_PPM is NOT defined
                      //  50 = fast but lot of errors (good for sensor testing...but quite a lot of noise)
                      // 300 = medium speed and less false indications (nice medium setting with 
                      //       a silence window (indoor)of -0.2 to +0.2)
                      // 1000 = conservative setting ;-)
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

/***************************************************************************************/
/* Parameters for the remote control option of the vario sensitivity                   */
/* These are parameters that can be used to fine tune the behaviour.                   */
/* ==>read the WiKi for details.                                                       */
/***************************************************************************************/
// These 2 values control on which ppm signal the programming mode will be initiated 
// a normal servo channel will be in the range of 1000..2000 microseconds
#define PPM_ProgrammingMode_minppm 900   // Pulse legth in microseconds 
#define PPM_ProgrammingMode_maxppm 1100  // Pulse legth in microseconds 

#define PPM_Range_min 981   // the toatal range min of the ppm input (Pulse legth in microseconds )
#define PPM_Range_max 1999  // the toatal range max of the ppm input (Pulse legth in microseconds )


#define PPM_ProgrammingMode_Seconds 30   // the programming mode lasts 30 seconds
// The KALMAN_R_MIN+MAX Parameters define the range in wehich you will be able to adjust 
// the sensitivity using your transmitter
#define KALMAN_R_MIN 50     // 1    the min value for KALMAN_R
#define KALMAN_R_MAX 1000  // 1000 the max value for KALMAN_R

/***************************************************************************************/
/* Calibration Offsets                                                                 */
/***************************************************************************************/
//const long PressureCalibrationOffset=-775 ; // pressure correction in 1/100 mbar to be added to the measured pressure
const long PressureCalibrationOffset=0 ; // pressure correction in 1/100 mbar to be added to the measured pressure
                                         // Calibration instruction will follow
const long TemperatureCalibrationOffset=-80 ; // Temperature correction in 1/10 of degree celsius

                                        

/*****************************************************************************************************/
/*****************************************************************************************************/
/*****************************************************************************************************/
/*****************************************************************************************************/
//                No changes below this line unless you know what you are doing                       /
/*****************************************************************************************************/
/*****************************************************************************************************/
/*****************************************************************************************************/
/*****************************************************************************************************/


#include <Wire.h>
#ifdef FRSKY
#include <SoftwareSerial.h>
#endif
#include <EEPROM.h> //Needed to access the eeprom read write functions

//#define DEBUG

/***************************************************************************************/
/* Other Debugging options                                                             */
/***************************************************************************************/
//#define PpmToPressure 1023 // If defined use the PPM SIgnal to adjust the pressure sensor, where the center of the 
                           // Pot will be the value of this define ( in Millibar)

//#define SimulateClimbRate 1 // Simulate a climbrate of <N> cm /s
// Software Serial is used including the Inverted Signal option ( the "true" in the line below ) 
// Pin PIN_SerialTX has to be connected to rx pin of the receiver
// we do not need the RX so we set it to 0
#ifdef FRSKY
  SoftwareSerial mySerial(0, PIN_SerialTX,true); // RX, TX
#endif

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

const int AverageValueCount=6; // the number of values that will be used to calculate a new verage pressure
const int calcIntervallMS = 25; //50; // the intervall for the calculation intervall (do not change!)
                                // the GetClimbRate() requires this to be a fixed value of 50ms currently
long avgPressure;

long pressureValues[AverageValueCount+1];
long pressureStart; // airpressure measured in setup() can be used to calculate total climb / relative altitude

unsigned int calibrationData[7]; // The factory calibration data of the ms5611
unsigned long millisStart,lastMillisCalc,lastMillisFrame1,lastMillisFrame2,lastMicrosLoop;
int Temp=0;
float alt=0,rawalt=0,lastAlt=0;
long climbRate=0;
int abweichung=0;

//*****************************************************************************************
//********* Additional vars for JETIDUPLEX PORT, some may be redundant ********************
//*****************************************************************************************
int16_t Height ;
int16_t DeltaHeight ;
int16_t HeightStart=0 ;
uint16_t i=0;
uint16_t FrameCnt=0 ;
int16_t LastVoltage=0;
int16_t LipoVoltage=0;
int16_t Relalt=0;           // Relative Altitude above ground
float Zeroalt=0;            // Reference for calculation of "Altitude above Ground"

#ifdef JETIDUPLEX
char ex[32];                // DUPLEX EX binary framebuffer
char JetiLine1[17];         // JETIBOX classic framebuffer Line 1
char JetiLine2[17];         // and Line 2
uint16_t ExFrameCnt=0;      // Counter for EX frame type selection / priorisation
unsigned char JetiKey=0;    // Pressed Key on Jetibox
unsigned char JetiKeyLast=0;
uint16_t JetiKeyCnt=0;       // Key Repeat Counter for Altitude Reset
uint8_t JetiVspeakMode=1;    // 1 = Mimikri Vspeak-Display
#endif // JETIDUPLEX

double kalman_q= KALMAN_Q; //process noise covariance
double kalman_r= KALMAN_R; //measurement noise covariance

boolean ppmProgMode=false;
unsigned long ppmProgModeMillis= PPM_ProgrammingMode_Seconds *1000;
boolean ledIsOn=false;
int ledcnt=0;
int countDownSeconds=65535;

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
#ifdef DEBUG
  Serial.begin(115200);
#endif
#ifdef KALMANDUMP undefine DEBUG
#endif

#ifdef DUOLED
  pinMode (PIN_greenLED, OUTPUT);
  pinMode (PIN_redLED, OUTPUT);
#endif

#ifdef JETIDUPLEX
  //------------ Setup Serial Port for DUPLEX EX output ----------------
  JetiUartInit();          // Requires JETIBOX.INO library...
  delay (500);
  for (i=i; i<40; i++)     // Display welcome message for 40 x 0,08 s on classic jetibox terminal
  {
    JetiSensor ("Jeti myEX Vario ", JetiVersionStr, 0, false);
    delay(80);

#ifdef DUOLED              // Flash DuoLEDs during Version display
    if ((i >> 1) & 1)
       { digitalWrite (PIN_greenLED, 0); digitalWrite (PIN_redLED, 1); }
    else
       { digitalWrite (PIN_greenLED, 1); digitalWrite (PIN_redLED, 0); }
#endif // DUOLED

  }
#endif // JETIDUPLEX

#ifdef FRSKY
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
#endif

#ifdef PIN_PPM   
  pinMode(PIN_PPM,INPUT); 
  digitalWrite(PIN_PPM,HIGH); // enable the internal pullup resistor
#endif

  //eepromIntWrite(0,0); // just for reset purposes...
  //Read Kalman_r from eeprom
#ifdef PPM_ProgrammingMode
  kalman_r=eepromIntRead(0);
  if ((kalman_r==65535)or (kalman_r==0)){
    // eeprom value is invalid, so write the default to the eeprom once
    kalman_r=KALMAN_R;
    eepromIntWrite(0,(unsigned int)kalman_r);
  }
#endif 

  // Setup the ms5611 Sensor and read the calibration Data
  setupSensor();
 
  // prefill the air pressure buffer
  for (int i = 0;i<AverageValueCount*2;i++){
    SavePressure(getPressure());
  }
  pressureStart= getAveragePress();
  alt=getAltitude(getPressure (),Temp+ TemperatureCalibrationOffset);
#ifdef FORCE_ABSOLUTE_ALT
  SendAlt(1);  // send initial height
  SendValue(0x00,int16_t(1)); //>> overwrite alt offset in open 9x in order to start with display of absolute altitude... 
  SendValue(0x30,(int16_t)(alt/100)+2); //>> overwrite min alt in open 9x
  SendValue(0x31,(int16_t)(alt/100)-2); //>> overwrite min alt in open 9x
  mySerial.write(0x5E); // End of Frame 1!
#endif

  Zeroalt = alt;                  // Store current altitude as zero for reference // Henning mod

#ifdef PPM_ProgrammingMode
  // on startup, check if the ppm signal is in the following range. ( for <n> seconds )
  if(ppmProgMode=CheckPPMProgMode()){
    SendGPSDist(uint16_t(1111));
    ledOn() ;delay(1000);
    SendGPSDist(uint16_t(kalman_r));
    ledOn();  delay(100);
    ledOff();  delay(100);
    ledOn();  delay(100);
    ledOff();  delay(100);
    ledOn();  delay(100);
    SendGPSDist(uint16_t(1111));
    ledOn();  delay(1000);
  }  
#endif
  millisStart=millis();
}

/********************************************************************************** Loop () */
void loop()
{
  long pressure;
#ifdef SimulateClimbRate
  // simulation of a defined climbRate for developing purposes
  Temp=210; //fake temperature
  pressure=101300; //fake pressure
  delay(10);
  alt= ((float)micros()/(float)1000-(float)millisStart) /1; //climbing!
#else
  // get the measurements
  pressure=getPressure();
  rawalt=getAltitude(pressure,Temp+ TemperatureCalibrationOffset);
  alt=kalman_update(rawalt);
#endif
  SaveClimbRate(alt);   

#ifdef PPM_ProgrammingMode
  if (ppmProgMode and millis()<(millisStart + ppmProgModeMillis)){
     //blink pattern in programming mode
     if (++ledcnt==10){
       ledcnt=0;
       if (ledIsOn)ledOff();else ledOn();
     }
     // Countdown in T1
     if(countDownSeconds> (   ((millisStart + ppmProgModeMillis)-millis())/1000)    ){
        countDownSeconds=  ((millisStart + ppmProgModeMillis)-millis())/1000;
         SendTemperature1(countDownSeconds*10);
     }
     
  }else if(ppmProgMode){
    // End Programming Mode!
    SaveToEEProm();
    ppmProgMode=false;
  }
#endif
 
  // ---------------------------------------------------- Frame 1 to send every 200ms 
  if( (lastMillisFrame1 + 200) <=millis()) 
  {
    lastMillisFrame1=millis(); 
    SavePressure(pressure);// Fill the pressure buffer
    avgPressure=getAveragePress(); 
    climbRate=GetClimbRate();
    
#ifdef LED_ClimbBlink 
    if (!ppmProgMode){
      if(climbRate >LED_ClimbBlink) ledOn();
      else ledOff();
    }
#endif

#ifdef DUOLED
   if (climbRate > LED_ClimbBlink) digitalWrite (PIN_greenLED, 0);   // green duoLED = Climbing
   else digitalWrite (PIN_greenLED, 1);

   if (climbRate < -LED_ClimbBlink) digitalWrite (PIN_redLED, 0);    // red duoLED = Sinking
   else digitalWrite (PIN_redLED, 1);
#endif

#ifdef DEBUG
    Serial.print(" Pressure:");    Serial.print(pressure,DEC);
    Serial.print(" AveragePressure:");    Serial.print((float)avgPressure/100,DEC);
    Serial.print(" Altitude:");      Serial.print(alt/100,DEC);
    Serial.print(" ClimbRate:");      Serial.print(climbRate,DEC);
    Serial.print(" K_R:");    Serial.print(kalman_r);
    Serial.print(" Temp:");    Serial.print(Temp,DEC);
    Serial.print(" VCC:"); Serial.print(readVccMv());
    Serial.println();
#endif

#ifdef JETIDUPLEX
/******************************************************************/
/*   This is the main JETIDUPLEX output loop for both protocols,  */
/*   Jetibox classic (Ascii) as well as Jeti Duplex EX (binary)   */
/*   According to Jeti it should run at 10Hz but 5Hz works also   */
/******************************************************************/
    Relalt = (int16_t)(alt - Zeroalt);                               // relative altitude above ground

    // Write 2 lines to the Jetibox Classic Terminal
    if (JetiVspeakMode == 1)                               // Display setup compatible to VSpeak-Vario
    {
      strcpy (JetiLine1, "myEX-Vario  0.0V");              // Line 1: Sensor name and Voltage [V]
      Dezipunkt_Zahl (JetiLine1, LipoVoltage/10, 10) ;
      strcpy (JetiLine2, "    312m  2.1m/s");              // Line 2: Altitude [m] and climb rate [m/s]
      Dezi_Integer (JetiLine2, Relalt/10, 1);
      Dezi_Integer99 (JetiLine2, (int16_t)climbRate/10, 8);
    }
    else
    {                                                      // my own display setup (with temperature)
      strcpy (JetiLine1, "  0.00V     2.0m");              // Line 1: Voltage and Altitude
      Dezipunkt_Zahl (JetiLine1, LipoVoltage/10, 0) ;
//    Dezipunkt_Zahl (JetiLine1, JetiKey, 0) ;             // Debug: Display keycode of Jetibox
      Dezi_Integer (JetiLine1, Relalt/10, 9);
      strcpy (JetiLine2, "  0.0\xDF\x43      m/s");        // Line 2: Temperature [°C] and climb rate [m/s]
      Dezi_Integer99 (JetiLine2, Temp + TemperatureCalibrationOffset, 0);
      Dezi_Integer99 (JetiLine2, (int16_t)climbRate/10, 8);
    }
    
    // Keyboard handling of JB (<L> and <R> => Altitude Reset)
    if (JetiKey == 112) JetiVspeakMode = 1;               // Kbd <L> => enable Vspeak Vario display mode
    if (JetiKey == 224) JetiVspeakMode = 0;               // Kbd <R> => disable Vspeak Vario display mode
    if ((JetiKey == 96) && (JetiKeyLast == 96))           // if Left & Right are pressed continously
    {
      JetiKeyCnt++ ;                                      // count the strokes
      strcpy (JetiLine1, "Relat. Altitude ");
      strcpy (JetiLine2, "Reset           ");             // Line 2: Display "Altitude Reset Progress Bar"
      for (i=0; i<JetiKeyCnt; i++)
        JetiLine2[6+i] = '>';
      if (JetiKeyCnt > 9)                                 // if Progress bar reaches 10 => Reset Altitude
      {
         Zeroalt = alt;                                   // Reset relative altitude above ground
         JetiKeyCnt = 0;
      }
    }
    else
      JetiKeyCnt = 0;
            
    // Define DUPLEX EX data frame and send it, followed by the Jetibox Classic Textframe
    if (ExFrameCnt++ > 255)                                // Häufigkeit der EX Protokollheader (1..9: Header, sonst: EX-Datenpakete)
      ExFrameCnt = 1;                                      // gezielter Zähler-Überlauf
    JetiKeyLast = JetiKey;                                 // Store last pressed JB key
    JetiSendEx (ExFrameCnt) ;                              // Jeti EX-Frame to Jetibox
    JetiSensor(JetiLine1, JetiLine2, 0, false);            // Jeti Classic Textframe too (including JB keyboard check)
#endif // JETIDUPLEX

#ifdef FRSKY

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
    LipoVoltage = ReadVoltage(PIN_VoltageCell1) ;        // with JETIDUPLEX only Cell 1 is cupported currently
    SendCellVoltage(1,LipoVoltage);
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
// ********************************* The Temp 1 Field
   if(!ppmProgMode){
#ifdef SEND_TEMP_T1      
    SendTemperature1(Temp+ TemperatureCalibrationOffset); //internal MS5611 voltage as temperature T1
#endif
#ifdef SEND_PressureAsT1 // pressure in T1 Field
    SendTemperature1(avgPressure-SEND_PressureAsT1*10); //pressure in T1 Field
#endif
   }
// ********************************* The Temp 2 Field
#ifdef SEND_TEMP_T2      
    SendTemperature2(Temp+ TemperatureCalibrationOffset); //internal MS5611 voltage as temperature T1
#endif
#ifdef SEND_SensitivityAsT2 // Kalman Param R in Temp2
    SendTemperature2(uint16_t(kalman_r)*10);
#endif
#ifdef SEND_PressureAsT2 // pressure in T2 Field
    SendTemperature2(avgPressure-SEND_PressureAsT2*10); //pressure in T2 Field
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

#endif // FRSKY

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
/* CheckPPMProgMode => check if the ppm signal is in the  */
/*   defined range to enter the programming mode          */
/**********************************************************/
boolean CheckPPMProgMode(){
  unsigned int ppm;
  const byte secToWait=5;
  for (int i=0;i<=secToWait*10;i+=1){
    // check every 100ms for 5 seconds
    if(i%5==0)ledOn(); else ledOff();
    if(i%10==0){
      SendTemperature1(secToWait*10- i);
      SendGPSDist(uint16_t((secToWait*10-i)/10*1111));
    }
    ppm=ReadPPM();
    if(( ppm >PPM_ProgrammingMode_minppm)and (ppm <PPM_ProgrammingMode_maxppm))return true;
    alt=getAltitude(getPressure (),Temp+ TemperatureCalibrationOffset); // feed the filter...
    delay(100);
  }
  return false;
}

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
  ledOn() ;delay(1000);
  SendGPSDist(uint16_t(kalman_r));
  ledOff();delay(200);
  ledOn() ;delay(200);
  ledOff();delay(200);
  ledOn() ;delay(200);
  ledOff();delay(200);
  ledOn() ;delay(200);
  ledOff();delay(200);
  SendGPSDist(uint16_t(9999));
  ledOn() ;delay(1000);
  ledOff();
}

/**********************************************************/
/* ProcessPPMSignal => read PPM signal from receiver and  */
/*   use its value to adjust sensitivity                  */
/**********************************************************/
#ifdef PIN_PPM
void ProcessPPMSignal(){
   static boolean SignalPresent= false;
   unsigned long ppm= ReadPPM();
   
#ifdef PPM_ProgrammingMode 
   static unsigned int ppm_min=65535;
   static unsigned int ppm_max=0;
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
      kalman_r=map(ppm, ppm_min,ppm_max,KALMAN_R_MIN/10,KALMAN_R_MAX/10)*10; // map value and change stepping to 10

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
  Serial.print("PPM=");Serial.println(ppm);
#endif
  if ((ppm>2500)or (ppm<500))ppm=0; // no signal!
 return ppm;
}
#endif

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
/* SaveClimbRate => calculate the current climbRate                            */
/* GetClimbRate = > Retrieve the average climbRate from the buffer             */
/*******************************************************************************/
const byte ClimbRateQueueLength=10; // averaging buffer for the climbrate
float climbRateQueue[ClimbRateQueueLength];
void SaveClimbRate(float alti){
  long now=micros();
  static long lastMicrosVerticalSpeed=micros();
  long timecalc=now-lastMicrosVerticalSpeed; // the time passed since last CR Calculation
  static float lastAlti=alti;
  static byte cnt=0;
  lastMicrosVerticalSpeed=now;
  float CurrentClimbRate=(float)(alti-lastAlti)*((float)1000000/(float)timecalc);
  climbRateQueue[cnt]=CurrentClimbRate; // store the current ClimbRate
  cnt+=1;
  if (cnt ==ClimbRateQueueLength)cnt=0;
  lastAlti=alti;
}
/* retrieve the average climbRate */
float GetClimbRate(){
  float myClimbRate=0;
  for(int i=0;i<ClimbRateQueueLength;){
    myClimbRate+=climbRateQueue[i];
    i++;
  }
  myClimbRate/=ClimbRateQueueLength;
  return myClimbRate;  
}

/**********************************************************/
/* SendValue => send a value as frsky sensor hub data     */
/**********************************************************/
#ifdef FRSKY
void SendValue(uint8_t ID, uint16_t Value) {
  uint8_t tmp1 = Value & 0x00ff;
  uint8_t tmp2 = (Value & 0xff00)>>8;
  mySerial.write(0x5E);  mySerial.write(ID);
  if(tmp1 == 0x5E) { mySerial.write(0x5D);    mySerial.write(0x3E);  } 
  else if(tmp1 == 0x5D) {    mySerial.write(0x5D);    mySerial.write(0x3D);  } 
  else {    mySerial.write(tmp1);  }
  if(tmp2 == 0x5E) {    mySerial.write(0x5D);    mySerial.write(0x3E);  } 
  else if(tmp2 == 0x5D) {    mySerial.write(0x5D);    mySerial.write(0x3D);  } 
  else {    mySerial.write(tmp2);  }
  // mySerial.write(0x5E);
}
#endif

/**********************************************************/
/* ReadVoltage => read the voltage of an analog input pin */
/**********************************************************/
uint16_t ReadVoltage(int pin)
{
   // Convert the analog reading (which goes from 0 - 1023) to a millivolt value
#ifdef MYVARIOEX
   return uint16_t((float)analogRead(pin)*(float)(readVccMv()/2946.6));  // adapted to Vario board voltage divider
#else
   return uint16_t((float)analogRead(pin)*(float)(readVccMv()/1023.0));  // adapted to Rainers Arduino board
#endif // MYVARIOEX
}

#ifdef FRSKY

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
  // The initial altitude setting in open9x seems not to  work if we send 0m. it works fine though if we use 1m, so as a workaround we increase all alt values by 1.
  uint16_t Centimeter =  uint16_t(abs(altcm)%100);
//  int16_t Meter = int16_t((altcm-(long)Centimeter)/(long)100);
  long Meter;
  if (altcm >0){
    Meter = (altcm-(long)Centimeter);
  }else{
    Meter = -1*(abs(altcm)+(long)Centimeter);
  }
  Meter=Meter/100;
#ifdef FORCE_ABSOLUTE_ALT
  Meter-=1; // To compensate for a Offset of 1 at the beginning
#endif
#ifdef xDEBUG
  Serial.print("Meter:");
  Serial.print(Meter);
  Serial.print(",");
  Serial.println(Centimeter);
#endif

  SendValue(FRSKY_USERDATA_BARO_ALT_B, (int16_t)Meter);
  SendValue(FRSKY_USERDATA_BARO_ALT_A, Centimeter);
}
/****************************************************************/
/* SendGPSAlt - send the a value to the GPS altitude field      */
/****************************************************************/
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

#endif // FRSKY

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
// Just for debugging and testing purposes:
float altToPressure(float altcm){
  const float sea_press = 1013.25;
  altcm/=100;
  //Serial.print("ALT to PRESSURE:");Serial.print(altcm);
  //Serial.print("=");Serial.println(sea_press*pow((1-((0.0065*altcm)/(21+273.15))),5.257) );
  return sea_press*pow((1-((0.0065*altcm)/(21+273.15))),5.257) ;
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
  
  #ifdef PpmToPressure
    return getPPMPressure();
  #else
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

  return (P + PressureCalibrationOffset); 
  
  //return P+475;
  //return P+1762;

  #endif
}
#ifdef PpmToPressure
long getPPMPressure(){
   long ppm=(long)ReadPPM();
   delay(10);
   //static long ppm_min=PPM_Range_min;
   //static long ppm_max=PPM_Range_max;
   static long ppm_min=900;
   static long ppm_max=2000  ;
   Temp = 210;
   if (ppm>0){
//Serial.print("PPM=");Serial.print(ppm);
/*Serial.print("returnP=");Serial.print(mapFloat(ppm, ppm_min, ppm_max,
                ((float)PpmToPressure*(float)100)-(float)2000,
                101325
             ) );
Serial.println();*/
delay(50);
      return mapFloat(ppm, ppm_min, ppm_max,
                ((float)PpmToPressure*(float)100)-(float)40000,
               101325           ); 
   }
}
#endif

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
  Serial.print(rawalt);Serial.print(",");Serial.print(x);Serial.print(",");Serial.println(abweichung);
#endif
  return x;
}

/***********************************************************************************/
/* Jeti DUPLEX EX support routines following for ascii and binary frame conversion */
/***********************************************************************************/
#ifdef JETIDUPLEX
//* 1/10 einer Dezimalzahl mit Dezimalpunkt in String ausgeben
void Dezipunkt_Zahl (char *buf, uint16_t Zahl, uint8_t Pos)
{
  if (Zahl > 9999) Zahl = 9999;			// limit values to 999.9

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
  if (Zahl > 999) Zahl = 999;			// limit values to 99.9

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
    Zahl = 0xFFFF - Zahl;	        // in positiven Wert umrechnen
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
    Zahl = 0xFFFF - Zahl;	        // in positiven Wert umrechnen
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
  uint8_t cryptcode[4] = { 0x52,0x1C,0x6C,0x23 };

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

//* Eine Zeichenkette ohne Endekennung in Jeti-(EX)-Buffer ab Position "n" einkopieren
uint8_t jeticpy_n (char *s1, char *exbuf, uint8_t n)
{
  uint8_t i ;
  i = n ;
  do
    exbuf[i++] = *s1++ ;
  while (*s1) ;
  return(i) ;							  // Länge des EX-Buffers zurückgeben
}

//* Signed Integer in Duplex-EX-Format "int14" wandeln
uint16_t uint14 (long value)                            
{
  if (value < 0)
    return ((value >> 8) & 0x1F) | 0x80;       	// 6-Bit-HighByte (negativ) oberste 3 Bits löschen, 0x80=negative
  else
    return (value >> 8);                   	// 6-Bit-HighByte
}

//* Eines von verschiedenen EX-Frames erzeugen und abschicken
//* Bei geradem x keine Infodaten, um mindestens jedes 2.Mal ein EX-Datenpaket zu senden
void JetiSendEx (uint8_t x)
{
  uint8_t n=0;
  uint8_t i=0;

  switch (x)
  {
  case 0:                                                      // bei 0 gar kein EX-Paket schicken
    return;
  case 1:
    ex[2] = 0x00; ex[8] = 0x00;  				// 2Bit Type(0-3) 0x40=Data, 0x00=Text, Identifier (0 = Sensorname)
    ex[9] = (9<<3) | 1;				                // 5Bit der Beschreibung / 3Bit Länge der Einheit (SPC=Dummy)
    n = jeticpy_n ("myVarioEX ", ex, 10);
    break;
  case 3:
    ex[2] = 0x00; ex[8] = 0x01;  				// 2Bit Type(0-3) 0x40=Data, 0x00=Text, Identifier (1 = Kanalname)
    ex[9] = (7<<3) | 1;				                // 5Bit der Beschreibung / 3Bit Länge der Einheit
    n = jeticpy_n ("VoltageV", ex, 10);                         // ab Position 10 in den Buffer kopieren
    break;
  case 5:
    ex[2] = 0x00; ex[8] = 0x02;					// 2Bit Type(0-3) 0x40=Data, 0x00=Text, Identifier (2 = Kanalname)
    ex[9] = (8<<3) | 1;				                // 5Bit der Beschreibung / 3Bit Länge der Einheit
    n = jeticpy_n ("Sealevelm", ex, 10);                        // ab Position 10 in den Buffer kopieren
    break;
  case 7:
    ex[2] = 0x00; ex[8] = 0x03;					// 2Bit Type(0-3) 0x40=Data, 0x00=Text, Identifier (3 = Kanalname)    						
    ex[9] = (8<<3) | 1;				                // 5Bit der Beschreibung / 3Bit Länge der Einheit
    n = jeticpy_n ("Altitudem", ex, 10);                        // ab Position 10 in den Buffer kopieren
    break;
  case 9:
    ex[2] = 0x00; ex[8] = 0x04;					// 2Bit Type(0-3) 0x40=Data, 0x00=Text, Identifier (4 = Kanalname)    						// 
    ex[9] = (9<<3) | 2;				                // 5Bit der Beschreibung / 3Bit Länge der Einheit
    n = jeticpy_n ("Temperat.\xB0\x43", ex, 10);                // ab Position 10 in den Buffer kopieren (Einheit "°C")
    break;
  case 11:
    ex[2] = 0x00; ex[8] = 0x05;					// 2Bit Type(0-3) 0x40=Data, 0x00=Text, Identifier (5 = Kanalname)
    ex[9] = (9<<3) | 3 ;            			        // 5Bit der Beschreibung / 3Bit Länge der Einheit
    n = jeticpy_n ("Climbratem/s", ex, 10);                     // ab Position 10 in den Buffer kopieren
    break;

  default:
    ex[2] = 0x40;						// 2Bit Type(0-3) 0x40=Data, 0x00=Text
    n=8;						        // start position in buffer

    //* Send the values of all EX sensor channels
    ex[n++] = 0x11;					// Identifier = 1 (Kanal 1= Spannung) + Data Type 1 (int14_t)
    ex[n++] = (LipoVoltage & 0xFF);			// LowByte Spannung
    ex[n++] = ((LipoVoltage >> 8) & 0xFF) | 0x40 ;  	// HighByte Spannung, 0x40 = Komma-Position an vorvorletzter Stelle

    ex[n++] = 0x21;					// Identifier = 2 (Kanal 2= Höhe über NN) + Data Type 1 (int14_t)
    ex[n++] = ((uint16_t)alt/10) & 0xFF;      	  	// LowByte Höhe
    ex[n++] = uint14 ((uint16_t)alt/10) | 0x20 ;      	// 6-Bit-HighByte Höhe, 0x20 = Komma an vorletzter Stelle

    ex[n++] = 0x31;					// Identifier = 3 (Kanal 3= Höhe) + Data Type 1 (int14_t)
    ex[n++] = (Relalt/10) & 0xFF;                     	// LowByte Höhe
    ex[n++] = uint14 (Relalt/10) | 0x20 ;              // 6-Bit-HighByte Höhe, 0x20 = Komma an vorletzter Stelle

    ex[n++] = 0x41;				  	// Identifier = 4 (Kanal 4= Temp) + Data Type 1 (int14_t)
    ex[n++] = (Temp+TemperatureCalibrationOffset) & 0xFF;            // LowByte Temperatur
    ex[n++] = uint14 (Temp+TemperatureCalibrationOffset) | 0x20;     // HighByte Temperatur, 0x20 = Komma an vorletzter Stelle

    ex[n++] = 0x51;				        // Identifier = 5 (Kanal 5= Steigrate) + Data Type 1 (int14_t)
#ifdef CLIMB2DEC
// Display Climbrate with 2 decimals to jugde sensor noise / filter performance
    ex[n++] = (climbRate) & 0xFF;       		// LowByte Steigrate
    ex[n++] = uint14 (climbRate) | 0x40;           	// HighByte Steigrate, 0x40 = Komma an vorvorletzter Stelle
#else
// Display Climbrate with 1 decimal for standard vario operation
    ex[n++] = (climbRate/10) & 0xFF;     		// LowByte Steigrate
    ex[n++] = uint14 (climbRate/10) | 0x20;           	// HighByte Steigrate, 0x20 = Komma an vorletzter Stelle
#endif
  }                                                     // in "n" bleibt die genutzte Paketlänge stehen

  jeti_crypt (ex, n) ;		                       // complete and encrypt the EX frame 

  JetiKey = JetiGetChar();                             // read keyboard of Jetibox / DC16 transmitter, dont evaluate it up to now
  UCSR0B &= ~(1<<RXEN0);                               // disable RX (just to make sure)
  UCSR0B |= (1<<TXEN0);                                // enable TX 

  JetiTransmitByte (0x7E, false);                      // send EX frame header
  for (i=1; i<=n; i++)                                 // followed by EX data frame
      JetiTransmitByte (ex[i], true);
}
#endif // JETIDUPLEX

