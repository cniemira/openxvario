// openxvario http://code.google.com/p/openxvario/
// started by R.Schloßhan 

// GND and +5V from a free Servo Port of the FrSky receiver goes to the Arduino. 
// (Voltage will be measured and transfered to open9x)
// The ppm Signal from that servo plug to pin 2
// The MS5611 has to be connected this way:
// MS5611 SCLK => A5
// MS5611 SDA_I => A4
// MS5611 VCC => +5V (depending on the MS5611 Module  you use, you might have to use 3.3V 
//                    instead or add in a resistor or voltage regulator)
// MS5611 GnD => GND
 
#define PIN_SerialTX 4        // 2  the pin to transmit the serial data to the frsky telemetry enabled receiver
#define PIN_ClimbLed 13       // 13 the led used to indicate lift ( led lights up above +0.10m/s)

// Define a voltagepin if you want to transmit the value to open9x:
//#define PIN_VoltageCell1 0    //  Pin for measuring Voltage of cell 1 ( Analog In Pin! )
//#define PIN_VoltageCell2 1    //  Pin for measuring Voltage of cell 2 ( Analog In Pin! )
//#define PIN_VoltageCell3 2    //  Pin for measuring Voltage of cell 3 ( Analog In Pin! )
//#define PIN_VoltageCell4 3    //  Pin for measuring Voltage of cell 4 ( Analog In Pin! )
//#define PIN_VoltageCell5 6    //  Pin for measuring Voltage of cell 5 ( Analog In Pin! )
//#define PIN_VoltageCell6 7    //  Pin for measuring Voltage of cell 6 ( Analog In Pin! )

// Choose if the led should be used for indicationg lift.
#define LED_ClimbBlink 5  // Blink when climbRate > 0.05cm. The numeric value can be changed to a different cm value

#define FORCE_ABSOLUTE_ALT // If defined, the height offset in open9x will be resetted upon startup, which results 
                           // in an absolute height display in open9x . (You can still change to a relative display 
                           // by pressing [MENU] in the telem.screens
                           // If not defined, open9x will use the first transmitted altitude as an internal offset, 
                           // which results in an initial height of 0m

const int I2CAdd=0x77;     // 0x77 The I2C Address of the MS5611 breakout board ( normally 0x76 or 0x77 configured on the MS5611
                           // module via a solder pin or fixed ...)

//************************** Which data to send in which field *****************************/
// The standard fields:
#define SEND_Alt        // Send alt in the altitude field
#define SEND_VERT_SPEED // Send vertical speed (climbrate) as id 0x38
#define SEND_VREF       // Voltage Reference as cell0
#define SEND_VFAS_NEW   // Voltage as VFAS 
//************************** Which data to send in the T1 (temperature 1) Field? (choose only one) 
//#define SEND_TEMP_T1         // MS5611 temperature as Temp1
#define SEND_PressureAsT1 9000 // pressure in 1/10th of mBar in T1 Field subtracted by the number in the define statment. 
                               // e.g. 950mbar => 9500 -offset of 9000 => 500 in display

//************************** Which data to send in the DIST Field? (choose only one)
#define SEND_AltAsDIST 0   // 0 Altitude in DIST the numeric value (in cm) is an offset that will be subtracted from the actual height for higher display precision.
                           // e.g: Actual height is 456,78 Meters ( DIsplay in DIst would be 456 7) if we subtract 300 display will be 156 78
                           // This is due to the fact that the highest precission in this field will only be transmitted up to an altitude of 327.68 m
//#define SEND_KalmanRasDist // Kalman Param R in DIST
//#define SEND_PressureAsDIST // pressure in DIST field

//************************** Which data to send in the T2 (temperature 2) Field? (choose only one)
//#define SEND_TEMP_T2    // MS5611 temperature as Temp2
#define SEND_KalmanRasT2 // Kalman Param R in Temp2

//************************** Which data to send in the RPM Field? (choose only one) (unprecise field! resolution is in steps of 30RPM!)
//#define SEND_AltAsRPM      // Altitude in RPM ;-)
//#define SEND_KalmanRasRPM  // Kalman Param R in RPM
//#define SEND_PressureAsRPM // pressure in RPM Field

//************************** Sensitivity / Data Filtering / kalman Filter Parameters *********************/
// The Kalman Filter is being used to remove noise from the MS5611 sensor data. It can be adjusted by 
// changing the value Q (process Noise) and R (Sensor Noise). Best pactise is to only change R, as both
// parametrers influence each other. (less Q requires more R to compensate and vica versa)
#define KALMAN_Q 0.05 // 0.05 do not tamper with this value unless you are looking for adventure ;-)
#define KALMAN_R 110  // default:110   change this value if you want to adjust the sensitivity!
                      //  50 = fast but more errors (good for sensor testing...but quite a lot of noise)
                      // 100 = medium speed and less false indications (nice medium setting with 
                      //       a silence window (indoor)of -0.2 to +0.2)
                      // 200 = conservative setting ;-)
                      // 500 = still useable maybe for a slow flyer?
                      // 1000 and q=0.001 is it moving at all?? in stable airpressure you can measure 
                      //      the height of your furniture with this setting. but do not use it for flying ;-)
                      // .. everything inbetween in relation to these values.
                    
//************************** Analog Output of vertical speed (to A1 or A2 on receiver *********************/
// Do NOT use this if you have an RS232 Port on your receiver. (that one produces better results!)
// if ANALOG_CLIMB_RATE has been defined you can connect Arduino Pin 3 to a 1kohm resistor. 
// the other end of that resistor will be connected to to A1 or A2 of FrSky Receiver ( e.g. D8R-II ) 
// and to a a 22uF electrolyte Capacitor (whichs other end goes to ground) 
// This will filter the PWM Output to a usable voltage for the open9x Vario code.
//#define ANALOG_CLIMB_RATE   // If defined, the clib rate will be written as PWM Signal to the defined port
#define OutputClimbRateMin -3 // -3this values eqaly 0V voltage to the receiver
#define OutputClimbRateMax  3 // 3 this values eqaly 3.2V voltage to the receiver
#define PIN_AnalogClimbRate 3 // 3 the pin used to write the data to the frsky a1 or a2 pin (could be 3,5,6,9,10,11)

//***************************************** PPM Signal from receiver **************************************/
// if you have the ppm signal from one of your channeld on the receiver connected to your arduino,
// you can use the value of that channel to "remote control" your openXvario.
// just uncomment the following line and set the number to the pin of your arduino that you connect the signal to.
// then choose which parameter you want to control by uncommenting one of the options with their min/max values
#define PIN_PPM 2          // default: 2 the pin to read the PPM Signal on coming from the receiver.           

// This makes the most sense.. use the ppm signal to control theopenXvario sensitivity
#define PPM_TO_KALMAN_R    // PPM Signal will be used to control the KALMAN_R (Sensor Noise) parameter
#define KALMAN_R_MIN 1     // 1    the min value for KALMAN_R
#define KALMAN_R_MAX 1000  // 1000 the max value for KALMAN_R
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
#include <SoftwareSerial.h>

//#define DEBUG

// Software Serial is used including the Inverted Signal option ( the "true" in the line below ) 
// Pin PIN_SerialTX has to be connected to rx pin of the receiver
// we do not need the RX so we set it to 0
SoftwareSerial mySerial(0, PIN_SerialTX,true); // RX, TX


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
unsigned long lastMillisCalc,lastMillisVerticalSpeed,lastMillisFrame1,lastMillisFrame2;
int Temp=0;
float alt=0,rawalt=0;
long climbRate=0;
int abweichung=0;

double kalman_q= KALMAN_Q; //process noise covariance
double kalman_r= KALMAN_R; //measurement noise covariance

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

#ifdef PIN_PPM   
  pinMode(PIN_PPM,INPUT); 
#endif

  Wire.begin();
#ifdef KALMANDUMP define DEBUG
#endif
#ifdef DEBUG
  Serial.begin(115200);
#endif
#ifdef KALMANDUMP undefine DEBUG
#endif
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);

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
}

/********************************************************************************** Loop () */
void loop()
{
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
    if(climbRate >LED_ClimbBlink) ledOn();else ledOff();
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
  // ---------------------------------------------------- Frame 1 to send every 200ms 
  if( (lastMillisFrame1 + 200) <=millis()) 
  {
    lastMillisFrame1=millis(); 
    avgPressure=getAveragePress(); 
#ifdef DEBUG
    Serial.print(" Pressure:");    Serial.print(pressure,DEC);
    Serial.print(" AveragePressure:");    Serial.print(avgPressure,DEC);
    Serial.print(" Altitude:");      Serial.print(alt,DEC);
    Serial.print(" ClimbRate:");      Serial.print(climbRate,DEC);
    Serial.print(" Temp:");    Serial.print(Temp,DEC);
    Serial.print(" VCC:"); readVccMv();
    Serial.println();
#endif

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
#ifdef SEND_KalmanRasDist
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
#ifdef SEND_KalmanRasT2 // Kalman Param R in Temp2
    SendTemperature2(uint16_t(kalman_r)*10); //internal MS5611 voltage as temperature T1
#endif

// ********************************* The RPM Field   
#ifdef SEND_AltAsRPM     
    SendRPM(alt);
#endif
#ifdef SEND_PressureAsRPM
   SendRPM(uint16_t(avgPressure/10));
#endif
#ifdef SEND_KalmanRasRPM
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

    // Read the ppm Signal from receiver
#ifdef PIN_PPM   
    int ppm= pulseIn(PIN_PPM, HIGH, 20000); 
    #ifdef PPM_TO_KALMAN_R
      if (ppm>0)kalman_r=map(ppm, 981,1999,KALMAN_R_MIN,KALMAN_R_MAX);
      else kalman_r=KALMAN_R;
    #endif
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
  mySerial.write(0x5E);  mySerial.write(ID);
  if(tmp1 == 0x5E) { mySerial.write(0x5D);    mySerial.write(0x3E);  } 
  else if(tmp1 == 0x5D) {    mySerial.write(0x5D);    mySerial.write(0x3D);  } 
  else {    mySerial.write(tmp1);  }
  if(tmp2 == 0x5E) {    mySerial.write(0x5D);    mySerial.write(0x3E);  } 
  else if(tmp2 == 0x5D) {    mySerial.write(0x5D);    mySerial.write(0x3D);  } 
  else {    mySerial.write(tmp2);  }
  // mySerial.write(0x5E);
}
/**********************************************************/
/* ReadVoltage => read the voltage of an analog input pin */
/**********************************************************/
uint16_t ReadVoltage(int pin)
{
    // Convert the analog reading (which goes from 0 - 1023) to a millivolt value
   return uint16_t(    (float) analogRead(pin) * (float)(readVccMv() /1023.0)     );
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
  // The initial altitude setting in open9x seems not to  work if we send 0m. it works fine though if we use 1m, so as a workaround we increase all alt values by 1.
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
//  return long(((pow((sea_press / result), 1/5.257) - 1.0) * ( (temp/100) + 273.15)) / 0.0065 *100)+10000;
  return (((pow((sea_press / result), 1/5.257) - 1.0) * ( (temp/100) + 273.15)) / 0.0065 *100)+10000;
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

//  D1 = getData(0x46, 5); //OSR=2048 0.018 mbar precision
//  D2 = getData(0x56, 1);

//  D1 = getData(0x44, 3); //OSR=1024 0.027 mbar precision
//  D2 = getData(0x54, 1);

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
}


void ledOff()
{
  digitalWrite(PIN_ClimbLed,0);
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

