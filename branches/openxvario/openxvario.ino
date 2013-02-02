// openxvario http://code.google.com/p/openxvario/
// started by R.Schloßhan 

//
// if ANALOG_CLIMB_RATE has been defined you can connect Arduino Pin 9 to a 1kohm resistor. the other end of that resistor will be connected to to A1 or A2 of FrSky Receiver ( e.g. D8R-II ) and to a a 22uF electrolyte Capacitor ( whichs other end goes to ground) 
// This will filter the PWM Output  to a usable voltage for the open9x Vario code.
//
// GND and +5V from a free Servo Port of the FrSky receiver goes to the Arduino. ( Voltage will be measured and transfered to open9x )

// The MS5611 has to be connected this way:
// MS5611 SCLK => A5
// MS5611 SDA_I => A4
// MS5611 VCC => +5V ( depending on the MS5611 Module  you use, you might have to use 3.3V instead or add in a resistor or voltage regulator)
// MS5611 GnD => GND

// a little warning: if the Serial Lib will be used to generate debugging output, this will create interference with the pwm output if the ANALOG_CLIMB_RATE is being used
 
#define PIN_SerialTX 10       // the pin to transmit the serial data to the frsky telemetry enabled receiver
#define PIN_ClimbLed 13       // the led used to indicate lift for debugging purposes

// Define a voltagepin if you want to transmit the value to open9x:
//#define PIN_VoltageCell1 0    //  Pin for measuring Voltage of cell 1 ( Analog In Pin! )
//#define PIN_VoltageCell2 1    //  Pin for measuring Voltage of cell 2 ( Analog In Pin! )
//#define PIN_VoltageCell3 2    //  Pin for measuring Voltage of cell 3 ( Analog In Pin! )
//#define PIN_VoltageCell4 3    //  Pin for measuring Voltage of cell 4 ( Analog In Pin! )
//#define PIN_VoltageCell5 6    //  Pin for measuring Voltage of cell 5 ( Analog In Pin! )
//#define PIN_VoltageCell6 7    //  Pin for measuring Voltage of cell 6 ( Analog In Pin! )

// Choose how what the led will be used for (only 1 should be defined):
#define LED_BufferBlink       // Blink every time the averaging buffer complets one cycle
//#define LED_ClimbBlink        // Blink when climbRate > 0.15cm
#define FORCE_ABSOLUTE_ALT // If defined, the height offset in open9x will be resetted upon startup, which results in an absolute height diplay.
                              // If not defined, open9x will use the first transmitted altitude as an internal offset, which results in an initial height of "0m"

const int I2CAdd=0x77;                 // The I2C Address of the MS5611 breakout board ( normally 0x76 or 0x77 configured on the MS5611 module via a solder pin or fixed ...)
const int AverageValueCount=15;        // the number of values that will be used to calculate a new verage value

//#define ANALOG_CLIMB_RATE   // If defined, the clib rate will be written as PWM Signal to the defined port( Only use if you have to use a receiver missing the serial connection)
#define OutputClimbRateMin = -3; 
#define OutputClimbRateMax = 3;
#define PIN_AnalogClimbRate 9 // the pin used to write the data to the frsky a1 or a2 pin (could be 3,5,6,9,10,11)

#define SEND_VERT_SPEED// Send vertical speed (climbrate) as id 0x38
#define SEND_VREF      // Voltage Reference as cell0
#define SEND_TEMP_T1   // MS5611 temperature as Temp1
#define SEND_TEMP_T2   // MS5611 temperature as Temp2
#define SEND_AltAsRPM  // Altitude in RPM ;-)
#define SEND_AltAsDIST // Altitude in DIST


/*****************************************************************************************************/
//                No changes below this line unless you know what you are doing                       /
/*****************************************************************************************************/










#include <Wire.h>
#include <SoftwareSerial.h>

//#define DEBUG
// Software Serial is used including the Inverted Signal option ( the "true" in the line below ) Pin 11 has to be connected to rx pin of the receiver
SoftwareSerial mySerial(0, PIN_SerialTX,true); // RX, TX

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

#define FRSKY_USERDATA_VERT_SPEED   0x38 // open9x Vario Mode Only

#define FRSKY_USERDATA_VOLTAGE_B    0x3A
#define FRSKY_USERDATA_VOLTAGE_A    0x3B

const int calcIntervallMS = 100; // the intervall that has to be passed before a new output will be written ( just to the analog pin!! 

long pressure,avgPressure,lastPressure;
long pressureValues[AverageValueCount+1];
long pressureStart; // airpressure measured in setup() can be used to calculate total climb / relative altitude
int  temperatureValues[AverageValueCount+1];
//float lastClimbRate;

unsigned int calibrationData[7]; // The factory calibration data of the ms5611
unsigned long lastMillisCalc,lastMillisFrame1,lastMillisFrame2;
unsigned long BufferRoundTrip =0;
float Temp=0;
long alt=0;
//long lastAlt=0;
long climbRate;
int avgTemp;

/********************************************************************************** Setup() */
void setup()
{
#ifdef ANALOG_CLIMB_RATE
  analogWrite(PIN_AnalogClimbRate,255/5*1.6); // initialize the output pin 
#endif
#ifdef PIN_VoltageCell1
  pinMode(PIN_VoltageCell1,INPUT); 
#endif
#ifdef PIN_VoltageCell2   pinMode(PIN_VoltageCell2,INPUT); 
#endif
#ifdef PIN_VoltageCell3   pinMode(PIN_VoltageCell3,INPUT); 
#endif
#ifdef PIN_VoltageCell4   pinMode(PIN_VoltageCell4,INPUT); 
#endif
#ifdef PIN_VoltageCell5   pinMode(PIN_VoltageCell5,INPUT);
#endif
#ifdef PIN_VoltageCell6   pinMode(PIN_VoltageCell6,INPUT); 
#endif
  
  Wire.begin();
#ifdef DEBUG
  Serial.begin(9600);
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
  
#ifdef FORCE_ABSOLUTE_ALT
  SendAlt(1);  // send initial height
  SendValue(0x00,int16_t(1)); //>> overwrite alt offset in open 9x in order to start with display of absolute altitude... 
  mySerial.write(0x5E); // End of Frame 1!
#endif
}

/********************************************************************************** Loop () */
void loop()
{
  SavePressure(getPressure()); // Fill the pressure buffer
  
  // ------------------------------------------------------ Do the math every 100ms
  if( (lastMillisCalc + calcIntervallMS) <=millis()) 
  {
    lastMillisCalc=millis(); 
        alt=getAverageAltitude();
    climbRate=GetClimbRate(alt);
  }
  // ---------------------------------------------------- Frame 1 to send every 200ms 
  if( (lastMillisFrame1 + 200) <=millis()) 
  {
    lastMillisFrame1=millis(); 
    avgTemp=getAverageTemperature(0);
    avgPressure=getAveragePress(); 
    
#ifdef DEBUG
    Serial.print("Pressure:");     Serial.print(pressure,DEC);
    Serial.print(" AveragePressure:");    Serial.print(avgPressure,DEC);
    Serial.print(" Altitude:");      Serial.print(alt,DEC);
    Serial.print(" Temp:");    Serial.print(avgTemp,DEC);
    Serial.print(" VCC:"); readVccMv();
    Serial.println();
#endif

    SendAlt(alt);
#ifdef SEND_AltAsRPM     SendRPM(alt);
#endif

#ifdef SEND_TEMP_T1      SendTemperature1(avgTemp); //internal MS5611 voltage as temperature T1
#endif
#ifdef SEND_TEMP_T2      SendTemperature2(avgTemp); //internal MS5611 voltage as temperature T1
#endif
#ifdef SEND_VERT_SPEED   SendValue(FRSKY_USERDATA_VERT_SPEED,(int16_t)climbRate); // ClimbRate in open9x Vario mode
#endif
#ifdef SEND_VREF         SendCellVoltage(0,readVccMv()); // internal voltage as cell 0 which is not optimal. somebody will probably have to add another id for this or change the the way the vfas voltage gets displayed 
#endif   
#ifdef PIN_VoltageCell1 SendCellVoltage(1,ReadVoltage(PIN_VoltageCell1));
#endif
#ifdef PIN_VoltageCell2 SendCellVoltage(2,ReadVoltage(PIN_VoltageCell2));
#endif
#ifdef PIN_VoltageCell3 SendCellVoltage(3,ReadVoltage(PIN_VoltageCell3));
#endif
#ifdef PIN_VoltageCell4 SendCellVoltage(4,ReadVoltage(PIN_VoltageCell4));
#endif
#ifdef PIN_VoltageCell5 SendCellVoltage(5,ReadVoltage(PIN_VoltageCell5));
#endif
    // SendCurrent(133.5);    // example to send a current. 
#ifdef SEND_AltAsDIST
     if (alt <= 32768) SendGPSDist(uint16_t(alt));
     else if (alt < 327680) SendGPSDist(uint16_t(alt/(long)10));
       else SendGPSDist(uint16_t(alt/(long)100));// If altitude gets higher than 327,68m alt/10 will be transmitted till 3276,8m then alt/100
       #endif 
    mySerial.write(0x5E); // End of Frame 1!
    #ifdef LED_ClimbBlink 
  if(climbRate >15) ledOn();else ledOff();
#endif
#ifdef ANALOG_CLIMB_RATE   SendAnalogClimbRate(climbRate); //Write the Clib/SinkRate to the output Pin
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
/*********************************************************/
/* GetClimbRate => calculate the current climbRate       */
/* has to be invoked every 100ms for a queue length of 10 */
/*********************************************************/
long GetClimbRate(long alti){
  static long climbRateQueue[10];
  static long lastAlti=alti;
  static byte cnt=0;
  climbRate -= climbRateQueue[cnt];  // overwrite the oldest value in the queue
  climbRateQueue[cnt]=alti-lastAlti; // store the current height difference

  cnt+=1;
  if (cnt ==10)cnt=0;
  
  climbRate += alti-lastAlti; // add the current height difference
  lastAlti=alti;
  return(climbRate);
}
   
  
void SendValue(uint8_t ID, uint16_t Value) {
  uint8_t tmp1 = Value & 0x00ff;
  uint8_t tmp2 = (Value & 0xff00)>>8;
  mySerial.write(0x5E);
  mySerial.write(ID);
  if(tmp1 == 0x5E) {
    mySerial.write(0x5D);    mySerial.write(0x3E);
  } 
  else if(tmp1 == 0x5D) {
    mySerial.write(0x5D);    mySerial.write(0x3D);
  } 
  else {
    mySerial.write(tmp1);
  }
  if(tmp2 == 0x5E) {
    mySerial.write(0x5D);    mySerial.write(0x3E);
  } 
  else if(tmp2 == 0x5D) {
    mySerial.write(0x5D);    mySerial.write(0x3D);
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
 //dist=constrain(dist,0,65535);
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
#ifdef DEBUG
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
  altcm+=100;// The initial altitude setting in open9x seems not to  work if we send 0m. it works fine though if we use 1m, so as a workaround we increase all alt values by 1.
//  int16_t Meter = int16_t(alt/100);
//  uint16_t Centimeter =  uint16_t((abs(alt/100) -abs(Meter)) *100);
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
long getAltitude(long press, int temp) {
  float   result;
  temp/=10;
  result=(float)press/100;
  const float sea_press = 1013.25;
  return long(((pow((sea_press / result), 1/5.257) - 1.0) * ( (temp/100) + 273.15)) / 0.0065 *100)+10000;
}

/****************************************************************/
/* SavePressure - save a new pressure value to the buffer       */
/*                                                              */
/****************************************************************/
/* Rotating Buffer for calculating the Average Pressure */
void SavePressure(long currentPressure){
  static unsigned long lastpmillis=0;
  static int cnt =0;
  pressureValues[cnt++]=currentPressure;
  if(cnt>AverageValueCount){
      cnt=0;
      BufferRoundTrip=millis()-lastpmillis;
      lastpmillis=millis();
      ledOn();
  }else if(cnt==1)ledOff(); //TODO: ifdef einbauen
}
/****************************************************************/
/* getAveragePress - calculate average pressure based on all    */
/* entries in the pressure buffer                               */
/****************************************************************/
long getAveragePress()
{
  long result=0;
  for (int i=0;i<=AverageValueCount;i++) result +=pressureValues[i];
  return result/AverageValueCount;
}

/****************************************************************/
/* getAverageAltitude - calculate average altitude based on all */
/* entries in the pressure buffer                               */
/* Additionally the average climb Rate will be calculated.      */
/****************************************************************/
float getAverageAltitude()
{
  
  float result=0;
   for (int i=0;i<AverageValueCount;i++){
     result +=getAltitude(pressureValues[i],Temp);
   }
  return result/AverageValueCount;
}

/* Rotating Buffer for calculating the Average Temperature */
/* temperature will be stored as centidegree ( temp*10) */
int getAverageTemperature(int tempc )
{
  static int cnt =0;
  long result=0;
  
  if (tempc != 0 ){ // if not invoked with parameter 0 we add a new value to the average array
      temperatureValues[cnt]=tempc;
      cnt+=1;
      if(cnt==AverageValueCount)cnt=0;
  }
  
  for (int i=0;i<AverageValueCount;i++) result +=temperatureValues[i];
  return result/AverageValueCount;
}
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Read the MS56111 Values */
long getPressure()
{
  long D1, D2, dT, P;
  float TEMP;
  int64_t OFF, SENS;

  D1 = getData(0x48, 10);
  D2 = getData(0x50, 1);

  dT = D2 - ((long)calibrationData[5] << 8);
  TEMP = (2000 + (((int64_t)dT * (int64_t)calibrationData[6]) >> 23)) / (float)100;
  getAverageTemperature((int)(TEMP*10)); // temp 31.5C => 315
  OFF = ((unsigned long)calibrationData[2] << 16) + (((int64_t)calibrationData[4] * dT) >> 7);
  SENS = ((unsigned long)calibrationData[1] << 15) + (((int64_t)calibrationData[3] * dT) >> 8);
  P = (((D1 * SENS) >> 21) - OFF) >> 15;
 // Serial.print("P=");
 // Serial.println(P,DEC);
  return P;
}
// Read Raw Data from the MS5611
long getData(byte command, byte del)
{
  long result = 0;
  twiSendCommand(I2CAdd, command);
  delay(del);
  twiSendCommand(I2CAdd, 0x00);
  Wire.requestFrom(I2CAdd, 3);
  if(Wire.available()!=3)
  #ifdef DEBUG
    Serial.println("Error: raw data not available")
  #endif
  ;
  for (int i = 0; i <= 2; i++)
  {
    result = (result<<8) | Wire.read(); 
  }
  return result;
}

/* Setup the MS5611 and read the calibration Data */
void setupSensor()
{
  Serial.println("Setup Sensor Start.");
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

