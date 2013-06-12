#ifndef OXS_CONFIG_h
#define OXS_CONFIG_h
// openxvario http://code.google.com/p/openxvario/
// started by R.Schloßhan 


/***************************************************************************************/
/* Remote controlling the sensitivity using a servo port on your receiver              */
/* ==> choose 0 to 1 option! You need to have the PIN_PPM defined as well              */
/***************************************************************************************/
#define PPM_AllwaysUsed  // PPM Signal always be used to control the sensitivity
// Choose this if you want to use a dedicated channel just for 
// adjusting the sensitivity

/***************************************************************************************/
/* Other Configuration Options  => various different values to take influence on       */
/*                    various stuff. Normaly you do not have to change any of these    */
/***************************************************************************************/
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
//#define SEND_mAhAsDist
//#define SEND_VRefAsDist // send the internal measured voltage as DIST
//#define SEND_DividerVoltageAsDist // send the calculated voltage on the voltage divider as DIST

//**************** the FUEL field  (choose only one)**********************/
//#define SEND_SensitivityAsFuel // sensitivity in DIST
//#define SEND_PressureAsFuel   // pressure in DIST field
//#define SEND_mAhAsFuel         // send the absolute consumed mAh value as fuel.
//#define SEND_mAhPercentageAsFuel 4000  // send a percentage of used capacity in the fuel field
                                       // this can be used in openTX to display a Fuel bar graphic.
                                       // the numeric is the usable capacity of your battery in mAh 
                                       // That value is 100%, the transfered value will be the capacity left in %
                                       
#define SEND_VRefAsFuel // send the internal measured voltage as Fuel
//#define SEND_DividerVoltageAsFuel // send the calculated voltage on the voltage divider as Fuel

//************************** the T1 (temperature 1) Field? (choose only one) **********/
//#define SEND_TEMP_T1         // MS5611 temperature as Temp1
//#define SEND_PressureAsT1 9000 // pressure in 1/10th of mBar in T1 Field subtracted by
// the number in the define statment. 
// e.g. 950mbar => 9500 -offset of 9000 => 500 in display
#define SEND_MilliampsAsT1 // send the measured current as T1


//************************** the T2 (temperature 2) Field? (choose only one) **********/
//#define SEND_TEMP_T2    // MS5611 temperature as Temp2
#define SEND_SensitivityAsT2  // Kalman Param R in Temp2
//#define SEND_PressureAsT2 9000 // pressure in 1/10th of mBar in T1 Field subtracted by
// the number in the define statment. 
// e.g. 950mbar => 9500 -offset of 9000 => 500 in display
//#define SEND_MilliampsAsT2 // send the measured current as T2
//#define SEND_LoopTimeAsT2 // for debuging /developing purposes only
//************************** the RPM Field? (choose only one) *************************/
//(unprecise field! resolution is in steps of 30RPM!)
//#define SEND_AltAsRPM      // Altitude in RPM ;-)
//#define SEND_SensitivityAsRPM  // Kalman Param R in RPM
//#define SEND_PressureAsRPM // pressure in RPM Field

/***************************************************************************************/
/* Hardware settings=>Here you can change which pins will be used for which connection */
/*                    and the I2C address of the connected MS5611 module               */
/*                    Normaly you do not have to change any of these                   */
/***************************************************************************************/
#define PIN_SerialTX        4  // 4  the pin to transmit the serial data to the frsky telemetry enabled receiver

#define PIN_PPM 2              // default: 2 the pin to read the PPM Signal on coming from the receiver.           
                               // you can uncomment this line if you want to completly disable the remote control functionality
                               
#define PIN_CurrentSensor   2  // the Analog pin the optional current Sensor is connected to 
#define PIN_VOLTAGE_DIVIDER 3  // Optional! a voltage divider to measure the battery pack voltage > 5v (see below)

#define I2CAdd           0x77 // 0x77 The I2C Address of the MS5611 breakout board 
                               // (normally 0x76 or 0x77 configured on the MS5611 module 
                               // via a solder pin or fixed)
                               
#define PIN_PushButton     10  // an optional push button to control the oXv          
                               // comment out to cpmpletly disable button code

#define PIN_LED            13  // The Signal LED (default=13=onboard LED)

/***************************************************************************************/
/* sensitivity / data filtering / Kalman filter parameters                             */
/* The Kalman Filter is being used to remove noise from the MS5611 sensor data. It can */
/* be adjusted by changing the value Q (process Noise)                                 */
/***************************************************************************************/
#define KALMAN_R 300  
//#define KALMAN_R 250  // default:300   change this value if you want to adjust the default sensitivity!
                      // this will only be used if PIN_PPM is NOT defined
                      //  50 = fast but lot of errors (good for sensor testing...but quite a lot of noise)
                      // 300 = medium speed and less false indications (nice medium setting with 
                      //       a silence window (indoor)of -0.2 to +0.2)
                      // 1000 = conservative setting ;-)
                      // .. everything inbetween in relation to these values.

/***************************************************************************************/
/* Optional Feature Current Mesaurement                                                */
/* Uncomment the #define sendCurrent toenable this feature.                            */
/***************************************************************************************/
#define SendCurrent  // Uncomment to enable a connected Current Sensor

//#define IdleMillivolts 2500        // Specify the mV value your sensor outputs for 5V supply bvoltage at 0A current flowing
//#define MillivoltsPerAmpere 185    // Specify the mv increase you get per 1Ampere current flowing
#define IdleMillivolts 491
#define MillivoltsPerAmpere 133

/***************************************************************************************/
/* Voltage Divider Setup - Measure your battery VOltage > 5V with your oXv             */
/* you can add an optional voltage divider to the oXv.                                 */
/* its only 2 resistors. One from an Analog pin defined above with the                 */
/* #define PIN_VOLTAGE_DIVIDER                                                         */
/* above. The first resistor goes from the analog pin (Default:A3) to GND.             */
/* The second resistor goes from the analog pin to the battery pack you want to measure*/
/***************************************************************************************/
//#define RESISTOR_ANALOG_TO_GND 3900        // 33kOhm resistor  Maximum allowed Voltage allowed on Voltage Divider=20.38
//#define RESISTOR_ANALOG_TO_BATTERY 12000   // 56kOhm resistor  Maximum allowed Voltage allowed on Voltage Divider=20.38

//#define RESISTOR_ANALOG_TO_GND 33000        // 33kOhm resistor Maximum allowed Voltage allowed on Voltage Divider=13.48
//#define RESISTOR_ANALOG_TO_BATTERY 56000   // 56kOhm resistor Maximum allowed Voltage allowed on Voltage Divider=13.48
#define RESISTOR_ANALOG_TO_GND 31480        // 33kOhm resistor Maximum allowed Voltage allowed on Voltage Divider=13.48
#define RESISTOR_ANALOG_TO_BATTERY 55500   // 56kOhm resistor Maximum allowed Voltage allowed on Voltage Divider=13.48
#define VOLTAGE_DIVIDER_CALIBRATION_OFFSET_MV -250 // you can define this one and set it to an apropiate millivolt value. 
                                                   // the value will be added to the calculated millivolt value
/***************************************************************************************/
/* Parameters for the remote control option of the vario sensitivity                   */
/* These are parameters that can be used to fine tune the behaviour.                   */
/* ==>read the WiKi for details.                                                       */
/***************************************************************************************/
#define PPM_Range_min 981   // the toatal range min of the ppm input (Pulse legth in microseconds )
#define PPM_Range_max 1999  // the toatal range max of the ppm input (Pulse legth in microseconds )

// The KALMAN_R_MIN+MAX Parameters define the range in wehich you will be able to adjust 
// the sensitivity using your transmitter
#define KALMAN_R_MIN 30     // 1    the min value for KALMAN_R
#define KALMAN_R_MAX 800  // 1000 the max value for KALMAN_R

/***************************************************************************************/
/* Optional Feature: Persistent memory.                                                */
/* If defined, some telemetry values will be stored in EEProm every 10 seconds.        */
/* These values will be restored every powerup. (e.g. mAh counted...                   */
/* By doing this we will get ongoing data counts even if the you turn off the modell   */
/* between flights                                                                     */
/***************************************************************************************/
//#define SAVE_TO_EEPROM      // Uncomment this to disable the persistent storage 
//#define SEND_MIN_MAX_ALT
//#define SEND_MAX_CURRENT

#endif

