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

/***************************************************************************************/
/* Which data to send in which telemetry field                                         */
/* ==> choose freely one value per Telemetry field                                     */
/***************************************************************************************/
//**************** the DIST field (GPS Distance) (choose only one)**********************/
//#define SEND_AltAsDIST 0   // 0 Altitude in DIST the numeric value (in cm) is an offset that will
// be subtracted from the actual height for higher display precision.
// e.g: Actual height is 456,78 Meters ( DIsplay in DIst would be 456 7)
// if we subtract 300 display will be 156 78
// This is due to the fact that the highest precission in this field
//  will only be transmitted up to an altitude of 327.68 m
//#define SEND_SensitivityAsDist // sensitivity in DIST
//#define SEND_PressureAsDIST    // pressure in DIST field
#define SEND_mAhAsDist

//**************** the FUEL field  (choose only one)**********************/
//#define SEND_SensitivityAsFuel // sensitivity in DIST
//#define SEND_PressureAsFuel   // pressure in DIST field
//#define SEND_mAhAsFuel         // send the absolute consumed mAh value as fuel.
#define SEND_mAhPercentageAsFuel 4000  // send apercentage of used capacity in the fuel field
// this can be used in openTX to display a Fuel bar graphic.
// the numeric is the usable capacity of your battery in mAh 
// That value is 100%, the transfered value will be the capacity left in %
//************************** the T1 (temperature 1) Field? (choose only one) **********/
#define SEND_TEMP_T1         // MS5611 temperature as Temp1
//#define SEND_PressureAsT1 9000 // pressure in 1/10th of mBar in T1 Field subtracted by
// the number in the define statment. 
// e.g. 950mbar => 9500 -offset of 9000 => 500 in display

//************************** the T2 (temperature 2) Field? (choose only one) **********/
//#define SEND_TEMP_T2    // MS5611 temperature as Temp2
#define SEND_SensitivityAsT2  // Kalman Param R in Temp2
//#define SEND_PressureAsT2 9000 // pressure in 1/10th of mBar in T1 Field subtracted by
// the number in the define statment. 
// e.g. 950mbar => 9500 -offset of 9000 => 500 in display

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
#define PIN_VoltageCell1 2    //  Pin for measuring Voltage of cell 1 ( Analog In Pin! )
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
#define PIN_SerialTX        4  // 4  the pin to transmit the serial data to the frsky telemetry enabled receiver
#define PIN_ClimbLed        13 // 13 the led used to indicate lift ( led lights up above +0.10m/s)
#define PIN_AnalogClimbRate 3  // 3 the pin used to optionally write the data to the frsky a1 or a2 pin (could be 3,5,6,9,10,11)
#define PIN_PPM 2              // default: 2 the pin to read the PPM Signal on coming from the receiver.           
// you can uncomment this line if you want to completly disable the remote control functionality
#define PIN_CurrentSensor   2  // the Analog pin the optional current Sensor is connected to 
const int I2CAdd=        0x77; // 0x77 The I2C Address of the MS5611 breakout board 
// (normally 0x76 or 0x77 configured on the MS5611 module 
// via a solder pin or fixed)
#define PIN_PushButton     10  // an optional push button to control the oXv          
// comment out to cpmpletly disable button code
#define PIN_LED            13  // The Signal LED (default=13=onboard LED)

/***************************************************************************************/
/* Optional Feature Current Mesaurement                                                */
/* Uncomment the #define sendCurrent toenable this feature.                            */
/***************************************************************************************/
#define SendCurrent  // Uncomment to enable a connected Current Sensor
//#define MinCurrentMilliamps -13510    // the lowest measured current (=0v input voltage)
//#define MaxCurrentMilliamps 13510     // the hioghest measured current (= input voltage= vRef)

//#define IdleMillivolts 2500
//#define MillivoltsPerAmpere 185
#define IdleMillivolts 490
#define MillivoltsPerAmpere 133


//#define ForceAbsolutCurrent  // If defined, all measured current values will be forced to be positive (e.g.: -4.5A => +4.5A)
//Here are some example values for standard ACS712 types of sensors
// Sensor Type    Min    Max     
// -5A  .. +5A   -13510  13510 
// -20A .. +20A  -25000	 25000
// -30A .. +30A  -37879	 37879


/***************************************************************************************/
/* Optional Feature: Persistent memory.                                                */
/* If defined, some telemetry values will be stored in EEProm every 10 seconds.        */
/* These values will be restored every powerup. (e.g. mAh counted...                   */
/* By doing this we will get ongoing data counts even if the you turn off the modell   */
/* between flights                                                                     */
/***************************************************************************************/
#define SAVE_TO_EEPROM


#endif

