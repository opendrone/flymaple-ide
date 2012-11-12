#include <stdio.h>
#include "wirish.h"
#include "i2c.h"
#include "kalman.h"

// This holds the output values that are supposed to be sent to the motors.
// The values can then be sent by using the motorControl() method. See MOTOR.pde for more information.
extern uint16 MotorData[6];  //Motor control register

// These 4 variables can hold the RF controller input values.
// See CapturePPM for more information.
extern volatile unsigned int chan1PPM;  //PPM capture value register
extern volatile unsigned int chan2PPM;
extern volatile unsigned int chan3PPM;
extern volatile unsigned int chan4PPM;

////////////////////////////////////////////////////////////////////////////////////
// Function prototype: void setup ()
// Parameter Description: None
// Return Value: None
// Description: FlyMaple board initialization function
///////////////////////////////////////////////////////////////////////////////////
void setup()
{
  initUI();
  
  SerialUSB.begin();

  // Initialize the AHRS
  SerialUSB.println("AHRS Initialization...");
  initAHRS();
  SerialUSB.println("AHRS Initialization... Done!");
}

////////////////////////////////////////////////////////////////////////////////////
// Function prototype: void loop ()
// Parameter Description: None
// Return Value: None
// Description: The main loop of the main function, the program
///////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // Uncomment the following to get all measured values after correction & filtering.
  //Display_Raw();

  // The following method is used to get 3D position for processing. Ask Jose for more information.
  AHRS_Cube();

  // Uncomment the following line to display Yaw, Pitch & Roll angles measured by the FlyMaple board
  //YPR_Display();

  // Uncomment the following line to display the values of RF controller input
  //capturePPMTest();

  // Uncomment the following to test the barometer (temperature, pressure, altitude). Seems to work
  //bmp085Test();

  // Uncomment the following to test the accelerometer. Works fine.
  //accelerometerTest();

  // Uncomment the following to test the gyroscope. Works fine.
  //GyroscopeTest();

  // Uncomment the following to test the compass. Seems to work.
  //compassTest();

}

