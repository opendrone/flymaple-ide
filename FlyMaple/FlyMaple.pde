#include <stdio.h>
#include "wirish.h"
#include "i2c.h"

// This holds the output values that are supposed to be sent to the motors.
// The values can then be sent by using the motorControl() method. See MOTOR.pde for more information.
extern uint16 MotorData[6];  //电机控制寄存器 

// These 4 variables can hold the RF controller input values.
// See CapturePPM for more information.
extern volatile unsigned int chan1PPM;  //PPM捕获值寄存器
extern volatile unsigned int chan2PPM;
extern volatile unsigned int chan3PPM;
extern volatile unsigned int chan4PPM;

void setup()
{
  SerialUSB.begin();
  
  // Initialize the AHRS
  SerialUSB.println("AHRS Initialization...");
  delay(10);
  initAHRS();
 
  
  // Some more required inits
//  motorInit();       //电机控制初始化 
//  capturePPMInit();  //捕获遥控器接收机PPM输入信号功能初始化   
  
  //some delay to let the sensors startup
  delay(200);
  SerialUSB.println("AHRS Initialization... Done!");
}


void loop()
{

  // The following method is used to get 3D position for another piece of software. Ask Jose for more information.
  //AHRS_Cube();

  // Uncomment the following line to display the values of Euler angles measured by the FlyMaple board
  //sixDOF_Display();

  // Uncomment the following line to display the values of RF controller input
  //capturePPMTest();

  // Uncomment the following to test the barometer (temperature, pressure, altitude)
  //bmp085Test();

  // Uncomment the following to test the accelerometer
  //accelerometerTest();

  // Uncomment the following to test the gyroscope
  //GyroscopeTest();

  // Uncomment the following to test the compass (not working for me yet)
  compassTest();
    
  delay(100); // to avoid crashing the IDE serial
}
