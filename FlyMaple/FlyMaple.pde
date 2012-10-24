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


char str[512]; 


////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void setup()               	     
//参数说明:  无                                        
//返回值:    无                                                               
//说明:      FlyMaple板 初始化函数
///////////////////////////////////////////////////////////////////////////////////
void setup()
{
  SerialUSB.begin();

  // Initialize the AHRS
  SerialUSB.println("AHRS Initialization...");
  initAHRS();
  SerialUSB.println("AHRS Initialization... Done!");
}


////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void loop()            	     
//参数说明:  无                                        
//返回值:    无                                                               
//说明:      主函数，程序主循环
///////////////////////////////////////////////////////////////////////////////////
void loop()
{

  // The following method is used to get 3D position for processing. Ask Jose for more information.
  //AHRS_Cube();

  // Uncomment the following line to display Yaw, Pitch & Roll angles measured by the FlyMaple board
  //YPR_Display_Raw();  //uncomment to display raw values
  YPR_Display();

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

