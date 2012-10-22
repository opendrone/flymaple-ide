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
  delay(10);
  
  //configure I2C port 1 (pins 5, 9) with no special option flags (second argument)
//  i2c_master_enable(I2C1, 0);
  
  // Some more required inits
//  motorInit();       //电机控制初始化 
//  capturePPMInit();  //捕获遥控器接收机PPM输入信号功能初始化   
//  initAcc();            //初始化加速度计
//  initGyro();           //初始化陀螺仪
//  bmp085Calibration();  //初始化气压高度计
//  compassInit(false);   //初始化罗盘
//  compassCalibrate(1);  //校准一次罗盘，gain为1.3Ga
//  commpassSetMode(0);  //设置为连续测量模式 
  
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
