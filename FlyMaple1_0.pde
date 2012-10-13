#include <stdio.h>
#include "wirish.h"
#include "i2c.h"

extern uint16 MotorData[6];  //电机控制寄存器 

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
}
////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void loop()            	     
//参数说明:  无                                        
//返回值:    无                                                               
//说明:      主函数，程序主循环
///////////////////////////////////////////////////////////////////////////////////
void loop()
{
  AHRS_Cube();
}
