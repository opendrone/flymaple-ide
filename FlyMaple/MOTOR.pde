
///////////////////////////////电机控制方法//////////////////////////////////
//KK四轴电机控制发出信号：
//周期：2.08MS
//在油门最小位置高电平：1.02ms
//中间位置：1.46ms
//最高位置：1.96MS
//当开电的时候给电调的信号是常低电平，当通过遥控器开启后，给电机控制的信号是1040US
//这时电机不转动
/////////////////////////////////////////////////////////////////////////////

///////////////////////电机控制端口定义/////////////////////////////////////////
#define	MOTOR0PIN  D28  //电机0控制端口 数字口D28
#define	MOTOR1PIN  D27  //电机1控制端口 数字口D27
#define	MOTOR2PIN  D11  //电机2控制端口 数字口D11
#define	MOTOR3PIN  D12  //电机3控制端口 数字口D12
#define MOTOR4PIN   D24  //电机3控制端口 数字口D24
#define MOTOR5PIN   D14  //电机3控制端口 数字口D14


//6个电机的控制信号量，值范围：1-999us 代表电机转速最小到最大速度
//当设置为0时将PWM占空比调整到0%，接口常低电平，当为1000时PWM占空比100%，接口常高电平。
uint16 MotorData[6] = {0,0,0,0,0,0};

////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void motorCcontrol(void)                 	     
//参数说明:  无                                        
//返回值:    无                                                               
//说明:      电机控制函数将电机控制值量计算出PWM控制信号
///////////////////////////////////////////////////////////////////////////////////
void motorControl(void)    
{
  uint16 PWMData[6] = {0,0,0,0,0,0};
  uint8 i;
  for(i=0;i<6;i++)
  {
    if(MotorData[i] <= 0)  PWMData[i] = 0;  //PWM占空比调整到0%，接口常低电平。
      else if(MotorData[i]  >= 1000) PWMData[i] = 50000;  //PWM占空比100%，接口常高电平。
        else  PWMData[i] = (1000 + MotorData[i])*24;
  }     
  //PWM最小1，最大499921，每24个数值对应1US，单值为0时为占空比为0%，当大于499920时为占空比100%
  pwmWrite(MOTOR0PIN,PWMData[0] );
  pwmWrite(MOTOR1PIN,PWMData[1] );
  pwmWrite(MOTOR2PIN,PWMData[2] );
  pwmWrite(MOTOR3PIN,PWMData[3] );
  pwmWrite(MOTOR4PIN,PWMData[4] );
  pwmWrite(MOTOR5PIN,PWMData[5] );
}

////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void motorInit(void)                                      	     
//参数说明:  无                                        
//返回值:    无                                                               
//说明:      初始化电机控制
///////////////////////////////////////////////////////////////////////////////////
void motorInit(void)   
{
  //将6个电机控制管脚都设置为推挽输出IO
  pinMode(MOTOR0PIN, PWM);
  pinMode(MOTOR1PIN, PWM);
  pinMode(MOTOR2PIN, PWM);
  pinMode(MOTOR3PIN, PWM);
  pinMode(MOTOR4PIN, PWM);
  pinMode(MOTOR5PIN, PWM);
  Timer3.setPeriod(2080);  //数字口D28，D27，D11，D12是Timer3的4个比较输出口，将Timer3的周期设置为2080us,电机更新频率为500HZ
  Timer4.setPeriod(2080);  //数字口D24，D14是Timer4的2个比较输出口，将Timer4的周期设置为2080us,电机更新频率为500HZ
  MotorData[0] = 0;
  MotorData[1] = 0;
  MotorData[2] = 0;
  MotorData[3] = 0;
  MotorData[4] = 0;
  MotorData[5] = 0; 
  motorControl();   //计算各个电机控制量之差,将这个值用于定时器产生中断改变相应电机脉冲高电平时间 
 }

 ////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void motorTest(void)
//参数说明:  无                                        
//返回值:    无                                                               
//说明:      测试电调电机
///////////////////////////////////////////////////////////////////////////////////
void motorTest(void)
{
   motorInit();       //电机控制初始化 
  ///////////////电调初始化设置行程，此时脉冲发出和电调电源一定要同时发生，设置最高行程///////////////////
  MotorData[0] = 999;  //首先将PWM设置为最高，设置电调最高行程数字
  MotorData[1] = 999;
  MotorData[2] = 999;
  MotorData[3] = 999;
  MotorData[4] = 999;
  MotorData[5] = 999; 
  motorControl();   //计算各个电机控制量之差,将这个值用于定时器产生中断改变相应电机脉冲高电平时间
  delay(3000);       //延迟至少3秒等待电调记住，需要根据实际的电调手册更改
  MotorData[0] = 10;  //将PWM设置为10，设置电调最低行程数字，解锁的时候只需要等于或者低于10就可以解锁
  MotorData[1] = 10;
  MotorData[2] = 10;
  MotorData[3] = 10;
  MotorData[4] = 10;
  MotorData[5] = 10; 
  motorControl();  //计算各个电机控制量之差,将这个值用于定时器产生中断改变相应电机脉冲高电平时间
  delay(3000);      //延迟至少3秒等待电调记住，需要根据实际的电调手册更改
  
  //////////////////////////解锁电调，发出低于设置的最小行程就可以这里小于/////////////////////////////
  MotorData[0] = 8;  //将PWM设置为10，设置电调最低行程数字，解锁的时候只需要等于或者低于10就可以解锁
  MotorData[1] = 8;
  MotorData[2] = 8;
  MotorData[3] = 8;
  MotorData[4] = 8;
  MotorData[5] = 8; 
  motorControl();  //计算各个电机控制量之差,将这个值用于定时器产生中断改变相应电机脉冲高电平时间
  delay(2000);      //延迟至少2秒等待电调记住，需要根据实际的电调手册更改
  while(1)
  {
    MotorData[0] = 100;  //控制6个电调使电机按照低速度运行
    MotorData[1] = 100;
    MotorData[2] = 100;
    MotorData[3] = 100;
    MotorData[4] = 100;
    MotorData[5] = 100; 
    motorControl();   //计算各个电机控制量之差,将这个值用于定时器产生中断改变相应电机脉冲高电平时间
    delay(3000);
    MotorData[0] = 500;  //控制6个电调使电机按照一半速度运行
    MotorData[1] = 500;
    MotorData[2] = 500;
    MotorData[3] = 500;
    MotorData[4] = 500;
    MotorData[5] = 500; 
    motorControl();   //计算各个电机控制量之差,将这个值用于定时器产生中断改变相应电机脉冲高电平时间
    delay(3000);   
  }  
}

