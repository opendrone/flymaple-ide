// 陀螺仪 ITG3205 
#define GYRO 0x68 // 定义传感器地址,将AD0连接到GND口，传感器地址为二进制数11101000 (请参考你接口板的原理图)
#define G_SMPLRT_DIV 0x15  //采样率寄存器地址
#define G_DLPF_FS 0x16     //检测灵敏度及其低通滤波器设置
#define G_INT_CFG 0x17     //中断配置寄存器
#define G_PWR_MGM 0x3E     //电源管理寄存器

#define G_TO_READ 8 // x,y,z 每个轴2个字节，另外再加上2个字节的温度

// 陀螺仪误差修正的偏移量,在陀螺仪初始化的时候将零值读取回来保存进去 
int16 g_offx = 0;
int16 g_offy = 0;
int16 g_offz = 0;

////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void initGyro(void)             	     
//参数说明:  无                                        
//返回值:    无                                                               
//说明:      初始化ITG3205陀螺仪
///////////////////////////////////////////////////////////////////////////////////
void initGyro(void)
{
   ///////////////////////////////////////////////
   // ITG 3200
   // 电源管理设定：
   // 时钟选择 =内部振荡器
   // 无复位, 无睡眠模式
   // 无待机模式
   // 采样率 = 125Hz
   // 参数为+ / - 2000度/秒
   // 低通滤波器=5HZ
   // 没有中断
   ///////////////////////////////////////////////////
   
  ////////////////////FreeIMU 的定义//////////////////////////////
//  writeTo(GYRO, G_SMPLRT_DIV, 0x00);//分频系数为0，不分频，采样率为8KHZ
//  writeTo(GYRO, G_DLPF_FS, 0x18);//采样频率8KHZ，带宽256HZ
//  writeTo(GYRO, G_PWR_MGM, 0x01);  //PLL with X Gyro reference
  
  writeTo(GYRO, G_PWR_MGM, 0x00);  //Internal oscillator
  writeTo(GYRO, G_SMPLRT_DIV, 0x07); // Fsample = 1kHz / (7 + 1) = 125Hz, or 8ms per sample.ITG3205 datasheet page 24    
  writeTo(GYRO, G_DLPF_FS, 0x1E);   //陀螺仪测量量程 +/- 2000 dgrs/sec, 1KHz 采样率,Low Pass Filter Bandwidth 5HZ
  
  writeTo(GYRO, G_INT_CFG, 0x00);   //关闭所有中断  
}
////////////////////////////////////////////////////////////////////////////////////
//函数原型:  zeroCalibrateGyroscope(unsigned int totSamples, unsigned int sampleDelayMS)           	     
//参数说明:  totSamples : 采集陀螺仪数据次数
//           sampleDelayMS: 采集间隔时间
//返回值:    无                                                               
//说明:      读取ITG3205陀螺仪静止状态下的零值将这个值记录后
///////////////////////////////////////////////////////////////////////////////////
void zeroCalibrateGyroscope(uint16 totSamples, uint16 sampleDelayMS) 
{
   //////////////////////////////////////
   // 陀螺仪ITG- 3205的I2C
   // 寄存器：
   // temp MSB = 1B, temp LSB = 1C
   // x axis MSB = 1D, x axis LSB = 1E
   // y axis MSB = 1F, y axis LSB = 20
   // z axis MSB = 21, z axis LSB = 22
   /////////////////////////////////////
  uint8 regAddress = 0x1D; // x axis MSB
  int16 xyz[3]; 
  float tmpOffsets[] = {0,0,0};
  uint8 buff[6];

  for (uint16 i = 0;i < totSamples;i++)
  {
    delay(sampleDelayMS);
    readFrom(GYRO, regAddress, 6, buff); //读取陀螺仪ITG3200 XYZ轴的数据
    xyz[0]= (((int16)buff[0] << 8) | buff[1]);
    xyz[1] = (((int16)buff[2] << 8) | buff[3]);
    xyz[2] = (((int16)buff[4] << 8) | buff[5]);
    tmpOffsets[0] += xyz[0];
    tmpOffsets[1] += xyz[1];
    tmpOffsets[2] += xyz[2];  
  }
  g_offx = -tmpOffsets[0] / totSamples;
  g_offy = -tmpOffsets[1] / totSamples;
  g_offz = -tmpOffsets[2] / totSamples;
}

////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void getGyroscopeRaw(int16 * result)  	     
//参数说明:  * result : 陀螺仪数据指针                                      
//返回值:    无                                                               
//说明:      读取ITG3205陀螺仪原始数据 加上零点修正值
///////////////////////////////////////////////////////////////////////////////////
void getGyroscopeRaw(int16 * result)
{
   //////////////////////////////////////
   // 陀螺仪ITG- 3200的I2C
   // 寄存器：
   // temp MSB = 1B, temp LSB = 1C
   // x axis MSB = 1D, x axis LSB = 1E
   // y axis MSB = 1F, y axis LSB = 20
   // z axis MSB = 21, z axis LSB = 22
   /////////////////////////////////////

  uint8 regAddress = 0x1B;
  int16 temp, x, y, z;
  uint8 buff[G_TO_READ];

  readFrom(GYRO, regAddress, G_TO_READ, buff); //读取陀螺仪ITG3200的数据

  result[0] = (((int16)buff[2] << 8) | buff[3]) + g_offx;
  result[1] = (((int16)buff[4] << 8) | buff[5]) + g_offy;
  result[2] = (((int16)buff[6] << 8) | buff[7]) + g_offz;
  result[3] = ((int16)buff[0] << 8) | buff[1]; // 温度
}

////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void getGyroscopeData(int16 * result)           	     
//参数说明:  * result : 陀螺仪数据指针                                      
//返回值:    无                                                               
//说明:      读取ITG3205陀螺仪角速度， 单位  度每秒 º/s 
///////////////////////////////////////////////////////////////////////////////////
void getGyroscopeData(float * result)
{
  int16 buff[4];
  getGyroscopeRaw(&buff[0]);  //读取原始数据
  result[0] = buff[0] / 14.375; // ITG3205 14.375  LSB/(º/s) 
  result[1] = buff[1] / 14.375;
  result[2] = buff[2] / 14.375;
}
void GyroscopeTest(void)  //ITG3205加速度读取测试例子
{
    float gyro[3];
     initGyro();           //初始化陀螺仪
    delay(1000);
    zeroCalibrateGyroscope(128,5);  //零值校正，记录陀螺仪静止状态输出的值将这个值保存到偏移量，采集128次，采样周期5ms
    while(1)
    {
      getGyroscopeData(gyro);    //读取陀螺仪      
      SerialUSB.print("Xg=");
      SerialUSB.print(gyro[0]);
      SerialUSB.print("    ");
      SerialUSB.print("Yg=");  
      SerialUSB.print(gyro[1]);
      SerialUSB.print("    ");
      SerialUSB.print("Zg=");  
      SerialUSB.print(gyro[2]);
      SerialUSB.println("    ");
      delay(100);
    }
}


