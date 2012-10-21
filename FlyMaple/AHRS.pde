
#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain 两倍比例增益
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain    两倍积分增益

float iq0, iq1, iq2, iq3;    //计算变量
float exInt, eyInt, ezInt;  // scaled integral error  比例积分误差
volatile float twoKp;      // 2 * proportional gain (Kp) 两倍比例增益 变量
volatile float twoKi;      // 2 * integral gain (Ki)  两倍积分增益 变量
volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame  传感器四元数变量
volatile float integralFBx,  integralFBy, integralFBz;
uint32 lastUpdate, now; // sample period expressed in milliseconds  采样周期变量，单位毫秒
float sampleFreq; // half the sample period expressed in seconds
int16 startLoopTime;

////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void initAHRS(void)       	     
//参数说明:  无                                      
//返回值:    无                                                               
//说明:      初始化AHRS模块
///////////////////////////////////////////////////////////////////////////////////
void initAHRS(void)
{
  // 初始化四元数 initialize quaternion
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
  
  
  // Initialize PID parameters
  twoKp = twoKpDef;
  twoKi = twoKiDef;
  
  
  integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
  
  lastUpdate = 0;
  now = 0;
  
  
  //configure I2C port 1 (pins 5, 9) with no special option flags (second argument)
  i2c_master_enable(I2C1, I2C_FAST_MODE);  //设置I2C1接口，主机模式
  
  
  // Accelerometer start
  SerialUSB.println("Initializing the Accelerometer...");
  initAcc();            //初始化加速度计
  SerialUSB.println("Initializing the Accelerometer... Done!");


  // Gyroscope start
  SerialUSB.println("Initializing the Gyroscope...");
  initGyro();           //初始化陀螺仪
  delay(1000);
  SerialUSB.println("Initializing the Gyroscope... Done!");
  
  SerialUSB.println("Calibrating the Gyroscope...");
  zeroCalibrateGyroscope(128,5);  //零值校正，记录陀螺仪静止状态输出的值将这个值保存到偏移量，采集128次，采样周期5ms
  SerialUSB.println("Calibrating the Gyroscope... Done!");


  // Barometer start
  SerialUSB.println("Calibrating the Barometer...");
  bmp085Calibration();  //初始化气压高度计
  SerialUSB.println("Calibrating the Barometer... Done!");
  
  
  // Compass start
  SerialUSB.println("Initializing the Compass...");
  compassInit(false);   //初始化罗盘
  SerialUSB.println("Initializing the Compass... Done!");
  
  SerialUSB.println("Calibrating the Compass...");
  compassCalibrate(1);  //校准一次罗盘，gain为1.3Ga
  SerialUSB.println("Calibrating the Compass... Done!");
  commpassSetMode(0);  //设置为连续测量模式  
}

////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void AHRSgetRawValues(int * raw_values)          	     
//参数说明:  * result : AHRS原始数据指针                                      
//返回值:    无                                                               
//说明:      读取AHRS原始数据
///////////////////////////////////////////////////////////////////////////////////
void AHRSgetRawValues(int16 * raw_values)
{
  getAccelerometerData(&raw_values[0]);  //读取 XYZ轴加速度原始数据
  getGyroscopeRaw(&raw_values[3]);    //读取 XYZ轴及其温度，陀螺仪原始数据
  //magn.getValues(&raw_values[6], &raw_values[7], &raw_values[8]); //读取罗盘原始数据
}

////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void AHRSgetValues(float * values)      	     
//参数说明:  * values : AHRS数据指针                                      
//返回值:    无                                                               
//说明:      读取AHRS数据
///////////////////////////////////////////////////////////////////////////////////
void AHRSgetValues(float * values) 
{  
  int16 val[4];
  getAccelerometerData(&val[0]);  //读取 XYZ轴加速度原始数据，然后转换成浮点数
  values[0] = ((float) val[0]);
  values[1] = ((float) val[1]);
  values[2] = ((float) val[2]);
  getGyroscopeData(&values[3]); //读取 XYZ轴陀螺仪 角速度
}

//http://blog.csdn.net/zhengjiaxiang135/article/details/6673639
float invSqrt(float x) //invertor square 
{
    float xhalf = 0.5f * x;  
    int32 i = *(int32*)&x;        // get bits for floating value  
    i =  0x5f375a86 - (i>>1);    // gives initial guess  
    x = *(float*)&i;            // convert bits back to float  
    x = x * (1.5f - xhalf*x*x); // Newton step  
    return x; 
}

// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from Sebastian Madgwick filter which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
//=====================================================================================================
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;

  /*
  // Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
   if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
   float hx, hy, bx, bz;
   float halfwx, halfwy, halfwz;
   
   // Normalise magnetometer measurement
   recipNorm = invSqrt(mx * mx + my * my + mz * mz);
   mx *= recipNorm;
   my *= recipNorm;
   mz *= recipNorm;
   
   // Reference direction of Earth's magnetic field
   hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
   hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
   bx = sqrt(hx * hx + hy * hy);
   bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
   
   // Estimated direction of magnetic field
   halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
   halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
   halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
   
   // Error is sum of cross product between estimated direction and measured direction of field vectors
   halfex = (my * halfwz - mz * halfwy);
   halfey = (mz * halfwx - mx * halfwz);
   halfez = (mx * halfwy - my * halfwx);
   }
   */

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) 
  {
    float halfvx, halfvy, halfvz;

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += (ay * halfvz - az * halfvy);
    halfey += (az * halfvx - ax * halfvz);
    halfez += (ax * halfvy - ay * halfvx);
  }

  // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
  if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) 
  {
    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) 
    {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else 
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}


void AHRSgetQ(float * q) 
{
  float val[9];
  AHRSgetValues(val);

  /*
  DEBUG_PRINT(val[3] * M_PI/180);
   DEBUG_PRINT(val[4] * M_PI/180);
   DEBUG_PRINT(val[5] * M_PI/180);
   DEBUG_PRINT(val[0]);
   DEBUG_PRINT(val[1]);
   DEBUG_PRINT(val[2]);
   DEBUG_PRINT(val[6]);
   DEBUG_PRINT(val[7]);
   DEBUG_PRINT(val[8]);
   */


  now = micros();
  sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
  lastUpdate = now;
  // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
  //AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
  // use the call below when using a 6DOF IMU
  AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], 0, 0, 0);
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

// Returns the Euler angles in radians defined with the Aerospace sequence.
// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
void AHRSgetEuler(float * angles) 
{
  float q[4]; // quaternion
  AHRSgetQ(q);
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI; // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180/M_PI; // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180/M_PI; // phi
}


void AHRSgetAngles(float * angles) 
{
  float a[3]; //Euler
  AHRSgetEuler(a);

  angles[0] = a[0];
  angles[1] = a[1];
  angles[2] = a[2];

  if(angles[0] < 0)angles[0] += 360;
  if(angles[1] < 0)angles[1] += 360;
  if(angles[2] < 0)angles[2] += 360;
}

void AHRSgetYawPitchRoll(float * ypr) 
{
  float q[4]; // quaternion
  float gx, gy, gz; // estimated gravity direction
  AHRSgetQ(q);

  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI;
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz))  * 180/M_PI;
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz))  * 180/M_PI;
}

void sixDOF_Display(void)
{
  float angles[3]; // yaw pitch roll
  float data[6];
  delay(5);
  while(1)
  {
    AHRSgetEuler(angles);  
    SerialUSB.print(angles[0]);
    SerialUSB.print(" | ");  
    SerialUSB.print(angles[1]);
    SerialUSB.print(" | ");
    SerialUSB.println(angles[2]);  
    /*
	AHRSgetValues(data);
	SerialUSB.print(data[0]);
	SerialUSB.print(" | ");  
	SerialUSB.print(data[1]);
	SerialUSB.print(" | ");  
	SerialUSB.print(data[2]);
	SerialUSB.print(" | ");  
	SerialUSB.print(data[3]);
	SerialUSB.print(" | ");  
	SerialUSB.print(data[4]);
	SerialUSB.print(" | ");  
	SerialUSB.println(data[5]);
	*/  
	delay(100);   
  }
}

void AHRS_Cube(void)
{
  float q[4]; //hold q values
  delay(5); 
  while(1)
  {
    AHRSgetQ(q);
//    SerialUSB.print(q[0]);
//    SerialUSB.print(" | ");  
//    SerialUSB.print(q[1]);
//    SerialUSB.print(" | ");  
//    SerialUSB.print(q[2]);
//    SerialUSB.print(" | ");  
//    SerialUSB.print(q[3]);
//    SerialUSB.print(" | ");  
    serialPrintFloatArr(q,4);
    SerialUSB.println(""); //line break
    delay(5);
  }
}



