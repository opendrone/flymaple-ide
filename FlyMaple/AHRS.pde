#include "AHRS.h"

#define twoKpDef  (2.0f * 0.1f) // 2 * proportional gain 两倍比例增益
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

// w,x,y,z - quoternion to rotate on yaw axis to 45 degree
#define YAW_45_DEGREE_FIX (float[4]){0.9238795325112867f, 0.0f, 0.0f, -0.3826834323650897f}

volatile int frameOrientation = FRAME_ORIENTATION_PLUS;

void toggleFrameOrientation()
{
  setFrameOrientation(frameOrientation == FRAME_ORIENTATION_X ? FRAME_ORIENTATION_PLUS : FRAME_ORIENTATION_X);
}

void setFrameOrientation(int orientation)
{
  frameOrientation = orientation;
  if (orientation == FRAME_ORIENTATION_PLUS)
    setBlink(0x00000001);
  else
    setBlink(0x00000101);
}

void qMult(float * qu1, float * qu2, int doNorm)
{
  float norm;
  float A, B, C, D, E, F, G, H;
  
  A = (qu1[0]+qu1[1])*(qu2[0]+qu2[1]);
  B = (qu1[3]-qu1[2])*(qu2[2]-qu2[3]);
  C = (qu1[1]-qu1[0])*(qu2[2]+qu2[3]);
  D = (qu1[2]+qu1[3])*(qu2[1]-qu2[0]);
  E = (qu1[1]+qu1[3])*(qu2[1]+qu2[2]);
  F = (qu1[1]-qu1[3])*(qu2[1]-qu2[2]);
  G = (qu1[0]+qu1[2])*(qu2[0]-qu2[3]);
  H = (qu1[0]-qu1[2])*(qu2[0]+qu2[3]);
  
  qu1[0] =  B + (-E - F + G + H) * 0.5;
  qu1[1] =  A - ( E + F + G + H) * 0.5;
  qu1[2] = -C + ( E - F + G - H) * 0.5;
  qu1[3] = -D + ( E - F - G + H) * 0.5;
  
  // Normalise resulted quaternion if needed
  if (doNorm) {
    norm = invSqrt(qu1[0] * qu1[0] + qu1[1] * qu1[1] + qu1[2] * qu1[2] + qu1[3] * qu1[3]);
    qu1[0] *= norm;
    qu1[1] *= norm;
    qu1[2] *= norm;
    qu1[3] *= norm;
  }
}

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


  integralFBx = 0.0f;
  integralFBy = 0.0f;
  integralFBz = 0.0f;

  lastUpdate = 0;
  now = 0;


  //configure I2C port 1 (pins 5, 9) with no special option flags (second argument)
  i2c_master_enable(I2C1, I2C_FAST_MODE);  //设置I2C1接口，主机模式

  delay(200);
  
  // Accelerometer start
  SerialUSB.println("Initializing the Accelerometer...");
  initAcc();            //初始化加速度计
  delay(200);
  SerialUSB.println("Initializing the Accelerometer... Done!");


  // Gyroscope start
  SerialUSB.println("Initializing the Gyroscope...");
  initGyro();           //初始化陀螺仪
  delay(200);
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
  compassSetMode(0);  //设置为连续测量模式  
}

////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void AHRSgetValues(float * values)      	     
//参数说明:  * values : AHRS数据指针                                      
//返回值:    无                                                               
//说明:      读取AHRS数据
///////////////////////////////////////////////////////////////////////////////////

// Kalman thanks to http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/
double kalman_q = 1;
double kalman_r = 32;
double kalman_p = 0;
kalman_state kalman_a_x = kalman_init(kalman_q,kalman_r,kalman_p,0);
kalman_state kalman_a_y = kalman_init(kalman_q,kalman_r,kalman_p,0);
kalman_state kalman_a_z = kalman_init(kalman_q,kalman_r,kalman_p,0);
kalman_state kalman_g_x = kalman_init(kalman_q,kalman_r,kalman_p,0);
kalman_state kalman_g_y = kalman_init(kalman_q,kalman_r,kalman_p,0);
kalman_state kalman_g_z = kalman_init(kalman_q,kalman_r,kalman_p,0);
kalman_state kalman_m_x = kalman_init(kalman_q,kalman_r,kalman_p,0);
kalman_state kalman_m_y = kalman_init(kalman_q,kalman_r,kalman_p,0);
kalman_state kalman_m_z = kalman_init(kalman_q,kalman_r,kalman_p,0);
void AHRSgetValues(float * values) 
{  
  float valG[3];
  getGyroscopeData(&valG[0]); //读取 XYZ轴陀螺仪 角速度
  // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
  values[0] = valG[0] * M_PI/180;
  values[1] = valG[1] * M_PI/180;
  values[2] = valG[2] * M_PI/180;
  int16 valA[3];
  getAccelerometerData(&valA[0]);  //读取 XYZ轴加速度原始数据，然后转换成浮点数
  values[3] = ((float) valA[0])/256;
  values[4] = ((float) valA[1])/256;
  values[5] = ((float) valA[2])/256;
  int16 valC[3];
  compassRead(&valC[0]);
  values[6] = ((float) valC[0]);
  values[7] = ((float) valC[1]);
  values[8] = ((float) valC[2]);
}

// apply a kalman filter to values
void AHRSgetFilteredValues(float * values)
{
  float unfiltered[9];
  AHRSgetValues(unfiltered);
  
  kalman_update(&kalman_g_x,unfiltered[0]);
  kalman_update(&kalman_g_y,unfiltered[1]);
  kalman_update(&kalman_g_z,unfiltered[2]);
  kalman_update(&kalman_a_x,unfiltered[3]);
  kalman_update(&kalman_a_y,unfiltered[4]);
  kalman_update(&kalman_a_z,unfiltered[5]);
  kalman_update(&kalman_m_x,unfiltered[6]);
  kalman_update(&kalman_m_y,unfiltered[7]);
  kalman_update(&kalman_m_z,unfiltered[8]);
  
  values[0] = kalman_g_x.x;
  values[1] = kalman_g_y.x;
  values[2] = kalman_g_z.x;
  values[3] = kalman_a_x.x;
  values[4] = kalman_a_y.x;
  values[5] = kalman_a_z.x;
  values[6] = kalman_m_x.x;
  values[7] = kalman_m_y.x;
  values[8] = kalman_m_z.x;
  
}


// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from Sebastian Madgwick filter which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
//=====================================================================================================
void AHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;        

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;	// apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } 
    else {
      integralFBx = 0.0f;	// prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
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


void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    AHRSupdateIMU(gx, gy, gz, ax, ay, az);
    return;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;     

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;   

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

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;	// apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } 
    else {
      integralFBx = 0.0f;	// prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
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
  AHRSgetFilteredValues(val);

  now = micros();
  sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
  lastUpdate = now;
  // 9DOF IMU
  AHRSupdate(val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8]);
  // 6DOF IMU
  //AHRSupdate(val[0], val[1], val[2], val[3], val[4], val[5], 0.0f, 0.0f, 0.0f);
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
  
  if (frameOrientation == FRAME_ORIENTATION_X)
    qMult(q, YAW_45_DEGREE_FIX, 0);
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

void YPR_Display(void)
{
  float angles[3];
  delay(5);
  while(1)
  {
    AHRSgetYawPitchRoll(angles);  
    SerialUSB.print(angles[0]);
    SerialUSB.print(" | ");  
    SerialUSB.print(angles[1]);
    SerialUSB.print(" | ");
    SerialUSB.println(angles[2]);  
    delay(100);   
  }
}

void Display_Raw(void)
{
  float data[6];
  delay(5);
  while(1)
  {  
    AHRSgetFilteredValues(data);
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
    SerialUSB.print(data[5]);
    SerialUSB.print(" | ");  
    SerialUSB.print(data[6]);
    SerialUSB.print(" | ");  
    SerialUSB.print(data[7]);
    SerialUSB.print(" | ");  
    SerialUSB.println(data[8]);
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




