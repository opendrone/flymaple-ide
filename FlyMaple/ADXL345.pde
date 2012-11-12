
////////Acceleration sensor ADXL345 function/////////////////////////////
#define ACC (0x53)    //Defined ADXL345 address, ALT ADDRESS pin is grounded
#define A_TO_READ (6) //the number of bytes to read(each axis accounted for two-byte)
#define XL345_DEVID   0xE5 //ADXL345 ID register，需要注意芯片有一个地址选择线将AD0连接到GND口
// ADXL345 Control register
#define ADXLREG_TAP_AXES     0x2A
#define ADXLREG_BW_RATE      0x2C
#define ADXLREG_POWER_CTL    0x2D
#define ADXLREG_INT_ENABLE   0x2E
#define ADXLREG_DATA_FORMAT  0x31
#define ADXLREG_FIFO_CTL     0x38
#define ADXLREG_DUR          0x21

//ADXL345 Data register
#define ADXLREG_DEVID        0x00
#define ADXLREG_DATAX0       0x32
#define ADXLREG_DATAX1       0x33
#define ADXLREG_DATAY0       0x34
#define ADXLREG_DATAY1       0x35
#define ADXLREG_DATAZ0       0x36
#define ADXLREG_DATAZ1       0x37

// Accelerometer correction offset
int16 a_offset[3];
////////////////////////////////////////////////////////////////////////////////////
// Function prototype: void initAcc (void)
// Parameter Description: None
// Return Value: None
// Description: Initialize ADXL345 accelerometer
///////////////////////////////////////////////////////////////////////////////////
void initAcc(void) 
{
  //all i2c transactions send and receive arrays of i2c_msg objects 
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
  uint8 msg_data[2]= {
    0x00,0x00  };
  msgs[0].addr = ACC;
  msgs[0].flags = 0; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;    
  i2c_master_xfer(I2C1, msgs, 1,0);

  msgs[0].addr = ACC;
  msgs[0].flags = I2C_MSG_READ; 
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C1, msgs, 1,0);
  // now we check msg_data for our 0xE5 magic number 
  uint8 dev_id = msg_data[0];
  //SerialUSB.print("Read device ID from xl345: ");
  //SerialUSB.println(dev_id,HEX);

  if (dev_id != XL345_DEVID) 
  {
    SerialUSB.println("Error, incorrect xl345 devid!");
    SerialUSB.println("Halting program, hit reset...");
    waitForButtonPress(0);
  }
  //invoke ADXL345
  writeTo(ACC,ADXLREG_POWER_CTL,0x00);
  writeTo(ACC,ADXLREG_POWER_CTL,0x08); //Set accelerometer to measure mode
  
  delay(100);
  // Calculate offset
  float accumulator[] = {0,0,0};
  int num_samples = 30;
  for(int i = 0 ; i < num_samples ; i++) {
    short acc[3];
    getAccelerometerData(acc);
    accumulator[0] += acc[0];
    accumulator[1] += acc[1];
    accumulator[2] += acc[2];
    delay(100);
  }
  for(int i = 0 ; i < 3 ; i++) accumulator[i] /= num_samples;
  accumulator[2] -= 256; // 1g at 2mg/LSB more or less.
  for(int i = 0 ; i < 3 ; i++) a_offset[i] = accumulator[i];
  for(int i = 0 ; i < 3 ; i++) SerialUSB.println(accumulator[i]);
}

////////////////////////////////////////////////////////////////////////////////////
// Function prototype: void getAccelerometerData (int16 * result)
// Parameter Description: * result: read acceleration values ​​pointer
// Return Value: None
// Description: Read the the ADXL345 accelerometer original data
///////////////////////////////////////////////////////////////////////////////////
void getAccelerometerData(int16 * result) 
{
  int16 regAddress = ADXLREG_DATAX0;    //start reading byte
  uint8 buff[A_TO_READ];

  readFrom(ACC, regAddress, A_TO_READ, buff); //read ADXL345 data and store it in buffer

  //Readings for each axis with 10-bit resolution, ie 2 bytes.  
  //We want to convert two bytes into an int variable
  result[0] = ((((int16)buff[1]) << 8) | buff[0]) - a_offset[0];   
  result[1] = ((((int16)buff[3]) << 8) | buff[2]) - a_offset[1];
  result[2] = ((((int16)buff[5]) << 8) | buff[4]) - a_offset[2];
}


void accelerometerTest(void)//ADXL345 accelerometer reading test example
{
  int16 acc[3];
  while(1)
  {
    getAccelerometerData(acc);  //Read acceleration
    SerialUSB.print("Xacc=");
    SerialUSB.print(acc[0]);
    SerialUSB.print("    ");
    SerialUSB.print("Yacc=");  
    SerialUSB.print(acc[1]);
    SerialUSB.print("    ");
    SerialUSB.print("Zacc=");  
    SerialUSB.print(acc[2]);
    SerialUSB.println("    ");
    delay(100);
  }
}



