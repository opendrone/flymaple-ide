

////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void writeTo(uint8 DEVICE, uint8 address, uint8 val)             	     
//参数说明:  DEVICE: I2C设备地址
//           address:操作寄存器地址 
//           val:写入寄存器值
//返回值:    无                                                               
//说明:      通过I2C总线将val写入到对应地址寄存器中
///////////////////////////////////////////////////////////////////////////////////
void writeTo(uint8 DEVICE, uint8 address, uint8 val) 
{
  // all i2c transactions send and receive arrays of i2c_msg objects 
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
 uint8 msg_data[2];
  
  msg_data = {address,val};  //写两个数据，一个地址，一个值
  msgs[0].addr = DEVICE;
  msgs[0].flags = 0; // 写操作
  msgs[0].length = 2; //写两个数据
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C1, msgs, 1,0);  //
}

////////////////////////////////////////////////////////////////////////////////////
//函数原型:  void readFrom(uint8 DEVICE, uint8 address, uint8 num, uint8 *msg_data)              	     
//参数说明:  DEVICE: I2C设备地址
//           address:操作寄存器地址 
//           num:读取数量
//           *msg_data:读取数据存放指针
//返回值:    无                                                               
//说明:      通过I2C总线读取数据
///////////////////////////////////////////////////////////////////////////////////
void readFrom(uint8 DEVICE, uint8 address, uint8 num, uint8 *msg_data) 
{
  i2c_msg msgs[1]; 
  msg_data[0] = address;
  
  msgs[0].addr = DEVICE;
  msgs[0].flags = 0; //标志为0，是写操作
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C1, msgs, 1,0);
  
  msgs[0].addr = DEVICE;
  msgs[0].flags = I2C_MSG_READ; //读取
  msgs[0].length = num; // 读取字节数
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C1, msgs, 1,0);
}

void serialPrintFloatArr(float * arr, uint8 length) 
{
  for(uint8 i=0; i<length; i++)
  {
    serialFloatPrint(arr[i]);
    SerialUSB.print(",");
  }
}


void serialFloatPrint(float f) 
{
  uint8 *b = (uint8 *)&f;
  for(uint8 i=0; i<4; i++) 
  {
    
    uint8 b1 = (b[i] >> 4) & 0x0f;
    uint8 b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    SerialUSB.print(c1);
    SerialUSB.print(c2);
  }
}


void writeArr(void * varr, uint8 arr_length, uint8 type_bytes) 
{
  uint8 *arr = (uint8 *) varr;
  for(uint8 i=0; i<arr_length; i++) 
  {
    writeVar(&arr[i * type_bytes], type_bytes);
  }
}


// thanks to Francesco Ferrara and the Simplo project for the following code!
void writeVar(void *val, uint8 type_bytes)
{
  uint8 * addr=(uint8 *)(val);
  for(uint8 i=0; i<type_bytes; i++) 
  { 
    SerialUSB.write(addr[i]);
  }
}
