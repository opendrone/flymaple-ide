

////////////////////////////////////////////////////////////////////////////////////
// Function prototype: void writeTo (uint8 DEVICE uint8 address, uint8 val)
// Parameter Description: DEVICE: I2C device address
// Address: Operation register address
// Val: write register values
// Return Value: None
// Description: val is written to the corresponding address register through the I2C bus
///////////////////////////////////////////////////////////////////////////////////
void writeTo(uint8 DEVICE, uint8 address, uint8 val) 
{
  // all i2c transactions send and receive arrays of i2c_msg objects 
  i2c_msg msgs[1]; // we dont do any bursting here, so we only need one i2c_msg object
 uint8 msg_data[2] = {address,val};
 
  msgs[0].addr = DEVICE;
  msgs[0].flags = 0; // write
  msgs[0].length = 2; //Write two data
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C1, msgs, 1,0);  //
}

////////////////////////////////////////////////////////////////////////////////////
// Function prototype: void readFrom (the uint8 DEVICE uint8 address, uint8 num, uint8 * msg_data)
// Parameter Description: DEVICE: I2C device address
// Address: Operation register address
// Num: the number of reads
// * Msg_data: read data stored pointer
// Return Value: None
// Description: I2C bus to read data
///////////////////////////////////////////////////////////////////////////////////
void readFrom(uint8 DEVICE, uint8 address, uint8 num, uint8 *msg_data) 
{
  i2c_msg msgs[1]; 
  msg_data[0] = address;
  
  msgs[0].addr = DEVICE;
  msgs[0].flags = 0; //write flag is 0
  msgs[0].length = 1; // just one byte for the address to read, 0x00
  msgs[0].data = msg_data;
  i2c_master_xfer(I2C1, msgs, 1,0);
  
  msgs[0].addr = DEVICE;
  msgs[0].flags = I2C_MSG_READ; //read
  msgs[0].length = num; // Read the number of bytes
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
