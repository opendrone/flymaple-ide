

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
