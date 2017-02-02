// Keller Pressure sensor

int kellerAddress = 0x40;
float pAt16384 = 0.0; //minimum pressure (bar)
float pAt49152 = 200.0; // maximum pressure (bar)

// read values of pressure sensor
int kellerInit(){
  if (printDiags) SerialUSB.println("Keller Init");
  byte temp[2];
  int i = 0;
   Wire.beginTransmission(kellerAddress);
   Wire.write((byte) 0);  
   Wire.endTransmission();
   delay(10);
   Wire.requestFrom(kellerAddress, 2); 
   delay(10);
   while(Wire.available())  
   { 
    temp[i] = Wire.read();  // receive one byte
    i++;
    if (printDiags) SerialUSB.println(temp[i-1]);
   } 
   return (i>0);  //return 1 if bytes read
}

void kellerConvert(){
   Wire.beginTransmission(kellerAddress);
   Wire.write(0xAC);  //Initiate pressure conversion takes >4 ms
   Wire.endTransmission();
}

void kellerRead(){
  int i = 0;
  byte temp[5];
  
  Wire.requestFrom(kellerAddress, 5);    // request 5 bytes from device; status; pressure(2); temperature (2)
  while(Wire.available())  
  { 
    temp[i] = Wire.read();  // receive one byte
    i++;
    if(printDiags) SerialUSB.println(temp[i-1]);
  } 
  float pressure = (float) ((uint16_t) temp[1] << 8 | (uint16_t) temp[2]);
  float milliBar = ((pressure - 16384.0) * (pAt49152 - pAt16384) / 32768.0 + pAt16384) * 1000.0;
  pressure_mbar = milliBar;
  depth = -(1010.0 - milliBar) / 1000.0;
  
  uint16_t tU16 = ((uint16_t) temp[3] << 8 | (uint16_t) temp[4]);
  temperature = (float) ((tU16 >> 4) - 24) * 0.05 - 50.0;
}

