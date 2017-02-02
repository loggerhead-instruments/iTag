// Measurement Specialties 5803 Pressure Sensor

int pressAddress = 0x77;

int pressInit()
{
  int i = 0;
  byte buff[2];
  int bytesread;

  if (printDiags) SerialUSB.println("MS5803 Init");
  // Reset so PROM is loaded
  Wire.beginTransmission(pressAddress);
  Wire.write(0x1E);  //Reset Command
  bytesread = Wire.endTransmission();
  if (printDiags) SerialUSB.println("Pressure Reset 0=success");
  if (printDiags) SerialUSB.println(bytesread);  
    
  delay(5);  //reset needs at least 2.8 ms
  
  // Read and store calibration coefficients
  Wire.beginTransmission(pressAddress);
  Wire.write(0xA2);  //PROM Read Pressure Sensitivity
  Wire.endTransmission();
  
  bytesread = Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  PSENS = ((uint16_t) buff[0]<<8)| (uint16_t) buff[1]; //pressure sensitivity  MSB first
  if (printDiags) SerialUSB.println(PSENS);
    
  i=0;
  Wire.beginTransmission(pressAddress);
  Wire.write(0xA4);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  POFF = ((uint16_t) buff[0]<<8) | (uint16_t) buff[1];  //Pressure offset
  if (printDiags) SerialUSB.println(POFF);
 
  i=0;
  Wire.beginTransmission(pressAddress);
  Wire.write(0xA6);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  TCSENS = ((uint16_t)  buff[0] << 8) | (uint16_t) buff[1];  //
  if (printDiags) SerialUSB.println(TCSENS);

  i=0;
  Wire.beginTransmission(pressAddress);
  Wire.write(0xA8);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }

  TCOFF = ((uint16_t) buff[0]<<8) | (uint16_t) buff[1];  //Temp coefficient of pressure offset
  if (printDiags) SerialUSB.println(TCOFF);
  
  i=0;
  Wire.beginTransmission(pressAddress);
  Wire.write(0xAA);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }

  TREF = ((uint16_t) buff[0]<<8) | (uint16_t) buff[1];  //Ref temperature
  if (printDiags) SerialUSB.println(TREF);
  
  i=0;
  Wire.beginTransmission(pressAddress);
  Wire.write(0xAC);  //PROM
  Wire.endTransmission();
  Wire.requestFrom(pressAddress, 2);    // request 2 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }

  TEMPSENS = ((uint16_t) buff[0]<<8) | (uint16_t) buff[1];  //Temperature sensitivity coefficient  
  if (printDiags) SerialUSB.println(TEMPSENS);

  return (i>0);  // return 1 if value returned for calibration coefficient; otherwise 0
}

void updatePress()
{
    Wire.beginTransmission(pressAddress);
    Wire.write(0x48);  //Initiate pressure conversion OSR-4096
    Wire.endTransmission();

}

void updateTemp()
{
   Wire.beginTransmission(pressAddress);
   Wire.write(0x58); //Initiate Temperature conversion OSR=4096
   Wire.endTransmission();
}

void readPress()
{
  int i = 0;
  
  Wire.beginTransmission(pressAddress);
  Wire.write((byte) 0);  //ADC Read Command
  Wire.endTransmission();
  
  Wire.requestFrom(pressAddress, 3);    // request 3 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    Pbuff[i] = Wire.read();  // receive one byte
    i++;
  }
}

void readTemp()
{
  int i = 0;
 
  Wire.beginTransmission(pressAddress);
  Wire.write((byte) 0);  //ADC Read Command
  Wire.endTransmission();
  
  Wire.requestFrom(pressAddress, 3);    // request 3 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    Tbuff[i] = Wire.read();  // receive one byte
    i++;
  }
}

// PSENS; //pressure sensitivity C1
// POFF;  //Pressure offset C2
// TCSENS; //Temp coefficient of pressure sensitivity C3
// TCOFF; //Temp coefficient of pressure offset C4
// TREF;  //Ref temperature C5
// TEMPSENS; //Temperature sensitivity coefficient C6
void calcPressTemp(){
  uint32_t D1 = (uint32_t)((((uint32_t)Pbuff[0]<<16) | ((uint32_t)Pbuff[1]<<8) | ((uint32_t) Pbuff[2])));
  uint32_t D2 = (uint32_t)((((uint32_t)Tbuff[0]<<16) | ((uint32_t)Tbuff[1]<<8) | ((uint32_t) Tbuff[2])));
  
  float dT = (float) D2 - ((float) TREF * 256.0);
  float T16 = 2000.0 + (dT * (float) TEMPSENS / (float) 8388608.0);
  
  float OFF = ((float) POFF * 65536.0)  + (((float) TCOFF * dT) / 128.0);
  float SENS = ((float) PSENS * 32768.0) + ((dT * (float) TCSENS) / 256.0);

  pressure_mbar = ((float) D1 * SENS / 2097152.0 - OFF) / MS5803_constant / 100.0;  // mbar
  float mbar_per_m = 1113.77;
  depth = -(1010.0 -  pressure_mbar) / mbar_per_m;
  temperature = T16 / 100.0;

  if (printDiags){
    SerialUSB.print("MS5803 constant:"); SerialUSB.println(MS5803_constant);
    SerialUSB.print("D1:"); SerialUSB.println(D1);
    SerialUSB.print("D2:"); SerialUSB.println(D2);
    SerialUSB.print("dT:"); SerialUSB.println(dT);
    SerialUSB.print("T16:"); SerialUSB.println(T16);
    SerialUSB.print("OFF:"); SerialUSB.println(OFF);
    SerialUSB.print("SENS:"); SerialUSB.println(SENS);
    SerialUSB.print("press:"); SerialUSB.println(pressure_mbar);
    SerialUSB.print("depth:"); SerialUSB.println(depth);
    SerialUSB.print("temp:"); SerialUSB.println(temperature);
  }
}
