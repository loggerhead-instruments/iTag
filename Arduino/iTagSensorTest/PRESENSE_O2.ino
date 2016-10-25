
int addr = 0x48;
#define STATUS 0x01
#define CONTROL 0x00
#define SRATE 0x10
#define PHASE 0x11
#define AMPLITUDE 0x12
#define TEMPERATURE 0x13

uint16_t o2Temp;
uint8_t oStatus;
float phase;
float amplitude;

int o2Status(){
  oStatus = read8(STATUS);
  return oStatus;
}

float o2Temperature(){
    o2Temp = read16(TEMPERATURE);
    return (float) o2Temp / 10.0;
}

float o2Phase(){
   phase = read32(PHASE);
   return phase;
}

float o2Amplitude(){
  amplitude = read32(AMPLITUDE);
  return amplitude;
}


//// Generic I2C read register (single byte)
//uint8_t read8(uint8_t reg)
//{
//  Wire.beginTransmission(addr);
//  Wire.write(reg);
//  Wire.endTransmission();
//  Wire.beginTransmission(addr);
//  Wire.requestFrom(addr,(uint8_t)1);
//  uint8_t data = Wire.read();
//  Wire.endTransmission();
//  
//  return data;
//}
//
//uint16_t read16(uint8_t reg)
//{
//  uint16_t data = 0x0000;
//  
//  Wire.beginTransmission(addr);
//  Wire.write(reg);
//  Wire.endTransmission();
//  
//  Wire.beginTransmission(addr);
//  Wire.requestFrom(addr, (uint8_t)2); // request 2 bytes of data
//  data = Wire.read();
//  data |= (Wire.read() << 8);
//  Wire.endTransmission();
//
//  return data;
//}
//
float read32(uint8_t reg)
{
  float data = 0x0000;
  union u_tag {
    byte b[4];
    float data;
  } u;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.beginTransmission(addr);
  Wire.requestFrom(addr, (uint8_t)4); // request 4 bytes of data
  int i = 0;
  while(Wire.available()){
    u.b[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
  return u.data;
}


