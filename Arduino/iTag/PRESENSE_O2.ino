// Address  Name        Size(Bytes) Access    Reset Value
// 0x00     Control             1       R/W     0x15
// 0x01     Status              1       RO      0x00
// 0x10     Sampling Rate       1       R/W     0x02
// 0x11     Sensor Phase Shift  4       RO      0x00
// 0x12     Sensor amplitude    4       RO      0x00
// 0x13     CPU temperature     2       RO      0x00

//2.2. Control register
//The control register allows for device function and performance control. Following bits are defined:
//Bit0: MODE
//          mode 0 – trigger (read by trigger)
//          mode 1 - continuous with predefined sampling rate (default = 1) - trigger
// Bit1: TRG: Trigger
// Bit2: TEMP - Temperature sensor 0 - off, 1 – on (default = 1)
// Bit4: GAIN0 - LED gain Lo (default = 1)
// Bit5: GAIN1 - LED gain Hi (default = 0)
// Bit6: - reserved
// Bit7:- reserved

//2.3. Status register
//The status register shows the current device status. Following bits are defined: 
//Bit0: DRDY - set on new data (phase shift, amplitude, temperature) ready
//Bit1: SLEEP - set when in trigger mode and no measurement
//Bit2:- reserved
//Bit3: - reserved
//Bit4:- reserved
//Bit5: ERR0 - error bit 0 = set on amplitude too low
//Bit6: ERR1 reserved - error bit 1 = set on amplitude too high
//Bit7: ERR2 - error bit 2 = reserved for future 

//2.4. Sensor phase shift register
//The phase shift register contains sensor value that is proportional to measured oxygen
//concentration. The value (4 bytes) represent float variable according to IEEE754.
//NOTE: For more information how to calculate oxygen concentration from phase shift please contact PreSens GmbH directly.

//2.5. Sensor amplitude register
//The amplitude is the parameter used for sensor and device fitness monitoring. The amplitude value (4 bytes) represent float variable according to IEEE754. The amplitude should be always between 1000 and 20000 units. The lower is the amplitude the higher phase shift noise occurs.

//2.6. CPU temperature register
//The temperature shows current device temperature. The temperature value (2 bytes) represent
//integer variable. It must be divided by 10 to get readings in degrees Celsius. Low byte is sent first. Example.: 0xD7 0x00 = 0xD700 -> 0x00D7 = 215dec / 10 = 21.5°C

int addr = 0x48;
#define STATUS 0x01
#define CONTROL 0x00
#define SRATE 0x10
#define PHASE 0x11
#define AMPLITUDE 0x12
#define TEMPERATURE 0x13

uint16_t o2Temp;
uint8_t oStatus;

int o2Status(){
  oStatus = read8(STATUS);
  return oStatus;
}

float o2Temperature(){
    o2Temp = read16(TEMPERATURE);
    O2temperature = (float) o2Temp / 10.0;
    return O2temperature;
}

float o2Phase(){
   O2phase = read32(PHASE);
   return O2phase;
}

float o2Amplitude(){
  O2amplitude = read32(AMPLITUDE);
  return O2amplitude;
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


