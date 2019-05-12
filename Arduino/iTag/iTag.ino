// iTag
// Board bootloader loaded using AtmelICE: Arduino Zero Programming Port
// Board programmed using USB via Arduino Zero Native USB port


#include <Wire.h>
  #include <SPI.h>
  #include <SdFat.h>
  #include <RTCZero.h>
  #include "amx32.h"
  #include "LowPower.h"
  
  SdFat sd;
  File dataFile;

// *** USER SETTINGS *********************//
int printDiags = 0;
float sensor_srate = 1.0;  
float imu_srate = 100.0;
int nbufsPerFile = 60; // number of seconds per file
// Select which MS5803 sensor is used on board to correctly calculate pressure in mBar
//#define MS5803_01bar
//#define MS5803_05bar
#define MS5803_30bar
//****************************************//

#ifdef MS5803_01bar
  #define MS5803_constant 32768.0
#endif
#ifdef MS5803_05bar
  #define MS5803_constant 32768.0
#endif
#ifdef MS5803_30bar
  #define MS5803_constant 8192.0
#endif

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

int bufCount = 0;
int ssCounter; // used to get different sample rates from one timer based on imu_srate
int file_count = 0;
int introPeriod = 1;
#define LEDSON 0  //change to 1 if want LEDs on all the time
#define LED_ON LOW
#define LED_OFF HIGH

// Header for amx files
DF_HEAD dfh;
SID_SPEC sidSpec[SID_MAX];
SID_REC sidRec[SID_MAX];
SENSOR sensor[SENSOR_MAX]; //structure to hold sensor specifications. e.g. MPU9250, MS5803, PA47LD, ISL29125

const int ledGreen = PIN_LED_TXL;
const int BURN = A5;
const int vSense = A4; 
const int VHF = PIN_LED_RXL;
const int O2POW = 4;
const int chipSelect = 10;
const int mpuInterrupt = 13;

byte toggleLED = 1;
int pressure_sensor;

// IMU
int FIFOpts;
#define IMUBUFFERSIZE 1800 // used this length because it is divisible by 18 bytes (e.g. A*3,M*3,G*3);
volatile byte imuBuffer[IMUBUFFERSIZE]; // buffer used to store IMU sensor data before writes in bytes
volatile byte time2writeIMU=0; 
volatile int IMUCounter = 0;
volatile int bufferposIMU = 0;
int halfbufIMU = IMUBUFFERSIZE/2;
volatile boolean firstwrittenIMU;
volatile uint8_t imuTempBuffer[20];
int16_t accel_x;
int16_t accel_y;
int16_t accel_z;
int16_t magnetom_x;
int16_t magnetom_y;
int16_t magnetom_z;
int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;
float gyro_temp;
int accel_scale = 0;

// RGB
int16_t islRed;
int16_t islBlue;
int16_t islGreen;

// Oxygen
float O2temperature;
float O2phase;
float O2amplitude;

// Pressure/Temp
byte Tbuff[3];
byte Pbuff[3];
volatile float depth, temperature, pressure_mbar;
boolean togglePress = 0; //flag to toggle conversion of pressure and temperature

//Pressure and temp calibration coefficients
uint16_t PSENS; //pressure sensitivity
uint16_t POFF;  //Pressure offset
uint16_t TCSENS; //Temp coefficient of pressure sensitivity
uint16_t TCOFF; //Temp coefficient of pressure offset
uint16_t TREF;  //Ref temperature
uint16_t TEMPSENS; //Temperature sensitivity coefficient

// Pressure, Temp double buffer
#define PTBUFFERSIZE 40
float PTbuffer[PTBUFFERSIZE];
byte time2writePT = 0; 
volatile byte bufferposPT=0;
byte halfbufPT = PTBUFFERSIZE/2;
boolean firstwrittenPT;

// RGB buffer
#define RGBBUFFERSIZE 120
byte RGBbuffer[RGBBUFFERSIZE];
byte time2writeRGB=0; 
int RGBCounter = 0;
volatile byte bufferposRGB=0;
byte halfbufRGB = RGBBUFFERSIZE/2;
boolean firstwrittenRGB;

// O2 buffer
#define O2BUFFERSIZE 60
float O2buffer[O2BUFFERSIZE];
byte time2writeO2=0; 
int O2Counter = 0;
volatile byte bufferposO2=0;
byte halfbufO2 = O2BUFFERSIZE/2;
boolean firstwrittenO2;

float depthThreshold = -2.0; // if > depthThreshold turn VHF on; depth is more negative for deeper

/* Create an rtc object */
RTCZero rtc;

volatile unsigned int irq_ovf_count = 0; // keep track of timer

/* Change these values to set the current initial time and date */
volatile byte second = 0;
volatile byte minute = 0;
volatile byte hour = 17;
volatile byte day = 1;
volatile byte month = 1;
volatile byte year = 17;

volatile byte burnSecond = 0;
volatile byte burnMinute = 10;
volatile byte burnHour = 17;
volatile byte burnDay = 1;
volatile byte burnMonth = 1;
volatile byte burnYear = 17;

unsigned int burnDurMin = 90; // how long burn wire is activated
unsigned int burnDelayMinutes = 0;
int burnFlag = 0; //0=no Burn; 1=burn burnDelayMinutes after start; 2=burn at specific time
int burnTriggered = 0; //set to 1 once burn happened so can use that to turn on VHF and go to sleep
long burnTime;
#define SECONDS_IN_MINUTE 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000
#define SECONDS_IN_LEAP 31622400

void setup() {
  dfh.Version = 20181019; //ULONG
  SerialUSB.begin(115200);
  pinMode(ledGreen, OUTPUT);
  digitalWrite(ledGreen,LED_ON);
  delay(10000);
  Wire.begin();
  Wire.setClock(400);  // set I2C clock to 400 kHz
  rtc.begin();

  SerialUSB.println("iTag");
  // see if the card is present and can be initialized:
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    SerialUSB.println("Card failed");
  }

  setupMenu();  
  sensorInit();

  if(printDiags) SerialUSB.println("Sensors initialized");
  setupDataStructures();
  if(printDiags) SerialUSB.println("Data structures initialized");

  resetGyroFIFO(); // reset Gyro FIFO
  FileInit();
  
  SerialUSB.println("Running"); 
  SerialUSB.println("USB disabled");
  SerialUSB.println("Ignore error");
  delay(500); // time to display
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
  startTimer((int) imu_srate); // start timer

  if (pressure_sensor==2) updateTemp();  // get first reading ready
  resetGyroFIFO(); // reset Gyro FIFO
}

byte newSecond;
byte oldSecond;
void loop() {
  //PM->SLEEP.reg |= PM_SLEEP_IDLE_CPU;  // Enable Idle0 mode - sleep CPU clock only
//  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
//  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; //disable USB
//  __WFI(); // enter sleep mode and wait for interrupt
//
//  // ... SLEEPING ...
//
//  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE; //enable USB
  
  // every second check depth and burn
  newSecond = rtc.getSeconds();
  if (newSecond != oldSecond) {
    //sampleSensors();
    if((depth > depthThreshold) | burnTriggered) {
      vhfOn();
    }
    else{
      vhfOff();
    }
    
    if(burnFlag){
      long curTime = RTCToUNIXTime(year, month, day, hour, minute, second);
      long diffMinutes = (curTime - burnTime) / 60;
      if((curTime > burnTime) & (diffMinutes < burnDurMin)) {
        digitalWrite(BURN, HIGH);
        burnTriggered = 1;
        }
      else{
        digitalWrite(BURN, LOW);
      }
    }
    oldSecond = newSecond;
    bufCount++;
  }

  // write IMU values to file
  if(time2writeIMU==1)
  {
    if(LEDSON | introPeriod) digitalWrite(ledGreen,LED_ON);
    if(dataFile.write((uint8_t *) & sidRec[3],sizeof(SID_REC))==-1) resetFunc();
    if(dataFile.write((uint8_t *) & imuBuffer[0], halfbufIMU)==-1) resetFunc(); 
    time2writeIMU = 0;
    if(LEDSON | introPeriod) digitalWrite(ledGreen,LED_OFF);
  }
  if(time2writeIMU==2)
  {
    if(dataFile.write((uint8_t *) & sidRec[3],sizeof(SID_REC))==-1) resetFunc();
    if(dataFile.write((uint8_t *) & imuBuffer[halfbufIMU], halfbufIMU)==-1) resetFunc();     
    time2writeIMU = 0;
  } 
  
  // write RGB values to file
  if(time2writeRGB==1){
    if(dataFile.write((uint8_t *)&sidRec[2],sizeof(SID_REC))==-1) resetFunc();
    if(dataFile.write((uint8_t *)&RGBbuffer[0], halfbufRGB)==-1) resetFunc(); 
    time2writeRGB = 0;
  }
  if(time2writeRGB==2){
    if(dataFile.write((uint8_t *)&sidRec[2],sizeof(SID_REC))==-1) resetFunc();
    if(dataFile.write((uint8_t *)&RGBbuffer[halfbufRGB], halfbufRGB)==-1) resetFunc();     
    time2writeRGB = 0;
  }
  
  // write Pressure & Temperature to file
  if(time2writePT==1){
    if(dataFile.write((uint8_t *)&sidRec[1],sizeof(SID_REC))==-1) resetFunc();
    if(dataFile.write((uint8_t *)&PTbuffer[0], halfbufPT * 4)==-1) resetFunc(); 
    time2writePT = 0;
  }
  if(time2writePT==2){
    if(dataFile.write((uint8_t *)&sidRec[1],sizeof(SID_REC))==-1) resetFunc();
    if(dataFile.write((uint8_t *)&PTbuffer[halfbufPT], halfbufPT * 4)==-1) resetFunc();     
    time2writePT = 0;
  }   

  // write RGB values to file
  if(time2writeRGB==1){
    if(dataFile.write((uint8_t *)&sidRec[2],sizeof(SID_REC))==-1) resetFunc();
    if(dataFile.write((uint8_t *)&RGBbuffer[0], halfbufRGB)==-1) resetFunc(); 
    time2writeRGB = 0;
  }
  if(time2writeRGB==2){
    if(dataFile.write((uint8_t *)&sidRec[2],sizeof(SID_REC))==-1) resetFunc();
    if(dataFile.write((uint8_t *)&RGBbuffer[halfbufRGB], halfbufRGB)==-1) resetFunc();     
    time2writeRGB = 0;
  } 

  // write O2 values to file
  if(time2writeO2==1){
    if(dataFile.write((uint8_t *)&sidRec[0],sizeof(SID_REC))==-1) resetFunc();
    if(dataFile.write((uint8_t *)&O2buffer[0], halfbufO2 * 4)==-1) resetFunc(); 
    time2writeO2 = 0;
  }
  if(time2writeO2==2){
    if(dataFile.write((uint8_t *)&sidRec[0],sizeof(SID_REC))==-1) resetFunc();
    if(dataFile.write((uint8_t *)&O2buffer[halfbufO2], halfbufO2 *4)==-1) resetFunc();     
    time2writeO2 = 0;
  }

  if(bufCount >= nbufsPerFile){       // time to stop?
    introPeriod = 0;  //LEDS on for first file
    digitalWrite(ledGreen,LED_OFF);
    dataFile.close();
    FileInit();  // make a new file
    bufCount = 0;
  }
}

void sensorInit(){
 // initialize and test sensors
  SerialUSB.println("Sensor Init");

  pinMode(ledGreen, OUTPUT);
  //REG_PORT_DIRSET0 = PORT_PA27;
  pinMode(BURN, OUTPUT);
  pinMode(VHF, OUTPUT);
  pinMode(vSense, INPUT);
  pinMode(O2POW, OUTPUT);

  // Digital IO
 // SerialUSB.println("Turning green LED on");
  digitalWrite(ledGreen, LED_ON);
  //REG_PORT_OUTSET0 = PORT_PA27;
  digitalWrite(BURN, HIGH);
  vhfOn();
  digitalWrite(O2POW, LOW);
  delay(100);
  digitalWrite(O2POW, HIGH); //O2 sensor needs to be on or ties up I2C bus
  delay(1000);
// battery voltage measurement
  SerialUSB.print("Battery: ");
  SerialUSB.println(readVoltage());
  
  delay(100); // this delay is needed to give sensors time
  
  digitalWrite(BURN, LOW);
  
  // RGB
  
  SerialUSB.print("RGBinit: ");
  SerialUSB.println(islInit()); 
  for(int n=0; n<4; n++){
      islRead();
      SerialUSB.print("R:"); SerialUSB.print(islRed); SerialUSB.print("\t");
      SerialUSB.print("G:"); SerialUSB.print(islGreen); SerialUSB.print("\t");
      SerialUSB.print("B:"); SerialUSB.println(islBlue);
      delay(200);
  }
  
// Pressure--auto identify which if any is present
  pressure_sensor = 0;
  // Keller
  if(kellerInit()) {
    pressure_sensor = 2;   // 2 if present
    SerialUSB.println("Keller Pressure Detected");
      for (int x=0; x<5; x++){
      kellerConvert();
      delay(100);
      kellerRead();
      SerialUSB.print("Depth: "); SerialUSB.print(depth);
      SerialUSB.print("  Temp: "); SerialUSB.println(temperature);
      delay(500);
    }
  }
  
  // Measurement Specialties
  if(pressInit()){
    pressure_sensor = 1;
    SerialUSB.print("MS5803 Pressure Detected: ");
    #ifdef MS5803_01bar
    SerialUSB.println("1 bar");
    #endif
    #ifdef MS5803_05bar
      SerialUSB.println("5 bar");
    #endif
    #ifdef MS5803_30bar
      SerialUSB.println("30 bar");
    #endif
    updatePress();
    delay(10);
    readPress();
    updateTemp();
    delay(10);
    readTemp();
    calcPressTemp();
    SerialUSB.print("Press (mBar): "); SerialUSB.print(pressure_mbar);
    SerialUSB.print("  Depth: "); SerialUSB.print(depth);
    SerialUSB.print("  Temp: "); SerialUSB.println(temperature);
  }

  // Presens O2
  SerialUSB.print("O2 status:");
  SerialUSB.println(o2Status());
  for (int n=0; n<2; n++){
    SerialUSB.print("O2 Temp:"); SerialUSB.print(o2Temperature());
    SerialUSB.print("  Phase:"); SerialUSB.print(o2Phase());
    SerialUSB.print("  Amplitude:"); SerialUSB.println(o2Amplitude());
  }

  // IMU
  SerialUSB.println(mpuInit(1));
    for(int i=0; i<10; i++){
      readImu();
      accel_x = (int16_t) ((int16_t)imuTempBuffer[0] << 8 | imuTempBuffer[1]);    
      accel_y = (int16_t) ((int16_t)imuTempBuffer[2] << 8 | imuTempBuffer[3]);   
      accel_z = (int16_t) ((int16_t)imuTempBuffer[4] << 8 | imuTempBuffer[5]);    
      
      gyro_temp = (int16_t) (((int16_t)imuTempBuffer[6]) << 8 | imuTempBuffer[7]);   
     
      gyro_x = (int16_t)  (((int16_t)imuTempBuffer[8] << 8) | imuTempBuffer[9]);   
      gyro_y = (int16_t)  (((int16_t)imuTempBuffer[10] << 8) | imuTempBuffer[11]); 
      gyro_z = (int16_t)  (((int16_t)imuTempBuffer[12] << 8) | imuTempBuffer[13]);   
      
      magnetom_x = (int16_t)  (((int16_t)imuTempBuffer[14] << 8) | imuTempBuffer[15]);   
      magnetom_y = (int16_t)  (((int16_t)imuTempBuffer[16] << 8) | imuTempBuffer[17]);   
      magnetom_z = (int16_t)  (((int16_t)imuTempBuffer[18] << 8) | imuTempBuffer[19]);  
  
      SerialUSB.print("a/g/m/t: \t");
      SerialUSB.print( accel_x); SerialUSB.print("\t");
      SerialUSB.print( accel_y); SerialUSB.print("\t");
      SerialUSB.print( accel_z); SerialUSB.print("\t");
      SerialUSB.print(gyro_x); SerialUSB.print("\t");
      SerialUSB.print(gyro_y); SerialUSB.print("\t");
      SerialUSB.print(gyro_z); SerialUSB.print("\t");
      SerialUSB.print(magnetom_x); SerialUSB.print("\t");
      SerialUSB.print(magnetom_y); SerialUSB.print("\t");
      SerialUSB.print(magnetom_z); SerialUSB.print("\t");
      SerialUSB.println(gyro_temp);
      delay(200);
    }


  //while(irq_ovf_count < 20);
  //stopTimer();

  digitalWrite(ledGreen, LED_OFF);
  vhfOff();

  SerialUSB.print("Burn flag:"); SerialUSB.println(burnFlag);
  SerialUSB.print("Burn UNIX time UTC:"); SerialUSB.println(burnTime);
  long curTime = RTCToUNIXTime(year, month, day, hour, minute, second);
  SerialUSB.print("Current UNIX time UTC:"); 
  SerialUSB.println(curTime);
  printTime();
  //REG_PORT_OUTCLR0 = PORT_PA27;
}

// increment PTbuffer position by 1 sample. This does not check for overflow, because collected at a slow rate
void incrementPTbufpos(){
  bufferposPT++;
   if(bufferposPT==PTBUFFERSIZE)
   {
     bufferposPT=0;
     time2writePT=2;  // set flag to write second half
     firstwrittenPT=0; 
   }
 
  if((bufferposPT>=halfbufPT) & !firstwrittenPT)  //at end of first buffer
  {
    time2writePT=1; 
    firstwrittenPT=1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

void incrementRGBbufpos(unsigned short val){
  RGBbuffer[bufferposRGB] = (uint8_t) val;
  bufferposRGB++;
  RGBbuffer[bufferposRGB] = (uint8_t) val>>8;
  bufferposRGB++;
  
   if(bufferposRGB==RGBBUFFERSIZE)
   {
     bufferposRGB = 0;
     time2writeRGB= 2;  // set flag to write second half
     firstwrittenRGB = 0; 
   }
 
  if((bufferposRGB>=halfbufRGB) & !firstwrittenRGB)  //at end of first buffer
  {
    time2writeRGB = 1; 
    firstwrittenRGB = 1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

void incrementO2bufpos(float val){
  O2buffer[bufferposO2] = val;
  bufferposO2++;
  
   if(bufferposO2==O2BUFFERSIZE)
   {
     bufferposO2 = 0;
     time2writeO2= 2;  // set flag to write second half
     firstwrittenO2 = 0; 
   }
 
  if((bufferposO2>=halfbufO2) & !firstwrittenO2)  //at end of first buffer
  {
    time2writeO2 = 1; 
    firstwrittenO2 = 1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

void incrementIMU(){
  for(int i=0; i<6; i++){
    imuBuffer[bufferposIMU] = (uint8_t) imuTempBuffer[i]; //accelerometer X,Y,Z
    bufferposIMU++;
  }
  // skipping IMU temperature in 6 and 7
  for(int i=8; i<20; i++){
    imuBuffer[bufferposIMU] = (uint8_t) imuTempBuffer[i]; //gyro and mag
    bufferposIMU++;
  }
  if(bufferposIMU==IMUBUFFERSIZE)
  {
    bufferposIMU = 0;
    time2writeIMU= 2;  // set flag to write second half
    firstwrittenIMU = 0; 
  }
  if((bufferposIMU>=halfbufIMU) & !firstwrittenIMU)  //at end of first buffer
  {
    time2writeIMU = 1; 
    firstwrittenIMU = 1;  //flag to prevent first half from being written more than once; reset when reach end of double buffer
  }
}

void setupDataStructures(void){
  // setup sidSpec and sidSpec buffers...hard coded for now
  
  // oxygen
  strncpy(sensor[0].chipName, "PRESENS", STR_MAX);
  sensor[0].nChan = 3;
  strncpy(sensor[0].name[0], "O2phase", STR_MAX);
  strncpy(sensor[0].name[1], "O2amplitude", STR_MAX);
  strncpy(sensor[0].name[2], "O2temperature", STR_MAX);
  strncpy(sensor[0].units[0], "deg", STR_MAX);
  strncpy(sensor[0].units[1], "V", STR_MAX);
  strncpy(sensor[0].units[2], "C", STR_MAX);
  sensor[0].cal[0] = 1; 
  sensor[0].cal[1] = 1;
  sensor[0].cal[2] = 1;


  // Pressure/Temperature
  if(pressure_sensor == 1) {
    strncpy(sensor[1].chipName, "MS5803", STR_MAX);
    sensor[1].nChan = 2;
    strncpy(sensor[1].name[0], "pressure", STR_MAX);
    strncpy(sensor[1].name[1], "temp", STR_MAX);
    strncpy(sensor[1].units[0], "mBar", STR_MAX);
    strncpy(sensor[1].units[1], "degreesC", STR_MAX);
    sensor[1].cal[0] = 1.0;
    sensor[1].cal[1] = 1.0;
  }
  else{
    strncpy(sensor[1].chipName, "PA7LD", STR_MAX);
    sensor[1].nChan = 2;
    strncpy(sensor[1].name[0], "pressure", STR_MAX);
    strncpy(sensor[1].name[1], "temp", STR_MAX);
    strncpy(sensor[1].units[0], "mBar", STR_MAX);
    strncpy(sensor[1].units[1], "degreesC", STR_MAX);
    sensor[1].cal[0] = 1.0;
    sensor[1].cal[1] = 1.0;
  }

  // RGB light
  strncpy(sensor[2].chipName, "ISL29125", STR_MAX);
  sensor[2].nChan = 3;
  strncpy(sensor[2].name[0], "red", STR_MAX);
  strncpy(sensor[2].name[1], "green", STR_MAX);
  strncpy(sensor[2].name[2], "blue", STR_MAX);
  strncpy(sensor[2].units[0], "uWpercm2", STR_MAX);
  strncpy(sensor[2].units[1], "uWpercm2", STR_MAX);
  strncpy(sensor[2].units[2], "uWpercm2", STR_MAX);
  sensor[2].cal[0] = 20.0 / 65536.0;
  sensor[2].cal[1] = 18.0 / 65536.0;
  sensor[2].cal[2] = 30.0 / 65536.0;


  // IMU
  strncpy(sensor[3].chipName, "MPU9250", STR_MAX);
  sensor[3].nChan = 9;
  strncpy(sensor[3].name[0], "accelX", STR_MAX);
  strncpy(sensor[3].name[1], "accelY", STR_MAX);
  strncpy(sensor[3].name[2], "accelZ", STR_MAX);
  //strncpy(sensor[3].name[3], "temp-21C", STR_MAX);
  strncpy(sensor[3].name[3], "gyroX", STR_MAX);
  strncpy(sensor[3].name[4], "gyroY", STR_MAX);
  strncpy(sensor[3].name[5], "gyroZ", STR_MAX);
  strncpy(sensor[3].name[6], "magX", STR_MAX);
  strncpy(sensor[3].name[7], "magY", STR_MAX);
  strncpy(sensor[3].name[8], "magZ", STR_MAX);
  strncpy(sensor[3].units[0], "g", STR_MAX);
  strncpy(sensor[3].units[1], "g", STR_MAX);
  strncpy(sensor[3].units[2], "g", STR_MAX);
  //strncpy(sensor[3].units[3], "degreesC", STR_MAX);
  strncpy(sensor[3].units[3], "degPerS", STR_MAX);
  strncpy(sensor[3].units[4], "degPerS", STR_MAX);
  strncpy(sensor[3].units[5], "degPerS", STR_MAX);
  strncpy(sensor[3].units[6], "uT", STR_MAX);
  strncpy(sensor[3].units[7], "uT", STR_MAX);
  strncpy(sensor[3].units[8], "uT", STR_MAX);
  
  float accelFullRange = 16.0; //ACCEL_FS_SEL 2g(00), 4g(01), 8g(10), 16g(11)
  int gyroFullRange = 1000.0;  // FS_SEL 250deg/s (0), 500 (1), 1000(2), 2000 (3)
  int magFullRange = 4800.0;  // fixed
  
  sensor[3].cal[0] = accelFullRange / 32768.0;
  sensor[3].cal[1] = accelFullRange / 32768.0;
  sensor[3].cal[2] = accelFullRange / 32768.0;
  //sensor[3].cal[3] = 1.0 / 337.87;
  sensor[3].cal[3] = gyroFullRange / 32768.0;
  sensor[3].cal[4] = gyroFullRange / 32768.0;
  sensor[3].cal[5] = gyroFullRange / 32768.0;
  sensor[3].cal[6] = magFullRange / 32768.0;
  sensor[3].cal[7] = magFullRange / 32768.0;
  sensor[3].cal[8] = magFullRange / 32768.0;
}

int addSid(int i, char* sid,  unsigned int sidType, unsigned long nSamples, SENSOR sensor, unsigned long dForm, float srate)
{
  unsigned long nBytes;
//  memcpy(&_sid, sid, 5);
//
//  memset(&sidSpec[i], 0, sizeof(SID_SPEC));
//        nBytes<<1;  //multiply by two because halfbuf
//
//  switch(dForm)
//  {
//    case DFORM_SHORT:
//      nBytes = nElements * 2;
//      break;            
//    case DFORM_LONG:
//      nBytes = nElements * 4;  //32 bit values
//      break;            
//    case DFORM_I24:
//      nBytes = nElements * 3;  //24 bit values
//      break;
//    case DFORM_FLOAT32:
//      nBytes = nElements * 4;
//      break;
//  }

  strncpy(sidSpec[i].SID, sid, STR_MAX);
  sidSpec[i].sidType = sidType;
  sidSpec[i].nSamples = nSamples;
  sidSpec[i].dForm = dForm;
  sidSpec[i].srate = srate;
  sidSpec[i].sensor = sensor;  
  
  if(dataFile.write((uint8_t *)&sidSpec[i], sizeof(SID_SPEC))==-1)  resetFunc();

  sidRec[i].nSID = i;
  sidRec[i].NU[0] = 100; //put in something easy to find when searching raw file
  sidRec[i].NU[1] = 200;
  sidRec[i].NU[2] = 300; 
}

void FileInit()
{
   char filename[20];
   getTime();
   // open file 
   sprintf(filename,"%02d%02d%02d%02d.AMX", day, hour, minute, second);  //filename is DDHHMM

   // log file
   float voltage = readVoltage();
   if(File logFile = sd.open("LOG.CSV",  O_CREAT | O_APPEND | O_WRITE)){
      logFile.print(filename);
      logFile.print(',');
      logFile.print(voltage); 
      logFile.print(',');
      logFile.println(dfh.Version);
      if(voltage < 3.7){
        stopTimer(); // stop sampling
        logFile.println("Stopping because Voltage less than 3.7 V");
        logFile.close();  
        
        islSleep(); // sleep RGB light sensor
        digitalWrite(ledGreen, LED_OFF);
        vhfOn(); // turn on VHF        

        // activate burn wire to make sure tag is off
        digitalWrite(BURN, HIGH); // burn on
        int burnDurNum = burnDurMin*60/8; // number of 8s lowpower delays
        for(i=0; i < burnDurNum; i++){
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // instead of delay(8000) ; 
        }
        digitalWrite(BURN, LOW);  // burn off
        
        // turn off microprocessor  - WHAT ABOUT IMU UNIT?
        mpuInit(0); // turn off MPU - CAN THIS BE DONE BEFORE BURN?

        // Finally, go into standby mode
        delay(1000);
        LowPower.standby(); 
      }
      logFile.close();
   }
   else{
    if(printDiags) SerialUSB.print("Log open fail.");
    resetFunc();
   }
   if(printDiags) SerialUSB.println("Log file initialized");
   
   dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
   if(printDiags) {
    SerialUSB.print("<<<<<<<<<<<<<<<<<<<<<");
    SerialUSB.println(filename);
   }
   
   
   while (!dataFile){
    file_count += 1;
    sprintf(filename,"F%06d.amx",file_count); //if can't open just use count
    dataFile = sd.open(filename, O_WRITE | O_CREAT | O_EXCL);
    if(printDiags) SerialUSB.println(filename);
   }
  //amx file header
  dfh.voltage = voltage;
  dfh.RecStartTime.sec = second;  
  dfh.RecStartTime.minute = minute;  
  dfh.RecStartTime.hour = hour;  
  dfh.RecStartTime.day = day;  
  dfh.RecStartTime.month = month;  
  dfh.RecStartTime.year = (int16_t) year;  
  dfh.RecStartTime.tzOffset = 0; //offset from GMT
  dataFile.write((uint8_t *) &dfh, sizeof(dfh));
  
  // write SID_SPEC depending on sensors chosen
  addSid(0, "O2TMP", RAW_SID, halfbufO2, sensor[0], DFORM_FLOAT32, sensor_srate);   
  addSid(1, "PRTMP", RAW_SID, halfbufPT, sensor[1], DFORM_FLOAT32, sensor_srate);    
  addSid(2, "LIGHT", RAW_SID, halfbufRGB / 2, sensor[2], DFORM_SHORT, sensor_srate);
  addSid(3, "3DAMG", RAW_SID, halfbufIMU / 2, sensor[3], DFORM_SHORT, imu_srate);
  addSid(4, "END", 0, 0, sensor[4], 0, 0);
  if(printDiags){
    SerialUSB.print("Buffers: ");
    SerialUSB.println(nbufsPerFile);
  }
}

void sampleSensors(void){  //interrupt at update_rate
  ssCounter++;
  
  readImu();
  incrementIMU();
//  if(printDiags){
//    accel_x = (int16_t) ((int16_t)imuTempBuffer[0] << 8 | imuTempBuffer[1]);
//    SerialUSB.print("accel:");
//    SerialUSB.println(accel_x);
//  }

  // MS5803 start temperature conversion half-way through
  if((ssCounter>=(0.5 * imu_srate / sensor_srate)) & (pressure_sensor==1)  & togglePress){ 
    readPress();   
    updateTemp();
    togglePress = 0;
  }
  
  if(ssCounter>=(int)(imu_srate/sensor_srate)){
    ssCounter = 0;
    // MS5803 pressure and temperature
    if (pressure_sensor==1){
        readTemp();
        updatePress();
        togglePress = 1;
    }
      
    // Keller PA7LD pressure and temperature
    if (pressure_sensor==2){
      kellerRead();
      kellerConvert();  // start conversion for next reading
    }    

    // Oxygen
    incrementO2bufpos(o2Temperature());
    incrementO2bufpos(o2Phase());
    incrementO2bufpos(o2Amplitude());

    // RGB
    islRead(); 
    incrementRGBbufpos(islRed);
    incrementRGBbufpos(islGreen);
    incrementRGBbufpos(islBlue);
    
    if (pressure_sensor==1) calcPressTemp(); // MS5803 pressure and temperature
    if (pressure_sensor>0){  //both Keller and MS5803
      PTbuffer[bufferposPT] = pressure_mbar;
      incrementPTbufpos();
      PTbuffer[bufferposPT] = temperature;
      incrementPTbufpos();
    }
  }
}
  
float readVoltage(){
  float vDivider = 0.5;
  float vReg = 3.3;
  float voltage = (float) analogRead(vSense) * vReg / (vDivider * 1024.0);
  return voltage;
}

void resetFunc(){

}

void getTime(){
  day = rtc.getDay();
  month = rtc.getMonth();
  year = rtc.getYear();
  hour = rtc.getHours();
  minute = rtc.getMinutes();
  second = rtc.getSeconds();
}

void vhfOn(){
  digitalWrite(VHF, HIGH);
}

void vhfOff(){
  digitalWrite(VHF, LOW);
}



// Calculates Accurate UNIX Time Based on RTC Timestamp
unsigned long RTCToUNIXTime(int uYear, int uMonth, int uDay, int uHour, int uMinute, int uSecond){
  int i;
  unsigned const char DaysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  unsigned long Ticks = 0;

  long yearsSince = uYear+30; // Same as tm->year + 2000 - 1970
  long numLeaps = yearsSince >> 2; // yearsSince / 4 truncated
  
  if((!(uYear%4)) && (uMonth>2)) Ticks+=SECONDS_IN_DAY;  //dm 8/9/2012  If current year is leap, add one day

  // Calculate Year Ticks
  Ticks += (yearsSince-numLeaps)*SECONDS_IN_YEAR;
  Ticks += numLeaps * SECONDS_IN_LEAP;

  // Calculate Month Ticks
  for(i=0; i < uMonth-1; i++){
       Ticks += DaysInMonth[i] * SECONDS_IN_DAY;
  }

  // Calculate Day Ticks
  Ticks += uDay * SECONDS_IN_DAY;
  
  // Calculate Time Ticks CHANGES ARE HERE
  Ticks += (ULONG)uHour * SECONDS_IN_HOUR;
  Ticks += (ULONG)uMinute * SECONDS_IN_MINUTE;
  Ticks += uSecond;

  return Ticks;
}

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

/*
This is a slightly modified version of the timer setup found at:
https://github.com/maxbader/arduino_tools
 */
void startTimer(int frequencyHz) {
  //GCLK->GENCTRL.reg |= GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_RUNSTDBY;
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID (GCM_TCC2_TC3)) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 );

  
  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;

  //TC->CTRLA.reg |= TC_CTRLA_RUNSTDBY;

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    sampleSensors();
  }
}

void stopTimer(){
  // TcCount16* TC = (TcCount16*) TC3; // get timer struct
   NVIC_DisableIRQ(TC3_IRQn);
}
