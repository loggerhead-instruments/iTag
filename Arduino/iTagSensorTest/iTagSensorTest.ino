// iTag
// Board bootloader loaded using AtmelICE and Board: Arduino Zero Programming Port
// Board programmed using USB via Arduino Zero Native USB port


#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// To Do:
// - O2 sensor
// - Keller sensor
// - SD test (write values to ASCII file)
// - Setup to use interrupt from IMU

int printDiags = 1;

const int ledGreen = PIN_LED_TXL;
const int BURN = A5;
const int vSense = A4; 
const int VHF = PIN_LED_RXL;
const int O2POW = 4;
const int chipSelect = 10;

int pressure_sensor;


// IMU
int FIFOpts;
#define BUFFERSIZE 140 // used this length because it is divisible by 20 bytes (e.g. A*3,M*3,G*3,T) and 14 (w/out mag)
byte imuBuffer[BUFFERSIZE]; // buffer used to store IMU sensor data before writes in bytes
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

// RGB
int16_t islRed;
int16_t islBlue;
int16_t islGreen;

// Pressure/Temp
byte Tbuff[3];
byte Pbuff[3];
volatile float depth, temperature;
boolean togglePress; //flag to toggle conversion of pressure and temperature

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
int ptCounter = 0;
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

File dataFile;

void setup() {
  SerialUSB.begin(57600);
  
  delay(5000);
  Wire.begin();
  
  SerialUSB.println("iTag sensor test");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    SerialUSB.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  SerialUSB.println("card initialized"); 
}

void loop() {
    sensorInit();
    SerialUSB.println("Test Complete");
    SerialUSB.println("Restarting in 5s");
    delay(5000);
}

void sensorInit(){
 // initialize and test sensors
  String dataString = "";
  dataFile = SD.open("log.txt", FILE_WRITE);
  SerialUSB.println("Sensor Init");
  pinMode(ledGreen, OUTPUT);
  pinMode(BURN, OUTPUT);
  pinMode(VHF, OUTPUT);
  pinMode(vSense, INPUT);
  pinMode(O2POW, OUTPUT);

  // Digital IO
  digitalWrite(ledGreen, HIGH);
  digitalWrite(BURN, HIGH);
  digitalWrite(VHF, HIGH);
  digitalWrite(O2POW, HIGH);

  
  SerialUSB.println("Green LED on");

  // RGB
  islInit(); 
  for(int n=0; n<10; n++){
      islRead();
      SerialUSB.print("R:"); SerialUSB.print(islRed); SerialUSB.print("\t");
      SerialUSB.print("G:"); SerialUSB.print(islGreen); SerialUSB.print("\t");
      SerialUSB.print("B:"); SerialUSB.println(islBlue);
      delay(1000);
  }
  
  digitalWrite(ledGreen, LOW);
  digitalWrite(BURN, LOW);
  digitalWrite(VHF, LOW);

  SerialUSB.println("Green LED off");

// battery voltage measurement
  SerialUSB.print("Battery: ");
  SerialUSB.println(analogRead(vSense));

  
// Pressure--auto identify which if any is present
  pressure_sensor = 0;
  // Keller
  if(kellerInit()) {
    pressure_sensor = 2;   // 2 if present
    SerialUSB.println("Keller Pressure Detected");
    kellerConvert();
    delay(5);
    kellerRead();
    SerialUSB.print("Depth: "); SerialUSB.println(depth);
    SerialUSB.print("Temperature: "); SerialUSB.println(temperature);
  }
  

  // Measurement Specialties
  if(pressInit()){
    pressure_sensor = 1;
    SerialUSB.println("MS Pressure Detected");
    updatePress();
    delay(10);
    readPress();
    updateTemp();
    delay(10);
    readTemp();
    calcPressTemp();
    SerialUSB.print("Depth: "); SerialUSB.println(depth);
    SerialUSB.print("Temperature: "); SerialUSB.println(temperature);
  }

  // Presens O2
  SerialUSB.print("O2 status:");
  SerialUSB.println(o2Status());
  SerialUSB.print("Temperature:"); SerialUSB.println(o2Temperature());
  SerialUSB.print("Phase:"); SerialUSB.println(o2Phase());
  SerialUSB.print("Amplitude:"); SerialUSB.println(o2Amplitude());

  // IMU
  mpuInit(1);
  for(int n=0; n<15; n++){
      delay(500);
      pollImu(); //will print out values from FIFO
  }

  dataString += String(islRed);
  dataString += ",";
  dataString += String(islGreen);
  dataString += ",";
  dataString += String(islBlue);
  dataFile.println(dataString);
  dataFile.close();
}

boolean pollImu(){
  FIFOpts=getImuFifo();
  //SerialUSB.print("IMU FIFO pts: ");
  //if (printDiags) SerialUSB.println(FIFOpts);
  if(FIFOpts>BUFFERSIZE)  //once have enough data for a block, download and write to disk
  {
     Read_Gyro(BUFFERSIZE);  //download block from FIFO
  
     
    if (printDiags){
    // print out first line of block
    // MSB byte first, then LSB, X,Y,Z
    accel_x = (int16_t) ((int16_t)imuBuffer[0] << 8 | imuBuffer[1]);    
    accel_y = (int16_t) ((int16_t)imuBuffer[2] << 8 | imuBuffer[3]);   
    accel_z = (int16_t) ((int16_t)imuBuffer[4] << 8 | imuBuffer[5]);    
    
    gyro_temp = (int16_t) (((int16_t)imuBuffer[6]) << 8 | imuBuffer[7]);   
   
    gyro_x = (int16_t)  (((int16_t)imuBuffer[8] << 8) | imuBuffer[9]);   
    gyro_y = (int16_t)  (((int16_t)imuBuffer[10] << 8) | imuBuffer[11]); 
    gyro_z = (int16_t)  (((int16_t)imuBuffer[12] << 8) | imuBuffer[13]);   
    
    magnetom_x = (int16_t)  (((int16_t)imuBuffer[14] << 8) | imuBuffer[15]);   
    magnetom_y = (int16_t)  (((int16_t)imuBuffer[16] << 8) | imuBuffer[17]);   
    magnetom_z = (int16_t)  (((int16_t)imuBuffer[18] << 8) | imuBuffer[19]);  

    SerialUSB.print("a/g/m/t:\t");
    SerialUSB.print( accel_x); SerialUSB.print("\t");
    SerialUSB.print( accel_y); SerialUSB.print("\t");
    SerialUSB.print( accel_z); SerialUSB.print("\t");
    SerialUSB.print(gyro_x); SerialUSB.print("\t");
    SerialUSB.print(gyro_y); SerialUSB.print("\t");
    SerialUSB.print(gyro_z); SerialUSB.print("\t");
    SerialUSB.print(magnetom_x); SerialUSB.print("\t");
    SerialUSB.print(magnetom_y); SerialUSB.print("\t");
    SerialUSB.print(magnetom_z); SerialUSB.print("\t");
    SerialUSB.println((float) gyro_temp/337.87+21);
    }
    
    return true;
  }
  return false;
}
