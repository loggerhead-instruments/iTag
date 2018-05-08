
// Include libraries this sketch will use
#include  <Wire.h>
#include  <MCP342X.h>

#define PULSE_EN 6

// Instantiate objects used in this project
MCP342X myADC(0x6E);

void setup() {
  Wire.begin();  // join I2C bus
  TWBR = 12;  // 400 kHz (maximum)
  
  Serial.begin(115200); // Open serial connection to send info to the host
  while (!Serial) {}  // wait for Serial comms to become ready
  Serial.println("Starting up");
  Serial.println("Testing device connection...");
    Serial.println(myADC.testConnection() ? "MCP342X connection successful" : "MCP342X connection failed");

  myADC.configure( MCP342X_MODE_CONTINUOUS |
                   MCP342X_CHANNEL_1 |
                   MCP342X_SIZE_16BIT |
                   MCP342X_GAIN_1X
                 );

  Serial.println(myADC.getConfigRegShdw(), HEX);
  pinMode(PULSE_EN, OUTPUT);
  
}  // End of setup()

int count;
void loop() {
  static int16_t  result;

  
  digitalWrite(PULSE_EN, HIGH);
  delay(1);
  digitalWrite(PULSE_EN, LOW);
  delay(1);
  count++;

  if(count>500){
    myADC.startConversion();
    myADC.getResult(&result);
    Serial.println(result);
    count = 0;
  }
}  // End of loop()
