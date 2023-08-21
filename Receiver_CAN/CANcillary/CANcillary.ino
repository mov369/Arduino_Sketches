#include <SPI.h>
#include "lookup.c"   //lookup tables for mappings
#include <mcp_can.h>

#define CAN_500KBPS 16     // CAN baud rate
#define SPI_CS_PIN  17     // setup chip select pin for CAN module
#define LED 23              //LED pin to show program active
#define STEERADC A2       // ADC for steer pot
#define THROADC A0        // ADC for Throttle pot
#define FLSENSE 12            // SW1 input
#define FRSENSE 6             // SW2 input
//#define HEADLIGHTS 5      // Headlights input
#define CH3ADC A1         /// Horn signal 
#define INDICATOR_L 4     // Left indicator
#define INDICATOR_R 21    // Right indicator

#define PROGRAM_MODE false     // set this to enable the serial port (the code will not run if serial enabled but not connected)

MCP_CAN CAN(SPI_CS_PIN);            // Set CS pin

void setup()
{
  
  pinMode(LED, OUTPUT);     // Set pins as digital inputs and outputs
  pinMode(FLSENSE, INPUT);
  pinMode(FRSENSE, INPUT);
  //pinMode(HEADLIGHTS, INPUT);
  pinMode(INDICATOR_L, INPUT_PULLUP);  //Indicators switch to ground so need a pullup
  pinMode(INDICATOR_R, INPUT_PULLUP);

  if (PROGRAM_MODE == true){
     Serial.begin(115200);
     while(!Serial);
  }
  while (CAN_OK != CAN.begin(CAN_500KBPS)){    // init can bus : baudrate = 500k
      if (PROGRAM_MODE == true){
       Serial.println("CAN BUS FAIL!");
      delay(100);
      }
  }
  if (PROGRAM_MODE == true){
      Serial.println("CAN BUS OK!");
  } 


  /*
        set mask, set both the mask to 0x3ff
    */
    CAN.init_Mask(0, 0, 0x3ff);                         // there are 2 mask in mcp2515, you need to set both of them
    CAN.init_Mask(1, 0, 0x3ff);

  CAN.init_Filt(1, 0, 0x113);
}

//JOYSTICK LEGACY STUFF
//int SteerRawValue = 0;  // raw value of steer pot
//int ThroRawValue = 0;   // raw value of throttle pot

//const unsigned int Steer_X[5] = {0x00, 0x1A0, 0x1EA, 0x210, 0x3A5};    //Arduino has 10 bit ADC so 0->2FF
//const int  Steer_Y[5] =         {-100,   0,      0,      0,      100};

//const unsigned int Torque_X[5] = {0xE1, 0x1B0, 0x1E5, 0x210, 0x3A5};
//const int  Torque_Y[5] =        {-100,   0,      0,      0,      100};

const unsigned long frameRepetitionTime=2500;  // repetition rate of the frame in ms  SET TO 1s for INITIAL TESTS
unsigned long previousTime;                    // last time for frame rate calcs

byte VehicleStateIndicators[8];
byte count = 0;  // rolling counter for alive signal
bool leds = true;


void loop()
{
//digitalWrite(THROADC, LOW);
unsigned long currentTime = millis();   // grab the current time
unsigned int CH3Input = 0;

if (currentTime-previousTime >= frameRepetitionTime) {

  digitalWrite(LED,leds);  // light/extinguish the led on alternate cycles
  leds=!leds;

  //SteerRawValue = analogRead(STEERADC);// read the value from the analog steer  channel
  // ThroRawValue = analogRead(THROADC);   // read the value from the analog throttle channel
  //CH3Input=analogRead(CH3ADC);          // read the value from Ch3 (strange oscillating small signal)






  //RECEIVE BIT
  uint8_t CANRX_data[8]={0,0,0,0,0,0,0,0};
  unsigned char len = 8;
  unsigned char buf[7];
  

  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        //unsigned long canId = CAN.getCanId();
        
        Serial.println("-----------------------------");
        Serial.print("Get data from ID: ");
        Serial.println(CAN.getCanId(),HEX);
        for(int i = 0; i<len; i++)    // print the data
        {
            Serial.print(buf[0]);
            Serial.print("\t");
        }
        Serial.println();
        if (buf[0]=2) {
          digitalWrite(THROADC, HIGH);
          }
          else if (buf[0]=0){
          digitalWrite(THROADC, LOW);}
          else if (buf[0]=1){
          digitalWrite(THROADC, LOW);}
          else if (buf[0]=3){
          digitalWrite(THROADC, LOW);}
          else if (buf[0]=4){
          digitalWrite(THROADC, LOW);}

  //CYCLIC STATE MESSAGE
  VehicleStateIndicators[0] = 0; 
  VehicleStateIndicators[1] = count++;
  VehicleStateIndicators[2] = 0;
  VehicleStateIndicators[3] = 0;
  VehicleStateIndicators[4] = 0;
  VehicleStateIndicators[5] = 0;  
  VehicleStateIndicators[6] = 0;
  VehicleStateIndicators[7] = 0;
  size_t dataLength = sizeof(VehicleStateIndicators);
  uint16_t crcResult = crc16_xmodem(VehicleStateIndicators, dataLength);
  VehicleStateIndicators[2] = byte(crcResult & 0x00ff);
  VehicleStateIndicators[3] = byte((crcResult >> 8) & 0x00ff); 
  CAN.sendMsgBuf(0x102, 0, 8, VehicleStateIndicators);

  

  previousTime = currentTime;   // set previous time for next timer loop
  }  // if currentTime-previousTime
}  // loop end
}

//CRC16 Generation
uint16_t crc16_xmodem(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
  return crc;
}
