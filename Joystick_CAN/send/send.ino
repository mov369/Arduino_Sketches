#include <SPI.h>
#include "lookup.c"   //lookup tables for mappings
#include <mcp_can.h>

#define CAN_500KBPS 16     // CAN baud rate
#define SPI_CS_PIN  17     // setup chip select pin for CAN module
#define LED 23              //LED pin to show program active
#define STEERADC A2       // ADC for steer pot
#define THROADC A0        // ADC for Throttle pot
#define SW1 12            // SW1 input
#define SW2 6             // SW2 input
#define INDICATORS_BOTH 5      // Both Indicators Input
#define CH3ADC A1         /// Horn signal 
#define INDICATOR_L 4     // Left indicator
#define INDICATOR_R 21    // Right indicator

#define PROGRAM_MODE false     // set this to enable the serial port (the code will not run if serial enabled but not connected)

MCP_CAN CAN(SPI_CS_PIN);            // Set CS pin

void setup()
{ //Pin setup
  pinMode(LED, OUTPUT);     // Set pins as digital inputs and outputs
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(INDICATORS_BOTH, INPUT);
  pinMode(INDICATOR_L, INPUT_PULLUP);  //Indicators switch to ground so need a pullup
  pinMode(INDICATOR_R, INPUT_PULLUP);

  //Message definitions
  //Wayve eTransit_Alpha_DBW-28-06-23.dbc
  byte ByWireControlAccelerator[8];
  byte ByWireControlBrake[8];
  byte ByWireControlSteering[8];
  byte ByWireControlIndicators[8];
  byte ByWireControlRequest[8];

  byte DriveByWireState[8]; //For testing Cancillaries

  //Serial monitoring set-up
  if (PROGRAM_MODE == true){
     Serial.begin(115200);
     while(!Serial);
  }
  while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
      if (PROGRAM_MODE == true){
       Serial.println("CAN BUS FAIL!");
      delay(100);
      }
    }
   if (PROGRAM_MODE == true){
      Serial.println("CAN BUS OK!");
   } 
}


int SteerRawValue = 0;  // raw value of steer pot
int ThroRawValue = 0;   // raw value of throttle pot

const unsigned int Steer_X[5] = {0x00, 0x1A0, 0x1EA, 0x210, 0x3A5}; //Arduino has 10 bit ADC so 0->2FF. 3FF?
const int  Steer_Y[5] =         {-7200,   0,      0,      0,      7200};

const unsigned int Accel_X[5] = {0xE1, 0x1B0, 0x1E5, 0x210, 0x3A5};
const int  Accel_Y[5] =        {0,   0,      0,      0,      1000};

const unsigned int Brake_X[5] = {0xE1, 0x1B0, 0x1E5, 0x210, 0x3A5};
const int  Brake_Y[5] =        {1000,   0,      0,      0,      0};

const unsigned long frameRepetitionTime=5; //repetition rate of the frame in ms
unsigned long previousTime; // last time for frame rate calcs
//rolling counters for alive signal
byte countAccel = 0;
byte countBrake = 0;  
byte countSteering = 0;
byte countIndicators = 0;
byte countRequest = 0;

bool leds = true;
int16_t SteerScaledValue; 
int16_t AccelScaledValue; 
int16_t BrakeScaledValue; 

size_t dataLength;

void loop()
{
  //Message names & lengths definitions
  //Wayve eTransit_Alpha_DBW-28-06-23.dbc
  byte ByWireControlAccelerator[8];
  byte ByWireControlBrake[8];
  byte ByWireControlSteering[8];
  byte ByWireControlIndicators[8];
  byte ByWireControlRequest[8];

  byte DriveByWireState[8]; //For testing Cancillaries
  //byte CRCTEST[8];
  unsigned long currentTime = millis();   // grab the current time
  unsigned int CH3Input = 0;

  if (currentTime-previousTime >= frameRepetitionTime) {
    SteerRawValue = analogRead(STEERADC);// read the value from the analog steer  channel
    ThroRawValue = analogRead(THROADC);   // read the value from the analog throttle channel
    CH3Input = analogRead(CH3ADC);          // read the value from Ch3 (strange oscillating small signal)
    SteerScaledValue = lookup_u16_s16(SteerRawValue, Steer_X, Steer_Y, 5);
    AccelScaledValue = lookup_u16_u16(ThroRawValue, Accel_X, Accel_Y, 5);
    BrakeScaledValue = lookup_u16_u16(ThroRawValue, Brake_X, Brake_Y, 5);
    digitalWrite(LED,leds);  // light/extinguish the led on alternate cycles
    leds=!leds;

    Serial.println(CH3Input);

    //ByWireControlBrake(0x111) data loading into the message
    ByWireControlAccelerator[0] = (byte)(AccelScaledValue & 0x00ff); //extracting lower 8 bits and setting higher bits to 0;
    ByWireControlAccelerator[1] = (byte)((AccelScaledValue >>8) & 0xFFFF); //bitshifting 8 positions to the right. Note that if the AccelScaledValue will be > 1000 max, the result raw output may be longer than specified bit length.
    ByWireControlAccelerator[2] = countAccel++;
    ByWireControlAccelerator[3] = 0;
    ByWireControlAccelerator[4] = 0;
    ByWireControlAccelerator[5] = 0;
    ByWireControlAccelerator[6] = 0;
    ByWireControlAccelerator[7] = 0;
    size_t dataLength1 = sizeof(ByWireControlAccelerator);
    uint16_t crcResult1 = crc16_xmodem(ByWireControlAccelerator, dataLength1);
    ByWireControlAccelerator[3] = byte(crcResult1 & 0x00ff);
    ByWireControlAccelerator[4] = byte((crcResult1 >> 8) & 0x00ff);
    CAN.sendMsgBuf(0x110, 0, 8, ByWireControlAccelerator);  

    //ByWireControlBrake(0x111) data loading into the message
    ByWireControlBrake[0] = (byte)(BrakeScaledValue & 0x00ff);
    ByWireControlBrake[1] = (byte)((BrakeScaledValue >>8) & 0xFFFF); //bitshifting 8 positions to the right. Note that if the BrakeScaledValue will be > 1000 max, the result raw output may be longer than specified bit length.
    ByWireControlBrake[2] = countBrake++; 
    ByWireControlBrake[3] = 0;
    ByWireControlBrake[4] = 0;
    ByWireControlBrake[5] = 0;
    ByWireControlBrake[6] = 0;
    ByWireControlBrake[7] = 0;
    size_t dataLength2 = sizeof(ByWireControlBrake);
    uint16_t crcResult2 = crc16_xmodem(ByWireControlBrake, dataLength2);
    ByWireControlBrake[3] = byte(crcResult2 & 0x00ff);
    ByWireControlBrake[4] = byte((crcResult2 >> 8) & 0x00ff);
    CAN.sendMsgBuf(0x111, 0, 8, ByWireControlBrake);  

    //ByWireControlSteering(0x112) data loading into the message
    ByWireControlSteering[0] = (byte)(SteerScaledValue & 0x00FF);
    ByWireControlSteering[1] = (byte)((SteerScaledValue >> 8) & 0x003F); //MSB. bitshifting 8 positions to the right and masking the unwanted byte and bits in the second nibble
    ByWireControlSteering[2] = countSteering++;
    ByWireControlSteering[3] = 0;
    ByWireControlSteering[4] = 0;
    ByWireControlSteering[5] = 0;
    ByWireControlSteering[6] = 0;
    ByWireControlSteering[7] = 0;
    size_t dataLength3 = sizeof(ByWireControlSteering);
    uint16_t crcResult3 = crc16_xmodem(ByWireControlSteering, dataLength3);
    ByWireControlSteering[3] = byte(crcResult3 & 0x00ff);
    ByWireControlSteering[4] = byte((crcResult3 >> 8) & 0x00ff); 
    CAN.sendMsgBuf(0x112, 0, 8, ByWireControlSteering);

    //ByWireControlIndicators(0x113) data loading into the message
    //Wayve .dbc Value table: 0 = reserved; 1 = off; 2 = left; 3 = right; 4 = both;
    //intialise all as 0
    ByWireControlIndicators[0] = 0; 
    if (digitalRead(INDICATORS_BOTH) == true) 
      { ByWireControlIndicators[0] = 0;
        ByWireControlIndicators[0] = ByWireControlIndicators[0] | B00000100;} //both
    else if(digitalRead(INDICATOR_R) == false) 
      { ByWireControlIndicators[0] = 0;
        ByWireControlIndicators[0] = ByWireControlIndicators[0] | B00000011;} //right
    else if(digitalRead(INDICATOR_L) == false) 
      { ByWireControlIndicators[0] = 0;
        ByWireControlIndicators[0] = ByWireControlIndicators[0] | B00000010;} //left
    else ByWireControlIndicators[0] = ByWireControlIndicators[0] | B00000001;
    ByWireControlIndicators[1] = countIndicators++;
    ByWireControlIndicators[2] = 0; 
    ByWireControlIndicators[3] = 0; 
    ByWireControlIndicators[4] = 0; 
    ByWireControlIndicators[5] = 0;
    ByWireControlIndicators[6] = 0;
    ByWireControlIndicators[7] = 0;
    size_t dataLength4 = sizeof(ByWireControlIndicators);
    uint16_t crcResult4 = crc16_xmodem(ByWireControlIndicators, dataLength4);
    ByWireControlIndicators[2] = byte(crcResult4 & 0x00ff);
    ByWireControlIndicators[3] = byte((crcResult4 >> 8) & 0x00ff); 
    CAN.sendMsgBuf(0x113, 0, 8, ByWireControlIndicators);

    //ByWireControlRequest(0x114) data loading into the message
    //Wayve .dbc Value table: 0 = off; 1 = on;
    //intialise all as 0
    ByWireControlRequest[0] = 0;
    if(digitalRead(SW2) == false) {ByWireControlRequest[0] += 0x01; }
    ByWireControlRequest[1] = countRequest++;
    ByWireControlRequest[2] = 0; //clear the bytes after
    ByWireControlRequest[3] = 0; //clear the bytes after
    ByWireControlRequest[4] = 0; 
    ByWireControlRequest[5] = 0;
    ByWireControlRequest[6] = 0;
    ByWireControlRequest[7] = 0; 
    size_t dataLength = sizeof(ByWireControlRequest);
    uint16_t crcResult = crc16_xmodem(ByWireControlRequest, dataLength);
    ByWireControlRequest[2] = byte(crcResult & 0x00ff);
    ByWireControlRequest[3] = byte((crcResult >> 8) & 0x00ff);
    CAN.sendMsgBuf(0x114, 0, 8, ByWireControlRequest);
    
    DriveByWireState[0] = 1;
    if(analogRead(CH3ADC) > 200) {

    DriveByWireState[0] = DriveByWireState[0] | B00000010;}
    DriveByWireState[1] = countRequest++;
    DriveByWireState[2] = 0;
    DriveByWireState[3] = 0;
    DriveByWireState[4] = 0;
    DriveByWireState[5] = 0;
    DriveByWireState[6] = 0;
    DriveByWireState[7] = 0;
    CAN.sendMsgBuf(0x1A0, 0, 8, DriveByWireState);
    //Serial.print("CRC16/XModem Checksum: 0x");
    //Serial.println(crcResult4, HEX);
  
    previousTime = currentTime;// set previous time for next timer loop
  }//if currentTime-previousTime
}//loop end


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
