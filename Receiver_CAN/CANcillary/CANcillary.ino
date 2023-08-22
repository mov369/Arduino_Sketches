#include <SPI.h>
#include <mcp_can.h>
#include "lookup.c"   //lookup tables for mappings

#define CAN_500KBPS 16     // CAN baud rate
#define SPI_CS_PIN  17     // setup chip select pin for CAN module
#define LED 23              //LED pin to show program active
#define IND_L_CONT A2       // ADC for steer pot
#define IND_R_CONT A0        // ADC for Throttle pot
#define FLSENSE 12            // SW1 input
#define FRSENSE 6             // SW2 input
#define CH3ADC A1         /// Horn signal 
#define IND_L_SENSE 4     // Left indicator
#define IND_R_SENSE 21    // Right indicator
//#define HEADLIGHTS 5      // Headlights input
#define PROGRAM_MODE false     // set this to enable the serial port (the code will not run if serial enabled but not connected)

MCP_CAN CAN(SPI_CS_PIN);            // Set CS pin

void setup()
{
  pinMode(LED, OUTPUT);     //Set pins as digital inputs and outputs
  pinMode(FLSENSE, INPUT);
  pinMode(FRSENSE, INPUT);
  pinMode(IND_L_CONT, OUTPUT);
  pinMode(IND_R_CONT, OUTPUT);
  //pinMode(HEADLIGHTS, INPUT);
  //pinMode(INDICATOR_L, INPUT_PULLUP);  //Indicators switch to ground so need a pullup
  //pinMode(INDICATOR_R, INPUT_PULLUP);

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
  CAN.init_Mask(0, 0, 0x3ff);  // there are 2 masks in mcp2515, you need to set both of them
  CAN.init_Mask(1, 0, 0x3ff);
  
  CAN.init_Filt(1, 0, 0x113);
  CAN.init_Filt(2, 0, 0x1A0);
}

//JOYSTICK LEGACY STUFF
//int SteerRawValue = 0;  // raw value of steer pot
//int ThroRawValue = 0;   // raw value of throttle pot

//const unsigned int Steer_X[5] = {0x00, 0x1A0, 0x1EA, 0x210, 0x3A5};    //Arduino has 10 bit ADC so 0->2FF
//const int  Steer_Y[5] =         {-100,   0,      0,      0,      100};

//const unsigned int Torque_X[5] = {0xE1, 0x1B0, 0x1E5, 0x210, 0x3A5};
//const int  Torque_Y[5] =        {-100,   0,      0,      0,      100};

const unsigned long frameRepetitionTime=1000;  // repetition rate of the frame in ms  SET TO 1s for INITIAL TESTS
unsigned long previousTime;                    // last time for frame rate calcs

byte VehicleStateIndicators[8];
byte ByWireControlIndicators[8];
byte DriveByWireState[8];

byte count = 0;  // rolling counter for alive signal
bool leds = true;
bool Autonomous_Mode = false;
unsigned int CH3Input = 0;
unsigned int indicatorcommand = 0;
uint16_t ms = 0;


//This Cal controls how frequently the indicators pulse 
int indicator_timer_cal = 400;
uint16_t can_rc_time_ms_104 = 0;
uint16_t can_rc_time_ms_100 = 0;
//int can_error_timeout_cal = 200;


void loop()
{
  if(0 == (ms % indicator_timer_cal)){
		if (Indicator_High_B){
			Indicator_High_B = false;
		}else{
			Indicator_High_B = true;
		}
	}
  //digitalWrite(THROADC, LOW);
  unsigned long currentTime = millis();   // grab the current time

  if (currentTime-previousTime >= frameRepetitionTime) {
    digitalWrite(LED,leds);  // light/extinguish the led on alternate cycles
    leds=!leds;

    //digitalWrite(IND_L_CONT, LOW);
    //digitalWrite(IND_R_CONT, LOW);
    //SteerRawValue = analogRead(STEERADC);// read the value from the analog steer  channel
    //ThroRawValue = analogRead(THROADC);   // read the value from the analog throttle channel
    //CH3Input=analogRead(CH3ADC);          // read the value from Ch3 (strange oscillating small signal)
    //RECEIVE BIT
    //uint8_t CANRX_data[8]={0,0,0,0,0,0,0,0};

    unsigned char len = 0;
    unsigned char buf[8];
  
  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
      CAN.readMsgBuf(&len, buf);        // read data,  len: data length, buf: data buf
      unsigned long canId = CAN.getCanId();
      
      if (canId == 0x113) {
        Serial.println("Received Message with ID 0x113:");
        for (int i = 0; i < len; i++) {
          Serial.print(buf[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        indicatorcommand = buf[0];
        ByWireControlIndicators[0] = buf[0]; 
        ByWireControlIndicators[1] = buf[1];
        ByWireControlIndicators[2] = 0; 
        ByWireControlIndicators[3] = 0; 
        ByWireControlIndicators[4] = buf[4]; 
        ByWireControlIndicators[5] = buf[5];
        ByWireControlIndicators[6] = buf[6];
        ByWireControlIndicators[7] = buf[7];
        size_t dataLength1 = sizeof(ByWireControlIndicators);
        uint16_t crcResult1 = crc16_xmodem(ByWireControlIndicators, dataLength1);
        byte highByte = buf[3];
        byte lowByte = buf[2];
        unsigned int combinedvalue;
        combinedvalue = (highByte << 8 )| lowByte;

        if(Autonomous_Mode == true){
          if (indicatorcommand == 2) {
            digitalWrite(IND_L_CONT, HIGH);
            digitalWrite(IND_R_CONT, LOW);
            //delay(indicator_timer_cal);
            } 
          else if (indicatorcommand == 3){
            digitalWrite(IND_L_CONT, LOW);
            digitalWrite(IND_R_CONT, HIGH);
            //delay(indicator_timer_cal);
            } 
          else if (indicatorcommand == 4){
            digitalWrite(IND_L_CONT, HIGH);
            digitalWrite(IND_R_CONT, HIGH);
            //delay(indicator_timer_cal);
            } 
          else{
            digitalWrite(IND_L_CONT, LOW);
            digitalWrite(IND_R_CONT, LOW);
            //delay(indicator_timer_cal);
            }
          }
        else{digitalWrite(IND_L_CONT, LOW);
            digitalWrite(IND_R_CONT, LOW);
            //delay(indicator_timer_cal);
            } // Auto mode dependancy
        } // End of message 113

       if (canId == 0x1A0) {             // Handle the first message here
        Serial.println("Received Message with ID 0x1A0:");
        for (int i = 0; i < len; i++) {
          Serial.print(buf[i], HEX);
          Serial.print(" ");}
        DriveByWireState[0] = buf[0]; 
        DriveByWireState[1] = buf[1];
        DriveByWireState[2] = buf[2]; 
        DriveByWireState[3] = buf[3]; 
        DriveByWireState[4] = buf[4]; 
        DriveByWireState[5] = buf[5];
        DriveByWireState[6] = buf[6];
        DriveByWireState[7] = buf[7];
        
        
        if (DriveByWireState[0] == 3){
          Autonomous_Mode = true;
        }
        else {Autonomous_Mode = false;}
      
        Serial.print(" MODE: ");
        Serial.print(Autonomous_Mode);
        Serial.println();
      } 


      //Serial.print("Data from ID: ");
      //Serial.println(CAN.getCanId(),HEX);
      //Serial.print("Buffer value: ");
      //Serial.println(buf[0]);
      //Serial.print("indicatorcommand Value: ");
      //Serial.print(indicatorcommand, DEC);
      //Serial.println();

      //Serial.print("\t");  
      //for(int i = 0; i<1; i++)    // print the data
      // {
      // Serial.print(buf[0]);
      //  Serial.print("\t");
      // if (buf[0]=2,HEX) {
      //    digitalWrite(THROADC, HIGH);
      //    }
      //  else if (buf[0]=0){
      //    digitalWrite(THROADC, LOW);}
      //  else if (buf[0]=1){
      //    digitalWrite(THROADC, LOW);}
      // else if (buf[0]=3){
      //    digitalWrite(THROADC, LOW);}
      // else {digitalWrite(THROADC, LOW);}
      //}
  
    

    //Serial.print("Actual CRC Bytes of the message from joystick: ");
    //Serial.print(buf[3],HEX);
    //Serial.print(" ");
    //Serial.println(buf[2],HEX);
    //Serial.print("Combined Value in dec - still message from joystick: ");
    //Serial.println(combinedvalue);
    //Serial.print("CRC Calculation on CANcillary module side: ");
    //Serial.println(crcResult1);
 




  
  
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
  }
} //if currentTime-previousTime if loop end
} //loop end

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
