/*
Project: IndiCANcillary Module (based on Arduino Leonardo)
Author: MJ
Date: 06/09/2023

Notes (Please read):

Program listens to the message "DriveByWireState (0x1A0)" & "ByWireControlIndicators (0x113)", and is continuously publishing message VehicleStateIndicators(0x102). 
Program only acts on the indicators control commands from message 0x113 if the drive by wire state in message 0x1A0 is set to autonomous (Byte 0 == 3).
Currently VehicleStateIndicators (0x102) only reports indicator state based on the condition of A2 & A0 pins, which are indicator control pins.
Sensing of the manual indicator switching through a stalk is not being reported, however untested bits of code for that are commented within this code.
Compared to S12Z-based CANcillary code, this program does not have CAN timeout implemented, but as with the above untested bits of code are commented within.

It's worth noting that it is possible to include CRC16 calculation/comparison on this board (function is included at the end of the program).
In fact, CRC16 is being generated for the message 
If necessary, this can be included as one of the && Conditions for acting upon the 0x1A0 and 0x113 commands in the instructions on lines 195-225.
Commented Serial.print-s are left for testing.
*/

#include <SPI.h>
#include <mcp_can.h>

#define CAN_500KBPS 16      // Setup CAN baud rate
#define SPI_CS_PIN  17      // Setup chip select pin for CAN module
#define LED 23              // LED pin to show program active
#define PROGRAM_MODE false  // Set this to enable the serial port (the code will not run if serial enabled but not connected)

#define IND_L_CONT A2       // True when "ByWireControlIndicators (0x113)" Byte 0 == 2 (as per eTransit_Alpha_DBW-28-06-23.dbc)
#define IND_R_CONT A0       // True when "ByWireControlIndicators (0x113)" Byte 0 == 3 (as per eTransit_Alpha_DBW-28-06-23.dbc)
//#define IND_L_SENSE 4     // Pin for sensing voltage on the indicators/stalk lines
//#define IND_R_SENSE 21    // Pin for sensing voltage on the indicators/stalk lines

MCP_CAN CAN(SPI_CS_PIN);    // Set CS pin

//CAN Messages
byte VehicleStateIndicators[8];
byte ByWireControlIndicators[8];
byte DriveByWireState[8];

//Timers
const unsigned long frameRepetitionTime=5;  // Frame repetition rate in ms
unsigned long previousTime = 0;             // End of loop time for frame rate calcs
int indicator_timer_cal = 400;              // This controls how frequently the indicators pulse 
unsigned long previousMillis = 0;           // Previous loop time for Indicator_High_B count

//int can_error_timeout_cal = 400;          // CAN timers for determining time since last CAN frame received
//uint16_t can_rc_time_ms_113 = 0;          // For message 113
//uint16_t can_rc_time_ms_1A0 = 0;          // For message 1A0

//Set initial values 
byte count = 0;                             // Rolling counter for VehicleStateIndicators (0x102) alive signal
bool leds = true;
bool Autonomous_Mode_B = false;
bool Indicator_High_B = false;
//bool CAN_Error_B = false;                 // for CAN Timer Error

void setup()
{
  // Set pins as digital inputs and outputs
  pinMode(LED, OUTPUT);                    
  //pinMode(FLSENSE, INPUT);                // Pin for sensing voltage on the indicators/stalk lines              
  //pinMode(FRSENSE, INPUT);                // Pin for sensing voltage on the indicators/stalk lines
  pinMode(IND_L_CONT, OUTPUT);
  pinMode(IND_R_CONT, OUTPUT);

  if (PROGRAM_MODE == true){
     Serial.begin(115200);
     while(!Serial);
  }
  while (CAN_OK != CAN.begin(CAN_500KBPS)){    // Init can bus : baudrate = 500k
      if (PROGRAM_MODE == true){
       Serial.println("CAN BUS FAIL!");
       delay(100);
      }
  }
  if (PROGRAM_MODE == true){
      Serial.println("CAN BUS OK!");
  } 
  
  // Set masks, both to 0x3ff
  CAN.init_Mask(0, 0, 0x3ff);  // there are 2 masks in mcp2515, you need to set both of them
  CAN.init_Mask(1, 0, 0x3ff);
  
  // Set filters to listen to messages of specific IDs
  CAN.init_Filt(1, 0, 0x113);
  CAN.init_Filt(2, 0, 0x1A0);
}


void loop()
{
  ByWireControlIndicators[0];

  //Swithing the Indicator_High_B to flash indicators at indicator_timer_cal rate
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= indicator_timer_cal) {   
    if (Indicator_High_B) {
      Indicator_High_B = false;
      } 
    else {
      Indicator_High_B = true;
      }
      previousMillis = currentMillis;
    }

  //Main Loop
  unsigned long currentTime = millis();
  if (currentTime-previousTime >= frameRepetitionTime);{
  digitalWrite(LED,leds);  // Light/extinguish the led on each loop cycle
  leds=!leds;

  unsigned char len = 0;
  unsigned char buf[8];
  
  if(CAN_MSGAVAIL == CAN.checkReceive())            // Enter loop if the data is on the CAN bus
    {
      CAN.readMsgBuf(&len, buf);                    // Read CAN bus data,  len: data length, buf: data buf
      unsigned long canId = CAN.getCanId();
      
      if (canId == 0x1A0) {                         // Handle the DriveByWireState (0x1A0) message
        //Serial.println("Received Message with ID 0x1A0:");
        //for (int i = 0; i < len; i++) {
        //  Serial.print(buf[i], HEX);
        //  Serial.print(" ");}
        DriveByWireState[0] = buf[0]; 
        DriveByWireState[1] = buf[1];
        DriveByWireState[2] = buf[2]; 
        DriveByWireState[3] = buf[3]; 
        DriveByWireState[4] = buf[4]; 
        DriveByWireState[5] = buf[5];
        DriveByWireState[6] = buf[6];
        DriveByWireState[7] = buf[7];
        if (DriveByWireState[0] == 3){
          Autonomous_Mode_B = true;
          }
        else {Autonomous_Mode_B = false;}
        //CRC CALCULATION FOR MESSAGE 0x1A0
        //size_t dataLength_1A0 = sizeof(DriveByWireState);
        //uint16_t crcResult_1A0 = crc16_xmodem(DriveByWireState, dataLength_1A0);
        //byte highByte = buf[3];
        //byte lowByte = buf[2];
        //unsigned int combinedvalue_1A0;
        //combinedvalue_1A0 = (highByte << 8 )| lowByte;

        //CAN MESSAGE TIMEOUT CALCULATION FOR MESSAGE 0x1A0
        //can_rc_time_ms_1A0 = ms;
			  //if ((ms - can_rc_time_ms_1A0) >= can_error_timeout_cal) {
				//  CAN_Error_B = false;
			  // }
        //else{
			  //  CAN_Error_B = false;
			  // }  
        } 

        if (canId == 0x113) {                   //Handle the DriveByWireState (0x1A0) message
          //Serial.println("Received Message with ID 0x113:");
          //for (int i = 0; i < len; i++) {
          //Serial.print(buf[i], HEX);
          //Serial.print(" ");
          //}
          
          ByWireControlIndicators[0] = buf[0]; 
          ByWireControlIndicators[1] = buf[1];
          ByWireControlIndicators[2] = 0; 
          ByWireControlIndicators[3] = 0; 
          ByWireControlIndicators[4] = buf[4]; 
          ByWireControlIndicators[5] = buf[5];
          ByWireControlIndicators[6] = buf[6];
          ByWireControlIndicators[7] = buf[7];

          //CRC CALCULATION FOR MESSAGE 0x113
          //size_t dataLength_113 = sizeof(ByWireControlIndicators);
          //uint16_t crcResult_113 = crc16_xmodem(ByWireControlIndicators, dataLength_113);
          //byte highByte = buf[3];
          //byte lowByte = buf[2];
          //unsigned int combinedvalue_113;
          //combinedvalue_113 = (highByte << 8 )| lowByte;

          //CAN MESSAGE TIMEOUT CALCULATION FOR MESSAGE 0x1A0
          //can_rc_time_ms_113 = millis();
			    //if ((millis() - can_rc_time_ms_113) >= can_error_timeout_cal) {
				  //CAN_Error_B = false;
			    //}else{
			  	//CAN_Error_B = false;
			    //}

          //Please add CAN_Error_B into the "if" conditions to implement CAN timer error tracking
          //For CRC16 error check, the calculated results for the "CRC CALCULATION FOR MESSAGE X" combinedvalue_x should be compared to the CRC value of the message.
          //If the above 2 are equal, set a separate bool to true and include as a condition in "if" statements below.
          if(ByWireControlIndicators[0] == 2 && Autonomous_Mode_B==true){
            if(Indicator_High_B==true){
              digitalWrite(IND_L_CONT, HIGH);
              digitalWrite(IND_R_CONT, LOW);
              }
            else{digitalWrite(IND_L_CONT, LOW);
            digitalWrite(IND_R_CONT, LOW);}
          } 
          else if(ByWireControlIndicators[0] == 3 && Autonomous_Mode_B==true){
            if(Indicator_High_B==true){
              digitalWrite(IND_L_CONT, LOW);
              digitalWrite(IND_R_CONT, HIGH);
              }
            else{digitalWrite(IND_L_CONT, LOW);
            digitalWrite(IND_R_CONT, LOW);}
          } 
          else if (ByWireControlIndicators[0] == 4 && Autonomous_Mode_B==true){
            if(Indicator_High_B==true){
              digitalWrite(IND_L_CONT, HIGH);
              digitalWrite(IND_R_CONT, HIGH);}
            else{
             digitalWrite(IND_L_CONT, LOW);
             digitalWrite(IND_R_CONT, LOW);}
          } 
          else {
            digitalWrite(IND_L_CONT, LOW);
            digitalWrite(IND_R_CONT, LOW);
          } // End of actions on 0x113 signals
          //Serial.print(ByWireControlIndicators[0]);
        } // End of message 0x113 data extraction
      }

      //Serial Prints for testing

      //Serial.print("Data from ID: ");
      //Serial.println(CAN.getCanId(),HEX);
      //Serial.print("Buffer value: ");
      
      //Serial.print("indicatorcommand Value: ");
      //Serial.print(ByWireControlIndicators[0], DEC);
      //Serial.println();

      //Serial.print("Actual CRC Bytes of the message from joystick: ");
      //Serial.print(buf[3],HEX);
      //Serial.print(" ");
      //Serial.println(buf[2],HEX);
      //Serial.print("Combined Value in dec - still message from joystick: ");
      //Serial.println(combinedvalue);
      //Serial.print("CRC Calculation on CANcillary module side: ");
      //Serial.println(crcResult1);
 

  //Cyclic State message, please add digitalRead(IND_L_SENSE) and digitalRead(IND_R_SENSE) into the "if" conditions to track & report the manual indicator line/stalk position.
  VehicleStateIndicators[0] = 0;
  if (digitalRead(IND_L_CONT) == false && digitalRead(IND_R_CONT) == false){
    VehicleStateIndicators[0] = 1;
  } 
  else if (digitalRead(IND_L_CONT) == true && digitalRead(IND_R_CONT) == false){
    VehicleStateIndicators[0] = 2;
  }
  else if (digitalRead(IND_L_CONT) == false && digitalRead(IND_R_CONT) == true){
    VehicleStateIndicators[0] = 3;
  }
  else if (digitalRead(IND_L_CONT) == true && digitalRead(IND_R_CONT) == true){
    VehicleStateIndicators[0] = 4;
  }
  else {
    VehicleStateIndicators[0] = 0;
  }
  VehicleStateIndicators[1] = count++;
  VehicleStateIndicators[2] = 0;
  VehicleStateIndicators[3] = 0;
  VehicleStateIndicators[4] = 0;
  VehicleStateIndicators[5] = 0;  
  VehicleStateIndicators[6] = 0;
  VehicleStateIndicators[7] = 0;
  size_t dataLength_102 = sizeof(VehicleStateIndicators);
  uint16_t crcResult_102 = crc16_xmodem(VehicleStateIndicators, dataLength_102);
  VehicleStateIndicators[2] = byte(crcResult_102 & 0x00ff);
  VehicleStateIndicators[3] = byte((crcResult_102 >> 8) & 0x00ff); 
  CAN.sendMsgBuf(0x102, 0, 8, VehicleStateIndicators);

  //Serial.println();
  //Serial.print("MODE: ");
  //Serial.print(Autonomous_Mode_B);

  //Serial.println();
  //Serial.println("millis: ");
  //Serial.println(millis());
  previousTime = currentTime;
  } //CAN Read, Unpack and Send loop end
} //Main loop end

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




  
