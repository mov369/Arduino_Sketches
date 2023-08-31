#include <SPI.h>
#include <mcp_can.h>
#include "lookup.c"   //lookup tables for mappings

#define CAN_500KBPS 16     // CAN baud rate
#define SPI_CS_PIN  17     // setup chip select pin for CAN module
#define LED 23              //LED pin to show program active
#define IND_L_CONT A2       // 2 in dbc - red on breadboard
#define IND_R_CONT A0        // 3 in dbc - green on breadboard
#define FLSENSE 12            // SW1 input
#define FRSENSE 6             // SW2 input
#define CH3ADC A1         /// Horn signal 
#define IND_L_SENSE 4     // Left indicator - 2 in dbc - red on breadboard
#define IND_R_SENSE 21    // Right indicator - 3 in dbc - green on breadboard
#define HEADLIGHTS 5      // Headlights input
#define PROGRAM_MODE false     // set this to enable the serial port (the code will not run if serial enabled but not connected)

MCP_CAN CAN(SPI_CS_PIN);            // Set CS pin

//Messages of interest
byte VehicleStateIndicators[8];
byte ByWireControlIndicators[8];
byte DriveByWireState[8];

//Timers
const unsigned long frameRepetitionTime=5;  // repetition rate of the frame in ms  SET TO 1s for INITIAL TESTS
unsigned long previousTime = 0; // last time for frame rate calcs
int indicator_timer_cal = 400; //This Cal controls how frequently the indicators pulse 
unsigned long previousMillis = 0; // last time for Indicator_High_B count

int can_error_timeout_cal = 400; //CAN timers for determining time since last CAN frame received
uint16_t can_rc_time_ms_113 = 0; 
uint16_t can_rc_time_ms_1A0 = 0;

byte count = 0; // rolling counter for alive signal

//Enables 
bool leds = true;
bool Autonomous_Mode_B = false;
bool Indicator_High_B = false;
bool CAN_Error_B = false;

//Preset int values
unsigned int CH3Input = 0;

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


void loop()
{
  ByWireControlIndicators[0];
  CAN_Error_B = false;
  unsigned long currentMillis = millis();
  //if (0 == millis() % indicator_timer_cal) {
    if (currentMillis - previousMillis >= indicator_timer_cal) {  
      if (Indicator_High_B) {
          Indicator_High_B = false;
        } 
      else {
          Indicator_High_B = true;
        }
        previousMillis = currentMillis;
    }
        Serial.println();
    Serial.print("Indicator Switch: ");
    Serial.println(Indicator_High_B);

  //if(Indicator_Switch = true){digitalWrite(IND_L_CONT, HIGH);}
  //if (currentTime-previousTime >= frameRepetitionTime) {

  //if(0 == (millis() % 20)){
    unsigned long currentTime = millis();
    if (currentTime-previousTime >= frameRepetitionTime);{
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
      
        if (canId == 0x1A0) {             // Handle the second message
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

          //can_rc_time_ms_1A0 = ms;
			    //if ((ms - can_rc_time_ms_1A0) >= can_error_timeout_cal) {
				  //CAN_Error_B = false;
			    //}else{
			  	//CAN_Error_B = false;
			    //}
          
        } 

        if (canId == 0x113) {
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
          size_t dataLength1 = sizeof(ByWireControlIndicators);
          uint16_t crcResult1 = crc16_xmodem(ByWireControlIndicators, dataLength1);
          byte highByte = buf[3];
          byte lowByte = buf[2];
          unsigned int combinedvalue;
          combinedvalue = (highByte << 8 )| lowByte;


          //can_rc_time_ms_113 = millis();
			    //if ((millis() - can_rc_time_ms_113) >= can_error_timeout_cal) {
				  //CAN_Error_B = false;
			    //}else{
			  	//CAN_Error_B = false;
			    //}

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
          
        } // End of message 0x113 extraction

       
    
      }

      //Serial.print("Data from ID: ");
      //Serial.println(CAN.getCanId(),HEX);
      //Serial.print("Buffer value: ");
      
      Serial.print("indicatorcommand Value: ");
      Serial.print(ByWireControlIndicators[0], DEC);
      Serial.println();

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
    size_t dataLength = sizeof(VehicleStateIndicators);
    uint16_t crcResult = crc16_xmodem(VehicleStateIndicators, dataLength);
    VehicleStateIndicators[2] = byte(crcResult & 0x00ff);
    VehicleStateIndicators[3] = byte((crcResult >> 8) & 0x00ff); 
    CAN.sendMsgBuf(0x102, 0, 8, VehicleStateIndicators);

    //previousTime = currentTime;   // set previous time for next timer loop

    Serial.println();
    Serial.print("MODE: ");
    Serial.print(Autonomous_Mode_B);

    //Serial.println();
    //Serial.println("millis: ");
    //Serial.println(millis());
  previousTime = currentTime;

  } //CAN Unpack & Send loop end
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




  
