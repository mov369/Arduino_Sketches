#include <SPI.h>
#include <mcp_can.h>
//#include "lookup.c"   //lookup tables for mappings

#define CAN_500KBPS 16     // CAN baud rate
#define SPI_CS_PIN  17     // setup chip select pin for CAN module
#define LED 23              //LED pin to show program active
#define IND_L_CONT A2       // ADC for steer pot 2 in dbc - red on breadboard
#define IND_R_CONT A0        // ADC for Throt 3 in dbc - green on breadboard
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
const long blinkInterval = 400;  // Blink interval in milliseconds
const unsigned long frameRepetitionTime=500;  // repetition rate of the frame in ms  SET TO 1s for INITIAL TESTS

unsigned long previousTime;                    // last time for frame rate calcs
byte count = 0;  // rolling counter for alive signal
uint16_t ms = 0;
int indicator_timer_cal = 1000; //This Cal controls how frequently the indicators pulse 
//uint16_t can_rc_time_ms_104 = 0; 
//uint16_t can_rc_time_ms_100 = 0;
//int can_error_timeout_cal = 200;

//Switches
bool leds = true;
bool Autonomous_Mode = false;
bool Indicator_Switch = false;

//Preset int values
unsigned int CH3Input = 0;
unsigned int indicatorcommand = 0;


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
  unsigned long currentTime = millis();
  if (0 == currentTime % indicator_timer_cal) { 
      if (Indicator_Switch) {
          Indicator_Switch = false;
        } 
      else {
          Indicator_Switch = true;
          delay(200);
        }
    }

    digitalWrite(IND_R_CONT, HIGH);
            digitalWrite(IND_L_CONT, LOW);

}