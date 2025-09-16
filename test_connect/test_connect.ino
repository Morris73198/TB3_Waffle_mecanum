#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84; 

const uint8_t MOTOR_ID = 1; 
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
void setup() {
  DEBUG_SERIAL.begin(57600);
  dxl.begin(1000000); 
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  dxl.setOperatingMode(MOTOR_ID, OP_VELOCITY);
  dxl.torqueOn(MOTOR_ID);
  
  DEBUG_SERIAL.println("Motor initialized!");
}

void loop() {
  dxl.setGoalVelocity(MOTOR_ID, 50); 
  delay(2000);
  
  dxl.setGoalVelocity(MOTOR_ID, 0);
  delay(1000);
  
  dxl.setGoalVelocity(MOTOR_ID, -50);
  delay(2000);
  
  dxl.setGoalVelocity(MOTOR_ID, 0);
  delay(1000);
}