#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  DEBUG_SERIAL.begin(57600);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
  
  DEBUG_SERIAL.println("=== Motor ID Changer ===");
  DEBUG_SERIAL.println("Connect ONE motor at a time");
  delay(2000);
  
  for(int id = 1; id <= 10; id++) {
    if(dxl.ping(id)) {
      DEBUG_SERIAL.print("Found motor with ID: ");
      DEBUG_SERIAL.println(id);
      
      int newID = 4;
      if(dxl.setID(id, newID)) {
        DEBUG_SERIAL.print("Successfully changed ID to: ");
        DEBUG_SERIAL.println(newID);
      } else {
        DEBUG_SERIAL.println("Failed to change ID");
      }
      break;
    }
  }
}

void loop() {
 
}