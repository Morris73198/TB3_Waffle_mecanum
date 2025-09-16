#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84;

const uint8_t FRONT_LEFT  = 1;  // 左前
const uint8_t FRONT_RIGHT = 2;  // 右前 
const uint8_t REAR_LEFT   = 3;  // 左后 
const uint8_t REAR_RIGHT  = 4;  // 右后

const float DXL_PROTOCOL_VERSION = 2.0;
const int BASE_SPEED = 80;  

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  DEBUG_SERIAL.begin(57600);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  DEBUG_SERIAL.println("控制");
  
  uint8_t motors[] = {FRONT_LEFT, FRONT_RIGHT, REAR_RIGHT, REAR_LEFT};
  
  for(int i = 0; i < 4; i++) {
    if(dxl.ping(motors[i])) {
      DEBUG_SERIAL.print("馬達 ");
      DEBUG_SERIAL.print(motors[i]);
      DEBUG_SERIAL.println(" 連接成功!");
      
      dxl.torqueOff(motors[i]);
      dxl.setOperatingMode(motors[i], OP_VELOCITY);
      dxl.torqueOn(motors[i]);
    } else {
      DEBUG_SERIAL.print("錯誤: 馬達 ");
      DEBUG_SERIAL.print(motors[i]);
      DEBUG_SERIAL.println(" 未找到!");
    }
  }
  
  DEBUG_SERIAL.println("\n控制命令:");
  DEBUG_SERIAL.println("w = 前進    s = 後退");
  DEBUG_SERIAL.println("a = 左移    d = 右移"); 
  DEBUG_SERIAL.println("q = 逆時針  e = 顺時針");
  DEBUG_SERIAL.println("x = 停止");
  DEBUG_SERIAL.println("1 = 左前移  3 = 右前移");
  DEBUG_SERIAL.println("z = 左後移  c = 右後移");
  
  stopAllMotors();
}

void loop() {
  if(DEBUG_SERIAL.available()) {
    char command = DEBUG_SERIAL.read();
    
    switch(command) {
      case 'w': 
        DEBUG_SERIAL.println("前進");
        moveForward();
        break;
        
      case 's':   
        DEBUG_SERIAL.println("後退");
        moveBackward();
        break;
        
      case 'a': 
        DEBUG_SERIAL.println("左移");
        moveLeft();
        break;
        
      case 'd': 
        DEBUG_SERIAL.println("右移");
        moveRight();
        break;
        
      case 'q': 
        DEBUG_SERIAL.println("逆時針旋轉");
        rotateCounterClockwise();
        break;
        
      case 'e': 
        DEBUG_SERIAL.println("顺時針旋轉");
        rotateClockwise();
        break;
        
      case '1': 
        DEBUG_SERIAL.println("左前移");
        moveFrontLeft();
        break;
        
      case '3': 
        DEBUG_SERIAL.println("右前移");
        moveFrontRight();
        break;
        
      case 'z': 
        DEBUG_SERIAL.println("左後移");
        moveBackLeft();
        break;
        
      case 'c': 
        DEBUG_SERIAL.println("右後移");
        moveBackRight();
        break;
        
      case 'x': 
        DEBUG_SERIAL.println("停止");
        stopAllMotors();
        break;
        
      default:
        DEBUG_SERIAL.println("???");
        break;
    }
  }
}

void moveForward() {
  dxl.setGoalVelocity(FRONT_LEFT,  BASE_SPEED);
  dxl.setGoalVelocity(FRONT_RIGHT, BASE_SPEED);
  dxl.setGoalVelocity(REAR_RIGHT,  BASE_SPEED);
  dxl.setGoalVelocity(REAR_LEFT,   BASE_SPEED);
}

void moveBackward() {
  dxl.setGoalVelocity(FRONT_LEFT,  -BASE_SPEED);
  dxl.setGoalVelocity(FRONT_RIGHT, -BASE_SPEED);
  dxl.setGoalVelocity(REAR_RIGHT,  -BASE_SPEED);
  dxl.setGoalVelocity(REAR_LEFT,   -BASE_SPEED);
}

void moveLeft() {
  dxl.setGoalVelocity(FRONT_LEFT,  -BASE_SPEED);
  dxl.setGoalVelocity(FRONT_RIGHT,  BASE_SPEED);
  dxl.setGoalVelocity(REAR_RIGHT,  -BASE_SPEED);
  dxl.setGoalVelocity(REAR_LEFT,    BASE_SPEED);
}

void moveRight() {
  dxl.setGoalVelocity(FRONT_LEFT,   BASE_SPEED);
  dxl.setGoalVelocity(FRONT_RIGHT, -BASE_SPEED);
  dxl.setGoalVelocity(REAR_RIGHT,   BASE_SPEED);
  dxl.setGoalVelocity(REAR_LEFT,   -BASE_SPEED);
}

void rotateClockwise() {
  dxl.setGoalVelocity(FRONT_LEFT,   BASE_SPEED);
  dxl.setGoalVelocity(FRONT_RIGHT, -BASE_SPEED);
  dxl.setGoalVelocity(REAR_RIGHT,  -BASE_SPEED);
  dxl.setGoalVelocity(REAR_LEFT,    BASE_SPEED);
}

void rotateCounterClockwise() {
  dxl.setGoalVelocity(FRONT_LEFT,  -BASE_SPEED);
  dxl.setGoalVelocity(FRONT_RIGHT,  BASE_SPEED);
  dxl.setGoalVelocity(REAR_RIGHT,   BASE_SPEED);
  dxl.setGoalVelocity(REAR_LEFT,   -BASE_SPEED);
}

void moveFrontLeft() {
  dxl.setGoalVelocity(FRONT_LEFT,  0);
  dxl.setGoalVelocity(FRONT_RIGHT, BASE_SPEED);
  dxl.setGoalVelocity(REAR_RIGHT,  0);
  dxl.setGoalVelocity(REAR_LEFT,   BASE_SPEED);
}

void moveFrontRight() {
  dxl.setGoalVelocity(FRONT_LEFT,  BASE_SPEED);
  dxl.setGoalVelocity(FRONT_RIGHT, 0);
  dxl.setGoalVelocity(REAR_RIGHT,  BASE_SPEED);
  dxl.setGoalVelocity(REAR_LEFT,   0);
}

void moveBackLeft() {
  dxl.setGoalVelocity(FRONT_LEFT,  -BASE_SPEED);
  dxl.setGoalVelocity(FRONT_RIGHT, 0);
  dxl.setGoalVelocity(REAR_RIGHT,  -BASE_SPEED);
  dxl.setGoalVelocity(REAR_LEFT,   0);
}

void moveBackRight() {
  dxl.setGoalVelocity(FRONT_LEFT,  0);
  dxl.setGoalVelocity(FRONT_RIGHT, -BASE_SPEED);
  dxl.setGoalVelocity(REAR_RIGHT,  0);
  dxl.setGoalVelocity(REAR_LEFT,   -BASE_SPEED);
}

void stopAllMotors() {
  dxl.setGoalVelocity(FRONT_LEFT,  0);
  dxl.setGoalVelocity(FRONT_RIGHT, 0);
  dxl.setGoalVelocity(REAR_RIGHT,  0);
  dxl.setGoalVelocity(REAR_LEFT,   0);
}