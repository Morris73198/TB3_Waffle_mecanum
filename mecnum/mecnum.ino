#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84;

const uint8_t FRONT_LEFT  = 1;  // ω1 左前 (FL)
const uint8_t FRONT_RIGHT = 2;  // ω2 右前 (FR)
const uint8_t REAR_LEFT   = 3;  // ω3 左後 (RL)
const uint8_t REAR_RIGHT  = 4;  // ω4 右後 (RR)

const float DXL_PROTOCOL_VERSION = 2.0;

// 麥克納姆輪運動學參數
const float WHEEL_RADIUS = 0.05;      // R: 輪子半徑 (米) - 根據實際調整
const float CHASSIS_LENGTH = 0.15;    // L: 車體長度半徑 (米) - 根據實際調整
const float CHASSIS_WIDTH = 0.15;     // W: 車體寬度半徑 (米) - 根據實際調整
const float BASE_SPEED = 0.3;         // 基礎線速度 (m/s) - 根據實際調整

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  DEBUG_SERIAL.begin(57600);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  DEBUG_SERIAL.println("=== 麥克納姆輪全向運動控制 ===");
  
  uint8_t motors[] = {FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT};
  
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
  DEBUG_SERIAL.println("w = 前進 (vx+)    s = 後退 (vx-)");
  DEBUG_SERIAL.println("a = 左移 (vy+)    d = 右移 (vy-)"); 
  DEBUG_SERIAL.println("q = 逆時針 (ω+)   e = 順時針 (ω-)");
  DEBUG_SERIAL.println("x = 停止");
  DEBUG_SERIAL.println("1 = 左前移    3 = 右前移");
  DEBUG_SERIAL.println("z = 左後移    c = 右後移");
  DEBUG_SERIAL.println("r = 前進+順時針  f = 前進+逆時針");
  
  stopAllMotors();
}

void loop() {
  if(DEBUG_SERIAL.available()) {
    char command = DEBUG_SERIAL.read();
    
    switch(command) {
      case 'w': 
        DEBUG_SERIAL.println("前進 (vx+)");
        setVelocity(BASE_SPEED, 0, 0);
        break;
        
      case 's':   
        DEBUG_SERIAL.println("後退 (vx-)");
        setVelocity(-BASE_SPEED, 0, 0);
        break;
        
      case 'a': 
        DEBUG_SERIAL.println("左移 (vy+)");
        setVelocity(0, BASE_SPEED, 0);
        break;
        
      case 'd': 
        DEBUG_SERIAL.println("右移 (vy-)");
        setVelocity(0, -BASE_SPEED, 0);
        break;
        
      case 'q': 
        DEBUG_SERIAL.println("逆時針旋轉 (ω+)");
        setVelocity(0, 0, 1.0);
        break;
        
      case 'e': 
        DEBUG_SERIAL.println("順時針旋轉 (ω-)");
        setVelocity(0, 0, -1.0);
        break;
        
      case '1': 
        DEBUG_SERIAL.println("左前移 (vx+, vy+)");
        setVelocity(BASE_SPEED * 0.707, BASE_SPEED * 0.707, 0);
        break;
        
      case '3': 
        DEBUG_SERIAL.println("右前移 (vx+, vy-)");
        setVelocity(BASE_SPEED * 0.707, -BASE_SPEED * 0.707, 0);
        break;
        
      case 'z': 
        DEBUG_SERIAL.println("左後移 (vx-, vy+)");
        setVelocity(-BASE_SPEED * 0.707, BASE_SPEED * 0.707, 0);
        break;
        
      case 'c': 
        DEBUG_SERIAL.println("右後移 (vx-, vy-)");
        setVelocity(-BASE_SPEED * 0.707, -BASE_SPEED * 0.707, 0);
        break;
        
      case 'r': 
        DEBUG_SERIAL.println("前進+順時針");
        setVelocity(BASE_SPEED, 0, -0.5);
        break;
        
      case 'f': 
        DEBUG_SERIAL.println("前進+逆時針");
        setVelocity(BASE_SPEED, 0, 0.5);
        break;
        
      case 'x': 
        DEBUG_SERIAL.println("停止");
        stopAllMotors();
        break;
        
      default:
        DEBUG_SERIAL.println("未知命令");
        break;
    }
  }
}

/**
 * 麥克納姆輪運動學逆解
 * 根據公式計算四個輪子的角速度
 * 
 * [ω1]   1/R * [ 1  -1  -(L+W) ] [vx ]
 * [ω2] =       [ 1   1   (L+W) ] [vy ]
 * [ω3]         [ 1   1  -(L+W) ] [ω  ]
 * [ω4]         [ 1  -1   (L+W) ]
 * 
 * @param vx: 車體前進速度 (m/s)
 * @param vy: 車體左移速度 (m/s)  
 * @param omega: 車體角速度 (rad/s)
 */
void setVelocity(float vx, float vy, float omega) {
  float L_plus_W = CHASSIS_LENGTH + CHASSIS_WIDTH;
  
  // 根據運動學公式計算各輪角速度 (rad/s)
  float w1 = (vx - vy - L_plus_W * omega) / WHEEL_RADIUS;  // 左前 (FL)
  float w2 = (vx + vy + L_plus_W * omega) / WHEEL_RADIUS;  // 右前 (FR)
  float w3 = (vx + vy - L_plus_W * omega) / WHEEL_RADIUS;  // 左後 (RL)
  float w4 = (vx - vy + L_plus_W * omega) / WHEEL_RADIUS;  // 右後 (RR)
  
  // 轉換為 Dynamixel 速度單位 (0.229 rpm)
  // 角速度 (rad/s) -> RPM: ω_rad/s * 60 / (2π) = ω_rad/s * 9.5493
  // RPM -> Dynamixel單位: RPM / 0.229
  float conversion = 9.5493 / 0.229;  // ≈ 41.7
  
  int speed1 = (int)(w1 * conversion);
  int speed2 = (int)(w2 * conversion);
  int speed3 = (int)(w3 * conversion);
  int speed4 = (int)(w4 * conversion);
  
  // 輸出調試信息
  DEBUG_SERIAL.print("速度命令 - FL:");
  DEBUG_SERIAL.print(speed1);
  DEBUG_SERIAL.print(" FR:");
  DEBUG_SERIAL.print(speed2);
  DEBUG_SERIAL.print(" RL:");
  DEBUG_SERIAL.print(speed3);
  DEBUG_SERIAL.print(" RR:");
  DEBUG_SERIAL.println(speed4);
  
  // 設置各馬達速度
  dxl.setGoalVelocity(FRONT_LEFT,  speed1);
  dxl.setGoalVelocity(FRONT_RIGHT, speed2);
  dxl.setGoalVelocity(REAR_LEFT,   speed3);
  dxl.setGoalVelocity(REAR_RIGHT,  speed4);
}

void stopAllMotors() {
  setVelocity(0, 0, 0);
}