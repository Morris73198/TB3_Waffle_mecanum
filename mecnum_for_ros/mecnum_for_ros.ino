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
const float WHEEL_RADIUS = 0.05;      // R: 輪子半徑 (米)
const float CHASSIS_LENGTH = 0.15;    // L: 車體長度半徑 (米)
const float CHASSIS_WIDTH = 0.15;     // W: 車體寬度半徑 (米)

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 用於接收序列埠數據
union FloatBytes {
  float value;
  byte bytes[4];
};

void setup() {
  DEBUG_SERIAL.begin(57600);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  DEBUG_SERIAL.println("=== 麥克納姆輪 ROS2 控制 ===");
  
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
  
  DEBUG_SERIAL.println("等待 ROS2 速度命令...");
  DEBUG_SERIAL.println("格式: [起始位元 0xFF] [vx(4 bytes)] [vy(4 bytes)] [omega(4 bytes)]");
  
  stopAllMotors();
}

void loop() {
  // 檢查是否有足夠的數據 (1 起始位元 + 12 bytes = 13 bytes)
  if(DEBUG_SERIAL.available() >= 13) {
    // 讀取起始位元
    byte startByte = DEBUG_SERIAL.read();
    
    if(startByte == 0xFF) {
      // 讀取 vx (4 bytes)
      FloatBytes vx_data;
      for(int i = 0; i < 4; i++) {
        vx_data.bytes[i] = DEBUG_SERIAL.read();
      }
      
      // 讀取 vy (4 bytes)
      FloatBytes vy_data;
      for(int i = 0; i < 4; i++) {
        vy_data.bytes[i] = DEBUG_SERIAL.read();
      }
      
      // 讀取 omega (4 bytes)
      FloatBytes omega_data;
      for(int i = 0; i < 4; i++) {
        omega_data.bytes[i] = DEBUG_SERIAL.read();
      }
      
      // 設定速度
      setVelocity(vx_data.value, vy_data.value, omega_data.value);
      
      // 輸出調試信息
      DEBUG_SERIAL.print("接收速度: vx=");
      DEBUG_SERIAL.print(vx_data.value, 3);
      DEBUG_SERIAL.print(" vy=");
      DEBUG_SERIAL.print(vy_data.value, 3);
      DEBUG_SERIAL.print(" omega=");
      DEBUG_SERIAL.println(omega_data.value, 3);
    } else {
      // 清空緩衝區
      while(DEBUG_SERIAL.available()) {
        DEBUG_SERIAL.read();
      }
    }
  }
}

void setVelocity(float vx, float vy, float omega) {
  float L_plus_W = CHASSIS_LENGTH + CHASSIS_WIDTH;
  
  // 根據運動學公式計算各輪角速度 (rad/s)
  float w1 = (vx - vy - L_plus_W * omega) / WHEEL_RADIUS;  // 左前 (FL)
  float w2 = (vx + vy + L_plus_W * omega) / WHEEL_RADIUS;  // 右前 (FR)
  float w3 = (vx + vy - L_plus_W * omega) / WHEEL_RADIUS;  // 左後 (RL)
  float w4 = (vx - vy + L_plus_W * omega) / WHEEL_RADIUS;  // 右後 (RR)
  
  // 轉換為 Dynamixel 速度單位
  float conversion = 9.5493 / 0.229;  // ≈ 41.7
  
  int speed1 = (int)(w1 * conversion);
  int speed2 = (int)(w2 * conversion);
  int speed3 = (int)(w3 * conversion);
  int speed4 = (int)(w4 * conversion);
  
  // 設置各馬達速度
  dxl.setGoalVelocity(FRONT_LEFT,  speed1);
  dxl.setGoalVelocity(FRONT_RIGHT, speed2);
  dxl.setGoalVelocity(REAR_LEFT,   speed3);
  dxl.setGoalVelocity(REAR_RIGHT,  speed4);
}

void stopAllMotors() {
  setVelocity(0, 0, 0);
}