#include <CAN.h>
#include <ESP32Servo.h>
// #include <WiFi.h>
// #include "mpu9250-main.h"

#define TX_GPIO_NUM 5
#define RX_GPIO_NUM 4
#define STEER_PIN 27
#define ESC_PIN 19
#define SERIAL_DEBUG false
#define STATUS_FINE 0xAA
#define STATUS_ERROR 0xAE
#define STATUS_SUCCESS 0xA0

// const uint8_t intPin = 32;
const uint16_t defaultCmd = 1500;
const uint16_t maxSpeed = 100;
const uint8_t defaultSteer = 90;
const uint16_t delayMicro = 20;
// const char* ssid = "ESP";
// const char* password = "88880000";

// const uint8_t myLed = 13;   // Set up pin 13 led for toggling
// float pitch, yaw, roll;
// float ax, ay, az, gx, gy, gz, mx, my, mz;
int16_t currentSpeed = defaultCmd;
int16_t speedCmd = 0;
// int16_t targetSpeed = currentSpeed;
uint8_t currentSteer = defaultSteer;

// MPU9250 mpu9250;
Servo steer_sv;
Servo esc;

void setup() {
  // Wire.begin();
  //  TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);
  delay(100);
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);
  if (!CAN.begin(500E3)) {
    if(SERIAL_DEBUG){
      ledDebug(STATUS_ERROR);
      Serial.println("Starting CAN failed!"); 
    }
    while (1);
  } else {
    ledDebug(STATUS_SUCCESS);
    if(SERIAL_DEBUG)
      Serial.println("CAN wired successfully!");
  }
  pinMode(LED_BUILTIN, OUTPUT);
  // Set up the interrupt pin, its set as active high, push-pull
  // pinMode(intPin, INPUT);
  // digitalWrite(intPin, LOW);

  // pinMode(myLed, OUTPUT);
  // digitalWrite(myLed, HIGH);
  // delay(800);

  // mpu9250.setDebug(false); // MPU debug
  // if(!mpu9250.begin()) {
  //   ledDebug(STATUS_ERROR);
  // } else {
  //   ledDebug(STATUS_SUCCESS);
  //   if(SERIAL_DEBUG)
  //     Serial.println("MPU wired successfully!");
  // }
  esc.attach(ESC_PIN, 1000, 2000);
  steer_sv.attach(STEER_PIN);
  esc.setPeriodHertz(50);
  steer_sv.setPeriodHertz(50);
  esc.writeMicroseconds(defaultCmd);
  steer_sv.write(defaultSteer);

  // WiFi.softAP(ssid, password);

  // ledDebug(STATUS_SUCCESS);
  ledDebug(STATUS_FINE);
}

void loop() {
  processCANControl();
  // sendCanTest();
  // if(SERIAL_DEBUG) {
  //     Serial.print("Speed | steer: ");
  //     Serial.print(currentSpeed);
  //     Serial.print("     |     ");
  //     Serial.println(currentSteer);
  // }
  // steer_sv.write(currentSteer);
  // If intPin goes high, all data registers have new data
  // calculate pitch roll yaw
  // mpu9250.calculate(pitch, roll, yaw);
  // int16_t xPitch = (int16_t) pitch;
  // int16_t xRoll = (int16_t) roll;
  // int16_t xYaw = (int16_t) yaw + 180;
  // Serial.print(xPitch);
  // Serial.print(" - ");
  // Serial.print(xRoll);
  // Serial.print(" - ");
  // Serial.print(xYaw);
  // Serial.println();
  // sendCanIMU(xPitch, xRoll, xYaw);
  // mpu9250.calculateRaw(ax, ay, az, gx, gy, gz, mx, my, mz);
//   //example output: 
//  ax = 710.82 ay = -328.92 az = 552.86 mg
//  gx = 16.14 gy = -3.44 gz = 4.32 deg/s
//  mx = -173 my = 334 mz = -91 mG
  // sendCanIMURaw();
}

// ========================= CAN Bus Function =================================================//
void sendCanTest() {
  CAN.beginPacket(0x12);
  CAN.write('h');
  CAN.write('e');
  CAN.write('l');
  CAN.write('l');
  CAN.write('o');
  CAN.endPacket();
  // Serial.println("sent");
}
void sendCanIMU(int16_t pitch, int16_t roll, int16_t yaw) {
  CAN.beginPacket(0x12);
  CAN.write((uint8_t*)&pitch, 2);
  CAN.write((uint8_t*)&roll, 2);
  CAN.write((uint8_t*)&yaw, 2);
  CAN.endPacket();
  // Serial.println("sent");
}
// void sendCanIMURaw() {
//   int16_t xAx = 1000*ax;
//   int16_t xAy = 1000*ay;
//   int16_t xAz = 1000*az;
//   int16_t xGx = 100*gx;
//   int16_t xGy = 100*gy;
//   int16_t xGz = 100*gz;
//   int16_t xMx = mx;
//   int16_t xMy = my;
//   int16_t xMz = mz;
//   // Serial.print(ax); Serial.print("   "); Serial.print(ay); Serial.print("   "); Serial.print(az); Serial.print("   ");
//   // // Serial.println();
//   // Serial.print(gx); Serial.print("   "); Serial.print(gy); Serial.print("   "); Serial.print(gz); Serial.print("   ");
//   // // Serial.println();
//   // Serial.print(mx); Serial.print("   "); Serial.print(my); Serial.print("   ");  Serial.print(mz); Serial.print("   "); 
//   // Serial.println();
//   // Serial.print(xAx); Serial.print("   "); Serial.print(xAy); Serial.print("   "); Serial.print(xAz); Serial.print("   ");
//   // // Serial.println();
//   // Serial.print(xGx); Serial.print("   "); Serial.print(xGy); Serial.print("   "); Serial.print(xGz); Serial.print("   ");
//   // // Serial.println();
//   // Serial.print(xMx); Serial.print("   "); Serial.print(xMy); Serial.print("   ");  Serial.print(xMz); Serial.print("   "); 
//   // Serial.println();
//   CAN.beginPacket(0x11);
//   CAN.write((uint8_t*)&xAx, 2);
//   CAN.write((uint8_t*)&xAy, 2);
//   CAN.write((uint8_t*)&xAz, 2);
//   CAN.endPacket();
//   CAN.beginPacket(0x12);
//   CAN.write((uint8_t*)&xGx, 2);
//   CAN.write((uint8_t*)&xGy, 2);
//   CAN.write((uint8_t*)&xGz, 2);
//   CAN.endPacket();
//   CAN.beginPacket(0x13);
//   CAN.write((uint8_t*)&xMx, 2);
//   CAN.write((uint8_t*)&xMy, 2);
//   CAN.write((uint8_t*)&xMz, 2);
//   CAN.endPacket();
//   // Serial.println("sent");
// }

void processCANControl() {
  int packetSize = CAN.parsePacket();
  if (packetSize) {
    // received a packet
    // if(SERIAL_DEBUG) {
    //   Serial.print("Received size ");
    //   Serial.print(packetSize);
    //   Serial.print(" | Id: 0x");
    //   Serial.println(CAN.packetId(), HEX);
    //   // Serial.print("Data: ");
    //   // while (CAN.available()) {
    //   //   Serial.print(CAN.read(), HEX); 
    //   //   Serial.print(" ");
    //   // }
    //   Serial.println();
    // }
    if(CAN.packetId() == 0x21 && packetSize == 4) {
      uint8_t canMsg[4];
      int i = 0;
      while(CAN.available()) {
        canMsg[i++] = (uint8_t)CAN.read();
      }
      int16_t speed = canMsg[0] << 8 | canMsg[1];   
      int16_t steering = canMsg[2] << 8 | canMsg[3]; 
      speedCmd = defaultCmd + constrain(speed, -maxSpeed, maxSpeed);
      currentSteer = constrain(steering, 10, 170);
      steer_sv.write(currentSteer);
      controlSpeed();
      if(SERIAL_DEBUG) {
        Serial.print("Speed: ");
        Serial.println(speed);
        Serial.print("Steer: ");
        Serial.println(steering);
        Serial.println();
      }
    }
  }
}

// ========================= Control Function ==================================================//
void controlSpeed() {
  while (currentSpeed < speedCmd) {
    esc.writeMicroseconds(currentSpeed++);
    delayMicroseconds(delayMicro);
  }
  while (currentSpeed > speedCmd) {
    esc.writeMicroseconds(currentSpeed--);
    delayMicroseconds(delayMicro);
  }
}
// =========================== DEBUG ======================================= //
void ledDebug(uint8_t status) {
  switch (status) {
    case STATUS_FINE:
      digitalWrite(LED_BUILTIN, HIGH);
      return;
    case STATUS_ERROR:
      while(1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(150);
        digitalWrite(LED_BUILTIN, LOW);
        delay(150);
      }
      break;
    case STATUS_SUCCESS:
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
  }
}
