
#include <Wire.h>    //Include wire library 
#include <Servo.h>
#include "Adafruit_VL53L0X.h"
#include <cmath>
#include <vector>


// L298N module (1)
const ENA = 13;
const IN1 = 22;
const IN2 = 23;
const IN3 = 24;
const IN4 = 25;
// L298N module (2)
const ENB = 12;
const IN1_2 = 27;
const IN2_2 = 29;
const IN3_2 = 31;
const IN4_2 = 33;
// Lisar Distance Sensors
const byte SENSOR1_ADDRESS = 0x29;  // Change this to the address of your first sensor
Adafruit_VL53L0X sensor1;
const byte SENSOR2_ADDRESS = 0x30;  // Change this to the address of your first sensor
Adafruit_VL53L0X sensor2;
const byte SENSOR3_ADDRESS = 0x31;  // Change this to the address of your first sensor
Adafruit_VL53L0X sensor3;
const byte SENSOR4_ADDRESS = 0x32;  // Change this to the address of your first sensor
Adafruit_VL53L0X sensor4;
int data[90];
// MPU-6050 Distance Sensor
const byte mpu_6050 = 0x68;
// Servo Motor
Servo servo; 
const int servoPin = 30;
// Angles
float xangle,yangle,zangle;
// Layers Array



void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
  // Initialize Laser sensors
  sensor1.begin(SENSOR1_ADDRESS);
  sensor2.begin(SENSOR2_ADDRESS);
  sensor3.begin(SENSOR3_ADDRESS);
  sensor4.begin(SENSOR4_ADDRESS);

}
void loop() {
  // Stop all motors to get angles correctly
  stopMotors();
  delay(2000);
  getCurrentAcceleration(mpu_6050);
  if (xangle > 3 || xangle < -3) {
    for (;;) {
      runForward();
      delay(1000);
      stopMotors();
      delay(1000);
      getCurrentAcceleration(mpu_6050);
    }
  }else {
    bool is_detected = detectObstacleAt90Degree();
    if (is_detected) {
      startScanningLayers();
    }
  }
}

void getCurrentAcceleration (MPU_addr){
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  xangle = AcX / 182.04;
  yangle = AcY / 182.04;
  zangle = AcZ / 182.04;
}
double calculateMean(float arr[], int size) {
  double sum = 0;
  for (int i = 0; i < size; ++i) {
    sum += arr[i];
  }
  return sum / size;
}
struct LayerData {
  float layer[4];
  float delta_h;
};
// This function will make a full 90-degrees scanning and also filtering the data
void startScanningLayers() {
  servo.write(90);
  delay(500)
  vector<LayerData> result; // Vector to store the result
  // Read initial sensor values
  float layer[4];
  layer[0] = sensor1.readRangeSingleMillimeters();
  layer[1] = sensor2.readRangeSingleMillimeters();
  layer[2] = sensor3.readRangeSingleMillimeters();
  layer[3] = sensor4.readRangeSingleMillimeters();

  float d1 = calculateMean(layer, 4);
  LayerData data;
  for (int j = 0; j < 4; j++) {
    data.layer[j] = layer[j];
  }
  data.delta_h = 0;
  // Add to the result vector
  result.push_back(data);

  for (int i = 1; i < 90; i++) {
    servo.write(90 + i);

    // Read sensor values
    layer[0] = sensor1.readRangeSingleMillimeters();
    layer[1] = sensor2.readRangeSingleMillimeters();
    layer[2] = sensor3.readRangeSingleMillimeters();
    layer[3] = sensor4.readRangeSingleMillimeters();

    float d2 = calculateMean(layer, 4);

    // Calculate delta_h
    float radians = (i + 90) * M_PI / 180.0;
    float delta_h = d1 * d1 + d2 * d2 - 2.0 * d1 * d2 * cos(radians);

    // Create a LayerData object and store data
    LayerData data;
    for (int j = 0; j < 4; j++) {
      data.layer[j] = layer[j];
    }
    data.delta_h = delta_h;

    // Add to the result vector
    result.push_back(data);
    d1 = d2;
    // Send Data to Raspberry pi
    Serial.print(data.layer[0]);
    Serial.print(',');
    Serial.print(data.layer[1]);
    Serial.print(',');
    Serial.print(data.layer[2]);
    Serial.print(',');
    Serial.print(data.layer[3]);
    Serial.print(',');
    Serial.print(data.delta_h);
  }
}
bool detectObstacleAt90Degree(){
  servo.write(90);
  float layer[4];
  layer[0] = sensor1.readRangeSingleMillimeters();
  layer[1] = sensor2.readRangeSingleMillimeters();
  layer[2] = sensor3.readRangeSingleMillimeters();
  layer[3] = sensor4.readRangeSingleMillimeters();
  for (int i = 0; i < size; ++i) {
    if (layer[i] != 0) {
      return true; // Found a value not equal to zero
    }
  }
  return false;
}
