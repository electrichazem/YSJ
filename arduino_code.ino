
#include <Wire.h>    //Include wire library 
#include <Servo.h>
#include "Adafruit_VL53L0X.h"



// L298N module (1)
const int ENA = 13;
const int IN1 = 22;
const int IN2 = 23;
const int IN3 = 24;
const int IN4 = 25;
// L298N module (2)
const int ENB = 12;
const int IN1_2 = 27;
const int IN2_2 = 29;
const int IN3_2 = 31;
const int IN4_2 = 33;
// Lisar Distance Sensors
const byte SENSOR1_ADDRESS = 0x29;  // Change this to the address of your first sensor
Adafruit_VL53L0X sensor1 = Adafruit_VL53L0X();
const byte SENSOR2_ADDRESS = 0x30;  // Change this to the address of your first sensor
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
const byte SENSOR3_ADDRESS = 0x31;  // Change this to the address of your first sensor
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();
const byte SENSOR4_ADDRESS = 0x32;  // Change this to the address of your first sensor
Adafruit_VL53L0X sensor4 = Adafruit_VL53L0X();
int data[90];
// MPU-6050 Distance Sensor
const byte MPU_addr = 0x68;
// Servo Motor
Servo servo; 
const int servoPin = 30;
// Angles
float xangle,yangle,zangle;
// Layers Array


  VL53L0X_RangingMeasurementData_t measure1;
  VL53L0X_RangingMeasurementData_t measure2;
  VL53L0X_RangingMeasurementData_t measure3;
  VL53L0X_RangingMeasurementData_t measure4;

  struct LayerData {
  float layer[4];
  float delta_h;
};
#define NUM_LAYERS 90
LayerData result[NUM_LAYERS];

// Define LayerData struct



void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
  // Initialize Laser sensors
  sensor1.begin(SENSOR1_ADDRESS);
  sensor2.begin(SENSOR2_ADDRESS);
  sensor3.begin(SENSOR3_ADDRESS);
  sensor4.begin(SENSOR4_ADDRESS);

  sensor1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  sensor2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  sensor3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  sensor4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!


}
void loop() {
  // Stop all motors to get angles correctly
  stopMotors();
  delay(2000);
  getCurrentAcceleration();
  if (xangle > 3 || xangle < -3) {
    for (;;) {
      runForward();
      delay(1000);
      stopMotors();
      delay(1000);
      getCurrentAcceleration();
    }
  } else {
    bool is_detected = detectObstacleAt90Degree();
    if (is_detected) {
      startScanningLayers();
    }
  }
}

void getCurrentAcceleration() {
  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
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
void startScanningLayers() {
  servo.write(90);
  delay(500);

  // Read initial sensor values
  VL53L0X_RangingMeasurementData_t measure1;
  sensor1.rangingTest(&measure1, false);
  VL53L0X_RangingMeasurementData_t measure2;
  sensor2.rangingTest(&measure2, false);
  VL53L0X_RangingMeasurementData_t measure3;
  sensor3.rangingTest(&measure3, false);
  VL53L0X_RangingMeasurementData_t measure4;
  sensor4.rangingTest(&measure4, false);

  float layer[4];
  layer[0] = measure1.RangeMilliMeter;
  layer[1] = measure2.RangeMilliMeter;
  layer[2] = measure3.RangeMilliMeter;
  layer[3] = measure4.RangeMilliMeter;

  float d1 = calculateMean(layer, 4);

  for (int j = 0; j < 4; j++) {
    result[0].layer[j] = layer[j];
  }
  result[0].delta_h = 0;

  for (int i = 1; i < NUM_LAYERS; i++) {
    servo.write(90 + i);

    // Read sensor values
    VL53L0X_RangingMeasurementData_t measure1;
    sensor1.rangingTest(&measure1, false);
    VL53L0X_RangingMeasurementData_t measure2;
    sensor2.rangingTest(&measure2, false);
    VL53L0X_RangingMeasurementData_t measure3;
    sensor3.rangingTest(&measure3, false);
    VL53L0X_RangingMeasurementData_t measure4;
    sensor4.rangingTest(&measure4, false);

    layer[0] = measure1.RangeMilliMeter;
    layer[1] = measure2.RangeMilliMeter;
    layer[2] = measure3.RangeMilliMeter;
    layer[3] = measure4.RangeMilliMeter;

    float d2 = calculateMean(layer, 4);

    // Calculate delta_h
    float radians = (i + 90) * M_PI / 180.0;
    float delta_h = d1 * d1 + d2 * d2 - 2.0 * d1 * d2 * cos(radians);

    for (int j = 0; j < 4; j++) {
      result[i].layer[j] = layer[j];
    }
    result[i].delta_h = delta_h;

    d1 = d2;
  }

  // Send Data to Raspberry pi
  for (int i = 0; i < NUM_LAYERS; i++) {
    Serial.print(result[i].layer[0]);
    Serial.print(',');
    Serial.print(result[i].layer[1]);
    Serial.print(',');
    Serial.print(result[i].layer[2]);
    Serial.print(',');
    Serial.print(result[i].layer[3]);
    Serial.print(',');
    Serial.print(result[i].delta_h);
    Serial.println();
  }
}
bool detectObstacleAt90Degree(){
  servo.write(90);
  float layer[4];
  layer[0] = measure1.RangeMilliMeter;
  layer[1] = measure2.RangeMilliMeter;
  layer[2] = measure3.RangeMilliMeter;
  layer[3] = measure4.RangeMilliMeter;
  for (int i = 0; i < 4; ++i) {
    if (layer[i] != 0) {
      return true; // Found a value not equal to zero
    }
  }
  return false;
}

void stopMotors() {
  // Stop Motors
}
void runForward() {
  // run Forward
}
