// LiteWing firmware ported from ESP32-S3 (Arduino, ESP-NOW joystick) to
// ESP8266, using WiFi SoftAP + UDP CRTP so it speaks the same protocol
// family as the LiteWing / ESP-Drone Android app.
//
// See README.md in this folder for wiring, setup and KNOWN LIMITATIONS -
// this has not been flight tested against the real app; you are the test
// pilot. Read it before you power on props.

#include <Wire.h>
#include <MPU6050.h>
#include "read_YPR.h"
#include "PID.h"
#include "motors.h"
#include "WiFi_CRTP.h"

// I2C pins for the MPU6050 on a Wemos D1 Mini / NodeMCU:
// D2 = GPIO4 (SDA), D1 = GPIO5 (SCL)
#define SDA_PIN 4
#define SCL_PIN 5

MPU6050 mpu;

unsigned long timer = 0;
unsigned long last_loop_time = 0;
float loop_time = 0.01; // Default 10ms

void setup() {
  Serial.begin(115200);

  // Bring up the SoftAP + UDP CRTP link the app connects to
  initWiFiCRTP();

  // Initialize I2C communication with specific pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // Fast I2C at 400kHz

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  mpu.setDLPFMode(MPU6050_DLPF_BW_42);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

  // Optional: Uncomment to calibrate the MPU6050
  calibrateMPU();

  // Initialize the Kalman filter
  setupMPU();

  // Initialize motors
  initializeMotors();

  Serial.println("Setup complete. Starting readings...");
  delay(1000);

  last_loop_time = micros();
}

void sendTelemetry() {
  Serial.print("TELEMETRY,");
  Serial.print(Desired_Roll_Angle); Serial.print(",");
  Serial.print(Desired_Pitch_Angle); Serial.print(",");
  Serial.print(Kalman_Roll_Angle); Serial.print(",");
  Serial.print(Kalman_Pitch_Angle); Serial.print(",");
  Serial.print(Kalman_Yaw_Angle); Serial.print(",");
  Serial.print(Roll_Rate); Serial.print(",");
  Serial.print(Pitch_Rate); Serial.print(",");
  Serial.print(Yaw_Rate); Serial.print(",");
  Serial.print(Desired_Roll_Rate); Serial.print(",");
  Serial.print(Desired_Pitch_Rate); Serial.print(",");
  Serial.print(Desired_Yaw_Rate); Serial.print(",");
  Serial.print(Roll_PID_Output); Serial.print(",");
  Serial.print(Pitch_PID_Output); Serial.print(",");
  Serial.print(Yaw_PID_Output); Serial.print(",");
  Serial.print(throttle); Serial.print(",");
  Serial.print(motor1_value); Serial.print(",");
  Serial.print(motor2_value); Serial.print(",");
  Serial.print(motor3_value); Serial.print(",");
  Serial.println(motor4_value);
}

void loop() {
  // Keep the UDP/CRTP link serviced every iteration (non-blocking)
  handleWiFiCRTP();

  unsigned long current_time = micros();
  loop_time = (current_time - last_loop_time) / 1000000.0;
  last_loop_time = current_time;

  if (loop_time > 0.1) loop_time = 0.01;

  updateAngles();

  calculateAnglePID();
  calculateRatePID();
  calculateMotorOutputs();
  writeToMotors();

  if (millis() - timer > 50) {
    sendTelemetry();
    timer = millis();
  }
}
