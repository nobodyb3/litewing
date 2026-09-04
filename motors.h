#ifndef MOTORS_H
#define MOTORS_H

#include "PID.h"

// ---- PWM Configuration ------------------------------------------------
// ESP8266 has no hardware LEDC like the ESP32-S3 original. analogWrite()
// uses a software-timer waveform generator instead. 20kHz keeps it out of
// the audible range on most cores, but if you get jittery/noisy motors,
// drop this to 1000-4000 Hz first when debugging.
#define FREQ 20000
#define PWM_RANGE 1023      // analogWriteRange - matches original 0-1023 duty math
#define MIN_DUTY_CYCLE 100
#define MIN_THROTTLE 1100   // Minimum throttle (us-style value) to arm motors
#define MAX_THROTTLE 1900   // Maximum throttle value

// ---- Motor pins (Wemos D1 Mini / NodeMCU) -----------------------------
// Chosen to avoid the ESP8266 boot-strapping pins (GPIO0/2/15 have boot
// requirements) and to leave D1/D2 (GPIO5/GPIO4) free for I2C to the MPU6050.
// D5=GPIO14, D6=GPIO12, D7=GPIO13, D8=GPIO15 (must be LOW at boot -> safe
// default state for an armed-low motor output).
#define MOTOR_1 14  // D5 - Front right
#define MOTOR_2 12  // D6 - Back left
#define MOTOR_3 13  // D7 - Back right
#define MOTOR_4 15  // D8 - Front left

// Default throttle (for bench testing only - normally comes from the app)
int throttle = 100;

int motor1_value = 0;
int motor2_value = 0;
int motor3_value = 0;
int motor4_value = 0;

void initializeMotors() {
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(MOTOR_3, OUTPUT);
  pinMode(MOTOR_4, OUTPUT);

  analogWriteRange(PWM_RANGE);
  analogWriteFreq(FREQ);

  analogWrite(MOTOR_1, 0);
  analogWrite(MOTOR_2, 0);
  analogWrite(MOTOR_3, 0);
  analogWrite(MOTOR_4, 0);

  // Spin each motor briefly so you can confirm wiring order:
  // brown, pink, blue, yellow
  analogWrite(MOTOR_1, 50); delay(100); analogWrite(MOTOR_1, 0);
  analogWrite(MOTOR_2, 50); delay(100); analogWrite(MOTOR_2, 0);
  analogWrite(MOTOR_3, 50); delay(100); analogWrite(MOTOR_3, 0);
  analogWrite(MOTOR_4, 50); delay(100); analogWrite(MOTOR_4, 0);
}

void calculateMotorOutputs() {
  if (throttle > MAX_THROTTLE) {
    throttle = MAX_THROTTLE;
  }

  /* Motor layout:
      M4(FL)    M1(FR)
         \      /
          \    /
           ----
          /    \
         /      \
      M3(BL)    M2(BR)
  */

  float m1 = throttle - Roll_PID_Output - Pitch_PID_Output + Yaw_PID_Output;  // Front Right
  float m2 = throttle - Roll_PID_Output + Pitch_PID_Output - Yaw_PID_Output;  // Back Right
  float m3 = throttle + Roll_PID_Output + Pitch_PID_Output + Yaw_PID_Output;  // Back Left
  float m4 = throttle + Roll_PID_Output - Pitch_PID_Output - Yaw_PID_Output;  // Front Left

  motor1_value = constrain((int)map(m1, 1000, 2000, 0, PWM_RANGE), 0, PWM_RANGE);
  motor2_value = constrain((int)map(m2, 1000, 2000, 0, PWM_RANGE), 0, PWM_RANGE);
  motor3_value = constrain((int)map(m3, 1000, 2000, 0, PWM_RANGE), 0, PWM_RANGE);
  motor4_value = constrain((int)map(m4, 1000, 2000, 0, PWM_RANGE), 0, PWM_RANGE);
}

void writeToMotors() {
  if (throttle > MIN_THROTTLE) {
    analogWrite(MOTOR_1, motor1_value);
    analogWrite(MOTOR_2, motor2_value);
    analogWrite(MOTOR_3, motor3_value);
    analogWrite(MOTOR_4, motor4_value);
  } else {
    analogWrite(MOTOR_1, 0);
    analogWrite(MOTOR_2, 0);
    analogWrite(MOTOR_3, 0);
    analogWrite(MOTOR_4, 0);

    Roll_Angle_I_term = 0;
    Pitch_Angle_I_term = 0;
    Roll_Rate_I_term = 0;
    Pitch_Rate_I_term = 0;
    Yaw_Rate_I_term = 0;
  }
}

#endif // MOTORS_H
