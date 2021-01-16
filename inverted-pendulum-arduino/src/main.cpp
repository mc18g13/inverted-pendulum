#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "DRV8825.h"
#include "i2c_t3.h"
#include "PIDController.h"


#define PRINT_WITH_NAME(NAME, X) Serial.print(NAME); Serial.print(" "); Serial.println(X);
#define NEW_LINE Serial.println();

/*

Stepping Modes for DRV8825

M0	   M1	    M2	  Microstep Resolution
Low	   Low	  Low	  Full step
High	 Low	  Low	  Half step
Low	   High	  Low	  1/4 step
High	 High	  Low	  1/8 step
Low	   Low	  High	1/16 step
High	 Low	  High	1/32 step
Low	   High	  High	1/32 step
High	 High	  High	1/32 step

*/

#define MICROSTEPS 32 // = 1/8 step resolution
#define MOTOR_STEPS 200 // Steps per revolution
#define RPM 50
#define DIR 22
#define STEP 23
#define BIT_MAX_12 4096
#define BIT_MAX_11 2048

DRV8825 stepper(MOTOR_STEPS, DIR, STEP);

uint16_t offset = 0;

float getCurrentPendulumAngle();
uint16_t getEncoderValue();
bool isPendulumAtRest();
void calibratePendulum();

void setup() {
    Serial.begin(9600);
    stepper.begin(RPM, MICROSTEPS);
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    delay(1000);
    while (!Serial); // Waiting for Serial Monitor
    delay(1000);
    Serial.println("started");
    calibratePendulum();
}

constexpr float32_t outputLimit = 75;
constexpr float32_t integralLimit = 75;
static_assert(outputLimit >= integralLimit, "cannot have and output limit for pid less than integral limit");

constexpr float32_t Kp = 10;
constexpr float32_t Ki = 0;
constexpr float32_t Kd = 5;

PIDController pendulumPIDController (Kp, Ki, Kd, outputLimit, integralLimit);

float currentAngle = 0;
void loop() {
  currentAngle = getCurrentPendulumAngle();
  if (abs(currentAngle) < 40.0f * PI / 180.0f) {
    pendulumPIDController.update(0, currentAngle);
    float output = pendulumPIDController.getOutput();
    // PRINT_WITH_NAME("Current Angle", currentAngle)
    PRINT_WITH_NAME("Current output", output)

    stepper.rotate(-output);
  } else {
    pendulumPIDController.reset();
    stepper.stop();
  }
  delay(10);
}

uint16_t getEncoderValue() {
  const byte ANGLE_ADDRESS_MSB = 0x0E;
  const byte DEVICE_ADDRESS = 0x36;

  Wire.beginTransmission(DEVICE_ADDRESS);
  delayMicroseconds(100);

  Wire.write(ANGLE_ADDRESS_MSB);
  delayMicroseconds(100);

  Wire.endTransmission();
  delayMicroseconds(100);

  Wire.requestFrom(DEVICE_ADDRESS, sizeof(uint16_t));
  delayMicroseconds(100);

  uint16_t t = millis();
  while (Wire.available() < 2 || (millis() - t > 1000)) {
    Serial.println("Waiting for encoder");
  }

  if (millis() - t > 1000) {
    Serial.println("broken encoder");
    stepper.stop();
    while(1);
  }

  byte data1 = Wire.receive();
  byte data2 = Wire.receive();
  return (data1 << 8) | data2;
}

float getCurrentPendulumAngle() {
  uint16_t encoder = getEncoderValue() - offset;
  // account for 16 bit overflow
  encoder %= BIT_MAX_12;
  PRINT_WITH_NAME("encoder ", encoder)
  
  float angleOutOf2PI = (float)encoder * 2 * PI / (float)BIT_MAX_12;
  float anglePlusMinusPI = angleOutOf2PI > PI ? angleOutOf2PI - (M_PI * 2) : angleOutOf2PI;
  return anglePlusMinusPI;
}

void calibratePendulum() {
  Serial.println("Calibrating pendulum");
  while (!isPendulumAtRest()) {
    Serial.println("Waiting for pendulum to be at rest to begin");
    delay(1000);
  };

  Serial.println("Finish calibrating pendulum");
  uint16_t startingEncoderValue = getEncoderValue();

  // encoder is 12 bit (4096 max). half way point is 11 bit (2^11 = 2048)
  offset = startingEncoderValue - BIT_MAX_11;
}

bool isPendulumAtRest() {
  const uint8_t COUNT_FOR_AT_REST_TEST = 50;
  uint16_t readings[COUNT_FOR_AT_REST_TEST];
  for (int i = 0; i < COUNT_FOR_AT_REST_TEST; i++) {
    readings[i] = getEncoderValue();
    delay(30);
  }

  float mean = 0;
  for (int i = 0; i < COUNT_FOR_AT_REST_TEST; i++) {
    mean += readings[i];
  }

  mean /= COUNT_FOR_AT_REST_TEST;

  float variance = 0;
  float differenceFromMeanForReading = 0;
  for (int i = 0; i < COUNT_FOR_AT_REST_TEST; i++) {
    differenceFromMeanForReading = (readings[i] - mean);
    variance += differenceFromMeanForReading * differenceFromMeanForReading;
  }

  variance /= COUNT_FOR_AT_REST_TEST;

  const float BOUNDARY_VARIANCE_FOR_AT_REST = 10;
  return (variance < BOUNDARY_VARIANCE_FOR_AT_REST);

}
