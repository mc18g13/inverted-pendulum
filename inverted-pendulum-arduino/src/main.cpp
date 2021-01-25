#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "DRV8825.h"
#include "i2c_t3.h"
#include "PIDController.h"

#define BIT_MAX_12 4096
#define BIT_MAX_11 2048


#define NEW_LINE Serial.println();

uint16_t offset = 0;

int16_t getCurrentPendulumAngle(float& anglePlusMinusPI);
int16_t getEncoderValue();
bool isPendulumAtRest();
void calibratePendulum();

#define CLOCKWISE 22
#define ANTI_CLOCKWISE 23
#define SPEED 21

// IN1	IN2	 IN3	IN4	Direction
// 0	  0	   0	   0	Stop
// 1	  0	   1	   0	Forward
// 0	  1  	 0	   1	Reverse
// 1	  0    0	   1	Left
// 0	  1	   1	   0	Right

void setup() {
    Serial.begin(115200);
    while (!Serial); // Waiting for Serial Monitor

    pinMode(CLOCKWISE, OUTPUT);
    pinMode(ANTI_CLOCKWISE, OUTPUT);
    pinMode(SPEED, OUTPUT);
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    delay(1000);
    Serial.println("started");
    const int PWM_FREQ_HZ = 1000;
    analogWriteFrequency(SPEED, PWM_FREQ_HZ);
    calibratePendulum();
}

constexpr float32_t outputLimit = 255;
constexpr float32_t integralLimit = 255;
static_assert(outputLimit >= integralLimit, "cannot have and output limit for pid less than integral limit");

constexpr float32_t Kp = 500;
constexpr float32_t Ki = 1.5;
constexpr float32_t Kd = 10000;

PIDController pendulumPIDController (Kp, Ki, Kd, outputLimit, integralLimit);

float currentAngle = 0;
uint32_t time = 0;
#define MIN_FOR_MOTOR_MOVEMENT 50

float fifteenDegreesAsRads = 30.0f * PI / 180.0f;
void loop() {

  auto reset = [&]() {
    pendulumPIDController.reset();
    digitalWrite(SPEED, LOW);
    digitalWrite(CLOCKWISE, LOW);
    digitalWrite(ANTI_CLOCKWISE, LOW);
  };

  uint8_t returnCode = getCurrentPendulumAngle(currentAngle);
  // PRINT_WITH_NAME("angle ", currentAngle);
  if (abs(currentAngle) < fifteenDegreesAsRads && returnCode > 0) {
    pendulumPIDController.update(0, currentAngle);
    float output = pendulumPIDController.getOutput();
    // PRINT_WITH_NAME("Current Angle", currentAngle)
    PRINT_WITH_NAME("Current output", output)
    // PRINT_WITH_NAME("time for move",  stepper.getTimeForMove(stepper.calcStepsForRotation(-output)))
    time = micros();
    if (output > 0) {
      analogWrite(SPEED, MIN_FOR_MOTOR_MOVEMENT + output);
      digitalWrite(ANTI_CLOCKWISE, LOW);
      digitalWrite(CLOCKWISE, HIGH);
    } else {
      analogWrite(SPEED, MIN_FOR_MOTOR_MOVEMENT + abs(output));
      digitalWrite(CLOCKWISE, LOW);
      digitalWrite(ANTI_CLOCKWISE, HIGH);
    }

  } else {
    reset();
  }

  // Serial.print(micros()); Serial.print(" "); Serial.println(getEncoderValue());
}

int16_t getEncoderValue() {
  const byte ANGLE_ADDRESS_MSB = 0x0E;
  const byte DEVICE_ADDRESS = 0x36;

  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(ANGLE_ADDRESS_MSB);
  Wire.endTransmission();
  Wire.requestFrom(DEVICE_ADDRESS, sizeof(uint16_t));

  uint16_t t = millis();
  while (Wire.available() < 2 && (millis() - t < 10000)) {
    Serial.println("Waiting for encoder");
  }

  if (millis() - t > 10000) {
    Serial.println("broken encoder");
    Wire.resetBus();
    Wire.finish();
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    return -1;
  }

  byte data1 = Wire.receive();
  byte data2 = Wire.receive();
  return (data1 << 8) | data2;
}

int16_t getCurrentPendulumAngle(float& anglePlusMinusPI) {
  int16_t encoder = getEncoderValue();
  if (encoder > 0) {
    
    // encoder = encoder - (offset - 5 * PI / 180);
    encoder = encoder - (offset);

    // account for 16 bit overflow
    encoder %= BIT_MAX_12;
    
    float angleOutOf2PI = (float)encoder * 2 * PI / (float)BIT_MAX_12;
    anglePlusMinusPI = angleOutOf2PI > PI ? angleOutOf2PI - (M_PI * 2) : angleOutOf2PI;
    return 1;
  } else {
    return encoder;
  }
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
  PRINT_WITH_NAME("Variance", variance);
  const float BOUNDARY_VARIANCE_FOR_AT_REST = 10;
  return (variance < BOUNDARY_VARIANCE_FOR_AT_REST);

}
