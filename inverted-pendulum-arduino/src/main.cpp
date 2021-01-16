#include "Arduino.h"

// #define dirPin 22
// #define stepPin 23
// #define stepsPerRevolution 200
// void setup() {
//   Serial.begin(1);
//   // Declare pins as output:
//   pinMode(stepPin, OUTPUT);
//   pinMode(dirPin, OUTPUT);
// }
// void loop() {
//   Serial.println("running");
//   // Set the spinning direction clockwise:
//   digitalWrite(dirPin, HIGH);
//   // Spin the stepper motor 1 revolution slowly:
//   for (int i = 0; i < stepsPerRevolution/7; i++) {
//     // These four lines result in 1 step:
//     digitalWrite(stepPin, HIGH);
//     delayMicroseconds(4000);
//     digitalWrite(stepPin, LOW);
//     delayMicroseconds(4000);
//   }
//   // delay(2000);
//   // Set the spinning direction counterclockwise:
//   digitalWrite(dirPin, LOW);
//   // Spin the stepper motor 1 revolution quickly:
//   for (int i = 0; i < stepsPerRevolution/7; i++) {
//     // These four lines result in 1 step:
//     digitalWrite(stepPin, HIGH);
//     delayMicroseconds(4000);
//     digitalWrite(stepPin, LOW);
//     delayMicroseconds(4000);
//   }

// }


#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 50

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 8

// All the wires needed for full functionality
#define DIR 22
#define STEP 23
//Uncomment line to use enable/disable functionality
//#define SLEEP 13

#include "DRV8825.h"
DRV8825 stepper(MOTOR_STEPS, DIR, STEP);


void setup() {
    stepper.begin(RPM, MICROSTEPS);
}

void loop() {
    stepper.rotate(90);
    delay(500);
    stepper.rotate(-90);
    delay(500);
}


// #include <AS5600.h>

// AS5600 encoder;
// float output;

// void setup() {
//   Serial.begin(9600);
//   Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);

//   pinMode(LED_BUILTIN, OUTPUT);
//   Serial.println("started");
//   while (!Serial); // Waiting for Serial Monitor

// }

// // void loop() {
// //   Serial.println("running");

// //   // get the angle in degrees of the encoder
// //   static int i = 0;
// //   i ++ ;
// //   if ( i  > 50 ) {
    
// //     // output = encoder.getPosition();
// //     Serial.println(output);

// //   }
// //   delay(1000);
// //   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
// // }

// void loop()
// {
//   Serial.println("Scanning...");

//   byte ANGLEAddressMSB = 0x0E;
//   byte ANGLEAddressLSB = 0x0F;
//   byte DEVICE_ADDRESS = 0x36;
//   Wire.beginTransmission(DEVICE_ADDRESS);
//   Wire.send(ANGLEAddressMSB);
//   Wire.endTransmission();
//   Wire.requestFrom(DEVICE_ADDRESS, 2);

//   while (Wire.available() < 2);
//   byte data1 = Wire.receive();
//   byte data2 = Wire.receive();
//   uint16_t data = (data1 << 8) | data2;
//   Serial.println(data);
//   delay(10); // wait 5 seconds for the next I2C scan
// }