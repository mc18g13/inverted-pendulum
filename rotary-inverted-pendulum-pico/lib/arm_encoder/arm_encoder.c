#include "arm_encoder.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "math.h"

#define I2C_PORT i2c0
 
#define LED_PIN 25
#define SDA_PIN 4
#define SCL_PIN 5
#define I2C_FREQ_HZ 400 * 1000

#define BIT_MAX_12 4096
#define BIT_MAX_11 2048

static uint16_t encoder_zero_offset = 0;
uint16_t get_encoder_value();

uint16_t get_encoder_value() {
  const uint8_t ANGLE_ADDRESS_MSB = 0x0E;
  const uint8_t DEVICE_ADDRESS = 0x36;

  uint8_t buffer[2];

  i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, &ANGLE_ADDRESS_MSB, 1, true);
  i2c_read_blocking(I2C_PORT, DEVICE_ADDRESS, buffer, sizeof(uint16_t), false);

  uint16_t angle = (buffer[0] << 8) | buffer[1];
  return angle;
}

bool arm_is_at_rest() {
  const uint8_t COUNT_FOR_AT_REST_TEST = 50;
  uint16_t readings[COUNT_FOR_AT_REST_TEST];
  for (int i = 0; i < COUNT_FOR_AT_REST_TEST; i++) {
    readings[i] = get_encoder_value();
    sleep_ms(30);
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

void calibrate_arm_encoder() {
  while (!arm_is_at_rest()) {
    printf("waiting for arm to be at rest");
    sleep_ms(1000);
  };

  uint16_t starting_encoder_value = get_encoder_value();

  // encoder is 12 bit (4096 max). half way point is 11 bit (2^11 = 2048)
  encoder_zero_offset = starting_encoder_value - BIT_MAX_11;
}


float get_arm_encoder_angle(void) {
  uint16_t encoder = get_encoder_value();
  encoder = encoder - encoder_zero_offset;

  // account for 16 bit overflow
  encoder %= BIT_MAX_12;

  float angle_out_of_2PI = (float)encoder * 2 * M_PI / (float)BIT_MAX_12;
  return angle_out_of_2PI > M_PI ? angle_out_of_2PI - (M_PI * 2) : angle_out_of_2PI;
}

void setup_arm_encoder(void) {
  i2c_init(i2c0, I2C_FREQ_HZ);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);
}