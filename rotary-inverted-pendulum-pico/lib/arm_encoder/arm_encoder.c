#include "arm_encoder.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
 
#define LED_PIN 25
#define SDA_PIN 4
#define SCL_PIN 5
#define I2C_FREQ_HZ 400 * 1000

uint16_t read_raw() {
    const uint8_t ANGLE_ADDRESS_MSB = 0x0E;
    const uint8_t DEVICE_ADDRESS = 0x36;

    uint8_t buffer[2];

    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, &ANGLE_ADDRESS_MSB, 1, true);
    i2c_read_blocking(I2C_PORT, DEVICE_ADDRESS, buffer, sizeof(uint16_t), false);

    uint16_t angle = (buffer[0] << 8) | buffer[1];
    return angle;
}


void setup_arm_encoder(void) {
  i2c_init(i2c0, I2C_FREQ_HZ);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);
}