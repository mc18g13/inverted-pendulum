#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
 
#define LED_PIN 25
#define SDA_PIN 4
#define SCL_PIN 5
#define I2C_FREQ_HZ 400 * 1000

static void read_raw(int16_t* angle) {
    const uint8_t ANGLE_ADDRESS_MSB = 0x0E;
    const uint8_t DEVICE_ADDRESS = 0x36;

    uint8_t buffer[2];

    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, &ANGLE_ADDRESS_MSB, 1, true);
    i2c_read_blocking(I2C_PORT, DEVICE_ADDRESS, buffer, sizeof(uint16_t), false);

    *angle = (buffer[0] << 8) | buffer[1];
}

int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    i2c_init(i2c0, I2C_FREQ_HZ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    int16_t angle;
    while (1) {
      read_raw(&angle);
      gpio_put(LED_PIN, !gpio_get(LED_PIN));
      printf("angle = %d\n", angle);
      // sleep_ms(100);
    }

    return 0;
}