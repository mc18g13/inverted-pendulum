#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

static int addr = 0x68;

#define I2C_PORT i2c0




static void read_raw(int16_t* angle) {
    const uint8_t ANGLE_ADDRESS_MSB = 0x0E;
    const uint8_t DEVICE_ADDRESS = 0x36;

    uint8_t buffer[2];

    i2c_write_blocking(I2C_PORT, DEVICE_ADDRESS, &ANGLE_ADDRESS_MSB, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_PORT, DEVICE_ADDRESS, buffer, sizeof(uint16_t), false);

    printf("buffer = %d\n", buffer[0]);
    *angle = (buffer[0] << 8) | buffer[1];
    
}

#define I2C_FREQ_HZ 400 * 1000

int main() {
    stdio_init_all();

    static const int LED_PIN = 25;
    i2c_init(I2C_PORT, I2C_FREQ_HZ);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_pull_up(4);
    gpio_pull_up(5);

    int16_t angle;
    while (1) {

      uint8_t error, address;
      int nDevices;

      printf("Scanning...\n");

      nDevices = 0;
      for (address = 1; address < 127; address++ ) 
      {

        i2c_write_blocking(I2C_PORT, address, 0, 1, true); // true to keep master control of bus
        i2c_read_blocking(I2C_PORT, address, error, 1, false);

        if (error == 0)
        {
          printf("I2C device found at address ");
          printf("%0x", address);
          printf("  !\n");

          nDevices++;
        }
        else
        {
          printf("Unknow error at address 0x\n");
          printf("%0x\n", address);
        }    
      }
      if (nDevices == 0)
        printf("No I2C devices found\n");
      else
        printf("done\n");

      sleep_ms(5000);           // wait 5 seconds for next scan

      // read_raw(&angle);
      gpio_put(LED_PIN, !gpio_get(LED_PIN));
        // printf("angle = %d\n", angle);

        // sleep_ms(100);
    }

    return 0;
}