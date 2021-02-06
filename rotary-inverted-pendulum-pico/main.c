#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}
 
#define LED_PIN 25
#define SDA_PIN 4
#define SCL_PIN 5
#define I2C_FREQ_HZ 400 * 1000

int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    i2c_init(i2c0, I2C_FREQ_HZ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    while (1) {
      printf("\nI2C Bus Scan\n");
      printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
  
      for (int addr = 0; addr < (1 << 7); ++addr) {
          if (addr % 16 == 0) {
              printf("%02x ", addr);
          }
  
          // Perform a 1-byte dummy read from the probe address. If a slave
          // acknowledges this address, the function returns the number of bytes
          // transferred. If the address byte is ignored, the function returns
          // -1.
  
          // Skip over any reserved addresses.
          int ret;
          uint8_t rxdata;
          if (reserved_addr(addr))
              ret = PICO_ERROR_GENERIC;
          else
              ret = i2c_read_blocking(i2c0, addr, &rxdata, 1, false);
  
          printf(ret < 0 ? "." : "@");
          printf(addr % 16 == 15 ? "\n" : "  ");
      }
      gpio_put(LED_PIN, !gpio_get(LED_PIN));
      printf("Done.\n");
      // sleep_ms(1000);
    }

    return 0;
}