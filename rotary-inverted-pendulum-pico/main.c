#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "motor_driver.h"

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

static char event_str[128];
static uint triggered_gpio = 0;
void gpio_event_string(char *buf, uint32_t events);
bool triggered = false;
static void gpio_callback(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    gpio_event_string(event_str, events);
    triggered_gpio = gpio;
    triggered = true;
}

static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

void gpio_event_string(char *buf, uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0') {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events) {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

#define ENCODER_INPUT_A 27
#define ENCODER_INPUT_B 26


int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    setup_motor_driver();

    gpio_init(ENCODER_INPUT_A);
    gpio_init(ENCODER_INPUT_B);
    gpio_set_irq_enabled_with_callback(ENCODER_INPUT_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(ENCODER_INPUT_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &gpio_callback);

    i2c_init(i2c0, I2C_FREQ_HZ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    int16_t angle;
    while (1) {
      if (triggered) {
        printf("GPIO %d %s\n", triggered_gpio, event_str);
        triggered = false;
      }
      printf("triggered %d \n", triggered);
      read_raw(&angle);
      gpio_put(LED_PIN, !gpio_get(LED_PIN));
      printf("angle = %d\n", angle);
      run_motors_clockwise();

      sleep_ms(10);
    }

    return 0;
}