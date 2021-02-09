#include "motor_encoder.h"
#include "hardware/gpio.h"

#define pulse_count_revolution 48.0f
#define gear_ratio 11.7f
#define pulse_count_for_geared_turn pulse_count_revolution * gear_ratio

static const char *gpio_irq_str[] = {
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
};

#define ENCODER_INPUT_A 27
#define ENCODER_INPUT_B 26

static char event_str[128];
static uint triggered_gpio = 0;
static uint trigger_count = 0;

void gpio_event_string(char *buf, uint32_t events);
static void gpio_callback(uint gpio, uint32_t events);

void setup_motor_encoder(void) {

  gpio_init(ENCODER_INPUT_A);
  gpio_init(ENCODER_INPUT_B);
  gpio_set_irq_enabled_with_callback(ENCODER_INPUT_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(ENCODER_INPUT_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL , true, &gpio_callback);
}

int get_trigger_count(void) {
  return trigger_count;
}

static void gpio_callback(uint gpio, uint32_t events) {
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    gpio_event_string(event_str, events);
    triggered_gpio = gpio;
    if (ENCODER_INPUT_A == gpio || ENCODER_INPUT_B == gpio) {
      trigger_count++;
    }
}

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