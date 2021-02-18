#include "motor_encoder.h"
#include "stdio.h"
#include "hardware/gpio.h"
#include "math.h"

#define pulse_count_revolution 48.0f
#define gear_ratio 11.7f
#define ENCODER_INPUT_A 27
#define ENCODER_INPUT_B 26

#define PULSE_COUNT_FOR_GEARED_TURN ((pulse_count_revolution) * (gear_ratio))
#define PULSES_TO_RADS ((2.0f) * (M_PI) / PULSE_COUNT_FOR_GEARED_TURN)

static uint triggered_gpio = 0;
static int displacement_count_from_start = 0;
static int direction = 0;
bool found_direction = false;

void calculate_direction(uint gpio, uint32_t events);
static void gpio_callback(uint gpio, uint32_t events);

void setup_motor_encoder(void) {
  gpio_init(ENCODER_INPUT_A);
  gpio_init(ENCODER_INPUT_B);
  gpio_set_irq_enabled_with_callback(ENCODER_INPUT_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(ENCODER_INPUT_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

float get_motor_displacement_radians(void) {
  return -displacement_count_from_start * PULSES_TO_RADS;
}

static void gpio_callback(uint gpio, uint32_t events) {
  if (!found_direction) {
    calculate_direction(gpio, events);
  }

  if (direction == 0) {
    printf("ERROR DIRECTION NOT SET\n");
  }

  if (triggered_gpio == gpio) {
    direction = -direction;
    // printf("Changed direction\n");
  }

  if (ENCODER_INPUT_A == gpio || ENCODER_INPUT_B == gpio) {
    displacement_count_from_start = displacement_count_from_start + direction;
  }

  triggered_gpio = gpio;
}

void calculate_direction(uint gpio, uint32_t events) {
  if (events & GPIO_IRQ_EDGE_FALL) {
    if (gpio == ENCODER_INPUT_A) {
      direction = 1;
    } else {
      direction = -1;
    }
    events &= ~GPIO_IRQ_EDGE_FALL;
  }

  if (events & GPIO_IRQ_EDGE_RISE) {
    if (gpio == ENCODER_INPUT_A) {
      direction = 1;
    } else {
      direction = -1;
    }
    events &= ~GPIO_IRQ_EDGE_RISE;
  }

  found_direction = true;
}