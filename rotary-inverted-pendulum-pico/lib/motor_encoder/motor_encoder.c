#include "motor_encoder.h"
#include "stdio.h"
#include "hardware/gpio.h"

#define pulse_count_revolution 48.0f
#define gear_ratio 11.7f
#define pulse_count_for_geared_turn pulse_count_revolution * gear_ratio

#define ENCODER_INPUT_A 27
#define ENCODER_INPUT_B 26

static uint triggered_gpio = 0;
static uint displacement_count_from_start = 0;
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

int get_motor_displacement_count_from_start(void) {
  return displacement_count_from_start;
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