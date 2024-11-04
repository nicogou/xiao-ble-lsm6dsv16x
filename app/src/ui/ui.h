#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

#define STACK_SIZE_UI 512

#define STEP_SIZE PWM_USEC(2000)
#define UI_COLOR_MAX 10000
#define UI_LED_INTENSITY_LOW 200
#define UI_LED_BLINK_MAX 100

typedef enum {
    UI_BTN_SHORTPRESS,
    UI_BTN_LONGPRESS,
} ui_instruction_type_t;

typedef struct
{
    uint16_t red;   /* Intensity 0-UI_COLOR_MAX */
    uint16_t green; /* Intensity 0-UI_COLOR_MAX */
    uint16_t blue;  /* Intensity 0-UI_COLOR_MAX */
    uint8_t blink;  /* Ratio off_duration/on_duration in % */
    uint8_t duration; /* blink cycle length in seconds */
} ui_rgb_t;

uint8_t ui_init();
uint8_t ui_set_rgb_on(uint16_t r, uint16_t g, uint16_t b, uint8_t k, uint8_t d);
uint8_t ui_set_rgb_off();

void button_resetsoft();
void button_resetall();
