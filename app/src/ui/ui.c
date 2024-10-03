#include "ui.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(ui, CONFIG_APP_LOG_LEVEL);

static const struct pwm_dt_spec red_pwm_led =
    PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
static const struct pwm_dt_spec green_pwm_led =
    PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
static const struct pwm_dt_spec blue_pwm_led =
    PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));

static ui_rgb_t off = {0, 0, 0, 0};
static ui_rgb_t red = {UI_COLOR_MAX, 0, 0, 0};

static ui_rgb_t ui_color = {0, 0, 0, 0};

uint8_t ui_set_rgb_on(ui_rgb_t *color)
{
    if (color->red >= UI_COLOR_MAX)
    {
        color->red = UI_COLOR_MAX;
    }
    if (color->green >= UI_COLOR_MAX)
    {
        color->green = UI_COLOR_MAX;
    }
    if (color->blue >= UI_COLOR_MAX)
    {
        color->blue = UI_COLOR_MAX;
    }

    ui_color.red = color->red;
    ui_color.green = color->green;
    ui_color.blue = color->blue;
    ui_color.blink = color->blink;
    ui_color.duration = color->duration;

    return 1;
}

uint8_t ui_set_rgb_off()
{
    return ui_set_rgb_on(&off);
}

uint8_t ui_init()
{
    if (!device_is_ready(red_pwm_led.dev) ||
        !device_is_ready(green_pwm_led.dev) ||
        !device_is_ready(blue_pwm_led.dev))
    {
        LOG_ERR("Error: one or more PWM devices not ready");
        return 0;
    }

    ui_set_rgb_on(&red);
    return 1;
}

void ui_process(void)
{
    int r, g, b, ret;
    bool k = true;

    ui_init();

    while (true)
    {
        if (ui_color.blink != 0 && k)
        {
            r = 0;
            g = 0;
            b = 0;
        }
        else
        {
            r = red_pwm_led.period / UI_COLOR_MAX * ui_color.red;
            g = green_pwm_led.period / UI_COLOR_MAX * ui_color.green;
            b = blue_pwm_led.period / UI_COLOR_MAX * ui_color.blue;
        }

        ret = pwm_set_pulse_dt(&red_pwm_led, r);
        if (ret != 0)
        {
            LOG_ERR("Error %d: red write failed.", ret);
            LOG_HEXDUMP_ERR(&ui_color, sizeof(ui_rgb_t), "Error while writing color :");
        }

        ret = pwm_set_pulse_dt(&green_pwm_led, g);
        if (ret != 0)
        {
            LOG_ERR("Error %d: green write failed.", ret);
            LOG_HEXDUMP_ERR(&ui_color, sizeof(ui_rgb_t), "Error while writing color :");
        }

        ret = pwm_set_pulse_dt(&blue_pwm_led, b);
        if (ret != 0)
        {
            LOG_ERR("Error %d: blue write failed.", ret);
            LOG_HEXDUMP_ERR(&ui_color, sizeof(ui_rgb_t), "Error while writing color :");
        }

        if (ui_color.blink != 0 && k)
        {
            k = false;
            k_sleep(K_MSEC(ui_color.duration * 10 * ui_color.blink));
        }
        else
        {
            k = true;
            k_sleep(K_MSEC(ui_color.duration * (1000 - 10 * ui_color.blink)));
        }
    }
}

K_THREAD_DEFINE(ui_thread, STACK_SIZE_UI, ui_process, NULL, NULL, NULL, 7, 0, 0);