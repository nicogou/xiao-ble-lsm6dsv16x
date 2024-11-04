#include "battery.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#if defined(CONFIG_XIAO_BLE_SHELL)
#include <zephyr/shell/shell.h>
#endif

#include <zephyr/logging/log.h>


#define CHARGING DT_ALIAS(charging)

LOG_MODULE_REGISTER(battery, CONFIG_APP_LOG_LEVEL);

#define VBATT DT_PATH(vbatt)
#define ZEPHYR_USER DT_PATH(zephyr_user)

/** A discharge curve specific to the power source. */
const struct battery_level_point levels[] = {
    // "Curve" here eyeballed from the battery datasheet.

    {10000, 4200},
    {9167, 4010},
    {8333, 3925},
    {7500, 3840},
    {6666, 3775},
    {5833, 3700},
    {5000, 3660},
    {4167, 3625},
    {3333, 3600},
    {2500, 3575},
    {1666, 3550},
    {833, 3525},
    {0, 3450},
};

static const struct gpio_dt_spec charging =
    GPIO_DT_SPEC_GET(CHARGING, gpios);

static bool _charging_state;

static struct gpio_callback charging_cb_data;

// Defining a timer to make the battery measurement regularly.
K_TIMER_DEFINE(timer_battery, battery_timer_expired_cb, NULL);
// Define a work for the system workqueue to process as battery can't be read from an interrupt.
K_WORK_DEFINE(battery_timer_expired_work, battery_timer_expired_work_handler);

#define BATTERY_ADC_GAIN ADC_GAIN_1_6

struct io_channel_config
{
    uint8_t channel;
};

struct divider_config
{
    struct io_channel_config io_channel;
    struct gpio_dt_spec power_gpios;
    /* output_ohm is used as a flag value: if it is nonzero then
     * the battery is measured through a voltage divider;
     * otherwise it is assumed to be directly connected to Vdd.
     */
    uint32_t output_ohm;
    uint32_t full_ohm;
};

static const struct divider_config divider_config = {
#if DT_NODE_HAS_STATUS(VBATT, okay)
    .io_channel = {
        DT_IO_CHANNELS_INPUT(VBATT),
    },
    .power_gpios = GPIO_DT_SPEC_GET_OR(VBATT, power_gpios, {}),
    .output_ohm = DT_PROP(VBATT, output_ohms),
    .full_ohm = DT_PROP(VBATT, full_ohms),
#else  /* /vbatt exists */
    .io_channel = {
        DT_IO_CHANNELS_INPUT(ZEPHYR_USER),
    },
#endif /* /vbatt exists */
};

struct divider_data
{
    const struct device *adc;
    struct adc_channel_cfg adc_cfg;
    struct adc_sequence adc_seq;
    int16_t raw;
};
static struct divider_data divider_data = {
#if DT_NODE_HAS_STATUS(VBATT, okay)
    .adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(VBATT)),
#else
    .adc = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(ZEPHYR_USER)),
#endif
};

static int divider_setup(void)
{
    const struct divider_config *cfg = &divider_config;
    const struct io_channel_config *iocp = &cfg->io_channel;
    const struct gpio_dt_spec *gcp = &cfg->power_gpios;
    struct divider_data *ddp = &divider_data;
    struct adc_sequence *asp = &ddp->adc_seq;
    struct adc_channel_cfg *accp = &ddp->adc_cfg;
    int rc;

    if (!device_is_ready(ddp->adc))
    {
        LOG_ERR("ADC device is not ready %s", ddp->adc->name);
        return -ENOENT;
    }

    if (gcp->port)
    {
        if (!device_is_ready(gcp->port))
        {
            LOG_ERR("%s: device not ready", gcp->port->name);
            return -ENOENT;
        }
        rc = gpio_pin_configure_dt(gcp, GPIO_OUTPUT_INACTIVE);
        if (rc != 0)
        {
            LOG_ERR("Failed to control feed %s.%u: %d",
                    gcp->port->name, gcp->pin, rc);
            return rc;
        }
    }

    *asp = (struct adc_sequence){
        .channels = BIT(0),
        .buffer = &ddp->raw,
        .buffer_size = sizeof(ddp->raw),
        .oversampling = 4,
        .calibrate = true,
    };

#ifdef CONFIG_ADC_NRFX_SAADC
    *accp = (struct adc_channel_cfg){
        .gain = BATTERY_ADC_GAIN,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
    };

    if (cfg->output_ohm != 0)
    {
        accp->input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + iocp->channel;
    }
    else
    {
        accp->input_positive = SAADC_CH_PSELP_PSELP_VDD;
    }

    asp->resolution = 14;
#else /* CONFIG_ADC_var */
#error Unsupported ADC
#endif /* CONFIG_ADC_var */

    rc = adc_channel_setup(ddp->adc, accp);
	if (rc != 0) {
	    LOG_ERR("ADC channel %u setup error %d", iocp->channel, rc);
	}

    return rc;
}

static bool battery_ok;

static int battery_setup(void)
{
    int rc = divider_setup();
	battery_ok = (rc == 0);
	if (rc != 0) {
	    LOG_ERR("Battery setup error: %d", rc);
	}

    return rc;
}

SYS_INIT(battery_setup, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

int battery_measure_enable(bool enable)
{
    int rc = -ENOENT;

    if (battery_ok)
    {
        const struct gpio_dt_spec *gcp = &divider_config.power_gpios;

        rc = 0;
        if (gcp->port)
        {
            rc = gpio_pin_set_dt(gcp, enable);
        }
    }
    return rc;
}

int battery_sample(void)
{
    int rc = -ENOENT;

	if (_charging_state) {
		LOG_ERR("Cannot read battery while charging");
		return -EACCES;
	}

    if (battery_ok)
    {
        struct divider_data *ddp = &divider_data;
        const struct divider_config *dcp = &divider_config;
        struct adc_sequence *sp = &ddp->adc_seq;

        rc = adc_read(ddp->adc, sp);
        sp->calibrate = false;
        if (rc == 0)
        {
            int32_t val = ddp->raw;

            adc_raw_to_millivolts(adc_ref_internal(ddp->adc),
                                  ddp->adc_cfg.gain,
                                  sp->resolution,
                                  &val);

            if (dcp->output_ohm != 0)
            {
                rc = val * (uint64_t)dcp->full_ohm / dcp->output_ohm;
                LOG_DBG("raw %u ~ %u mV => %d mV",
                        ddp->raw, val, rc);
            }
            else
            {
                rc = val;
                LOG_DBG("raw %u ~ %u mV", ddp->raw, val);
            }
        }
    }

    return rc;
}

unsigned int battery_level_pptt(unsigned int batt_mV,
                                const struct battery_level_point *curve)
{
    const struct battery_level_point *pb = curve;

    if (batt_mV >= pb->lvl_mV)
    {
        /* Measured voltage above highest point, cap at maximum. */
        return pb->lvl_pptt;
    }
    /* Go down to the last point at or below the measured voltage. */
    while ((pb->lvl_pptt > 0) && (batt_mV < pb->lvl_mV))
    {
        ++pb;
    }
    if (batt_mV < pb->lvl_mV)
    {
        /* Below lowest point, cap at minimum */
        return pb->lvl_pptt;
    }

    /* Linear interpolation between below and above points. */
    const struct battery_level_point *pa = pb - 1;

    return pb->lvl_pptt + ((pa->lvl_pptt - pb->lvl_pptt) * (batt_mV - pb->lvl_mV) / (pa->lvl_mV - pb->lvl_mV));
}

static void charging_event(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    _charging_state = gpio_pin_get_dt(&charging);
    if (_charging_state)
    {
		LOG_INF("Charging has started");
    }
    else
    {
		LOG_INF("Charging has stopped");
    }
}

unsigned int battery_read()
{
    battery_measure_enable(true);
    int batt_mV = battery_sample();
    battery_measure_enable(false);

    if (batt_mV < 0)
    {
        LOG_ERR("Failed to read battery voltage: %d",
                batt_mV);
        return 0;
    }

    unsigned int batt_pptt = battery_level_pptt(batt_mV, levels);

    LOG_INF("[%s]: %d mV; %u pptt", "now",
            batt_mV, batt_pptt);

	return batt_pptt;
}

void battery_timer_expired_work_handler(struct k_work *work)
{
    // Read battery from workqueue. Only if battery is not charging in order to avoid damaging pin P0.31 (https://wiki.seeedstudio.com/XIAO_BLE/#q3-what-are-the-considerations-when-using-xiao-nrf52840-sense-for-battery-charging)
    if (!_charging_state)
    {
#if defined(CONFIG_XIAO_BLE_SHELL)
		// This sends the reading to the BLE Shell as well as logging it.
		shell_execute_cmd(NULL, "battery read");
#else
        battery_read();
#endif
    }
}

void battery_timer_expired_cb()
{
    // Submit work to the system workqueue as battery can't be read from an interrupt (which a timer expiring is).
    k_work_submit(&battery_timer_expired_work);
}

uint8_t battery_prepare_wakeup()
{
    /* Charging pin setup */
    /* Configure to generate PORT event (wakeup) on button 1 press. */
    // nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(CHG_CHG, gpios),
    //                    NRF_GPIO_PIN_PULLUP);

    // nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(CHG_CHG, gpios),
    //                        NRF_GPIO_PIN_SENSE_LOW);

    return 1;
}

bool battery_is_charging()
{
    return _charging_state;
}

uint8_t battery_init()
{

    uint8_t ret;

    /* chg_chg pin setup */
    if (!device_is_ready(charging.port))
    {
        LOG_ERR("Error: charging device %s is not ready",
                charging.port->name);
        return 0;
    }

    ret = gpio_pin_configure_dt(&charging, GPIO_INPUT);
    if (ret != 0)
    {
        LOG_ERR("Error %d: failed to configure %s pin %d",
                ret, charging.port->name, charging.pin);
        return 0;
    }

    ret = gpio_pin_interrupt_configure_dt(&charging,
                                          GPIO_INT_EDGE_BOTH);
    if (ret != 0)
    {
        LOG_ERR("Error %d: failed to configure interrupt on %s pin %d",
                ret, charging.port->name, charging.pin);
        return 0;
    }

    gpio_init_callback(&charging_cb_data, charging_event, BIT(charging.pin));
    gpio_add_callback(charging.port, &charging_cb_data);

    _charging_state = gpio_pin_get_dt(&charging);

    int rc = battery_measure_enable(false);
    if (rc != 0)
    {
        LOG_ERR("Failed initialize battery measurement: %d", rc);
        return 0;
    }

    // Starting the timer to make battery measurements regularly.
    k_timer_start(&timer_battery, K_SECONDS(0), K_SECONDS(60));

    return 1;
}
