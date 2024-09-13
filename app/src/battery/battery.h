#ifndef APPLICATION_BATTERY_H_
#define APPLICATION_BATTERY_H_

#include <zephyr/kernel.h>

/** Enable or disable measurement of the battery voltage.
 *
 * @param enable true to enable, false to disable
 *
 * @return zero on success, or a negative error code.
 */
int battery_measure_enable(bool enable);

/** Measure the battery voltage.
 *
 * @return the battery voltage in millivolts, or a negative error
 * code.
 */
int battery_sample(void);

/** A point in a battery discharge curve sequence.
 *
 * A discharge curve is defined as a sequence of these points, where
 * the first point has #lvl_pptt set to 10000 and the last point has
 * #lvl_pptt set to zero.  Both #lvl_pptt and #lvl_mV should be
 * monotonic decreasing within the sequence.
 */
struct battery_level_point
{
    /** Remaining life at #lvl_mV. */
    uint16_t lvl_pptt;

    /** Battery voltage at #lvl_pptt remaining life. */
    uint16_t lvl_mV;
};

/** Calculate the estimated battery level based on a measured voltage.
 *
 * @param batt_mV a measured battery voltage level.
 *
 * @param curve the discharge curve for the type of battery installed
 * on the system.
 *
 * @return the estimated remaining capacity in parts per ten
 * thousand.
 */
unsigned int battery_level_pptt(unsigned int batt_mV,
                                const struct battery_level_point *curve);

/** Initializes the battery and charger.
 *
 * @return 0 if not initialized properly. 1 otherwise.
 */
uint8_t battery_init();
uint8_t battery_prepare_wakeup();
bool battery_is_charging();
void battery_detection_enable(bool b);
unsigned int battery_read();
void battery_timer_expired_cb();
void battery_timer_expired_work_handler(struct k_work *work);

#endif /* APPLICATION_BATTERY_H_ */
