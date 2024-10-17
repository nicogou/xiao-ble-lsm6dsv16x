#include "impulse.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <ei_wrapper.h>

LOG_MODULE_REGISTER(impulse, CONFIG_APP_LOG_LEVEL);

static void result_ready_cb(int err)
{
	if (err) {
		LOG_ERR("Result ready callback returned error (err: %d)", err);
		return;
	}

	const char *label;
	float value;
	float anomaly;

	LOG_INF("Classification results");
	LOG_INF("======================");

	while (true) {
		err = ei_wrapper_get_next_classification_result(&label, &value, NULL);
		if (err) {
			if (err == -ENOENT) {
				err = 0;
			}
			break;
		}

		LOG_INF("Value: %.2f\tLabel: %s", value, label);
	}

	if (err) {
		LOG_ERR("Cannot get classification results (err: %d)", err);
	} else {
		if (ei_wrapper_classifier_has_anomaly()) {
			err = ei_wrapper_get_anomaly(&anomaly);
			if (err) {
				LOG_ERR("Cannot get anomaly (err: %d)", err);
			} else {
				LOG_INF("Anomaly: %.2f", anomaly);
			}
		}
	}

	err = ei_wrapper_start_prediction(1, 0);
	if (err) {
		LOG_ERR("Cannot restart prediction (err: %d)", err);
	} else {
		LOG_INF("Prediction restarted...");
	}
}

int impulse_add_data(float* data, size_t len)
{
	int res = ei_wrapper_add_data(data, len);
	if (res) {
		LOG_ERR("Cannot provide input data (err: %d)", res);
		LOG_ERR("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE");
	}
	return res;
}

int impulse_start_predicting()
{
	int err = ei_wrapper_start_prediction(0, 0);
	if (err) {
		LOG_ERR("Cannot start prediction (err: %d)", err);
	} else {
		LOG_INF("Prediction started...");
	}
	return err;
}

int impulse_stop_predicting()
{
	int err = ei_wrapper_clear_data(NULL);
	if (err) {
		LOG_ERR("Cannot stop prediction (err: %d)", err);
	} else {
		LOG_INF("Prediction stopped.");
	}
	return err;
}

int impulse_init()
{
	int err = ei_wrapper_init(result_ready_cb);

	if (err) {
		LOG_ERR("Edge Impulse wrapper failed to initialize (err: %d)",
		       err);
		return 0;
	};

	LOG_INF("Machine learning model window size %u - frame size %u", ei_wrapper_get_window_size(), ei_wrapper_get_frame_size());

	LOG_INF("Machine learning model sampling frequency: %zu",
	       ei_wrapper_get_classifier_frequency());
	LOG_INF("Labels assigned by the model:");
	for (size_t i = 0; i < ei_wrapper_get_classifier_label_count(); i++) {
		LOG_INF("- %s", ei_wrapper_get_classifier_label(i));
	}

	return 0;
}
