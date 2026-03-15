#define DT_DRV_COMPAT bosch_mybme280_custom
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <drivers/sensor/bme280.h>

LOG_MODULE_REGISTER(mybme280, CONFIG_SENSOR_LOG_LEVEL);

/*
 * mybme280_data_ready_sem — signals app when sensor measurement complete
 * Weak extern: driver declares weak, app defines and owns it
 */
extern struct k_sem mybme280_data_ready_sem __attribute__((weak));
#define SIGNAL_DATA_READY() \
do { \
    if ((uintptr_t)&mybme280_data_ready_sem != 0U) { \
        k_sem_give(&mybme280_data_ready_sem); \
    } \
} while (0)

/* Read calibration data from sensor NVM */
static int bme280_read_calib(const struct device *dev)
{
	struct bme280_data *data = dev->data;
	const struct bme280_config *config = dev->config;
	uint8_t calib[26];
	uint8_t calib_h[7];

	/* Read compensation data — 26 bytes from 0x88 */
	if (i2c_burst_read_dt(&config->i2c, 0x88, calib, sizeof(calib)) < 0) {
		LOG_ERR("Failed to read calibration data");
		return -EIO;
	}

	/* Read humidity calibration — 7 bytes from 0xE1 */
	if (i2c_burst_read_dt(&config->i2c, 0xE1, calib_h, sizeof(calib_h)) < 0) {
		LOG_ERR("Failed to read humidity calibration");
		return -EIO;
	}

	/* Parse temperature calibration */
	data->calib.dig_T1 = (calib[1] << 8) | calib[0];
	data->calib.dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
	data->calib.dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);

	/* Parse pressure calibration */
	data->calib.dig_P1 = (calib[7] << 8) | calib[6];
	data->calib.dig_P2 = (int16_t)((calib[9] << 8) | calib[8]);
	data->calib.dig_P3 = (int16_t)((calib[11] << 8) | calib[10]);
	data->calib.dig_P4 = (int16_t)((calib[13] << 8) | calib[12]);
	data->calib.dig_P5 = (int16_t)((calib[15] << 8) | calib[14]);
	data->calib.dig_P6 = (int16_t)((calib[17] << 8) | calib[16]);
	data->calib.dig_P7 = (int16_t)((calib[19] << 8) | calib[18]);
	data->calib.dig_P8 = (int16_t)((calib[21] << 8) | calib[20]);
	data->calib.dig_P9 = (int16_t)((calib[23] << 8) | calib[22]);

	/* Parse humidity calibration */
	data->calib.dig_H1 = calib[25];
	data->calib.dig_H2 = (int16_t)((calib_h[1] << 4) | (calib_h[0] & 0x0F));
	data->calib.dig_H3 = calib_h[2];
	data->calib.dig_H4 = (int16_t)((calib_h[3] << 4) | (calib_h[4] & 0x0F));
	data->calib.dig_H5 = (int16_t)((calib_h[5] << 4) | ((calib_h[4] >> 4) & 0x0F));
	data->calib.dig_H6 = (int8_t)calib_h[6];

	return 0;
}

/* Compensation formulas */
static int32_t bme280_compensate_T(struct bme280_data *data, int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((adc_T >> 3) - ((int32_t)data->calib.dig_T1 << 1));
	var2 = (var1 * (int32_t)data->calib.dig_T2) >> 11;
	var1 = ((adc_T >> 4) - (int32_t)data->calib.dig_T1);
	var1 = (((var1 * var1) >> 12) * (int32_t)data->calib.dig_T3) >> 14;
	data->calib.t_fine = var1 + var2;
	T = (data->calib.t_fine * 5 + 128) >> 8;
	return T;
}

static uint32_t bme280_compensate_P(struct bme280_data *data, int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)data->calib.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)data->calib.dig_P6;
	var2 = var2 + ((var1 * (int64_t)data->calib.dig_P5) << 17);
	var2 = var2 + (((int64_t)data->calib.dig_P4) << 35);
	var1 = (((var1 * var1 * (int64_t)data->calib.dig_P3) >> 8) +
		((var1 * (int64_t)data->calib.dig_P2) << 12));
	var1 = (((1LL << 47) + var1) * ((int64_t)data->calib.dig_P1)) >> 33;
	if (var1 == 0) return 0;
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)data->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)data->calib.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)data->calib.dig_P7) << 4);
	return (uint32_t)p;
}

static uint32_t bme280_compensate_H(struct bme280_data *data, int32_t adc_H)
{
	int32_t var_H;
	var_H = (data->calib.t_fine) - 76800;
	var_H = (adc_H - (((int32_t)data->calib.dig_H4 << 4) | 
	         ((int32_t)(data->calib.dig_H5 & 0x0F)))) *
	        (((int32_t)data->calib.dig_H2) / 65536.0);
	var_H = (var_H * (1.0 + (((int32_t)data->calib.dig_H6) / 67108864.0)));
	var_H = var_H * (1.0 - (((int32_t)data->calib.dig_H1) * var_H / 524288.0));
	if (var_H > 100000) var_H = 100000;
	else if (var_H < 0) var_H = 0;
	return (uint32_t)var_H;
}

/* Soft reset and chip verification */
static int bme280_reset(const struct device *dev)
{
	const struct bme280_config *config = dev->config;
	uint8_t id;

	if (i2c_reg_read_byte_dt(&config->i2c, BME280_REG_ID, &id) < 0) {
		LOG_ERR("Failed to read chip ID");
		return -EIO;
	}

	if (id != BME280_CHIP_ID) {
		LOG_ERR("Unexpected chip ID: 0x%02x (expected 0x%02x)", id, BME280_CHIP_ID);
		return -EINVAL;
	}

	if (i2c_reg_write_byte_dt(&config->i2c, BME280_REG_RESET, BME280_SOFT_RESET_CODE) < 0) {
		LOG_ERR("Failed to reset device");
		return -EIO;
	}

	k_sleep(K_MSEC(10));
	return 0;
}

/* Configure sensor with oversampling */
static int bme280_configure(const struct device *dev)
{
	const struct bme280_config *config = dev->config;
	uint8_t ctrl_hum, ctrl_meas, config_reg;

	ctrl_hum = CONFIG_MYBME280_OVERSAMPLE_HUM & 0x07;
	if (i2c_reg_write_byte_dt(&config->i2c, BME280_REG_CTRL_HUM, ctrl_hum) < 0) {
		LOG_ERR("Failed to set humidity oversampling");
		return -EIO;
	}

	ctrl_meas = 0x03 |
	            ((CONFIG_MYBME280_OVERSAMPLE_TEMP & 0x07) << 5) |
	            ((CONFIG_MYBME280_OVERSAMPLE_PRESS & 0x07) << 2);

	if (i2c_reg_write_byte_dt(&config->i2c, BME280_REG_CTRL_MEAS, ctrl_meas) < 0) {
		LOG_ERR("Failed to set measurement control");
		return -EIO;
	}

	config_reg = 0xA0;
	if (i2c_reg_write_byte_dt(&config->i2c, BME280_REG_CONFIG, config_reg) < 0) {
		LOG_ERR("Failed to set config");
		return -EIO;
	}

	return 0;
}

/* Read raw ADC and apply compensation */
static int bme280_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct bme280_data *data = dev->data;
	const struct bme280_config *config = dev->config;
	uint8_t adc_data[8];
	int32_t adc_T, adc_P, adc_H;
	int ret;

	if (chan != SENSOR_CHAN_ALL &&
	    chan != SENSOR_CHAN_AMBIENT_TEMP &&
	    chan != SENSOR_CHAN_HUMIDITY &&
	    chan != SENSOR_CHAN_PRESS) {
		return -ENOTSUP;
	}

	uint8_t status;
	int retries = 10;
	do {
		ret = i2c_reg_read_byte_dt(&config->i2c, BME280_REG_STATUS, &status);
		if (ret < 0) {
			LOG_ERR("Failed to read status");
			return -EIO;
		}
		if ((status & 0x09) == 0) break;
		k_sleep(K_MSEC(50));
	} while (--retries > 0);

	if (retries == 0) {
		LOG_WRN("Measurement timeout");
		return -EAGAIN;
	}

	ret = i2c_burst_read_dt(&config->i2c, BME280_REG_ADC_PRES, adc_data, sizeof(adc_data));
	if (ret < 0) {
		LOG_ERR("Failed to read ADC data");
		return -EIO;
	}

	adc_P = ((int32_t)adc_data[0] << 12) | ((int32_t)adc_data[1] << 4) | ((int32_t)adc_data[2] >> 4);
	adc_T = ((int32_t)adc_data[3] << 12) | ((int32_t)adc_data[4] << 4) | ((int32_t)adc_data[5] >> 4);
	adc_H = ((int32_t)adc_data[6] << 8) | adc_data[7];

	int32_t T_fine = bme280_compensate_T(data, adc_T);
	data->temperature.val1 = T_fine / 5120;
	data->temperature.val2 = (T_fine % 5120) * 195312 / 5120;

	uint32_t P = bme280_compensate_P(data, adc_P);
	data->pressure.val1 = P / 256;
	data->pressure.val2 = (P % 256) * 3906;

	uint32_t H = bme280_compensate_H(data, adc_H);
	data->humidity.val1 = H / 1024;
	data->humidity.val2 = (H % 1024) * 976;

	return 0;
}

/* Return cached value */
static int bme280_channel_get(const struct device *dev,
                               enum sensor_channel chan,
                               struct sensor_value *val)
{
	struct bme280_data *data = dev->data;

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		*val = data->temperature;
	} else if (chan == SENSOR_CHAN_HUMIDITY) {
		*val = data->humidity;
	} else if (chan == SENSOR_CHAN_PRESS) {
		*val = data->pressure;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

#if CONFIG_MYBME280_MODE_CONTINUOUS
static void bme280_acq_thread(const struct device *dev)
{
	LOG_INF("BME280 acquisition thread started");

	while (true) {
		int ret = bme280_sample_fetch(dev, SENSOR_CHAN_ALL);
		if (ret < 0) {
			LOG_ERR("BME280 fetch failed: %d", ret);
		} else {
			SIGNAL_DATA_READY();
		}

		k_sleep(K_MSEC(CONFIG_MYBME280_MEASUREMENT_INTERVAL_MS));
	}
}
#endif

/* Public API — app calls this after network ready */
int mybme280_runtime_start(const struct device *dev)
{
#if CONFIG_MYBME280_MODE_CONTINUOUS
	struct bme280_data *data = dev->data;
	static uint8_t acq_stack[CONFIG_MYBME280_ACQ_THREAD_STACK_SIZE];

	if (data->initialized) {
		LOG_WRN("BME280 runtime already started");
		return -EALREADY;
	}

	k_thread_create(&data->acq_thread, acq_stack, sizeof(acq_stack),
	                (k_thread_entry_t)bme280_acq_thread, (void *)dev, NULL, NULL,
	                K_PRIO_PREEMPT(CONFIG_MYBME280_ACQ_THREAD_PRIORITY),
	                0, K_NO_WAIT);

	k_thread_name_set(&data->acq_thread, "mybme280");
	data->initialized = true;
	LOG_INF("BME280 acquisition thread created");
	return 0;
#else
	LOG_INF("BME280 ready for on-demand polling");
	return 0;
#endif
}

/* Boot-time initialization */
static int bme280_init(const struct device *dev)
{
	struct bme280_data *data = dev->data;
	const struct bme280_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	if (bme280_reset(dev) < 0) {
		LOG_ERR("BME280 reset failed");
		return -EIO;
	}

	if (bme280_read_calib(dev) < 0) {
		LOG_ERR("BME280 calibration read failed");
		return -EIO;
	}

	if (bme280_configure(dev) < 0) {
		LOG_ERR("BME280 configuration failed");
		return -EIO;
	}

	data->initialized = false;
	LOG_INF("BME280 initialized (deferred start)");
	return 0;
}

static const struct sensor_driver_api bme280_api = {
	.sample_fetch = bme280_sample_fetch,
	.channel_get = bme280_channel_get,
};

#define BME280_DEFINE(inst) \
	static struct bme280_data bme280_data_##inst; \
	static const struct bme280_config bme280_config_##inst = { \
		.i2c = I2C_DT_SPEC_GET(DT_INST_PARENT(inst)), \
		.chip_id = BME280_CHIP_ID, \
	}; \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, bme280_init, NULL, \
	                              &bme280_data_##inst, \
	                              &bme280_config_##inst, \
	                              POST_KERNEL, \
	                              CONFIG_MYBME280_INIT_PRIORITY, \
	                              &bme280_api);

DT_INST_FOREACH_STATUS_OKAY(BME280_DEFINE)
