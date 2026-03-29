#ifndef CUSTOM_BME280_H_
#define CUSTOM_BME280_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

/* Register addresses */
#define CUSTOM_BME280_REG_ID           0xD0
#define CUSTOM_BME280_REG_RESET        0xE0
#define CUSTOM_BME280_REG_CTRL_HUM     0xF2
#define CUSTOM_BME280_REG_STATUS       0xF3
#define CUSTOM_BME280_REG_CTRL_MEAS    0xF4
#define CUSTOM_BME280_REG_CONFIG       0xF5
#define CUSTOM_BME280_REG_PRESS_MSB    0xF7
#define CUSTOM_BME280_REG_ADC_PRES     0xF7
#define CUSTOM_BME280_REG_ADC_TEMP     0xFA
#define CUSTOM_BME280_REG_ADC_HUM      0xFD

#define CUSTOM_BME280_CHIP_ID          0x60
#define CUSTOM_BME280_SOFT_RESET_CODE  0xB9

/* Compensation calibration data */
struct custom_bme280_calib_data {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
	int32_t t_fine;
};

struct custom_bme280_config {
	struct i2c_dt_spec i2c;
	uint8_t chip_id;
};

struct custom_bme280_data {
	struct custom_bme280_calib_data calib;
	struct sensor_value temperature;
	struct sensor_value humidity;
	struct sensor_value pressure;
	bool initialized;
#if CONFIG_CUSTOM_BME280_MODE_CONTINUOUS
	struct k_thread acq_thread;
#endif
};

#ifdef __cplusplus
extern "C" {
#endif

int custom_bme280_runtime_start(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_BME280_H_ */
