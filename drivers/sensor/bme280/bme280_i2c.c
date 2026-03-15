#define DT_DRV_COMPAT bosch_mybme280_custom

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(bosch_bme280, CONFIG_SENSOR_LOG_LEVEL);

/* I2C transport layer — all I2C operations in bme280.c using i2c_dt_spec from DT.
   This file is a placeholder for future transport-specific code. */
