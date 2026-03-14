#define DT_DRV_COMPAT bosch_bme280

/* I2C transport layer — minimal, device tree handles all binding */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(bme280, CONFIG_SENSOR_LOG_LEVEL);

/* All I2C operations are in bme280.c using i2c_dt_spec from DT.
   This file is a placeholder for future transport-specific code.
   Kept for build structure consistency with pH driver. */