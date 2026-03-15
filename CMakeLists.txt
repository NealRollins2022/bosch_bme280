
if(CONFIG_MYBME280)

  target_include_directories(zephyr_interface BEFORE INTERFACE
    ${ZEPHYR_CURRENT_MODULE_DIR}/include
  )

  zephyr_library_sources_ifdef(
    CONFIG_MYBME280
    drivers/sensor/bme280/bme280.c
    drivers/sensor/bme280/bme280_i2c.c
  )

  if(EXISTS ${ZEPHYR_CURRENT_MODULE_DIR}/drivers/sensor/bme280/CMakeLists.txt)
    add_subdirectory(drivers/sensor/bme280)
  endif()

endif()
