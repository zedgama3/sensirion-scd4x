# Sensirion Embedded I2C SCD4x Driver

This is an ESP-IDF implementation of the generic driver for the [Sensirion SCD4x Carbon Dioxide Sensor](https://www.sensirion.com/scd4x/).
It enables developers to communicate with the SCD4x sensor using the basic ESP-IDF library.

# Getting started

**Includes:**
```
#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

#include "driver/i2c_master.h"
```

**CMake:**

If this is not in your ESP-IDF components folder, make sure to add the following to then end of your root CMMakeLists.txt:
```
set(EXTRA_COMPONENT_DIRS "./components")
```
*This example assumes that the driver directory will be in the `/components/` folder of your project.*

**Setup:**

Create a bus configuration and then initiate the library.

```
i2c_master_bus_config_t i2c_mst_config = {
	.scl_io_num = 7,
	.sda_io_num = 6,
};

sensirion_i2c_hal_init(i2c_mst_config);

ESP_LOGD(TAG, "Clean up potential SCD40 states");
scd4x_wake_up();
ESP_LOGD(TAG, "scd4x_stop_periodic_measurement()");
scd4x_stop_periodic_measurement();
ESP_LOGD(TAG, "scd4x_reinit()");
scd4x_reinit();
```

# Notes

This is based on the generic driver provided by Sensirion and should provide all of the function calls listed in their documentation.

Errors about NACK after calling `scd4x_wake_up()` are expected.

Finally,
This is my first ESP32 project, so please forgive any errors or ommisions.  Please check any code yourself before executing.