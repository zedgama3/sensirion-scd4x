idf_component_register(SRCS     "scd4x_i2c.c" 
                                "sensirion_i2c.c" 
                                "sensirion_i2c_hal.c" 
                                "sensirion_common.c"
                        INCLUDE_DIRS "include"
                        REQUIRES esp_driver_i2c)

