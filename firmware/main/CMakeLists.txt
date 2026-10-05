idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES esp_driver_i2s esp_driver_gpio espressif__esp-dsp esp_driver_usb_serial_jtag
                       INCLUDE_DIRS ".")
