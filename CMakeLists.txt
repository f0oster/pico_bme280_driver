cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)

set(PICO_BOARD pico_w)
project(bme280_driver)
pico_sdk_init()

include_directories(./include)

add_executable(main
    main.c
    bme280.c
)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)

# Need to generate UF2 file for upload to RP2040
pico_add_extra_outputs(main)

# Add pico_stdlib library, add more if used
target_link_libraries(main pico_stdlib hardware_i2c)