#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "bme280.h"

int main()
{

    stdio_init_all();

    BME280 bme280;
    if (bme280_init(&bme280) == -1)
    {
        printf("Failed to initialize BME280!");
        return -1;
    }

    while (true)
    {
        if (bme280_get_reading(&bme280) == 0) {
            printf("Raw Temperature Data: %d\r\n", bme280.raw_data.temperature);
            printf("Compensated Temperature Value: %dc\r\n", bme280.compensated_data.temperature); // 4520 = 45.20celsius
            printf("Compensated Divided Temperature Value: %dc\r\n", bme280.compensated_data.temperature / 100);

            printf("Raw pressure: %d\r\n", bme280.raw_data.pressure);
            printf("Compensated pressure (Pa): %d\r\n", bme280.compensated_data.pressure / 256); // Pa
            printf("Compensated pressure (hPa): %d\r\n", (bme280.compensated_data.pressure / 256) / 100); // hPa

            printf("Raw humidity: %d\r\n", bme280.raw_data.humidity);
            printf("Compensated humidity data: %lu\r\n", (bme280.compensated_data.humidity / 1000)); // Relative Humidity
        }

        sleep_ms(5000);
    }

}
