#include "bme280.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

int reg_read_seq(BME280* bme280, const uint8_t reg, uint8_t* buf, const uint8_t num_bytes) {

    if (num_bytes < 1) {
        return 0;
    }

    int num_bytes_read = 0;

    i2c_write_blocking(bme280->i2c, bme280->address, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(bme280->i2c, bme280->address, buf, num_bytes, false);

    return num_bytes_read;
}

int reg_write_seq(BME280* bme280, const uint8_t reg, uint8_t* buf, const uint8_t num_bytes) {
    
    if (num_bytes < 1) {
        return 0;
    }

    int num_bytes_written = 0;
    uint8_t msg[num_bytes + 1];

    // Preprend the target register address to the message
    msg[0] = reg;
    memcpy(&msg[1], buf, num_bytes);

    num_bytes_written = i2c_write_blocking(bme280->i2c, bme280->address, msg, (num_bytes + 1), false);

    return num_bytes_written;
}

void bme280_store_contiguous_calib_temppress_data(BME280* bme280, uint8_t* reg_buffer) {

    bme280->calib_data.dig_t1 = COMBINE_BYTES(reg_buffer[1], reg_buffer[0]);
    bme280->calib_data.dig_t2 = (int16_t)COMBINE_BYTES(reg_buffer[3], reg_buffer[2]);
    bme280->calib_data.dig_t3 = (int16_t)COMBINE_BYTES(reg_buffer[5], reg_buffer[4]);
    bme280->calib_data.dig_p1 = COMBINE_BYTES(reg_buffer[7], reg_buffer[6]);
    bme280->calib_data.dig_p2 = (int16_t)COMBINE_BYTES(reg_buffer[9], reg_buffer[8]);
    bme280->calib_data.dig_p3 = (int16_t)COMBINE_BYTES(reg_buffer[11], reg_buffer[10]);
    bme280->calib_data.dig_p4 = (int16_t)COMBINE_BYTES(reg_buffer[13], reg_buffer[12]);
    bme280->calib_data.dig_p5 = (int16_t)COMBINE_BYTES(reg_buffer[15], reg_buffer[14]);
    bme280->calib_data.dig_p6 = (int16_t)COMBINE_BYTES(reg_buffer[17], reg_buffer[16]);
    bme280->calib_data.dig_p7 = (int16_t)COMBINE_BYTES(reg_buffer[19], reg_buffer[18]);
    bme280->calib_data.dig_p8 = (int16_t)COMBINE_BYTES(reg_buffer[21], reg_buffer[20]);
    bme280->calib_data.dig_p9 = (int16_t)COMBINE_BYTES(reg_buffer[23], reg_buffer[22]);
    bme280->calib_data.dig_h1 = reg_buffer[25];

}


void bme280_store_contiguous_calib_hum_data(BME280* bme280, uint8_t* reg_buffer) {

    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    bme280->calib_data.dig_h2 = COMBINE_BYTES(reg_buffer[1], reg_buffer[0]);
    bme280->calib_data.dig_h3 = reg_buffer[2];

    dig_h4_msb = reg_buffer[3] << 4;
    dig_h4_lsb = (reg_buffer[4] & 0x0F);
    bme280->calib_data.dig_h4 = dig_h4_msb | dig_h4_lsb;

    dig_h5_msb = reg_buffer[5] << 4;
    dig_h5_lsb = (reg_buffer[4] >> 4);
    bme280->calib_data.dig_h5 = dig_h5_msb | dig_h5_lsb;

    bme280->calib_data.dig_h6 = reg_buffer[6];

}

int bme280_init(BME280* bme280) {

    printf("\r\n\r\n\r\n");
    printf("=====================\r\n");
    printf("... Initialising BME280! ...\r\n");
    printf("=====================\r\n");
    printf("...\r\n");
    printf("..\r\n");
    printf(".\r\n");
    printf("\r\n");

    bme280->address = BME280_ADDR;

    // Configuration
    bme280->p_mode = 5;
    bme280->t_mode = 2;
    bme280->h_mode = 1; // Oversampling x1
    bme280->iir = 1;

    // Ports
    bme280->i2c = i2c0;
    // Initialize I2C port at 400 kHz
    printf("Initialising I2C port at %dkHz\r\n", I2C_FREQ);
    i2c_init(bme280->i2c, I2C_FREQ * 1000);

    // Configure the pins
    printf("Configuring pins for I2C: SDA (pin %d) and SCL (pin %d)\r\n", SDA_PIN, SCL_PIN);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // Buffer for reading and writing to registers
    uint8_t buf[CALIB_DATA_FIRST_CHUNK_LENGTH];

    // Query for the device ID
    printf("Attempting to communicate with BME280...\r\n");
    reg_read_seq(bme280, REG_DEVID, buf, 1);
    if (buf[0] != DEVID) {
        printf("ERROR: Could not communicate with BME280 via I2C, received DEVID = %X\r\n", buf[0]);
        return 1;
    }
    else {
        printf("Communicated with BME280 via I2C\r\n");
    }

    // Fetch and store the calibration data
    // Temperature and pressure calibration data
    printf("read %d bytes starting from REG_TEMP_PRESS_CALIB (%x)\r\n", reg_read_seq(bme280, REG_TEMP_PRESS_CALIB, buf, CALIB_DATA_FIRST_CHUNK_LENGTH), REG_TEMP_PRESS_CALIB);
    bme280_store_contiguous_calib_temppress_data(bme280, buf);
    // Humidity calibration data
    printf("read %d bytes starting from REG_HUM_CALIB (%x)\r\n", reg_read_seq(bme280, REG_HUM_CALIB, buf, 7), REG_HUM_CALIB);
    bme280_store_contiguous_calib_hum_data(bme280, buf);

    // Configuration
    buf[0] = bme280->h_mode;
    printf("writing to REG_CTRL_HUM register...");
    reg_write_seq(bme280, REG_CTRL_HUM, buf, 1);

    buf[0] = CTRL_MEAS_SETTING_TOS1_POS1_MODESLEEP;
    printf("writing to REG_CTRL_MEAS register...");
    reg_write_seq(bme280, REG_CTRL_MEAS, buf, 1);

    
    // Bit 7,6,5 [2:0] t_sb: 0.5ms standby duration (000)
    // Bit 2,3,4 [2:0] filter: 4 Filter coefficient (010)
    // Bit 0 - [0] spi3w_en: SPI disabled (0)
    buf[0] = (bme280->iir << 2); // 0x03 or 0b00000100

    reg_write_seq(bme280, REG_CONFIG, buf, 1);
    printf("writing to REG_CONFIG register...");
    printf("Initalized BME280 successfully!\r\n");
    return 0;

}

int bme280_get_raw_sensor_data(BME280* bme280) {

    uint8_t buf[8];

    // Bit 7,6,5 [2:0] osrs_t: 16x Temp Oversampling(101)
    // Bit 2,3,4 [2:0] osrs_p: 2x Pressure Oversampling (010)
    // Bit 1,0 - [1:0] mode: Forced mode (01)
    buf[0] = (bme280->p_mode << 5 | bme280->t_mode << 2 | 1); // 0xA9 or 0b10101001
    printf("Setting REG_CTRL_MEAS to 0x%X\r\n", buf[0]);
    reg_write_seq(bme280, REG_CTRL_MEAS, buf, 1);

    // TODO: Implement sleep that matches the measurement delays outlined in the datasheet
    // Appendix B: Measurement time and current calculation (pg 51)
    printf("Sleeping for 5 seconds\r\n");
    sleep_ms(5000);

    uint16_t combined = 0;
    do {
        reg_read_seq(bme280, REG_STATUS, buf, 2);
        combined = COMBINE_BYTES(buf[0], buf[1]);
        sleep_ms(100);
    } while ((combined & 0x08) == 0);

    int res = reg_read_seq(bme280, SENSOR_BURST_READ_STARTING_ADDRESS, buf, SENSOR_BURST_READ_REG_COUNT);
    printf("read %d bytes starting from SENSOR_BURST_READ_STARTING_ADDRESS (%x)\r\n", res, SENSOR_BURST_READ_STARTING_ADDRESS);

    uint32_t press_msb = (uint32_t)buf[0] << 12; // shift our msb 12 bits to the left to make room for lsb and xlsb
    uint32_t press_lsb = (uint32_t)buf[1] << 4; // shift our lsb 4 bits to the left, filling in the space after msb
    uint32_t press_xlsb = (uint32_t)buf[2] >> 4; // dispose of last 4 bits
    bme280->raw_data.pressure = (press_msb | press_lsb | press_xlsb); 

    uint32_t temp_msb = (uint32_t)buf[3] << 12; // shift our msb 12 bits to the left to make room for lsb and xlsb
    uint32_t temp_lsb = (uint32_t)buf[4] << 4; // shift our lsb 4 bits to the left, filling in the space after msb
    uint32_t temp_xlsb = (uint32_t)buf[5] >> 4; // dispose of last 4 bits
    bme280->raw_data.temperature = (temp_msb | temp_lsb | temp_xlsb);

    bme280->raw_data.humidity = COMBINE_BYTES(buf[6], buf[7]); 

    return 0;
    
}

int bme280_convert_sensor_data(BME280* bme280) {

    int32_t var1, var2, temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    // Calculate temperature using calibration data
    var1 = (int32_t)((bme280->raw_data.temperature / 8) - ((int32_t)bme280->calib_data.dig_t1 * 2));
    var1 = (var1 * ((int32_t)bme280->calib_data.dig_t2)) / 2048;
    var2 = (int32_t)((bme280->raw_data.temperature / 16) - ((int32_t)bme280->calib_data.dig_t1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)bme280->calib_data.dig_t3)) / 16384;
    bme280->calib_data.t_fine = var1 + var2;
    temperature = (bme280->calib_data.t_fine * 5 + 128) / 256;

    if (temperature < temperature_min) {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max) {
        temperature = temperature_max;
    }

    bme280->compensated_data.temperature = temperature;

    // Calculate pressure using calibration data
    int64_t pvar1, pvar2, p;
    pvar1 = ((int64_t)bme280->calib_data.t_fine) - 128000;
    pvar2 = pvar1 * pvar1 * (int64_t)bme280->calib_data.dig_p6;
    pvar2 = pvar2 + ((pvar1 * (int64_t)bme280->calib_data.dig_p5) << 17);
    pvar2 = pvar2 + (((int64_t)bme280->calib_data.dig_p4) << 35);
    pvar1 = ((pvar1 * pvar1 * (int64_t)bme280->calib_data.dig_p3) >> 8) + ((pvar1 * (int64_t)bme280->calib_data.dig_p2) << 12);
    pvar1 = (((((int64_t)1) << 47) + pvar1)) * ((int64_t)bme280->calib_data.dig_p1) >> 33;
    if (pvar1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - bme280->raw_data.pressure;
    p = (((p << 31) - pvar2) * 3125) / pvar1;
    pvar1 = (((int64_t)bme280->calib_data.dig_p9) * (p >> 13) * (p >> 13)) >> 25;
    pvar2 = (((int64_t)bme280->calib_data.dig_p8) * p) >> 19;
    p = ((p + pvar1 + pvar2) >> 8) + (((int64_t)bme280->calib_data.dig_p7) << 4);

    bme280->compensated_data.pressure = p;

    // Calculate humidity using calibration data
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = bme280->calib_data.t_fine - ((int32_t)76800);
    var2 = (int32_t)(bme280->raw_data.humidity * 16384);
    var3 = (int32_t)(((int32_t)bme280->calib_data.dig_h4) * 1048576);
    var4 = ((int32_t)bme280->calib_data.dig_h5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)bme280->calib_data.dig_h6)) / 1024;
    var3 = (var1 * ((int32_t)bme280->calib_data.dig_h3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)bme280->calib_data.dig_h2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)bme280->calib_data.dig_h1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }

    bme280->compensated_data.humidity = humidity;

    return 0;
}


int bme280_get_reading(BME280* bme280) {

    bme280_get_raw_sensor_data(bme280);
    bme280_convert_sensor_data(bme280);

    return 0;
}