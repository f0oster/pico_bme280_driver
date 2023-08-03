#ifndef bme280_h
#define bme280_h

#include "hardware/i2c.h"

// I2C Configuration
#define I2C_ID 0        // Bus ID
#define I2C_FREQ 400    // 400kHz
#define SDA_PIN 8
#define SCL_PIN 9

// BME280
#define BME280_ADDR 0x77
#define DEVID 0x60

// Registers (refer to BME280 datasheet for details, registers may span across multiple addresses)
#define REG_DEVID 0xD0              // Chip identification number chip_id[7:0] (0x60)
#define REG_RESET 0xE0              // Write 0xB6 to initialize a reset via the power-on-reset procedure
#define REG_TEMP_PRESS_CALIB 0x88   // Beginning of first chunk of calibration data registers (0x88 -> 0xA1)
#define REG_HUM_CALIB 0xE1          // Beginning of second chunk of calibration data registers (0xE1 -> 0xF0)
#define REG_CTRL_HUM 0xF2           // Humidity oversampling settings. Should be written *before* REG_CTRL_MEAS.
#define REG_STATUS 0xF3             // Device status (idle/performing measurment/updating NVM with measurement)
#define REG_CTRL_MEAS 0xF4          // Pressure and temperature data acquisition settings. Must be written *after* changing REG_CTRL_HUM for changes to apply
#define REG_CONFIG 0xF5             // Rate, filtering and interface settings.
#define REG_RAW_PRES 0xF7           // First chunk of raw pressure (0xF7 - 0xFA, 20bits)
#define REG_RAW_TEMP 0xFA           // First chunk of raw temperature (0xFA -> 0xFC, 20bits)
#define REG_RAW_HUM 0xFD            // First chunk of raw humidity (0xFD -> 0xFE, 16bits)

// Read "press" registers (0xF7 -> 0xF9) (_msb, _lsb, _xlsb) (20bit)
// Read "temp" registers (0xFA -> 0xFC) (_msb, _lsb, _xlsb) (20bit)
// Read "hum" registers (0xFD -> 0xFE) (_msb, _lsb) (16bit)
#define SENSOR_BURST_READ_STARTING_ADDRESS REG_RAW_PRES
#define SENSOR_BURST_READ_REG_COUNT 8

/*
0x24 = 00100100
Mode = Sleep mode
00
Temperature Oversampling = Oversampling x1
001
Pressure Oversampling = Oversampling x1
001 
*/
#define CTRL_MEAS_SETTING_TOS1_POS1_MODESLEEP 0x24

// Lengths
#define CALIB_DATA_FIRST_CHUNK_LENGTH 26
#define CALIB_DATA_SECOND_CHUNK_LENGTH 8

// Combine two bytes, representing one 16 bit register.
#define COMBINE_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

typedef struct raw_data
{
    int32_t temperature;
    int32_t pressure;
    int32_t humidity;
} raw_data;

typedef struct compensated_data {
    int32_t temperature;
    uint32_t pressure;
    uint32_t humidity;
} compensated_data;

typedef struct calib_data
{
    // Temperature
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    // Pressure
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;

    // Humidity
    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;

    // Used during humidity calculations
    int32_t t_fine;
} calib_data;

typedef struct BME280
{
    uint8_t address;
    i2c_inst_t* i2c;

    uint8_t t_mode;
    uint8_t p_mode;
    uint8_t h_mode;
    uint8_t iir;

    calib_data calib_data;
    raw_data raw_data;
    compensated_data compensated_data;

} BME280;

/**
 * @brief Reads data from the BME280 sensor registers via I2C.
 *
 * Read num_bytes bytes sequentially starting from register address on the BME280 sensor over the I2C interface.
 * It writes the register address to the sensor first and then reads the specified number of bytes
 * from the sensor into the provided buffer.
 *
 * @param[in] bme280 A pointer to the initialized BME280 struct representing the sensor.
 * @param[in] reg The register address from which to start reading data.
 * @param[out] buf A pointer to the buffer where the read data will be stored.
 * @param[in] num_bytes The number of bytes to read from the sensor.
 *
 * @return Returns the number of bytes successfully read from the sensor. It may be less than `num_bytes`
 *         if there was an error during communication. If `num_bytes` is less than 1, the function returns 0.
 */
int reg_read_seq(BME280* bme280, const uint8_t reg, uint8_t* buf, const uint8_t num_bytes);

/*
Write num_bytes bytes sequentially, starting from the specified register address
*/

/**
 * @brief Writes data to the BME280 sensor registers via I2C.
 *
 * Writes num_bytes bytes sequentially starting from register address to the BME280 sensor over the I2C interface.
 * It prepares a message containing the register address followed by the data to be written, and then
 * sends the message to the sensor using I2C communication.
 *
 * @param[in] bme280 A pointer to the initialized BME280 struct representing the sensor.
 * @param[in] reg The register address to which the data will be written.
 * @param[in] buf A pointer to the buffer containing the data to be written to the sensor.
 * @param[in] num_bytes The number of bytes to write to the sensor.
 *
 * @return Returns the number of bytes written to the sensor.
 */
int reg_write_seq(BME280* bme280, const uint8_t reg, uint8_t* buf, const uint8_t num_bytes);

/**
 * @brief Stores temperature and pressure calibration data into the BME280 struct.
 *
 * This function extracts temperature and pressure calibration data from the contiguous memory buffer
 * `reg_buffer` and stores it into the `BME280` struct's `calib_data` member, specifically the following fields:
 * dig_t1, dig_t2, dig_t3, dig_p1, dig_p2, dig_p3, dig_p4, dig_p5, dig_p6, dig_p7, dig_p8, dig_p9, and dig_h1.
 *
 * @param[in,out] bme280 A pointer to the initialized BME280 struct representing the sensor.
 * @param[in] reg_buffer A pointer to the contiguous memory buffer containing the temperature and pressure calibration data.
 *                       The buffer should be at least 26 bytes long.
 */
void bme280_store_contiguous_calib_temppress_data(BME280* bme280, uint8_t* reg_buffer);

/**
 * @brief Stores humidity calibration data into the BME280 struct.
 *
 * This function extracts humidity calibration data from the contiguous memory buffer `reg_buffer`
 * and stores it into the `BME280` struct's `calib_data` member, specifically the humidity-related fields:
 * dig_h2, dig_h3, dig_h4, dig_h5, and dig_h6.
 *
 * @param[in,out] bme280 A pointer to the initialized BME280 struct representing the sensor.
 * @param[in] reg_buffer A pointer to the contiguous memory buffer containing the humidity calibration data.
 *                       The buffer should be at least 7 bytes long.
 */
void bme280_store_contiguous_calib_hum_data(BME280* bme280, uint8_t* reg_buffer);

/**
 * @brief Initializes the BME280 sensor.
 *
 * This function initializes the BME280 sensor by configuring the necessary I2C
 * port, reading the device ID to verify communication, fetching and storing the
 * calibration data, and setting the sensor's operating modes.
 *
 * @param[in,out] bme280 A pointer to the BME280 struct representing the sensor.
 *
 * @return Returns 0 on successful initialization, otherwise a non-zero value is returned.
 *         A non-zero value may indicate a communication error with the BME280 sensor.
 */
int bme280_init(BME280* bme280);

/**
 * @brief Reads raw sensor data from the BME280 sensor.
 *
 * This function sets the appropriate configuration for temperature and pressure oversampling,
 * triggers a single forced mode measurement, and reads the raw sensor data from the BME280 sensor.
 * The raw sensor data for temperature, pressure, and humidity are stored in the `BME280` struct
 * provided by the user.
 *
 * @param[in,out] bme280 A pointer to the initialized BME280 struct representing the sensor.
 *
 * @return Returns 0 on successful reading of raw sensor data, otherwise a non-zero value is returned.
 *         A non-zero value may indicate a communication error with the BME280 sensor or a failure
 *         during the sensor measurement process.
 */

int bme280_get_raw_sensor_data(BME280* bme280);

/**
 * @brief Converts raw sensor data to compensated sensor data for temperature, pressure, and humidity.
 *
 * This function takes the raw sensor data read from the BME280 sensor and converts it to
 * compensated sensor data for temperature, pressure, and humidity. The conversion involves
 * applying calibration data stored in the `BME280` struct to obtain accurate temperature,
 * pressure, and humidity values.
 *
 * @param[in,out] bme280 A pointer to the initialized BME280 struct representing the sensor.
 *
 * @return Returns 0 on successful conversion of raw sensor data, otherwise a non-zero value is returned.
 *         A non-zero value may indicate a communication error with the BME280 sensor or a failure
 *         during the conversion process.
 */
int bme280_convert_sensor_data(BME280* bme280);

/**
 * @brief Obtains a fully compensated sensor reading from the BME280 sensor.
 *
 * This function reads raw sensor data from the BME280 sensor using `bme280_get_raw_sensor_data`,
 * and then converts the raw data to compensated sensor data for temperature, pressure, and humidity
 * using `bme280_convert_sensor_data`. The compensated sensor data is stored in the `BME280` struct
 * provided by the user.
 *
 * @param[in,out] bme280 A pointer to the initialized BME280 struct representing the sensor.
 *
 * @return Returns 0 on successful reading and conversion of sensor data, otherwise a non-zero value is returned.
 *         A non-zero value may indicate a communication error with the BME280 sensor, a failure during the
 *         sensor measurement process, or an error during the data conversion.
 */
int bme280_get_reading(BME280* bme280);

#endif