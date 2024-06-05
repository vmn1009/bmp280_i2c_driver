/*
    Author: Minh Nhat
    Date: 16/5/2024
    Description: Write a simple BMP280 driver
*/

#ifndef __BMP280_H__
#define __BMP280_H__

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/types.h> 
#include <linux/ioctl.h>
/**
 * BMP280 or BME280 address is 0x77 if SDO pin is high, and is 0x76 if SDO pin is low.
 */

#define BMP280_I2C_ADDRESS_0           0x76     // BMP280 I2C address when SDO pin is low
#define BMP280_I2C_ADDRESS_1           0x77     // BMP280 I2C address when SDO pin is high
#define BMP280_CHIP_ID                 0x58     // BMP280 sensor has chip id 0x58
#define BME280_CHIP_ID                 0x60     // BME280 sensor has chip id 0x60

// Address of some registers for BMP280
#define BMP280_RESET_REG               0xE0     // Reset register
#define BMP280_STATUS_REG              0xF3     // Status register
#define BMP280_CTRL_MEAS_REG           0xF4     // Control and measurement register
#define BMP280_CONFIG_REG              0xF5     // Configuration register
#define BMP280_ID_REG                  0xD0     // ID register
#define BMP280_RESET_VALUE             0xB6     // Reset value

// Pressure registers
#define BMP280_PRESS_MSB_REG           0xF7     // Pressure MSB register
#define BMP280_PRESS_LSB_REG           0xF8     // Pressure LSB register
#define BMP280_PRESS_XLSB_REG          0xF9     // Pressure XLSB register

// Temperature registers
#define BMP280_TEMP_MSB_REG            0xFA     // Temperature MSB register
#define BMP280_TEMP_LSB_REG            0xFB     // Temperature LSB register
#define BMP280_TEMP_XLSB_REG           0xFC     // Temperature XLSB register

// Calibration data registers
#define BMP280_DIG_T1_REG              0x88     // Calibration data T1
#define BMP280_DIG_T2_REG              0x8A     // Calibration data T2
#define BMP280_DIG_T3_REG              0x8C     // Calibration data T3

#define BMP280_DIG_P1_REG              0x8E     // Calibration data P1
#define BMP280_DIG_P2_REG              0x90     // Calibration data P2
#define BMP280_DIG_P3_REG              0x92     // Calibration data P3
#define BMP280_DIG_P4_REG              0x94     // Calibration data P4
#define BMP280_DIG_P5_REG              0x96     // Calibration data P5
#define BMP280_DIG_P6_REG              0x98     // Calibration data P6
#define BMP280_DIG_P7_REG              0x9A     // Calibration data P7
#define BMP280_DIG_P8_REG              0x9C     // Calibration data P8
#define BMP280_DIG_P9_REG              0x9E     // Calibration data P9

#define BMP280_PRESSUREDATA            BMP280_PRESS_MSB_REG // Pressure data register
#define BMP280_TEMPDATA                BMP280_TEMP_MSB_REG  // Temperature data register

// IOCTL commands   
#define BMP280_MAGIC_NUMBER                 'a' // Magic number for IOCTL
#define BMP280_GET_RAW_VALUE                _IOR(BMP280_MAGIC_NUMBER, 1, bmp280_raw_data_t *)       // Get raw value command
#define BMP280_SET_MODE                     _IOW(BMP280_MAGIC_NUMBER, 2, BMP280_Mode *)             // Set mode command
#define BMP280_SET_FILTER                   _IOW(BMP280_MAGIC_NUMBER, 3, BMP280_Filter*)            // Set filter command
#define BMP280_SET_OVERSAMPLING_PRESSURE    _IOW(BMP280_MAGIC_NUMBER, 4, BMP280_Oversampling*)      // Set oversampling for pressure
#define BMP280_SET_OVERSAMPLING_TEMPERATURE _IOW(BMP280_MAGIC_NUMBER, 5, BMP280_Oversampling*)      // Set oversampling for temperature
#define BMP280_SET_STANDBY                  _IOW(BMP280_MAGIC_NUMBER, 6, BMP280_StandbyTime*)       // Set standby time
#define BMP280_I2C_SET_ADDR                 _IOW(BMP280_MAGIC_NUMBER, 7, int32_t*)                  // Set I2C address

#define SWAP_2BYTES(x)                 (((x & 0xFFFF) >> 8) | ((x & 0xFF) << 8))                    // Macro to swap two bytes
#define BMP280_DEVICE_NAME             "bmp280_i2c_device"                                          // Device name

/**
 * BMP280 module operation modes.
 * Forced - Measurement is initiated by user.
 * Normal - Continuous measurement.
 */
typedef enum {
    BMP280_MODE_SLEEP            = 0x00,                    // Sleep mode
    BMP280_MODE_FORCED           = 0x01,                    // Forced mode
    BMP280_MODE_NORMAL           = 0x03                     // Normal mode
} BMP280_Mode;

/**
 * BMP280 filter settings.
 */
typedef enum {
    BMP280_FILTER_OFF            = 0,                       // No filter
    BMP280_FILTER_2              = 1,                       // Filter coefficient 2
    BMP280_FILTER_4              = 2,                       // Filter coefficient 4
    BMP280_FILTER_8              = 3,                       // Filter coefficient 8
    BMP280_FILTER_16             = 4                        // Filter coefficient 16
} BMP280_Filter;

/**
 * Pressure oversampling settings.
 */
typedef enum {
    BMP280_SKIPPED               = 0,                       // No measurement
    BMP280_ULTRA_LOW_POWER       = 1,                       // Oversampling x1
    BMP280_LOW_POWER             = 2,                       // Oversampling x2
    BMP280_STANDARD              = 3,                       // Oversampling x4
    BMP280_HIGH_RES              = 4,                       // Oversampling x8
    BMP280_ULTRA_HIGH_RES        = 5                        // Oversampling x16
} BMP280_Oversampling;

/**
 * Standby time between measurements in normal mode.
 */
typedef enum {
    BMP280_STANDBY_05            = 0,                       // Standby time 0.5ms
    BMP280_STANDBY_62            = 1,                       // Standby time 62.5ms
    BMP280_STANDBY_125           = 2,                       // Standby time 125ms
    BMP280_STANDBY_250           = 3,                       // Standby time 250ms
    BMP280_STANDBY_500           = 4,                       // Standby time 500ms
    BMP280_STANDBY_1000          = 5,                       // Standby time 1s
    BMP280_STANDBY_2000          = 6,                       // Standby time 2s for BMP280, 10ms for BME280
    BMP280_STANDBY_4000          = 7,                       // Standby time 4s for BMP280, 20ms for BME280
} BMP280_StandbyTime;

/**
 * Configuration parameters for BMP280 module.
 * Use function bmp280_init_default_params to use default configuration.
 */
typedef struct {
    BMP280_Mode                   mode;                     // Operation mode
    BMP280_Filter                 filter;                   // Filter setting
    BMP280_StandbyTime            standby;                  // Standby time
    BMP280_Oversampling           oversampling_pressure;    // Pressure oversampling
    BMP280_Oversampling           oversampling_temperature; // Temperature oversampling
    BMP280_Oversampling           oversampling_humidity;    // Humidity oversampling (only for BME280)
} bmp280_params_t;

/**
 * Structure to hold raw data and calibration parameters.
 */
typedef struct {
    unsigned short int             dig_T1;                  // Calibration parameter T1
    short int                      dig_T2;                  // Calibration parameter T2
    short int                      dig_T3;                  // Calibration parameter T3
    unsigned short int             dig_P1;                  // Calibration parameter P1
    short int                      dig_P2;                  // Calibration parameter P2
    short int                      dig_P3;                  // Calibration parameter P3
    short int                      dig_P4;                  // Calibration parameter P4
    short int                      dig_P5;                  // Calibration parameter P5
    short int                      dig_P6;                  // Calibration parameter P6
    short int                      dig_P7;                  // Calibration parameter P7
    short int                      dig_P8;                  // Calibration parameter P8
    short int                      dig_P9;                  // Calibration parameter P9
    int                            raw_temperature;         // Raw temperature data
    int                            raw_pressure;            // Raw pressure data
} bmp280_raw_data_t;

/**
 * Structure to hold BMP280 device context.
 */
typedef struct {
    uint8_t                        ctrl_meas_reg;           // Control and measurement register value
    uint8_t                        config_reg;              // Configuration register value
    uint8_t                        id;                      // Chip ID
    uint16_t                       addr;                    // I2C address
    struct cdev                    cdev;                    // Character device structure
    struct class                   *dev_class;              // Device class
    bmp280_params_t                config;                  // Configuration parameters
    bmp280_raw_data_t              raw_data;                // Raw data and calibration parameters
    struct i2c_client              *client;                 // I2C client
} BMP280_HandleTypedef;

// Function prototypes
/**
 * @brief Open function for the ETX driver.
 *
 * This function is called when the device is opened.
 *
 * @param inode Pointer to the inode structure.
 * @param file Pointer to the file structure.
 * @return Returns 0 on success, or a negative error code on failure.
 */
static int etx_open(struct inode *inode, struct file *file);

/**
 * @brief Release function for the ETX driver.
 *
 * This function is called when the device is closed.
 *
 * @param inode Pointer to the inode structure.
 * @param file Pointer to the file structure.
 * @return Returns 0 on success, or a negative error code on failure.
 */
static int etx_release(struct inode *inode, struct file *file);

/**
 * @brief Read function for the ETX driver.
 *
 * This function is called when data is read from the device.
 *
 * @param filp Pointer to the file structure.
 * @param buf Pointer to the user buffer.
 * @param len Length of the data to read.
 * @param off Pointer to the current file offset.
 * @return Returns the number of bytes read, or a negative error code on failure.
 */
static ssize_t etx_read(struct file *filp, char __user *buf, size_t len, loff_t *off);

/**
 * @brief Write function for the ETX driver.
 *
 * This function is called when data is written to the device.
 *
 * @param filp Pointer to the file structure.
 * @param buf Pointer to the data buffer.
 * @param len Length of the data to write.
 * @param off Pointer to the current file offset.
 * @return Returns the number of bytes written, or a negative error code on failure.
 */
static ssize_t etx_write(struct file *filp, const char *buf, size_t len, loff_t *off);

/**
 * @brief IOCTL function for the ETX driver.
 *
 * This function is called to perform device-specific operations.
 *
 * @param file Pointer to the file structure.
 * @param cmd IOCTL command.
 * @param arg Argument for the IOCTL command.
 * @return Returns 0 on success, or a negative error code on failure.
 */
static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

/**
 * @brief Function to read raw data from a register of the BMP280 sensor.
 *
 * @param bmp280 Pointer to the BMP280 handle structure.
 * @param reg Register address to read from.
 * @return No return value.
 */
void read_raw(BMP280_HandleTypedef *bmp280, int reg);

/**
 * @brief Function to initialize default parameters for the BMP280 sensor.
 *
 * @param bmp280 Pointer to the BMP280 handle structure.
 * @return No return value.
 */
void bmp280_init_default_params(BMP280_HandleTypedef *bmp280);

/**
 * @brief Function to read calibration data from the BMP280 sensor.
 *
 * @param bmp280 Pointer to the BMP280 handle structure.
 * @return No return value.
 */
static void read_calibration_data(BMP280_HandleTypedef *bmp280);

/**
 * @brief Function to read an 8-bit register value from the BMP280 sensor.
 *
 * @param bmp280 Pointer to the BMP280 handle structure.
 * @param addr Register address to read from.
 * @param data Pointer to store the read value.
 * @return Returns true if successful, false otherwise.
 */
static bool bmp280_read_reg_value_8bit(BMP280_HandleTypedef *bmp280, uint8_t addr, uint8_t *data);

/**
 * @brief Function to write an 8-bit register value to the BMP280 sensor.
 *
 * @param bmp280 Pointer to the BMP280 handle structure.
 * @param addr Register address to write to.
 * @param data Value to write.
 * @return Returns true if successful, false otherwise.
 */
static bool bmp280_write_reg_value_8bit(BMP280_HandleTypedef *bmp280, uint8_t addr, uint8_t data);

/**
 * @brief Function to read a 16-bit register value from the BMP280 sensor.
 *
 * @param bmp280 Pointer to the BMP280 handle structure.
 * @param addr Register address to read from.
 * @param data Pointer to store the read value.
 * @return Returns 0 on success, or a negative error code on failure.
 */
static int bmp280_read_reg_value_16bit(BMP280_HandleTypedef *bmp280, uint8_t addr, uint16_t *data);


#endif // __BMP280_H__
