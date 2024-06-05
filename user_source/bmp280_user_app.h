#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define DEVICE_FILE                            "/dev/bmp280_i2c_device"
// #define GET_RAW_VALUE _IOR('a', 1, bmp280_raw_data_t *)

#define BMP280_MAGIC_NUMBER                     'a'                                                     /**< Magic number for BMP280 IOCTL commands. */
#define BMP280_GET_RAW_VALUE                    _IOR(BMP280_MAGIC_NUMBER, 1, bmp280_raw_data_t *)       /**< IOCTL command to get raw sensor values. */
#define BMP280_SET_MODE                         _IOW(BMP280_MAGIC_NUMBER, 2, BMP280_Mode *)             /**< IOCTL command to set sensor mode. */
#define BMP280_SET_FILTER                       _IOW(BMP280_MAGIC_NUMBER, 3, BMP280_Filter*)            /**< IOCTL command to set filter setting. */
#define BMP280_SET_OVERSAMPLING_PRESSURE        _IOW(BMP280_MAGIC_NUMBER, 4, BMP280_Oversampling*)      /**< IOCTL command to set pressure oversampling. */
#define BMP280_SET_OVERSAMPLING_TEMPERATURE     _IOW(BMP280_MAGIC_NUMBER, 5, BMP280_Oversampling*)      /**< IOCTL command to set temperature oversampling. */
#define BMP280_SET_STANDBY                      _IOW(BMP280_MAGIC_NUMBER, 6, BMP280_StandbyTime *)      /**< IOCTL command to set standby time. */
#define BMP280_I2C_SET_ADDR                     _IOW(BMP280_MAGIC_NUMBER, 7, int32_t*)                  /**< IOCTL command to set I2C address. */
#define RD_VALUE                                _IOR('a','b',bmp280_raw_data_t*)                        /**< Custom IOCTL command for reading sensor values. */
#define WR_VALUE                                _IOW('a','a',int32_t*)                                  /**< Custom IOCTL command for writing sensor values. */


/**
 * @brief Enumeration for BMP280 sensor operating modes.
 */
typedef enum {
    BMP280_MODE_SLEEP   = 0x00,                         /**< Sleep mode. */
    BMP280_MODE_FORCED  = 0x01,                         /**< Forced mode. */
    BMP280_MODE_NORMAL  = 0x03                          /**< Normal mode. */
} BMP280_Mode;

/**
 * @brief Enumeration for BMP280 sensor filter settings.
 */
typedef enum {
    BMP280_FILTER_OFF   = 0,                            /**< Filter off. */
    BMP280_FILTER_2     = 1,                            /**< Filter coefficient 2. */
    BMP280_FILTER_4     = 2,                            /**< Filter coefficient 4. */
    BMP280_FILTER_8     = 3,                            /**< Filter coefficient 8. */
    BMP280_FILTER_16    = 4                             /**< Filter coefficient 16. */
} BMP280_Filter;

/**
 * @brief Enumeration for BMP280 sensor pressure oversampling settings.
 */
typedef enum {
    BMP280_SKIPPED            = 0,                      /**< No measurement. */
    BMP280_ULTRA_LOW_POWER    = 1,                      /**< Oversampling x1. */
    BMP280_LOW_POWER          = 2,                      /**< Oversampling x2. */
    BMP280_STANDARD           = 3,                      /**< Oversampling x4. */
    BMP280_HIGH_RES           = 4,                      /**< Oversampling x8. */
    BMP280_ULTRA_HIGH_RES     = 5                       /**< Oversampling x16. */
} BMP280_Oversampling;

/**
 * @brief Enumeration for BMP280 sensor standby time settings.
 */
typedef enum {
    BMP280_STANDBY_05   = 0,                            /**< Standby time 0.5ms. */
    BMP280_STANDBY_62   = 1,                            /**< Standby time 62.5ms. */
    BMP280_STANDBY_125  = 2,                            /**< Standby time 125ms. */
    BMP280_STANDBY_250  = 3,                            /**< Standby time 250ms. */
    BMP280_STANDBY_500  = 4,                            /**< Standby time 500ms. */
    BMP280_STANDBY_1000 = 5,                            /**< Standby time 1s. */
    BMP280_STANDBY_2000 = 6,                            /**< Standby time 2s for BMP280, 10ms for BME280. */
    BMP280_STANDBY_4000 = 7                             /**< Standby time 4s for BMP280, 20ms for BME280. */
} BMP280_StandbyTime;





/**
 * @brief Structure to hold raw data from BMP280 sensor.
 *
 * This structure holds raw data read from the BMP280 sensor along with compensation coefficients.
 */
typedef struct {
    unsigned short int      dig_T1;                     /**< Calibration data for temperature compensation. */
    short int               dig_T2;                     /**< Calibration data for temperature compensation. */
    short int               dig_T3;                     /**< Calibration data for temperature compensation. */
    unsigned short int      dig_P1;                     /**< Calibration data for pressure compensation. */
    short int               dig_P2;                     /**< Calibration data for pressure compensation. */
    short int               dig_P3;                     /**< Calibration data for pressure compensation. */
    short int               dig_P4;                     /**< Calibration data for pressure compensation. */
    short int               dig_P5;                     /**< Calibration data for pressure compensation. */
    short int               dig_P6;                     /**< Calibration data for pressure compensation. */
    short int               dig_P7;                     /**< Calibration data for pressure compensation. */
    short int               dig_P8;                     /**< Calibration data for pressure compensation. */
    short int               dig_P9;                     /**< Calibration data for pressure compensation. */
    int                     raw_temperature;            /**< Raw temperature data read from the sensor. */
    int                     raw_pressure;               /**< Raw pressure data read from the sensor. */
} bmp280_raw_data_t;




/**
 * @brief Initializes the BMP280 sensor.
 *
 * This function initializes the BMP280 sensor with specified parameters.
 *
 * @param bmp280_fd           File descriptor for the BMP280 sensor.
 * @param power_mode          Power mode to set for the BMP280 sensor.
 * @param filter              Filter setting for the BMP280 sensor.
 * @param oversampling_pressure   Oversampling setting for pressure measurement.
 * @param oversampling_temperature   Oversampling setting for temperature measurement.
 * @param standby_time        Standby time setting for the BMP280 sensor.
 * 
 * @return                    Returns 0 on success, or a negative value on failure.
 */
int bmp280_init(int bmp280_fd, BMP280_Mode power_mode, 
                BMP280_Filter filter, BMP280_Oversampling oversampling_pressure,
                BMP280_StandbyTime oversampling_temperature, BMP280_StandbyTime standby_time);

/**
 * @brief Reads temperature and pressure data from the BMP280 sensor.
 *
 * This function reads temperature and pressure data from the BMP280 sensor in fixed point format.
 *
 * @param bmp280              Pointer to the structure containing raw data from the BMP280 sensor.
 * @param temperature         Pointer to store the temperature data.
 * @param pressure            Pointer to store the pressure data.
 *
 * @return                    Returns 0 on success, or a negative value on failure.
 */
int bmp280_read_fixed(bmp280_raw_data_t *bmp280, int32_t *temperature, uint32_t *pressure);

/**
 * @brief Reads temperature and pressure data from the BMP280 sensor.
 *
 * This function reads temperature and pressure data from the BMP280 sensor in floating point format.
 *
 * @param bmp280              Pointer to the structure containing raw data from the BMP280 sensor.
 * @param temperature         Pointer to store the temperature data.
 * @param pressure            Pointer to store the pressure data.
 *
 * @return                    No return value.
 */
void bmp280_read_float(bmp280_raw_data_t *bmp280, int32_t *temperature, uint32_t *pressure);

/**
 * @brief Compensates temperature data.
 *
 * This function compensates the raw temperature data read from the BMP280 sensor.
 *
 * @param bmp280              Pointer to the structure containing raw data from the BMP280 sensor.
 * @param adc_temp            Raw temperature data.
 * @param fine_temp           Pointer to store the compensated temperature data.
 *
 * @return                    No return value.
 */
static int32_t compensate_temperature(bmp280_raw_data_t *bmp280, int32_t adc_temp, int32_t *fine_temp);

/**
 * @brief Compensates pressure data.
 *
 * This function compensates the raw pressure data read from the BMP280 sensor.
 *
 * @param bmp280              Pointer to the structure containing raw data from the BMP280 sensor.
 * @param adc_press           Raw pressure data.
 * @param fine_temp           Compensated temperature data.
 *
 * @return                    Returns compensated pressure data.
 */
static uint32_t compensate_pressure(bmp280_raw_data_t *bmp280, int32_t adc_press, int32_t fine_temp);


