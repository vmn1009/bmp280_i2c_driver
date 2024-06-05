#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define DEVICE_FILE                            "/dev/bmp280_i2c_device"
// #define GET_RAW_VALUE _IOR('a', 1, bmp280_raw_data_t *)

#define BMP280_MAGIC_NUMBER                     'a'
#define BMP280_GET_RAW_VALUE                    _IOR(BMP280_MAGIC_NUMBER, 1, bmp280_raw_data_t *)
#define BMP280_SET_MODE                         _IOW(BMP280_MAGIC_NUMBER, 2, BMP280_Mode *)
#define BMP280_SET_FILTER                       _IOW(BMP280_MAGIC_NUMBER, 3, BMP280_Filter*)
#define BMP280_SET_OVERSAMPLING_PRESSURE        _IOW(BMP280_MAGIC_NUMBER, 4, BMP280_Oversampling*)
#define BMP280_SET_OVERSAMPLING_TEMPERATURE     _IOW(BMP280_MAGIC_NUMBER, 5, BMP280_Oversampling*)
#define BMP280_SET_STANDBY                      _IOW(BMP280_MAGIC_NUMBER, 6, BMP280_StandbyTime *)
#define BMP280_I2C_SET_ADDR                     _IOW(BMP280_MAGIC_NUMBER, 7, int32_t*)
#define RD_VALUE _IOR('a','b',bmp280_raw_data_t*)
#define WR_VALUE _IOW('a','a',int32_t*)


typedef enum {
    BMP280_MODE_SLEEP       = 0x00,
    BMP280_MODE_FORCED      = 0x01,
    BMP280_MODE_NORMAL      = 0x03
}BMP280_Mode;

typedef enum {
    BMP280_FILTER_OFF       = 0,
    BMP280_FILTER_2         = 1,
    BMP280_FILTER_4         = 2,
    BMP280_FILTER_8         = 3,
    BMP280_FILTER_16        = 4
}BMP280_Filter;

/**
 * Pressure oversampling settings
 */
typedef enum {
    BMP280_SKIPPED          = 0,            /* no measurement  */
    BMP280_ULTRA_LOW_POWER  = 1,            /* oversampling x1 */
    BMP280_LOW_POWER        = 2,            /* oversampling x2 */
    BMP280_STANDARD         = 3,            /* oversampling x4 */
    BMP280_HIGH_RES         = 4,            /* oversampling x8 */
    BMP280_ULTRA_HIGH_RES   = 5             /* oversampling x16 */
} BMP280_Oversampling;

typedef enum {
    BMP280_STANDBY_05       = 0,            /* stand by time 0.5ms */
    BMP280_STANDBY_62       = 1,            /* stand by time 62.5ms */
    BMP280_STANDBY_125      = 2,            /* stand by time 125ms */
    BMP280_STANDBY_250      = 3,            /* stand by time 250ms */
    BMP280_STANDBY_500      = 4,            /* stand by time 500ms */
    BMP280_STANDBY_1000     = 5,            /* stand by time 1s */
    BMP280_STANDBY_2000     = 6,            /* stand by time 2s BMP280, 10ms BME280 */
    BMP280_STANDBY_4000     = 7,            /* stand by time 4s BMP280, 20ms BME280 */
} BMP280_StandbyTime;

typedef struct {
    unsigned short int      dig_T1;
    short int               dig_T2;
    short int               dig_T3;
    unsigned short int      dig_P1;
    short int               dig_P2;
    short int               dig_P3;
    short int               dig_P4;
    short int               dig_P5;
    short int               dig_P6;
    short int               dig_P7;
    short int               dig_P8;
    short int               dig_P9;    
    int                     raw_temperature;
    int                     raw_pressure;
} bmp280_raw_data_t;



int             bmp280_init(int bmp280_fd, BMP280_Mode power_mode, BMP280_Filter filter,BMP280_Oversampling oversampling_pressure,BMP280_StandbyTime oversampling_temperature,BMP280_StandbyTime standby_time);
int             bmp280_read_fixed(bmp280_raw_data_t *bmp280, int32_t *temperature, uint32_t *pressure);
void            bmp280_read_float(bmp280_raw_data_t *bmp280, int32_t *temperature, uint32_t *pressure);
static int32_t  compensate_temperature(bmp280_raw_data_t *bmp280, int32_t adc_temp, int32_t *fine_temp);
static uint32_t compensate_pressure(bmp280_raw_data_t *bmp280, int32_t adc_press, int32_t fine_temp);

