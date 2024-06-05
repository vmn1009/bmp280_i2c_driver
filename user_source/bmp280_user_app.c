#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include "bmp280_user_app.h"


static int32_t compensate_temperature(bmp280_raw_data_t *bmp280, int32_t adc_temp, int32_t *fine_temp) {
	int32_t var1, var2;

	var1 = ((((adc_temp >> 3) - ((int32_t) bmp280->dig_T1 << 1))) * (int32_t) bmp280->dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) bmp280->dig_T1) * ((adc_temp >> 4) - (int32_t) bmp280->dig_T1)) >> 12) * (int32_t) bmp280->dig_T3) >> 14;
	*fine_temp = var1 + var2;
	return (*fine_temp * 5 + 128) >> 8;
}

static uint32_t compensate_pressure(bmp280_raw_data_t *bmp280, int32_t adc_press, int32_t fine_temp) {
	int64_t var1, var2, p;

	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) bmp280->dig_P6;
	var2 = var2 + ((var1 * (int64_t) bmp280->dig_P5) << 17);
	var2 = var2 + (((int64_t) bmp280->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) bmp280->dig_P3) >> 8)	+ ((var1 * (int64_t) bmp280->dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) bmp280->dig_P1) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) bmp280->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) bmp280->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) bmp280->dig_P7 << 4);
	return p;
}



int bmp280_read_fixed(bmp280_raw_data_t *bmp280, int32_t *temperature, uint32_t *pressure) {
    int32_t adc_temp = bmp280->raw_temperature;
    uint32_t adc_pressure = bmp280->raw_pressure;
    int32_t fine_temp;

    *temperature = compensate_temperature(bmp280, adc_temp, &fine_temp);  // Cập nhật hàm gọi nếu cần
    *pressure = compensate_pressure(bmp280, adc_pressure, fine_temp);  // Cập nhật hàm gọi nếu cần

    return 0;
}


void bmp280_read_float(bmp280_raw_data_t *bmp280, int32_t *temperature, uint32_t *pressure) {
    int32_t fixed_temperature;
    uint32_t fixed_pressure;
    bmp280_read_fixed(bmp280, &fixed_temperature, &fixed_pressure);
    *temperature =  fixed_temperature / 100;
    *pressure =  fixed_pressure / 256;
}

int bmp280_init(int bmp280_fd, BMP280_Mode power_mode, BMP280_Filter filter,BMP280_Oversampling oversampling_pressure,BMP280_StandbyTime oversampling_temperature,BMP280_StandbyTime standby_time)
{

    unsigned short int value;

    if (ioctl(bmp280_fd, BMP280_SET_MODE, &power_mode) < 0) {
        perror("Failed to set");
        close(bmp280_fd);
        return -1;
    }
    if (ioctl(bmp280_fd, BMP280_SET_FILTER, &filter) < 0) {
        perror("Failed to get raw data");
        close(bmp280_fd);
        return -1;
    }
    if (ioctl(bmp280_fd, BMP280_SET_OVERSAMPLING_PRESSURE, &oversampling_pressure) < 0) {
        perror("Failed to get raw data");
        close(bmp280_fd);
        return -1;
    }
    if (ioctl(bmp280_fd, BMP280_SET_OVERSAMPLING_TEMPERATURE, &oversampling_temperature) < 0) {
        perror("Failed to get raw data");
        close(bmp280_fd);
        return -1;
    }
    if (ioctl(bmp280_fd, BMP280_SET_STANDBY, &standby_time) < 0) {
        perror("Failed to get raw data");
        close(bmp280_fd);
        return -1;
    }

}

