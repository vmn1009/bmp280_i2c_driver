#include "bmp280_user_app.c"

int main() {
    bmp280_raw_data_t raw_data_ioctl;
    int32_t temp;
    uint32_t press;
    int fd;
    int32_t value, number, count = 0;

    printf("\nOpening Driver\n");

    fd = open(DEVICE_FILE, O_RDWR);
    if(fd < 0) {
            printf("Cannot open device file...\n");
            return 0;
    }

    bmp280_init(fd, BMP280_MODE_NORMAL, BMP280_FILTER_4,BMP280_STANDARD, BMP280_STANDARD, BMP280_STANDBY_250);

    while(1) {
        if(ioctl(fd, BMP280_GET_RAW_VALUE, (bmp280_raw_data_t*) &raw_data_ioctl) < 0){
            perror("Failed to get raw data");
            close(fd);
            return -1;
        }
        bmp280_read_float(&raw_data_ioctl, &temp, &press);
        printf("\n--------------LINE %d--------------\n", count++);
        printf("Temperature: %d \n", temp);
        printf("Pressure %d\n", press/100);
        printf("----------------------------------\n");
        sleep(1);
    }
    close(fd);

    return 0;
}
