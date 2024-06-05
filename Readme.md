Group: 8  
Date: 20/5/2024  
Description: User manual for how can use bmp280 library

##### Group Team consist of:
+ Vo Thanh Minh Nhat - 21146132
+ Le Khac Chan Nguyen - 21146126
+ Nguyen Huu Minh - 21146129
##### Instructors: Bui Ha Duc

# Table of contents

* [Introduce bmp280 sensor](#Introduce-bmp280-sensor)
* [Features](#Features)
* [Specification](#Specification)
* [Raspberry Pi setup](#Raspberry-Pi-setup)
* [Connecting bmp280 sensor with raspberry pi ](#Connecting-bmp280-sensor-with-raspberry-pi)
* [Download, Configuration and Usage source code ](#Download,-Configuration-and-Usage-source-code)
* [How to use the bmp280 sensor library ](#How-to-use-the-bmp280-sensor-library)


### Introduce bmp280 sensor

This document describes how to build and use bmp280 sensor driver on raspberry pi 4 board
The BMP280 is an absolute barometric pressure sensor especially designed for mobile applications. 
Its small dimensions and its low power consumption of 2.7 µA @1Hz allow the implementation in battery driven devices such as mobile phones, GPS modules or watches. As the successor to the widely adopted BMP180
The BMP280 delivers high performance in all applications that require precise pressure measurement. The BMP280 operates at lower noise, supports new filter modes and an SPI interface within a footprint 63% smaller than the BMP180. The BMP280 support I2C interface to communicate with microctroller or SOC board have I2C protocol

### Features
    Bmp280 sensor used to measure
    Temperature from -40 deg C to 85 deg C
    Pressure from 300 to 1000hPa (corresponding with hight from + 9000...-500m compared to sea level)
### Specification
    Operation voltage: 3.3V
    Size: 11.5 x 14.5 mm
    Relative precision: +- 12hPa (corresponding with +- 1m950 ~ 1050hPa @ 25 deg C)
    Absolute precision: +- 1hPa (950-1050hPa, 0 ~+ 40 deg C)
    Temperature compensation: 1.5 Pa/K, corresponding to At 12.6cm/K(About 25-40 deg C at 900hPa)
---
### Raspberry Pi setup

> [!IMPORTANT]
> Before download source code form github, you should install some packages bellow:

```
$sudo apt install linux-headers-$(uname -r) -y
$sudo apt install -y libi2c-dev i2c-tools
$sudo apt install -y build-essential
```
Enable I2C protocol step by step bellow:
```
$sudo apt update & sudo apt upgrade
$sudo raspi-config
```
After direct to Interfacing Options and enter you will see I2C. The system has question do you turn on I2C protocol ? Choose Yes and exit
You need to need to reboot to update options that you was just changed
```
$sudo reboot
```
### Connecting bmp280 sensor with raspberry pi 
    The BMP280 sensor has 4 pins for I2C communication
     + VCC pin connects to the PIN 17 on the Raspberry Pi board
     + GND pin connects to the PIN 9 of the Raspberr Pi board
     + SCL pin connects to the (GPIO 3) PIN 5 of the Raspberry Pi
     + SDA pin connects to the (GPIO 2) PIN 3 of ther Raspberry Pi 
    You can see the picture bellow to check:
![alt text](https://docs.sunfounder.com/projects/umsk/en/latest/_images/Lesson_20_bmp280_pi_bb.png "Diagram raspberry pi 4 connect bmp280") 
---
### Functional description
BMP280 provides highest flexibility to the designer and can be adapted to the requirements regarding accuracy, measurement time and power consumption by selecting from a high number of possible combinations of the sensor settings.

##### Power Modes
* Sleep Mode: No measurements are performed.
* Normal Mode: Cycles between active measurement and inactive standby periods (see how to config standby at standby configuration).
* Forced Mode: Performs a single measurement and then returns to sleep mode.

| mode [1:0]            | Mode          |  
| --------------------- |:-------------:|   
| BMP280_MODE_SLEEP     | sleep mode    |                          
| BMP280_MODE_FORCED    | forced mode   |  
| BMP280_MODE_NORMAL    | normal mode   |  

##### Measurement and Oversampling Settings
The BMP280 provides predefined settings for different use-cases by combining pressure and temperature measurement oversampling rates, ranging from 0 to 16 times oversampling. These settings can be independently selected for:
* Temperature Measurement
* Ultra Low Power
* Low Power
* Standard Resolution
* High Resolution
* Ultra High Resolution

##### Pressure measurement 
Pressure measurement can be enabled or skipped. Skipping the measurement is useful if the BMP280 is used solely as a temperature sensor. When pressure measurement is enabled, several oversampling options are available. Each oversampling step reduces noise and increases output resolution by one bit.

 | osrs_p [2:0]                  | Oversampling setting          | Pressure oversampling | Typical pressure resolution  | Reccommended temperate oversampling |
 | ----------------------------- | ----------------------------- |:---------------------:| :--------------------------: | :---------------------------------: | 
 | BMP280_SKIPPED                | Pressure measurement skipped  | Skipped               | None                         | As needed                           |                        
 | BMP280_ULTRA_LOW_POWER        | Ultra low power               | x1                    | 16 bit / 2.62 Pa             | x1                                  |
 | BMP280_LOW_POWER              | Low power                     | x2                    | 17 bit / 1.31 Pa             | x1                                  |
 | BMP280_STANDARD               | Standard resolution           | x4                    | 18 bit / 0.66 Pa             | x1                                  |
 | BMP280_HIGH_RES               | High resolution               | x8                    | 19 bit / 0.33 Pa             | x1                                  |
 | BMP280_ULTRA_HIGH_RES         | Ultra high resolution         | x16                   | 20 bit / 0.16 Pa             | x2                                  |

##### Temperature measurement
Temperature measurement can be enabled or skipped. Skipping the measurement could be useful to measure pressure extremely rapidly. When enabled, several oversampling options exist. Each oversampling step reduces noise and increases the output resolution by one bit,

| osrs_t [2:0]                  | Oversampling setting          | Pressure oversampling | Typical temperate resolution | 
| ----------------------------- |:-----------------------------:|:---------------------:| :--------------------------: |  
| BMP280_SKIPPED                | Temperate measurement skipped | Skipped               |  None                    |                         
| BMP280_ULTRA_LOW_POWER        | Ultra low power               | x1                    | 16 bit / 0.0050 degC         | 
| BMP280_LOW_POWER              | Low power                     | x2                    | 17 bit / 0.0025 degC         | 
| BMP280_STANDARD               | Standard resolution           | x4                    | 18 bit / 0.0012 degC         | 
| BMP280_HIGH_RES               | High resolution               | x8                    | 19 bit / 0.0006 degC         | 
| BMP280_ULTRA_HIGH_RES         | Ultra high resolution         | x16                   | 20 bit / 0.0003 degC         | 

##### Filter settings
The environmental pressure is subject to many short-term changes, such as those caused by slamming doors, windows, or wind. To suppress these disturbances in the output data without causing additional interface traffic and processor workload, the BMP280 features an internal Infinite Impulse Response (IIR) filter. The filter reduces the bandwidth of the output signals, smoothing out sudden changes.

| filter [2:0]      | Filter coefficient | Samples to reach >= 75% of step response | 
| ----------------- |:------------------:| :--------------------------------------: |  
| BMP280_FILTER_OFF | Filter off         | 1                                        |                         
| BMP280_FILTER_2   | 2                  | 2                                        | 
| BMP280_FILTER_4   | 4                  | 5                                        | 
| BMP280_FILTER_8   | 8                  | 11                                       | 
| BMP280_FILTER_16  | 16                 | 22                                       | 

##### Standby configuration
In Normal Mode, the BMP280 sensor continuously cycles between active measurement and inactive standby periods, defined by tstandby. The current during standby (IDDSB) is slightly higher than in sleep mode. Once the mode, measurement, and filter options are set, the last measurement results can be retrieved from the data registers without further write operations. Normal mode is recommended when using the IIR filter and is useful for applications that need to filter out short-term disturbances, such as blowing into the sensor.

| t_sb [2:0]            | time_stanby [ms]   |  
| --------------------- |:------------------:|   
| BMP280_STANBY_05      | 0.5                |                          
| BMP280_STANBY_62      | 62.5               |  
| BMP280_STANBY_125     | 125                |  
| BMP280_STANBY_250     | 250                |  
| BMP280_STANBY_500     | 500                |  
| BMP280_STANBY_1000    | 1000               |
| BMP280_STANBY_2000    | 2000               |
| BMP280_STANBY_4000    | 4000               |              

---
### Download, Configuration and Usage source code
Step 1: You can download source code with this link below:
```
cd ~
mkdir bmp280_driver
sudo git clone https://github.com/vmn1009/bmp280-driver.git
```

Step 2: overlay application bmp280 for pi4
```
cd kernel_source
make dt
sudo dtoverlay bmp280_overlay.dtbo
``` 

Step 3: compile and insmod module
```
cd kernel_source
make module
sudo insmod bmmp280_driver.ko
```
> [!TIP]
> To determine whether the driver has been succesfully insmoded. Please check with this command.
> + $dmesg -tail.
> + If you see the notification line is: Device Driver Insert...Done!!! it indicates that the insertion was successful.

Step 4: Change mode for device file
```
sudo chmod 0777 /dev/bmp280_driver.ko
```

Step 5: get data throught user app 
```
cd ..
cd user_source
make
./bmp280_user_test
```

### How to use the bmp280 sensor library
Step 1: you need to include library headers:
```C
#include "bmp280_user_app.h"
```

Step 2: open the device file, as follows:

```C
int fd;
fd = open(DEVICE_FILE, O_RDWR);
if(fd < 0) {
    printf("Cannot open device file...\n");
    exit(1);
}
```

Step 3: When you have opened the device, you will call bmp280_init function to config some modes for bmp280 you want to communicate:

```C
bmp280_init(int bmp280_fd, power_mode, filter, oversampling_pressure, oversampling_temperature, standby_time);
```

> [!NOTE]
> Optionals of these mode you will see:
> + power_mode see the optinal table: [Power Modes](#Power-Modes)
> + filter see the optinal table: [Filter settings](#Filter-settings)
> + oversampling_pressure see the optinal table: [Oversampling pressure](#Pressure-measurement)
> + oversampling_temperature see the optinal table: [Oversampling temperature](#Temperature-measurement)
> + standby_time see the optinal table: [Standby configuration](#Standby-configuration)

Step 3: You must get raw data from BMP280 sensor using the ioctl system call:

```C
if (ioctl(fd, BMP280_GET_RAW_VALUE, (bmp280_raw_data_t*) &raw_data_ioctl) < 0) {
    perror("Failed to get raw data");
    close(fd);
    return -1;
}

```

Step 4: To read temperatue and pressure data you will call bmp280_read_float and pass 2 parameter one after another (temperature, pressure)
```C
int32_t temp;
uint32_t press;
bmp280_read_float(&raw_data_ioctl, &temp, &press);
printf("Temperature: %d\n", temp);
printf("Pressure %d\n", press/100);
```

Step 5: close device file

```C
close(fd);
```

> [!NOTE]
> You can see all example code in user_source folder at file main.c:

















