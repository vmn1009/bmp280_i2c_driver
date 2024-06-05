obj-m += bmp280_driver.o

all: module dt
	echo Builded Device Tree Overlay and kernel module

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
dt: bmp280_overlay.dts
	dtc -@ -I dts -O dtb -o bmp280_overlay.dtbo bmp280_overlay.dts
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf testoverlay.dtbo
	
insmod:
	make
	sudo insmod bmp280_driver.ko
	sudo chmod 0777 /dev/bmp280_i2c_device

rmmod:
	sudo rmmod bmp280_driver
	make clean
