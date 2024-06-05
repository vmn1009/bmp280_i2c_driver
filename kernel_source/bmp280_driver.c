

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include<linux/slab.h>                 //kmalloc()
#include<linux/uaccess.h>              //copy_to/from_user()
#include <linux/ioctl.h>
#include <linux/err.h>
#include "bmp280.h"




#define BMP280_DEVICE_NAME      "bmp280_i2c_device"
int32_t data_ioctl = 0;
int32_t value = 0;
dev_t dev = 0;


/*
** Function Prototypes
*/

static bool bmp280_read_reg_value_8bit(BMP280_HandleTypedef *bmp280, uint8_t addr, uint8_t *data) {
    uint8_t ret = i2c_smbus_read_byte_data(bmp280->client, addr);
    if (ret < 0) {
        return false; 
    }
    *data = (uint8_t)ret;
    return true;
}

static bool bmp280_write_reg_value_8bit(BMP280_HandleTypedef *bmp280, uint8_t addr, uint8_t data) {
    uint8_t ret = i2c_smbus_write_byte_data(bmp280->client, addr, data);
    if (ret < 0) {
        return false;  
    }
    return true;  
}

static int bmp280_read_reg_value_16bit(BMP280_HandleTypedef *bmp280, uint8_t addr, uint16_t *data) {
    uint16_t ret = i2c_smbus_read_word_data(bmp280->client, addr);
    if(ret < 0) {
        pr_info("Can not send data to reg 16bit");
        return false;
    }
    *data = i2c_smbus_read_word_data(bmp280->client, addr);
    return true;  
}



static bool bmp280_init(BMP280_HandleTypedef *bmp280) {

	bmp280->config_reg = (bmp280->config.standby << 5) | (bmp280->config.filter << 2);
	if (bmp280_write_reg_value_8bit(bmp280, BMP280_CONFIG_REG, bmp280->config_reg) < 0) {
        pr_info("Can not send data to BMP280_CONFIG reg!");
		return false;
	}
    if (bmp280->config.mode == BMP280_MODE_FORCED) {
		bmp280->config.mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
	}
    bmp280->ctrl_meas_reg = (bmp280->config.oversampling_temperature << 5)
			| (bmp280->config.oversampling_pressure << 2) | (bmp280->config.mode);
    
    if (bmp280_write_reg_value_8bit(bmp280, BMP280_CTRL_MEAS_REG, bmp280->ctrl_meas_reg) < 0) {
        pr_info("Can not send data to BMP280_CONTROL reg!");
		return false;
	}

	return true;
}

void bmp280_init_default_params(BMP280_HandleTypedef *bmp280) {
	bmp280->config.mode = BMP280_MODE_NORMAL;
	bmp280->config.filter = BMP280_FILTER_OFF;
	bmp280->config.oversampling_pressure = BMP280_STANDARD;
	bmp280->config.oversampling_temperature = BMP280_STANDARD;
	bmp280->config.oversampling_humidity = BMP280_STANDARD;
	bmp280->config.standby = BMP280_STANDBY_250;
}


static void read_calibration_data(BMP280_HandleTypedef *bmp280) {

    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_T1_REG, &bmp280->raw_data.dig_T1);
    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_T2_REG, &bmp280->raw_data.dig_T2);
    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_T3_REG, &bmp280->raw_data.dig_T3);  

    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_P1_REG, &bmp280->raw_data.dig_P1);    
    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_P2_REG, &bmp280->raw_data.dig_P2);    
    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_P3_REG, &bmp280->raw_data.dig_P3);
    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_P4_REG, &bmp280->raw_data.dig_P4);    
    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_P5_REG, &bmp280->raw_data.dig_P5);
    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_P6_REG, &bmp280->raw_data.dig_P6);
    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_P7_REG, &bmp280->raw_data.dig_P7);    
    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_P8_REG, &bmp280->raw_data.dig_P8);    
    bmp280_read_reg_value_16bit(bmp280, BMP280_DIG_P9_REG, &bmp280->raw_data.dig_P9);
}

void read_raw(BMP280_HandleTypedef *bmp280, int reg) {
    uint16_t data;
    bmp280_read_reg_value_16bit(bmp280, reg, &data);
    int raw = SWAP_2BYTES(data) << 8;
    uint8_t data_1;
    bmp280_read_reg_value_8bit(bmp280, reg + 2, &data_1);
    raw |= data_1;
    raw >>= 4;

    if (reg == BMP280_PRESSUREDATA) {
        bmp280->raw_data.raw_pressure = raw;
    } else if (reg == BMP280_TEMPDATA) {
        bmp280->raw_data.raw_temperature = raw;
    }
}

/*
** File operation sturcture
*/
static struct file_operations fops =
{
        .owner          = THIS_MODULE,
        .read           = etx_read,
        .write          = etx_write,
        .open           = etx_open,
        .unlocked_ioctl = etx_ioctl,
        .release        = etx_release,
};

/*
** This function will be called when we open the Device file
*/
static int etx_open(struct inode *inode, struct file *file)
{

    BMP280_HandleTypedef *bmp280 = container_of(inode->i_cdev, BMP280_HandleTypedef, cdev);

    // Ensure bmp280 is valid
    if (!bmp280) {
        pr_err("BMP280 not initialized\n");
        return -EFAULT;
    }

    // Assign device's data structure to struct file so as to use for different methods
    file->private_data = bmp280;

    pr_info("Device file opened successfully\n");
    return 0;
}

/*
** This function will be called when we close the Device file
*/
static int etx_release(struct inode *inode, struct file *file)
{
        pr_info("Device File Closed...!!!\n");
        return 0;
}

/*
** This function will be called when we read the Device file
*/
static ssize_t etx_read(struct file *filp, char __user *buf, size_t len, loff_t *off)
{
        pr_info("Read Function\n");
        return 0;
}

/*
** This function will be called when we write the Device file
*/
static ssize_t etx_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
        pr_info("Write function\n");
        return len;
}

/*
** This function will be called when we write IOCTL on the Device file
*/
static long etx_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    // BMP280_HandleTypedef *bmp280 = file->private_data;

   BMP280_HandleTypedef *bmp280 = (BMP280_HandleTypedef*)file->private_data;
    struct i2c_client *client = bmp280->client;

    if (!bmp280) {
        pr_err("BMP280 not initialized\n");
        return -EFAULT;
    }

    switch (cmd) {

        case BMP280_GET_RAW_VALUE:{
            bmp280_raw_data_t *bmp280_rawdata =  NULL;
            bmp280_rawdata = &(bmp280->raw_data);
            read_calibration_data(bmp280);
            read_raw(bmp280, BMP280_PRESSUREDATA);
            read_raw(bmp280, BMP280_TEMPDATA);

            if(copy_to_user((int __user *) arg, bmp280_rawdata, sizeof(bmp280_raw_data_t))) {
                pr_err("Error copying raw data to user space\n");
                return -EFAULT;
            }
            break;
        }

        case BMP280_SET_MODE: {
            BMP280_Mode bmp280_mode;
            if (copy_from_user(&bmp280_mode, (BMP280_Mode *)arg, sizeof(BMP280_Mode))) {
                pr_err("Error copying raw data from user space\n");
                return -EFAULT;
            }
            uint8_t data_hex = (uint8_t)bmp280_mode;
            bmp280->ctrl_meas_reg &= ~((1 << 0) | (1 << 1));
            bmp280->ctrl_meas_reg |= data_hex;
            if (bmp280_write_reg_value_8bit(bmp280, BMP280_CTRL_MEAS_REG, bmp280->ctrl_meas_reg) < 0) {
                return -1;
            }
            break;
        }

        case BMP280_SET_FILTER: {
            BMP280_Filter bmp280_filter;
            if(copy_from_user(&bmp280_filter ,(BMP280_Filter *)arg, sizeof(BMP280_Filter))) {
                pr_err("Error copying raw data to user space\n");
                return -EFAULT;
            }
            uint8_t data_hex = (uint8_t)bmp280_filter;
            bmp280->config_reg &= ~((1 << 2) | (1 << 3) | (1 << 4));
            bmp280->config_reg |= (data_hex << 2);
            if (bmp280_write_reg_value_8bit(bmp280, BMP280_CONFIG_REG, bmp280->config_reg) < 0) {
                return -1;
            }
            break;
        }

        case BMP280_SET_STANDBY: {
            BMP280_StandbyTime bmp280_standby;
            if(copy_from_user(&bmp280_standby ,(BMP280_StandbyTime *)arg, sizeof(BMP280_StandbyTime))) {
                pr_err("Error copying raw data to user space\n");
                return -EFAULT;
            }
            uint8_t data_hex = (uint8_t)bmp280_standby;
            bmp280->config_reg &= ~((1 << 5) | (1 << 6) | (1 << 7));
            bmp280->config_reg |= (data_hex << 5);
            if (bmp280_write_reg_value_8bit(bmp280, BMP280_CONFIG_REG, bmp280->config_reg) < 0) {
                return -1;
            }
            break;
        }
        case BMP280_SET_OVERSAMPLING_PRESSURE: {
            BMP280_Oversampling bmp280_pressure;
            if(copy_from_user(&bmp280_pressure ,(BMP280_Oversampling *)arg, sizeof(BMP280_Oversampling))) {
                pr_err("Error copying raw data to user space\n");
                return -EFAULT;
            }
            uint8_t data_hex = (uint8_t)bmp280_pressure;
            bmp280->ctrl_meas_reg &= ~((1 << 2) | (1 << 3) | (1 << 4));
            bmp280->ctrl_meas_reg |= (data_hex << 2);
            if (bmp280_write_reg_value_8bit(bmp280, BMP280_CTRL_MEAS_REG, bmp280->ctrl_meas_reg) < 0) {
                return -1;
            }
            break;
        }

        case BMP280_SET_OVERSAMPLING_TEMPERATURE: {
            BMP280_Oversampling bmp280_temperature;
            if(copy_from_user(&bmp280_temperature ,(BMP280_Oversampling *)arg, sizeof(BMP280_Oversampling))) {
                pr_err("Error copying raw data to user space\n");
                return -EFAULT;
            }
            uint8_t data_hex = (uint8_t)bmp280_temperature;
            bmp280->ctrl_meas_reg &= ~((1 << 5) | (1 << 6) | (1 << 7));
            bmp280->ctrl_meas_reg |= (data_hex << 5);
            if (bmp280_write_reg_value_8bit(bmp280, BMP280_CTRL_MEAS_REG, bmp280->ctrl_meas_reg) < 0) {
                return -1;
            }
            break;
        }

        default:
            pr_err("Unknown IOCTL command\n");
            return -EINVAL;
    }
    return 0; // Success
}

 
/*
** Module Init function
*/
static int bmp280_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    struct device *dev_num_dr = &client->dev;
    BMP280_HandleTypedef *bmp280 = devm_kzalloc(dev_num_dr, sizeof(BMP280_HandleTypedef), GFP_KERNEL);

    if (!bmp280) {
        pr_err("Failed to allocate memory using kzalloc\n");
        return -ENOMEM;
    }
    i2c_set_clientdata(client, bmp280);
    bmp280->client = client;
    // bmp280 = kmalloc(sizeof(BMP280_HandleTypedef), GFP_KERNEL);
    if(bmp280->client->addr == 0x76) {
        pr_info("Addrr is true!");
    }
    /*Allocating Major number*/
    if((alloc_chrdev_region(&dev, 0, 1, "etx_Dev")) <0){
            pr_err("Cannot allocate major number\n");
            return -1;
    }
    pr_info("Major = %d Minor = %d \n",MAJOR(dev), MINOR(dev));

    /*Creating cdev structure*/
    cdev_init(&bmp280->cdev,&fops);

    /*Adding character device to the system*/
    if((cdev_add(&bmp280->cdev,dev,1)) < 0){
        pr_err("Cannot add the device to the system\n");
        goto r_class;
    }

    /*Creating struct class*/
    if(IS_ERR(bmp280->dev_class = class_create(THIS_MODULE,"bmp280_class"))){
        pr_err("Cannot create the struct class\n");
        goto r_class;
    }

    /*Creating device*/
    if(IS_ERR(device_create(bmp280->dev_class ,NULL,dev,NULL,BMP280_DEVICE_NAME))){
        pr_err("Cannot create the Device 1\n");
        goto r_device;
    }

	if(client->addr == 0x76) {
		pr_info("Address is true!");
	}
    bmp280_init_default_params(bmp280);
    bmp280_init(bmp280);
    read_calibration_data(bmp280);


    pr_info("Device Driver Insert...Done!!!\n");
    return 0;
    
r_device:
        class_destroy(bmp280->dev_class);
r_class:
        unregister_chrdev_region(dev,1);
        return -1;
}


static void bmp280_remove(struct i2c_client *client)
{
    BMP280_HandleTypedef *bmp280 = i2c_get_clientdata(client);
    device_destroy(bmp280->dev_class,dev);
    class_destroy(bmp280->dev_class);
    cdev_del(&bmp280->cdev);
    unregister_chrdev_region(dev, 1);
    pr_info("Device BMP280 Driver Remove...Done!!!\n");

}
static struct i2c_device_id bmp280_i2c_device[] = {
	{BMP280_DEVICE_NAME, 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, bmp280_i2c_device);

static const struct of_device_id bmp280_dt_match[] =
{
    { .compatible = "bosch,bmp280" },
    { }
};
MODULE_DEVICE_TABLE(of, bmp280_dt_match);


static struct i2c_driver bmp280_driver =
{
    .driver = 
    {
        .name = BMP280_DEVICE_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(bmp280_dt_match),
    },
    .probe = bmp280_probe,
    .remove = bmp280_remove,
	.id_table = bmp280_i2c_device,

}; 

module_i2c_driver(bmp280_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vo Thanh Minh Nhat");
MODULE_DESCRIPTION("Simple BMP280 device driver");
MODULE_VERSION("1.5");
