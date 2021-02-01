#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/version.h>

#include <linux/string.h>
#include "wilc_sbridge.h"
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/host.h>
#include <linux/pm_runtime.h>
#define SAMA5D4_ARDUINO_CONNECTOR 1

#define SDIO_MODALIAS "wilc_sdio"
/*WILC RESET/CHIP_EN/IRQN PIN CONNECTION*/
#ifdef SAMA5D4_ARDUINO_CONNECTOR
#define GPIO_NUM		58 
#define GPIO_CHIP_EN	94 
#define GPIO_RESET		60 
#else /*SAMA5D4_EXT1_CONNECTOR*/
#define GPIO_NUM		58
#define GPIO_CHIP_EN	59
#define GPIO_RESET		149
#endif

typedef struct  {
	uint32_t cmd;
	uint32_t addr;
	uint32_t val;

}cmd_hdr;

typedef struct {
	uint32_t cmd;
	uint32_t addr;
	uint32_t val;
	uint8_t b_buffer[4*1024];
}block_cmd_hdr;

#define CMD_READ_REG 		_IOR('q', 1, cmd_hdr *)
#define CMD_WRITE_REG 		_IOW('q', 2, cmd_hdr *)
#define CMD_READ_BLOCK_REG 	_IOR('q', 3, cmd_hdr *)
#define CMD_WRITE_BLOCK_REG 	_IOW('q', 4, cmd_hdr *)

#define SDIO_VENDOR_ID_WILC 0x0296
#define SDIO_DEVICE_ID_WILC 0x5347

static const struct sdio_device_id wilc_sdio_ids[] = {
	{ SDIO_DEVICE(SDIO_VENDOR_ID_WILC, SDIO_DEVICE_ID_WILC) },
	{ },
};

#define FIRST_MINOR 0
#define MINOR_CNT 1

#define WILC_SDIO_BLOCK_SIZE 512

struct wilc_sdio {
	u32 block_size;
	int nint;
#define MAX_NUN_INT_THRPT_ENH2 (5) /* Max num interrupts allowed in registers 0xf7, 0xf8 */
	int has_thrpt_enh3;
};

static struct wilc_sdio g_sdio;
static int sdio_write(struct wilc *wilc, u32 addr, u8 *buf, u32 size);
static int sdio_read(struct wilc *wilc, u32 addr, u8 *buf, u32 size);
static int sdio_write_reg(struct wilc *wilc, u32 addr, u32 data);
static int sdio_read_reg(struct wilc *wilc, u32 addr, u32 *data);
static int sdio_init(struct wilc *wilc, bool resume);

static dev_t dev;
static struct cdev c_dev;
static struct class *cl;
struct wilc *wilc;
struct device *dt_dev= NULL;


#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)) 
static void wilc_wlan_power(struct wilc *wilc, int power)
{
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_chip_en;

	pr_info("wifi_pm : %d\n", power);

	gpio_reset = gpiod_get(wilc->dev, "reset-gpios", GPIOD_ASIS);
	if (IS_ERR(gpio_reset)) {
		printk( "failed to get Reset GPIO, try default\r\n");
		gpio_reset = gpio_to_desc(GPIO_RESET);
		if (!gpio_reset) {
			printk("failed to get default Reset GPIO\r\n");
			return;
		}
	} else {
		printk( "succesfully got gpio_reset\r\n");
	}

	gpio_chip_en = gpiod_get(wilc->dev, "chip_en-gpios", GPIOD_ASIS);
	if (IS_ERR(gpio_chip_en)) {
		printk( "failed to get Chi_en GPIO, try default\r\n");
		gpio_chip_en = gpio_to_desc(GPIO_CHIP_EN);
		if (!gpio_chip_en) {
			printk("failed to get default chip_en GPIO\r\n");
			gpiod_put(gpio_reset);
			return;
		}
	} else {
		printk("succesfully got gpio_chip_en\r\n");
	}

	if (power) {
		gpiod_direction_output(gpio_chip_en, 1);
		mdelay(100);
		gpiod_direction_output(gpio_reset, 1);
	} else {
		gpiod_direction_output(gpio_reset, 0);
		gpiod_direction_output(gpio_chip_en, 0);
	}
	gpiod_put(gpio_chip_en);
	gpiod_put(gpio_reset);
}
#else
static void wilc_wlan_power(struct wilc *wilc, int power)
{
	int gpio_reset;
	int gpio_chip_en;
	struct device_node *of_node = wilc->dev->of_node;

	pr_info("wifi_pm : %d\n", power);

	gpio_reset = of_get_named_gpio_flags(of_node, "reset-gpios", 0, NULL);

	if (gpio_reset < 0) {
		gpio_reset = GPIO_RESET;
		pr_info("wifi_pm : load default reset GPIO %d\n", gpio_reset);
	}

	gpio_chip_en = of_get_named_gpio_flags(of_node, "chip_en-gpios", 0,
					       NULL);

	if (gpio_chip_en < 0) {
		gpio_chip_en = GPIO_CHIP_EN;
		pr_info("wifi_pm : load default chip_en GPIO %d\n",
			gpio_chip_en);
	}

	if (gpio_request(gpio_chip_en, "CHIP_EN") == 0 &&
	    gpio_request(gpio_reset, "RESET") == 0) {
		gpio_direction_output(gpio_chip_en, 0);
		gpio_direction_output(gpio_reset, 0);
		if (power) {
			gpio_set_value(gpio_chip_en, 1);
			mdelay(5);
			gpio_set_value(gpio_reset, 1);
		} else {
			gpio_set_value(gpio_reset, 0);
			gpio_set_value(gpio_chip_en, 0);
		}
		gpio_free(gpio_chip_en);
		gpio_free(gpio_reset);
	} else {
		dev_err(wilc->dev,
			"Error requesting GPIOs for CHIP_EN and RESET");
	}

}
#endif

void wilc_wlan_power_on_sequence(void)
{
	wilc_wlan_power(wilc, 0);
	wilc_wlan_power(wilc, 1);
}

void wilc_wlan_power_off_sequence(void)
{
	wilc_wlan_power(wilc, 0);
}

static int my_open(struct inode *i, struct file *f)
{
    return 0;
}
static int my_close(struct inode *i, struct file *f)
{
    return 0;
}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int my_ioctl(struct inode *i, struct file *f, unsigned int cmd, unsigned long arg)
#else
static long my_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
#endif
{
    cmd_hdr dat;
    block_cmd_hdr b_dat;
    unsigned int val32=0;

    switch (cmd)
    {
        case CMD_READ_REG:
	    if (copy_from_user(&dat, (cmd_hdr *)arg, sizeof(cmd_hdr)))
            {
                return -EACCES;
            }
	    sdio_read_reg(wilc,  dat.addr, &val32);
            dat.val = (int) val32;
            if (copy_to_user((cmd_hdr *)arg, &dat, sizeof(cmd_hdr)))
            {
                return -EACCES;
            }
            break;
	case CMD_WRITE_REG:
            if (copy_from_user(&dat, (cmd_hdr *)arg, sizeof(cmd_hdr)))
            {
                return -EACCES;
            }
	    sdio_write_reg(wilc, dat.addr, dat.val);
            break;
        case CMD_WRITE_BLOCK_REG:
	    if (copy_from_user(&b_dat, (block_cmd_hdr *)arg, sizeof(block_cmd_hdr)))
           {
                return -EACCES;
           }
           sdio_write(wilc, b_dat.addr, b_dat.b_buffer, (b_dat.cmd>>16) & 0xFFFF);
           break;
	case CMD_READ_BLOCK_REG:
	    if (copy_from_user(&b_dat, (block_cmd_hdr *)arg, sizeof(block_cmd_hdr)))
            {
                return -EACCES;
            }
            sdio_read(wilc, b_dat.addr, b_dat.b_buffer, (b_dat.cmd>>16) & 0xFFFF);
            if (copy_to_user((block_cmd_hdr *)arg, &b_dat, sizeof(block_cmd_hdr)))
            {
                return -EACCES;
            }
            break;
        default:
            return -EINVAL;
    }
    return 0;
}
 

static struct file_operations query_fops =
{
    .owner = THIS_MODULE,
    .open = my_open,
    .release = my_close,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    .ioctl = my_ioctl
#else
    .unlocked_ioctl = my_ioctl
#endif
};
 
static int query_ioctl_init(void)
{
    int ret;
    struct device *dev_ret;
 
     if ((ret = alloc_chrdev_region(&dev, FIRST_MINOR, MINOR_CNT, "query_ioctl")) < 0)
    {
        return ret;
    }
 
    cdev_init(&c_dev, &query_fops);
 
    if ((ret = cdev_add(&c_dev, dev, MINOR_CNT)) < 0)
    {
        return ret;
    }
     
    if (IS_ERR(cl = class_create(THIS_MODULE, "char")))
    {
        cdev_del(&c_dev);
        unregister_chrdev_region(dev, MINOR_CNT);
        return PTR_ERR(cl);
    }
    if (IS_ERR(dev_ret = device_create(cl, NULL, dev, NULL, "query")))
    {
        class_destroy(cl);
        cdev_del(&c_dev);
        unregister_chrdev_region(dev, MINOR_CNT);
        return PTR_ERR(dev_ret);
    }
 
    return 0;
}
 
static void query_ioctl_exit(void)
{
    device_destroy(cl, dev);
    class_destroy(cl);
    cdev_del(&c_dev);
    unregister_chrdev_region(dev, MINOR_CNT);
}

u32 wilc_get_chipid(struct wilc *wilc)
{
	u32 chipid = 0;
	int ret = 0;
	
	ret = sdio_read_reg(wilc, 0x3b0000, &chipid);
	if (!ret)
		printk("[wilc start]: fail read reg 0x3b0000\n");
	if (!is_wilc3000(chipid)) {
		sdio_read_reg(wilc, 0x1000,&chipid);
		if (!is_wilc1000(chipid)) {				
			printk("Its not WILC1000 Chipid: %x\n", chipid);
			chipid = 0;
		}
		if (chipid < 0x1003a0) {
			printk("WILC1002 isn't suported %x\n", chipid);
			chipid = 0;				
		}
	}
	return chipid;
}
 
int init_wilc_chip(struct wilc *wilc)
{
	u32 chipid;
	u32 reg, ret = 0;
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);

	chipid = wilc_get_chipid(wilc);
	printk("WILC ChipID: %x \n",chipid);

	ret = sdio_read_reg(wilc, 0x1118, &reg);
	if (!ret) {
		dev_err(&func->dev, "fail read reg 0x1118\n");
		return ret;
	}
	reg |= BIT(0);
	ret = sdio_write_reg(wilc, 0x1118, reg);
	if (!ret) {
		dev_err(&func->dev, "fail write reg 0x1118\n");
		return ret;
	}
	ret = sdio_write_reg(wilc, 0xc0000, 0x71);
	if (!ret) {
		dev_err(&func->dev, "fail write reg 0xc0000\n");
		return ret;
	}		
	//if (is_wilc3000(chipid)) {
		ret =sdio_read_reg(wilc, 0x207ac, &reg);

		ret = sdio_write_reg(wilc, 0x4f0000,0x71);
		if (!ret) {
			dev_err(&func->dev, "fail write reg 0x4f0000\n");
			return ret;
		}
	//}	
	return ret;
}



static int wilc_bus_probe(struct sdio_func *func,
			    const struct sdio_device_id *id)
{
	dt_dev = &func->card->dev;
	
	printk("This is wilc bus probe\n");
	wilc = kzalloc(sizeof(*wilc), GFP_KERNEL);
	if (!wilc)
		return -ENOMEM;
	printk("after power on sequence\n"); 

	mutex_init(&wilc->hif_cs);
	sdio_set_drvdata(func, wilc);
	wilc->dev = &func->dev;
/* To be enabled when using the custom WILC board */
/* Not for WILC1000 SD / WICL3000 SD/Shield Boards*/
#ifdef WILC_PWR_ON 
	wilc_wlan_power_on_sequence();
#endif	
	query_ioctl_init();
	//msleep(100);
	sdio_init(wilc,false);
	//msleep(100);
	init_wilc_chip(wilc);
	pm_runtime_get_sync(mmc_dev(func->card->host));
	printk("WILC SDIO probe success\n");
	return 0;
}

static void wilc_bus_remove(struct sdio_func *func)
{
	printk("wilc_bus_remove SDIO card removed \n");
	query_ioctl_exit();
}

static int wilc_sdio_cmd52(struct wilc *wilc, struct sdio_cmd52 *cmd)
{
	struct sdio_func *func = container_of(wilc->dev, struct sdio_func, dev);
	int ret;
	u8 data;

	sdio_claim_host(func);

	func->num = cmd->function;
	if (cmd->read_write) {  /* write */
		if (cmd->raw) {
			sdio_writeb(func, cmd->data, cmd->address, &ret);
			data = sdio_readb(func, cmd->address, &ret);
			cmd->data = data;
		} else {
			sdio_writeb(func, cmd->data, cmd->address, &ret);
		}
	} else {        /* read */
		data = sdio_readb(func, cmd->address, &ret);
		cmd->data = data;
	}

	sdio_release_host(func);

	if (ret)
		dev_err(&func->dev, "%s..failed, err(%d)\n", __func__, ret);
	return ret;
}

static int wilc_sdio_cmd53(struct wilc *wilc, struct sdio_cmd53 *cmd)
{
	struct sdio_func *func = container_of(wilc->dev, struct sdio_func, dev);
	int size, ret;

	sdio_claim_host(func);

	func->num = cmd->function;
	func->cur_blksize = cmd->block_size;
	if (cmd->block_mode)
		size = cmd->count * cmd->block_size;
	else
		size = cmd->count;

	if (cmd->read_write) {  /* write */
		ret = sdio_memcpy_toio(func, cmd->address,
				       (void *)cmd->buffer, size);
	} else {        /* read */
		ret = sdio_memcpy_fromio(func, (void *)cmd->buffer,
					 cmd->address,  size);
	}

	sdio_release_host(func);

	if (ret)
		dev_err(&func->dev, "%s..failed, err(%d)\n", __func__,  ret);

	return ret;
}
static int sdio_reset(struct wilc *wilc)
{
	struct sdio_cmd52 cmd;
	int ret;
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);

	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = 0x6;
	cmd.data = 0x8;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret)
		dev_err(&func->dev, "Fail cmd 52, reset cmd\n");
	return ret;
}

static int wilc_sdio_suspend(struct device *dev)
{
	return 0;
}

static int wilc_sdio_resume(struct device *dev)
{
	return 0;
}

static const struct of_device_id wilc_of_match[] = {
	{ .compatible = "microchip,wilc1000", },
	{ .compatible = "microchip,wilc3000", },
	{ /* sentinel */}
};
MODULE_DEVICE_TABLE(of, wilc_of_match);

static const struct dev_pm_ops wilc_sdio_pm_ops = {
	.suspend = wilc_sdio_suspend,
	.resume = wilc_sdio_resume,
};

static struct sdio_driver wilc_sdio_driver = {
	.name		= SDIO_MODALIAS,
	.id_table	= wilc_sdio_ids,
	.probe		= wilc_bus_probe,
	.remove		= wilc_bus_remove,
	.drv = {
		.pm = &wilc_sdio_pm_ops,
		.of_match_table = wilc_of_match,
	}
};
module_driver(wilc_sdio_driver,
	      sdio_register_driver,
	      sdio_unregister_driver);
MODULE_LICENSE("GPL");

/********************************************
 *
 *      Function 0
 *
 ********************************************/

static int sdio_set_func0_csa_address(struct wilc *wilc, u32 adr)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct sdio_cmd52 cmd;
	int ret;

	/**
	 *      Review: BIG ENDIAN
	 **/
	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = 0x10c;
	cmd.data = (u8)adr;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set 0x10c data...\n");
		goto fail;
	}

	cmd.address = 0x10d;
	cmd.data = (u8)(adr >> 8);
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set 0x10d data...\n");
		goto fail;
	}

	cmd.address = 0x10e;
	cmd.data = (u8)(adr >> 16);
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set 0x10e data...\n");
		goto fail;
	}

	return 1;
fail:
	return 0;
}

static int sdio_set_func0_block_size(struct wilc *wilc, u32 block_size)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct sdio_cmd52 cmd;
	int ret;

	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = 0x10;
	cmd.data = (u8)block_size;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set 0x10 data...\n");
		goto fail;
	}

	cmd.address = 0x11;
	cmd.data = (u8)(block_size >> 8);
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set 0x11 data...\n");
		goto fail;
	}

	return 1;
fail:
	return 0;
}

/********************************************
 *
 *      Function 1
 *
 ********************************************/

static int sdio_set_func1_block_size(struct wilc *wilc, u32 block_size)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct sdio_cmd52 cmd;
	int ret;

	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = 0x110;
	cmd.data = (u8)block_size;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set 0x110 data...\n");
		goto fail;
	}
	cmd.address = 0x111;
	cmd.data = (u8)(block_size >> 8);
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Failed cmd52, set 0x111 data...\n");
		goto fail;
	}

	return 1;
fail:
	return 0;
}

/********************************************
 *
 *      Sdio interfaces
 *
 ********************************************/
static int sdio_write_reg(struct wilc *wilc, u32 addr, u32 data)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	int ret;

	cpu_to_le32s(&data);

	if (addr >= 0xf0 && addr <= 0xff) {
		struct sdio_cmd52 cmd;

		cmd.read_write = 1;
		cmd.function = 0;
		cmd.raw = 0;
		cmd.address = addr;
		cmd.data = data;
		ret = wilc_sdio_cmd52(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd 52, write reg %08x ...\n", addr);
			goto fail;
		}
	} else {
		struct sdio_cmd53 cmd;

		/**
		 *      set the AHB address
		 **/
		if (!sdio_set_func0_csa_address(wilc, addr))
			goto fail;

		cmd.read_write = 1;
		cmd.function = 0;
		cmd.address = 0x10f;
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = 4;
		cmd.buffer = (u8 *)&data;
		cmd.block_size = g_sdio.block_size; /* johnny : prevent it from setting unexpected value */
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd53, write reg (%08x)...\n", addr);
			goto fail;
		}
	}

	return 1;

fail:

	return 0;
}

static int sdio_write(struct wilc *wilc, u32 addr, u8 *buf, u32 size)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	u32 block_size = g_sdio.block_size;
	struct sdio_cmd53 cmd;
	int nblk, nleft, ret;

	cmd.read_write = 1;
	if (addr > 0) {
		/**
		 *      has to be word aligned...
		 **/
		if (size & 0x3) {
			size += 4;
			size &= ~0x3;
		}

		/**
		 *      func 0 access
		 **/
		cmd.function = 0;
		cmd.address = 0x10f;
	} else {
		/**
		 *      has to be word aligned...
		 **/
		if (size & 0x3) {
			size += 4;
			size &= ~0x3;
		}

		/**
		 *      func 1 access
		 **/
		cmd.function = 1;
		cmd.address = 0;
	}

	nblk = size / block_size;
	nleft = size % block_size;

	if (nblk > 0) {
		cmd.block_mode = 1;
		cmd.increment = 1;
		cmd.count = nblk;
		cmd.buffer = buf;
		cmd.block_size = block_size;
		if (addr > 0) {
			if (!sdio_set_func0_csa_address(wilc, addr))
				goto fail;
		}
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd53 [%x], block send...\n", addr);
			goto fail;
		}
		if (addr > 0)
			addr += nblk * block_size;
		buf += nblk * block_size;
	}

	if (nleft > 0) {
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = nleft;
		cmd.buffer = buf;

		cmd.block_size = block_size;

		if (addr > 0) {
			if (!sdio_set_func0_csa_address(wilc, addr))
				goto fail;
		}
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd53 [%x], bytes send...\n", addr);
			goto fail;
		}
	}

	return 1;

fail:

	return 0;
}

static int sdio_read_reg(struct wilc *wilc, u32 addr, u32 *data)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	int ret;

	if (addr >= 0xf0 && addr <= 0xff) {
		struct sdio_cmd52 cmd;

		cmd.read_write = 0;
		cmd.function = 0;
		cmd.raw = 0;
		cmd.address = addr;
		ret = wilc_sdio_cmd52(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd 52, read reg (%08x) ...\n", addr);
			goto fail;
		}
		*data = cmd.data;
	} else {
		struct sdio_cmd53 cmd;

		if (!sdio_set_func0_csa_address(wilc, addr))
			goto fail;

		cmd.read_write = 0;
		cmd.function = 0;
		cmd.address = 0x10f;
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = 4;
		cmd.buffer = (u8 *)data;

		cmd.block_size = g_sdio.block_size; /* johnny : prevent it from setting unexpected value */
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd53, read reg (%08x)...\n", addr);
			goto fail;
		}
	}

	le32_to_cpus(data);

	return 1;

fail:

	return 0;
}

static int sdio_read(struct wilc *wilc, u32 addr, u8 *buf, u32 size)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	u32 block_size = g_sdio.block_size;
	struct sdio_cmd53 cmd;
	int nblk, nleft, ret;

	cmd.read_write = 0;
	if (addr > 0) {
		/**
		 *      has to be word aligned...
		 **/
		if (size & 0x3) {
			size += 4;
			size &= ~0x3;
		}

		/**
		 *      func 0 access
		 **/
		cmd.function = 0;
		cmd.address = 0x10f;
	} else {
		/**
		 *      has to be word aligned...
		 **/
		if (size & 0x3) {
			size += 4;
			size &= ~0x3;
		}

		/**
		 *      func 1 access
		 **/
		cmd.function = 1;
		cmd.address = 0;
	}

	nblk = size / block_size;
	nleft = size % block_size;

	if (nblk > 0) {
		cmd.block_mode = 1;
		cmd.increment = 1;
		cmd.count = nblk;
		cmd.buffer = buf;
		cmd.block_size = block_size;
		if (addr > 0) {
			if (!sdio_set_func0_csa_address(wilc, addr))
				goto fail;
		}
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd53 [%x], block read...\n", addr);
			goto fail;
		}
		if (addr > 0)
			addr += nblk * block_size;
		buf += nblk * block_size;
	}       /* if (nblk > 0) */

	if (nleft > 0) {
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = nleft;
		cmd.buffer = buf;

		cmd.block_size = block_size; /* johnny : prevent it from setting unexpected value */

		if (addr > 0) {
			if (!sdio_set_func0_csa_address(wilc, addr))
				goto fail;
		}
		ret = wilc_sdio_cmd53(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Failed cmd53 [%x], bytes read...\n", addr);
			goto fail;
		}
	}

	return 1;

fail:

	return 0;
}

/********************************************
 *
 *      Bus interfaces
 *
 ********************************************/

static int sdio_init(struct wilc *wilc, bool resume)
{
	struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct sdio_cmd52 cmd;
	int loop, ret;
	u32 chipid;
	static bool init_done;

	/** Patch for sdio interrupt latency issue*/
	pm_runtime_get_sync(mmc_dev(func->card->host));
	
	if (init_done)
		return 1;

	if (!resume) {
		memset(&g_sdio, 0, sizeof(struct wilc_sdio));		
	}

	/**
	 *      function 0 csa enable
	 **/
	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 1;
	cmd.address = 0x100;
	cmd.data = 0x80;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Fail cmd 52, enable csa...\n");
		goto fail;
	}

	/**
	 *      function 0 block size
	 **/
	if (!sdio_set_func0_block_size(wilc, WILC_SDIO_BLOCK_SIZE)) {
		dev_err(&func->dev, "Fail cmd 52, set func 0 block size...\n");
		goto fail;
	}
	g_sdio.block_size = WILC_SDIO_BLOCK_SIZE;

	/**
	 *      enable func1 IO
	 **/
	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 1;
	cmd.address = 0x2;
	cmd.data = 0x2;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev,
			"Fail cmd 52, set IOE register...\n");
		goto fail;
	}

	/**
	 *      make sure func 1 is up
	 **/
	cmd.read_write = 0;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = 0x3;
	loop = 3;
	do {
		cmd.data = 0;
		ret = wilc_sdio_cmd52(wilc, &cmd);
		if (ret) {
			dev_err(&func->dev,
				"Fail cmd 52, get IOR register...\n");
			goto fail;
		}
		if (cmd.data == 0x2)
			break;
	} while (loop--);

	if (loop <= 0) {
		dev_err(&func->dev, "Fail func 1 is not ready...\n");
		goto fail;
	}

	/**
	 *      func 1 is ready, set func 1 block size
	 **/
	if (!sdio_set_func1_block_size(wilc, WILC_SDIO_BLOCK_SIZE)) {
		dev_err(&func->dev, "Fail set func 1 block size...\n");
		goto fail;
	}

	/**
	 *      func 1 interrupt enable
	 **/
	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 1;
	cmd.address = 0x4;
	cmd.data = 0x3;
	ret = wilc_sdio_cmd52(wilc, &cmd);
	if (ret) {
		dev_err(&func->dev, "Fail cmd 52, set IEN register...\n");
		goto fail;
	}

	/**
	 *      make sure can read back chip id correctly
	 **/
	if (!resume) {
		chipid = wilc_get_chipid(wilc);
			if (is_wilc3000(chipid)) {
				goto _pass_;
			}
			else if (is_wilc1000(chipid)) {
				goto _pass_;
			}
		else {
			dev_err(&func->dev, "Fail cmd read chip id...\n");
			goto fail;
		}
	}
_pass_:
	init_done = 1;
	return 1;

fail:

	return 0;
}
