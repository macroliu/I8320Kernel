/*
 * Linux kernel driver for ST LIS302DL 3-axis accelerometer
 *
 * Copyright (C) 2009 0xlab
 * Author: Matt Hsu <matt@0xlab.org>
 *
 * Copyright (C) 2007-2008 by Openmoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 *         Andy Green <andy@openmoko.com>
 *         Simon Kagstrom <simon.kagstrom@gmail.com>
 *
 * This driver is based on Openmoko accelerometer driver
 * but with I2C interface.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>

#include <linux/lis302dl.h>

#define LIS302DL_DRIVER_NAME 		"lis302dl"
#define mg_weight 			18

struct lis302dl{
	struct device *dev;
	struct input_dev *input_dev;
	struct lis302dl_platform_data *pdata;
	struct i2c_client *client;
	struct mutex lock;
	struct work_struct work;
	int irq;
};

static int __lis302dl_reg_write(struct lis302dl *lis, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(lis->client, reg, value);
}

static int reg_write(struct lis302dl *lis, u_int8_t reg, u_int8_t val)
{
	int ret;

	mutex_lock(&lis->lock);
	ret = __lis302dl_reg_write(lis, reg, val);
	mutex_unlock(&lis->lock);

	return ret;
}

static int __lis302dl_reg_read(struct lis302dl *lis, u8 reg)
{
	return i2c_smbus_read_byte_data(lis->client, reg);
}

static u_int8_t reg_read(struct lis302dl *lis, u_int8_t reg)
{
	int32_t ret;

	mutex_lock(&lis->lock);
	ret = __lis302dl_reg_read(lis, reg);
	mutex_unlock(&lis->lock);

	return ret & 0xff;
}

static int reg_set_bit_mask(struct lis302dl *lis,
			    u_int8_t reg, u_int8_t mask, u_int8_t val)
{
	int ret;
	u_int8_t tmp;

	val &= mask;

	mutex_lock(&lis->lock);

	tmp = __lis302dl_reg_read(lis, reg);
	tmp &= ~mask;
	tmp |= val;
	ret = __lis302dl_reg_write(lis, reg, tmp);

	mutex_unlock(&lis->lock);

	return ret;
}

/* interrupt handling related */

enum lis302dl_intmode {
	LIS302DL_INTMODE_GND		= 0x00,
	LIS302DL_INTMODE_FF_WU_1	= 0x01,
	LIS302DL_INTMODE_FF_WU_2	= 0x02,
	LIS302DL_INTMODE_FF_WU_12	= 0x03,
	LIS302DL_INTMODE_DATA_READY	= 0x04,
	LIS302DL_INTMODE_CLICK		= 0x07,
};

enum lis302dl_axis {
	X_OUTPUT  				= 0,
	Y_OUTPUT 				= 1,
	Z_OUTPUT 				= 2,
	AXIS_OUTPUT_SIZE 		= 3,
};

static void lis_work(struct work_struct *work)
{
	struct lis302dl *lis =
				container_of(work, struct lis302dl, work);

	s8 out[AXIS_OUTPUT_SIZE];
	u_int8_t status = reg_read(lis, LIS302DL_REG_STATUS);

	/* deal with input and report to user space */
	if (LIS302DL_STATUS_XYZDA & status)
	{
		out[X_OUTPUT] = reg_read(lis, LIS302DL_REG_OUT_X);
		input_report_abs(lis->input_dev, ABS_X, out[X_OUTPUT] * mg_weight);

		out[Y_OUTPUT] = reg_read(lis, LIS302DL_REG_OUT_Y);
		input_report_abs(lis->input_dev, ABS_Y, out[Y_OUTPUT] * mg_weight);

		out[Z_OUTPUT] = reg_read(lis, LIS302DL_REG_OUT_Z);
		input_report_abs(lis->input_dev, ABS_Z, out[Z_OUTPUT] * mg_weight);

		dev_dbg(lis->dev, "status %2x:X %2x:Y %2x:Z %2x\n",
						status, out[X_OUTPUT], out[Y_OUTPUT], out[Z_OUTPUT]);
		input_sync(lis->input_dev);
	}
	enable_irq(lis->irq);
}

static irqreturn_t lis302dl_irq(int irq, void *data)
{
	struct lis302dl *lis = data;

	dev_dbg(lis->dev, "entering\n");
	disable_irq(lis->irq);
	schedule_work(&lis->work);

	return IRQ_HANDLED;
}

static void __enable_data_collection(struct lis302dl *lis)
{
	reg_set_bit_mask(lis, LIS302DL_REG_CTRL3,
			LIS302DL_INTMODE_CLICK, LIS302DL_INTMODE_DATA_READY);

	/* threshold */
}

static int lis302dl_input_open(struct input_dev *inp)
{
	struct lis302dl *lis = input_get_drvdata(inp);

	__enable_data_collection(lis);
	return 0;
}

static void lis302dl_input_close(struct input_dev *inp)
{
	struct lis302dl *lis = input_get_drvdata(inp);

	reg_set_bit_mask(lis, LIS302DL_REG_CTRL3,
			LIS302DL_INTMODE_CLICK, LIS302DL_INTMODE_GND);
}

#ifdef CONFIG_PM
static int lis302dl_suspend(struct device *dev, pm_message_t state)
{
	return 0;
}

static int lis302dl_resume(struct device *dev)
{
	return 0;
}
#else
#define lis302dl_suspend NULL
#define lis302dl_resume NULL
#endif

static int __devinit lis302dl_probe(struct i2c_client *client,
				const struct i2c_device_id *ids)
{
	struct lis302dl *lis;
	struct lis302dl_platform_data *pdata = client->dev.platform_data;
	int ret = 0;

	lis = kzalloc(sizeof(struct lis302dl), GFP_KERNEL);

	if (!lis)
		return -ENOMEM;

	lis->client = client;
	lis->irq = client->irq;
	lis->dev = &client->dev;
	i2c_set_clientdata(client, lis);

	lis->pdata = pdata;
	mutex_init(&lis->lock);

	dev_info(lis->dev, "Who_am_I magic:%02x \n", reg_read(lis, LIS302DL_REG_WHO_AM_I));
	/* create sysfs group */

	/* allocate input layer */
	lis->input_dev = input_allocate_device();
	if (!lis->input_dev) {
		dev_err(lis->dev, "Unable to allocate input device\n");
		goto exit_fail;
	}

	input_set_drvdata(lis->input_dev, lis);

	lis->input_dev->name = "LIS302DL Motion Sensor";
	lis->input_dev->id.bustype = BUS_I2C;
	lis->input_dev->open = lis302dl_input_open;
	lis->input_dev->close = lis302dl_input_close;

	ret = input_register_device(lis->input_dev);
	if (ret) {
		dev_err(lis->dev, "registering input device: %d\n", ret);
		goto exit_input_dev;
	}

	set_bit(EV_ABS, lis->input_dev->evbit);
	input_set_abs_params(lis->input_dev, ABS_X, 0, 0, 0, 0);
	input_set_abs_params(lis->input_dev, ABS_Y, 0, 0, 0, 0);
	input_set_abs_params(lis->input_dev, ABS_Z, 0, 0, 0, 0);

	INIT_WORK(&lis->work, lis_work);

	/* power up device */
	reg_write(lis, LIS302DL_REG_CTRL1, LIS302DL_CTRL1_PD
							| LIS302DL_CTRL1_Xen
							| LIS302DL_CTRL1_Yen
							| LIS302DL_CTRL1_Zen);

	/* setup INT source */
	reg_write(lis, LIS302DL_REG_CTRL3, LIS302DL_CTRL3_PP_OD | LIS302DL_CTRL3_IHL);

	/* reset devie */
	reg_write(lis, LIS302DL_REG_CTRL2, LIS302DL_CTRL2_BOOT);

	reg_read(lis, LIS302DL_REG_STATUS);
	reg_read(lis, LIS302DL_REG_FF_WU_SRC_1);
	reg_read(lis, LIS302DL_REG_FF_WU_SRC_2);
	reg_read(lis, LIS302DL_REG_CLICK_SRC);

	/* allocate IRQ resource */
	if (lis->irq) {
		ret = request_irq(lis->irq, lis302dl_irq,
					IRQF_TRIGGER_LOW, LIS302DL_DRIVER_NAME, lis);
		if (ret) {
			dev_err(lis->dev, "request IRQ failed\n");
			goto exit_input_reg;
		}
	} else {
		dev_err(lis->dev, "No IRQ allocated \n");
	}

	return ret;

exit_input_reg:
	input_unregister_device(lis->input_dev);
exit_input_dev:
	input_free_device(lis->input_dev);
exit_fail:
	kfree(lis);
	return ret;
}

static int __devexit lis302dl_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_device_id lis302dl_id_table[] = {
	/* the slave address is passed by i2c_boardinfo */
	{LIS302DL_DRIVER_NAME, },
	{/* end of list */}
};

static struct i2c_driver lis302dl_driver = {
	.driver = {
		.name	= LIS302DL_DRIVER_NAME,
		.suspend = lis302dl_suspend,
		.resume	= lis302dl_resume,
	},
	.id_table = lis302dl_id_table,
	.probe = lis302dl_probe,
	.remove = __devexit_p(lis302dl_remove),
};

static int __init lis302dl_init(void)
{
	return i2c_add_driver(&lis302dl_driver);
}

static void __exit lis302dl_exit(void)
{
	i2c_del_driver(&lis302dl_driver);
}

MODULE_DESCRIPTION("I2C chip driver for STM LIS302DL");
MODULE_AUTHOR("Matt Hsu <matt@0xlab.org>");
MODULE_LICENSE("GPL");

module_init(lis302dl_init);
module_exit(lis302dl_exit);
