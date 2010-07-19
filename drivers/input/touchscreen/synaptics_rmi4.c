/*
 * Synaptics RMI4 touchscreen driver
 *
 * Copyright (C) 2010
 * Author: Joerie de Gram <j.de.gram@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input.h>

#define GPIO_IRQ	142

#define ABS_X_MAX	480
#define ABS_Y_MAX	800

static struct workqueue_struct *synaptics_wq;

struct synaptics_rmi4_data {
	struct input_dev *input;
	struct i2c_client *client;
	struct hrtimer timer;
	struct work_struct work;
	unsigned int irq;
	unsigned char f01_control_base;
	unsigned char f01_data_base;
	unsigned char f11_data_base;
};

static enum hrtimer_restart synaptics_rmi4_timer_func(struct hrtimer *timer)
{
	struct synaptics_rmi4_data *ts = container_of(timer, struct synaptics_rmi4_data, timer);

	queue_work(synaptics_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static void synaptics_rmi4_work(struct work_struct *work)
{
	struct synaptics_rmi4_data *ts = container_of(work, struct synaptics_rmi4_data, work);
	s32 finger_status, finger_contact, x_msb, y_msb, xy_lsb;

	/* Finger status */
	finger_status = i2c_smbus_read_byte_data(ts->client, ts->f11_data_base + 0); /* F11_2D_Data0 */

	/* X & Y */
	x_msb = i2c_smbus_read_byte_data(ts->client, ts->f11_data_base + 1); /* F11_2D_Data1 */
	y_msb = i2c_smbus_read_byte_data(ts->client, ts->f11_data_base + 2); /* F11_2D_Data2 */
	xy_lsb = i2c_smbus_read_byte_data(ts->client, ts->f11_data_base + 3); /* F11_2D_Data3 */

	/* Finger contact */
	finger_contact = i2c_smbus_read_byte_data(ts->client, ts->f11_data_base + 5); /* F11_2D_Data5 */

	input_report_key(ts->input, BTN_TOUCH, (finger_status == 0x01 || finger_status == 0x02) ? 1 : 0);
	input_report_abs(ts->input, ABS_X, (x_msb << 4) | (xy_lsb & 0x0f));
	input_report_abs(ts->input, ABS_Y, (y_msb << 4) | ((xy_lsb & 0xf0) >> 4));
	input_report_abs(ts->input, ABS_PRESSURE, finger_contact);

	input_sync(ts->input);

	if(ts->irq) {
		/* Clear interrupt status register */
		i2c_smbus_read_word_data(ts->client, ts->f01_data_base + 1);
		enable_irq(ts->irq);
	}
}

static void synaptics_rmi4_irq_setup(struct synaptics_rmi4_data *ts)
{
	i2c_smbus_write_byte_data(ts->client, ts->f01_control_base + 1, 0x04);
}

static irqreturn_t synaptics_rmi4_irq_handler(int irq, void *dev_id)
{
	struct synaptics_rmi4_data *ts = dev_id;

	disable_irq_nosync(ts->irq);
	queue_work(synaptics_wq, &ts->work);

	return IRQ_HANDLED;
}

static int __devinit synaptics_rmi4_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_rmi4_data *ts;
	struct input_dev *input_dev;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		return -EIO;
	}

	ts = kzalloc(sizeof(struct synaptics_rmi4_data), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!ts || !input_dev) {
		pr_err("synaptics-rmi4: failed to allocate memory\n");
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->irq = GPIO_IRQ;
	ts->f01_control_base = 0x23;
	ts->f01_data_base = 0x13;
	ts->f11_data_base = 0x15; /* FIXME */

	ts->client = client;
	ts->input = input_dev;

	INIT_WORK(&ts->work, synaptics_rmi4_work);

	input_dev->name = "Synaptics RMI4 touchscreen";
	input_dev->phys = "synaptics-rmi4/input0";
	input_dev->id.bustype = BUS_I2C;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, ABS_X_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ABS_Y_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	err = input_register_device(input_dev);
	if (err) {
		pr_err("synaptics-rmi4: failed to register input device\n");
		goto err_free_mem;
	}

	i2c_set_clientdata(client, ts);


	if(ts->irq) {
		err = request_irq(gpio_to_irq(ts->irq), synaptics_rmi4_irq_handler, IRQF_TRIGGER_FALLING, "SYNAPTICS_ATTN", ts);
	}

	if(!err && ts->irq) {
		synaptics_rmi4_irq_setup(ts);
	} else {
		pr_info("synaptics-rmi4: GPIO IRQ missing, falling back to polled mode\n");
		ts->irq = 0;

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_rmi4_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	return 0;

 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit synaptics_rmi4_remove(struct i2c_client *client)
{
	struct synaptics_rmi4_data *ts = i2c_get_clientdata(client);
	input_unregister_device(ts->input);

	if(ts->irq) {
		free_irq(ts->irq, ts);
	} else {
		hrtimer_cancel(&ts->timer);
	}

	kfree(ts);

	return 0;
}

static struct i2c_device_id synaptics_rmi4_idtable[] = {
	{ "synaptics-rmi4", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_idtable);

static struct i2c_driver synaptics_rmi4_driver = {
	.probe		= synaptics_rmi4_probe,
	.remove		= __devexit_p(synaptics_rmi4_remove),
	.id_table	= synaptics_rmi4_idtable,
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "synaptics-rmi4"
	},
};

static int __init synaptics_rmi4_init(void)
{
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");

	if (!synaptics_wq) {
		return -ENOMEM;
	}

	return i2c_add_driver(&synaptics_rmi4_driver);
}

static void __exit synaptics_rmi4_exit(void)
{
	i2c_del_driver(&synaptics_rmi4_driver);
}

module_init(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

MODULE_AUTHOR("Joerie de Gram <j.de.gram@gmail.com");
MODULE_DESCRIPTION("Synaptics RMI4 touchscreen driver");
MODULE_LICENSE("GPL");

