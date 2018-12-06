#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>

#define REG_VER		0x01
#define REG_CFG		0x02
#define REG_INT		0x03
#define REG_KEY		0x04
#define REG_BKL		0x05
#define REG_DEB		0x06
#define REG_FRQ		0x07
#define REG_RST		0x08
#define REG_FIF		0x09

#define WRITE_MASK	BIT(7)

#define CFG_PANIC_INT		BIT(5)
#define CFG_KEY_INT			BIT(4)
#define CFG_NUMLOCK_INT		BIT(3)
#define CFG_CAPSLOCK_INT	BIT(2)
#define CFG_OVERFLOW_INT	BIT(1)
#define CFG_OVERFLOW_ON		BIT(0)

#define INT_PANIC		BIT(4)
#define INT_KEY			BIT(3)
#define INT_NUMLOCK		BIT(2)
#define INT_CAPSLOCK	BIT(1)
#define INT_OVERFLOW	BIT(0)

//#define KEY_NUMLOCK		BIT(6)
//#define KEY_CAPSLOCK	BIT(5)

#define KEY_COUNT_MASK	0x1F

#define STATE_RELEASE	3
#define STATE_HOLD		2
#define STATE_PRESS		1
#define STATE_IDLE		0

struct bbq10pmod_data {
	struct i2c_client *client;
	struct input_dev *input;
};

static int bbq10pmod_write_reg(struct bbq10pmod_data *drv_data, u8 reg)
{
	struct i2c_client *client;
	int error;
	
	client = drv_data->client;
	
	struct i2c_msg msgs[] = {
		{ .addr = client->addr, .flags = I2C_M_STOP, .len = sizeof(u8), .buf = &reg, },
	};
	
	error = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (error != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev,
		"%s failed, reg: %d, error: %d\n",
		__func__, reg, error);
		
		return error;
	}
	
	return 0;
}

static int bbq10pmod_write_data(struct bbq10pmod_data *drv_data, u8 reg, u8 *buf, u8 len)
{
	struct i2c_client *client;
	int error;
	
	client = drv_data->client;
	
	struct i2c_msg msgs[] = {
		{ .addr = client->addr | 0xC0, .flags = 0, .len = sizeof(u8), .buf = &reg, },
		{ .addr = client->addr | 0xC0, .flags = I2C_M_STOP, .len = len, .buf = buf, },
	};
	
	error = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (error != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev,
		"%s failed, reg: %d, error: %d\n",
		__func__, reg, error);
		
		return error;
	}
	
	return 0;
}

static int bbq10pmod_read_reg(struct bbq10pmod_data *drv_data, u8 reg, u8 *buf, u8 len)
{
	struct i2c_client *client;
	int error;
	
	client = drv_data->client;
	
	
	struct i2c_msg msgs[] = {
		{ .addr = client->addr, .flags = 0, .len = sizeof(u8), .buf = &reg, },
		{ .addr = client->addr, .flags = I2C_M_RD | I2C_M_STOP, .len = len, .buf = buf }
	};
	
	error = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (error != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev,
		"%s failed, reg: %d, error: %d\n",
		__func__, reg, error);
		
		return error;
	}
	
	return 0;
}

static irqreturn_t bbq10pmod_irq_handler(int irq, void *dev_id)
{
	struct bbq10pmod_data *drv_data = dev_id;
	u8 data[2];
	int error;
	
	printk("%s fired\n", __func__);
	
	/*error = bbq10pmod_read_reg(drv_data, REG_KEY, data, sizeof(u8));
	if (error < 0)
		dev_err(&drv_data->client->dev, "Failed to read KEY\n");*/
	
	return IRQ_HANDLED;
}

static int bbq10pmod_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct bbq10pmod_data *drv_data;
	struct input_dev *input;
	int error;
	u8 reg;
	
	drv_data = devm_kzalloc(dev, sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;
	
	drv_data->client = client;
	
	//error = bbq10pmod_read_reg(drv_data, REG_VER, &reg, sizeof(reg));
	error = bbq10pmod_write_reg(drv_data, REG_RST);
	if (error)
		return -ENODEV;
		
	//msleep(100);
	
	//reg = 0;
	//error = bbq10pmod_write_data(drv_data, REG_BKL, &reg, 1);
	//if (error)
	//	return -ENODEV;
	
	//error = bbq10pmod_read_reg(drv_data, REG_VER, &reg, sizeof(reg));
	//if (error)
	//	return -ENODEV;
		
	//printk("%s: version: 0x%02X\n", __func__, reg);
	
	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
		
	drv_data->input = input;
	
	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->id.vendor  = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0001;
	
	if (device_property_read_bool(dev, "keypad,autorepeat"))
		__set_bit(EV_REP, input->evbit);
	
	input_set_capability(input, EV_MSC, MSC_SCAN);
	
	error = devm_request_threaded_irq(dev, client->irq,
										NULL, bbq10pmod_irq_handler,
										IRQF_SHARED | IRQF_ONESHOT,
										client->name, drv_data);
	if (error) {
		dev_err(dev, "Unable to claim irq %d; error %d\n", client->irq, error);
		return error;
	}
	
	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n", error);
		return error;
	}
	
	return 0;
}

static const struct i2c_device_id bbq10pmod_id[] = {
	{ "bbq10pmod", 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bbq10pmod_id);

static const struct of_device_id bbq10pmod_dt_ids[] = {
	{ .compatible = "arturo182,bbq10pmod", },
	{ }
};
MODULE_DEVICE_TABLE(of, bbq10pmod_dt_ids);

static struct i2c_driver bbq10pmod_driver = {
	.driver = {
		.name = "bbq10pmod",
		.of_match_table = bbq10pmod_dt_ids,
	},
	.probe		= bbq10pmod_probe,
	.id_table	= bbq10pmod_id,
};

static int __init bbq10pmod_init(void)
{
	return i2c_add_driver(&bbq10pmod_driver);
}
subsys_initcall(bbq10pmod_init);

static void __exit bbq10pmod_exit(void)
{
	i2c_del_driver(&bbq10pmod_driver);
}
module_exit(bbq10pmod_exit);

MODULE_AUTHOR("arturo182 <arturo182@tlen.pl>");
MODULE_DESCRIPTION("Keyboard driver for BBQ10 PMOD");
MODULE_LICENSE("GPL");
