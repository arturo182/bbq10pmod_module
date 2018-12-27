#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>

//#define DEBUG

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

#define CFG_USE_MODS		BIT(7)
#define CFG_REPORT_MODS		BIT(6)
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

#define _KEY_NUMLOCK	BIT(6)
#define _KEY_CAPSLOCK	BIT(5)
#define KEY_COUNT_MASK	0x1F

#define STATE_RELEASE	3
#define STATE_HOLD		2
#define STATE_PRESS		1
#define STATE_IDLE		0

#define NUM_KEYCODES	256

static unsigned short keycodes[NUM_KEYCODES] = {
	[0x01] = KEY_UP,
	[0x02] = KEY_DOWN,
	[0x03] = KEY_LEFT,
	[0x04] = KEY_RIGHT,
	[0x05] = KEY_ENTER,
	[0x06] = KEY_MENU,
	[0x07] = KEY_BACK,

	[17] = KEY_LEFTALT,
	[18] = KEY_LEFTSHIFT,
	[19] = KEY_LEFTCTRL,

	['A'] = KEY_A,
	['B'] = KEY_B,
	['C'] = KEY_C,
	['D'] = KEY_D,
	['E'] = KEY_E,
	['F'] = KEY_F,
	['G'] = KEY_G,
	['H'] = KEY_H,
	['I'] = KEY_I,
	['J'] = KEY_J,
	['K'] = KEY_K,
	['L'] = KEY_L,
	['M'] = KEY_M,
	['N'] = KEY_N,
	['O'] = KEY_O,
	['P'] = KEY_P,
	['Q'] = KEY_Q,
	['R'] = KEY_R,
	['S'] = KEY_S,
	['T'] = KEY_T,
	['U'] = KEY_U,
	['W'] = KEY_W,
	['V'] = KEY_V,
	['X'] = KEY_X,
	['Y'] = KEY_Y,
	['Z'] = KEY_Z,

	[' '] = KEY_SPACE,
	['~'] = KEY_0,
	['$'] = KEY_GRAVE,

	['\b'] = KEY_BACKSPACE,
	['\n'] = KEY_ENTER,
};

struct bbq10pmod_data {
	unsigned short keycode[NUM_KEYCODES];
	struct i2c_client *client;
	struct input_dev *input;
};

static int bbq10pmod_write_reg(struct bbq10pmod_data *drv_data, u8 reg)
{
	struct i2c_client *client = drv_data->client;
	int error;

	struct i2c_msg msgs[] = {
		{ .addr = client->addr, .flags = I2C_M_STOP, .len = sizeof(u8), .buf = &reg, },
	};

	error = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (error != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s failed, reg: %d, error: %d\n",
		__func__, reg, error);

		return error;
	}

	return 0;
}

static int bbq10pmod_write_data(struct bbq10pmod_data *drv_data, u8 reg, const u8 *buf, u8 len)
{
	struct i2c_client *client = drv_data->client;
	int error;

	reg |= 0x80;

	struct i2c_msg msgs[] = {
		{ .addr = client->addr, .flags = 0, .len = sizeof(u8), .buf = &reg, },
		{ .addr = client->addr, .flags = 0, .len = len, .buf = (u8*)buf, },
	};

	error = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (error != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s failed, reg: %d, error: %d\n",
			__func__, reg, error);

		return error;
	}

	return 0;
}

static int bbq10pmod_read_reg(struct bbq10pmod_data *drv_data, u8 reg, u8 *buf, u8 len)
{
	struct i2c_client *client = drv_data->client;
	int error;

	struct i2c_msg msgs[] = {
		{ .addr = client->addr, .flags = client->flags, .len = sizeof(u8), .buf = &reg, },
		{ .addr = client->addr, .flags = client->flags | I2C_M_RD, .len = len, .buf = buf }
	};

	error = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (error != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s failed, reg: %d, error: %d\n",
			__func__, reg, error);

		return error;
	}

	return 0;
}

static void bbq10pmod_update_locks(struct bbq10pmod_data *drv_data)
{
	struct input_dev *input = drv_data->input;
	struct i2c_client *client = drv_data->client;
	u8 data;
	int error;

	error = bbq10pmod_read_reg(drv_data, REG_KEY, &data, sizeof(u8));
	if (error < 0) {
		dev_err(&client->dev, "%s failed to read REG_KEY, error: %d\n",
			__func__, error);
		return;
	}

	pr_warn("%s status: 0x%02X\n", __func__, data);

	//input_report_key(input, KEY_NUMLOCK, data & _KEY_NUMLOCK);
	//input_report_key(input, KEY_CAPSLOCK, data & _KEY_CAPSLOCK);

	input_sync(input);
}

static void bbq10pmod_read_fifo(struct bbq10pmod_data *drv_data)
{
	struct input_dev *input = drv_data->input;
	struct i2c_client *client = drv_data->client;
	unsigned int keycode;
	u8 data[2];
	u8 count;
	int error;

	error = bbq10pmod_read_reg(drv_data, REG_KEY, data, sizeof(u8));
	if (error < 0) {
		dev_err(&client->dev, "%s failed to read REG_KEY, error: %d\n",
			__func__, error);
		return;
	}

	pr_debug("%s status: 0x%02X\n", __func__, data[0]);

	count = (data[0] & KEY_COUNT_MASK);

	while (count > 0) {
		error = bbq10pmod_read_reg(drv_data, REG_FIF, data, sizeof(u8) * 2);
		if (error < 0) {
			dev_err(&client->dev, "%s failed to read REG_FIF, error: %d\n",
				__func__, error);
			return;
		}

		pr_debug("%s key %d/%c, state: %d\n",
			__func__, data[1], data[1], data[0]);

		count -= 1;

		if (data[0] == STATE_PRESS || data[0] == STATE_RELEASE) {
			input_event(input, EV_MSC, MSC_SCAN, data[1]);

			keycode = drv_data->keycode[data[1]];
			if (keycode == KEY_UNKNOWN) {
				pr_warn("%s code 0x%02X UNKNOWN\n", __func__, data[1]);
			} else {
				input_report_key(input, keycode, data[0] == STATE_PRESS);
			}
		}
	}

	input_sync(input);
}

static irqreturn_t bbq10pmod_irq_handler(int irq, void *dev_id)
{
	struct bbq10pmod_data *drv_data = dev_id;
	struct i2c_client *client = drv_data->client;
	int error;
	u8 reg;

	pr_debug("%s fired\n", __func__);

	error = bbq10pmod_read_reg(drv_data, REG_INT, &reg, sizeof(u8));
	if (error < 0) {
		dev_err(&client->dev, "%s: failed to read KEY: %d\n",
			__func__, error);
		return IRQ_NONE;
	}

	pr_debug("int: 0x%02x\n", reg);

	if (reg == 0x00)
		return IRQ_NONE;

	if (reg & INT_OVERFLOW)
		dev_warn(&client->dev, "overflow occurred\n");

	if (reg & INT_CAPSLOCK || reg & INT_NUMLOCK)
		bbq10pmod_update_locks(drv_data);

	if (reg & INT_KEY)
		bbq10pmod_read_fifo(drv_data);

	reg = 0x00;
	error = bbq10pmod_write_data(drv_data, REG_INT, &reg, sizeof(reg));
	if (error < 0)
		dev_err(&client->dev, "%s: failed to clear REG_INT, error: %d\n",
			__func__, error);

	return IRQ_HANDLED;
}

static int bbq10pmod_backlight_update_status(struct backlight_device *bd)
{
	const struct backlight_properties *p = &bd->props;
	const u8 intensity = (p->power == FB_BLANK_UNBLANK) ? p->brightness : 0;
	struct bbq10pmod_data *drv_data = dev_get_drvdata(&bd->dev);

	return bbq10pmod_write_data(drv_data, REG_BKL, &intensity, sizeof(intensity));
}

static const struct backlight_ops backlight_ops = {
	.options		= BL_CORE_SUSPENDRESUME,
	.update_status	= bbq10pmod_backlight_update_status,
};

static struct backlight_properties backlight_props = {
	.type			= BACKLIGHT_PLATFORM,
	.max_brightness = 255,
	.brightness		= 127,
};

static int bbq10pmod_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct bbq10pmod_data *drv_data;
	struct backlight_device *bd;
	struct input_dev *input;
	int error;
	int i;
	u8 reg;

	drv_data = devm_kzalloc(dev, sizeof(*drv_data), GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

	drv_data->client = client;
	memcpy(drv_data->keycode, keycodes, sizeof(drv_data->keycode));

	error = bbq10pmod_write_reg(drv_data, REG_RST);
	if (error)
		return -ENODEV;

	msleep(100);

	error = bbq10pmod_read_reg(drv_data, REG_VER, &reg, sizeof(reg));
	if (error)
		return -ENODEV;

	printk("%s version: 0x%02X\n", __func__, reg);

	reg = CFG_OVERFLOW_ON | CFG_OVERFLOW_INT | CFG_CAPSLOCK_INT |
			CFG_NUMLOCK_INT | CFG_KEY_INT | CFG_REPORT_MODS;
	error = bbq10pmod_write_data(drv_data, REG_CFG, &reg, sizeof(reg));
	if (error)
		return -ENODEV;

	bd = devm_backlight_device_register(dev, client->name, dev,
					    drv_data,
					    &backlight_ops,
					    &backlight_props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	error = backlight_update_status(bd);
	if (error)
		return -ENODEV;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	drv_data->input = input;

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->id.vendor  = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0001;
	input->keycode = drv_data->keycode;
	input->keycodesize = sizeof(drv_data->keycode[0]);
	input->keycodemax = ARRAY_SIZE(drv_data->keycode);

	for (i = 0; i < NUM_KEYCODES; i++)
		__set_bit(drv_data->keycode[i], input->keybit);

	__clear_bit(KEY_RESERVED, input->keybit);

	__set_bit(EV_REP, input->evbit);
	__set_bit(EV_KEY, input->evbit);

	input_set_capability(input, EV_MSC, MSC_SCAN);

	error = devm_request_threaded_irq(dev, client->irq,
										NULL, bbq10pmod_irq_handler,
										IRQF_SHARED | IRQF_ONESHOT,
										client->name, drv_data);
	if (error) {
		dev_err(dev, "Failed to claim irq %d; error %d\n", client->irq, error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Failed to register input device, error: %d\n", error);
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
