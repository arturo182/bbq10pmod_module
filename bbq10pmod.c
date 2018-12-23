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
#include <linux/input/sparse-keymap.h>

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

struct bbq10pmod_data {
	struct i2c_client *client;
	struct input_dev *input;
};

static const struct key_entry bbq10pmod_keys[] = {
	{ KE_KEY, 0x01, { KEY_UP } },
	{ KE_KEY, 0x02, { KEY_DOWN } },
	{ KE_KEY, 0x03, { KEY_LEFT } },
	{ KE_KEY, 0x04, { KEY_RIGHT } },
	{ KE_KEY, 0x05, { KEY_ENTER } },
	{ KE_KEY, 0x06, { KEY_MENU } },
	{ KE_KEY, 0x07, { KEY_BACK } },

	{ KE_KEY, 17, { KEY_LEFTALT } },
	{ KE_KEY, 18, { KEY_LEFTSHIFT } },
	{ KE_KEY, 19, { KEY_LEFTCTRL } },

	{ KE_KEY, 'A', { KEY_A } },
	{ KE_KEY, 'B', { KEY_B } },
	{ KE_KEY, 'C', { KEY_C } },
	{ KE_KEY, 'D', { KEY_D } },
	{ KE_KEY, 'E', { KEY_E } },
	{ KE_KEY, 'F', { KEY_F } },
	{ KE_KEY, 'G', { KEY_G } },
	{ KE_KEY, 'H', { KEY_H } },
	{ KE_KEY, 'I', { KEY_I } },
	{ KE_KEY, 'J', { KEY_J } },
	{ KE_KEY, 'K', { KEY_K } },
	{ KE_KEY, 'L', { KEY_L } },
	{ KE_KEY, 'M', { KEY_M } },
	{ KE_KEY, 'N', { KEY_N } },
	{ KE_KEY, 'O', { KEY_O } },
	{ KE_KEY, 'P', { KEY_P } },
	{ KE_KEY, 'Q', { KEY_Q } },
	{ KE_KEY, 'R', { KEY_R } },
	{ KE_KEY, 'S', { KEY_S } },
	{ KE_KEY, 'T', { KEY_T } },
	{ KE_KEY, 'U', { KEY_U } },
	{ KE_KEY, 'W', { KEY_W } },
	{ KE_KEY, 'V', { KEY_V } },
	{ KE_KEY, 'X', { KEY_X } },
	{ KE_KEY, 'Y', { KEY_Y } },
	{ KE_KEY, 'Z', { KEY_Z } },
	{ KE_KEY, '\b', { KEY_BACKSPACE } },
	{ KE_KEY, ' ', { KEY_SPACE } },
	{ KE_KEY, '\n', { KEY_ENTER } },
	{ KE_KEY, '~', { KEY_GRAVE } },

	{ KE_KEY, 254, { KEY_NUMLOCK } },
	{ KE_KEY, 255, { KEY_CAPSLOCK } },
};

static unsigned int sparse_keymap_get_key_index(struct input_dev *dev,
						const struct key_entry *k)
{
	struct key_entry *key;
	unsigned int idx = 0;

	for (key = dev->keycode; key->type != KE_END; key++) {
		if (key->type == KE_KEY) {
			if (key == k)
				break;
			idx++;
		}
	}

	return idx;
}

static struct key_entry *sparse_keymap_entry_by_index(struct input_dev *dev,
						      unsigned int index)
{
	struct key_entry *key;
	unsigned int key_cnt = 0;

	for (key = dev->keycode; key->type != KE_END; key++)
		if (key->type == KE_KEY)
			if (key_cnt++ == index)
				return key;

	return NULL;
}

struct key_entry *sparse_keymap_entry_from_scancode(struct input_dev *dev,
						    unsigned int code)
{
	struct key_entry *key;

	for (key = dev->keycode; key->type != KE_END; key++)
		if (code == key->code)
			return key;

	return NULL;
}

struct key_entry *sparse_keymap_entry_from_keycode(struct input_dev *dev,
						   unsigned int keycode)
{
	struct key_entry *key;

	for (key = dev->keycode; key->type != KE_END; key++)
		if (key->type == KE_KEY && keycode == key->keycode)
			return key;

	return NULL;
}

static struct key_entry *sparse_keymap_locate(struct input_dev *dev,
					const struct input_keymap_entry *ke)
{
	struct key_entry *key;
	unsigned int scancode;

	if (ke->flags & INPUT_KEYMAP_BY_INDEX)
		key = sparse_keymap_entry_by_index(dev, ke->index);
	else if (input_scancode_to_scalar(ke, &scancode) == 0)
		key = sparse_keymap_entry_from_scancode(dev, scancode);
	else
		key = NULL;

	return key;
}

static int sparse_keymap_getkeycode(struct input_dev *dev,
				    struct input_keymap_entry *ke)
{
	const struct key_entry *key;

	if (dev->keycode) {
		key = sparse_keymap_locate(dev, ke);
		if (key && key->type == KE_KEY) {
			ke->keycode = key->keycode;
			if (!(ke->flags & INPUT_KEYMAP_BY_INDEX))
				ke->index =
					sparse_keymap_get_key_index(dev, key);
			ke->len = sizeof(key->code);
			memcpy(ke->scancode, &key->code, sizeof(key->code));
			return 0;
		}
	}

	return -EINVAL;
}

static int sparse_keymap_setkeycode(struct input_dev *dev,
				    const struct input_keymap_entry *ke,
				    unsigned int *old_keycode)
{
	struct key_entry *key;

	if (dev->keycode) {
		key = sparse_keymap_locate(dev, ke);
		if (key && key->type == KE_KEY) {
			*old_keycode = key->keycode;
			key->keycode = ke->keycode;
			set_bit(ke->keycode, dev->keybit);
			if (!sparse_keymap_entry_from_keycode(dev, *old_keycode))
				clear_bit(*old_keycode, dev->keybit);
			return 0;
		}
	}

	return -EINVAL;
}

int sparse_keymap_setup(struct input_dev *dev,
			const struct key_entry *keymap,
			int (*setup)(struct input_dev *, struct key_entry *))
{
	size_t map_size = 1; /* to account for the last KE_END entry */
	const struct key_entry *e;
	struct key_entry *map, *entry;
	int i;
	int error;

	for (e = keymap; e->type != KE_END; e++)
		map_size++;

	map = devm_kmemdup(&dev->dev, keymap, map_size * sizeof(*map),
			   GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	for (i = 0; i < map_size; i++) {
		entry = &map[i];

		if (setup) {
			error = setup(dev, entry);
			if (error)
				return error;
		}

		switch (entry->type) {
		case KE_KEY:
			__set_bit(EV_KEY, dev->evbit);
			__set_bit(entry->keycode, dev->keybit);
			break;

		case KE_SW:
		case KE_VSW:
			__set_bit(EV_SW, dev->evbit);
			__set_bit(entry->sw.code, dev->swbit);
			break;
		}
	}

	if (test_bit(EV_KEY, dev->evbit)) {
		__set_bit(KEY_UNKNOWN, dev->keybit);
		__set_bit(EV_MSC, dev->evbit);
		__set_bit(MSC_SCAN, dev->mscbit);
	}

	dev->keycode = map;
	dev->keycodemax = map_size;
	dev->getkeycode = sparse_keymap_getkeycode;
	dev->setkeycode = sparse_keymap_setkeycode;

	return 0;
}

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

static int bbq10pmod_write_data(struct bbq10pmod_data *drv_data, u8 reg, u8 *buf, u8 len)
{
	struct i2c_client *client = drv_data->client;
	int error;

	reg |= 0x80;

	struct i2c_msg msgs[] = {
		{ .addr = client->addr, .flags = 0, .len = sizeof(u8), .buf = &reg, },
		{ .addr = client->addr, .flags = 0, .len = len, .buf = buf, },
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

	input_report_key(input, KEY_NUMLOCK, data & _KEY_NUMLOCK);
	input_report_key(input, KEY_CAPSLOCK, data & _KEY_CAPSLOCK);

	input_sync(input);
}

static void bbq10pmod_read_fifo(struct bbq10pmod_data *drv_data)
{
	struct input_dev *input = drv_data->input;
	struct i2c_client *client = drv_data->client;
	const struct key_entry *ke;
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

		if (data[1] != 0 && data[1] != 0xFF && (data[0] == STATE_PRESS || data[0] == STATE_RELEASE)) {
			ke = sparse_keymap_entry_from_scancode(input, data[1]);
			keycode = ke ? ke->keycode : KEY_UNKNOWN;

			pr_debug("%s input data 0x%04x--> keycode %d\n",
				__func__, data[1], keycode);

			if (keycode == KEY_UNKNOWN)
				pr_warn("%s input data 0x%04x --> unknown\n",
					__func__, data[1]);

			input_report_key(input, keycode, data[0] == STATE_PRESS);
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

	pr_warn("int: 0x%02x\n", reg);

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

	error = bbq10pmod_write_reg(drv_data, REG_RST);
	if (error)
		return -ENODEV;

	msleep(100);

	error = bbq10pmod_read_reg(drv_data, REG_VER, &reg, sizeof(reg));
	if (error)
		return -ENODEV;

	printk("%s version: 0x%02X\n", __func__, reg);

	reg = 0x55; // TODO: make a sysfs device
	error = bbq10pmod_write_data(drv_data, REG_BKL, &reg, sizeof(reg));
	if (error)
		return -ENODEV;

	reg = CFG_OVERFLOW_ON | CFG_OVERFLOW_INT | CFG_CAPSLOCK_INT |
			CFG_NUMLOCK_INT | CFG_KEY_INT | CFG_REPORT_MODS;
	error = bbq10pmod_write_data(drv_data, REG_CFG, &reg, sizeof(reg));
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

	//if (device_property_read_bool(dev, "keypad,autorepeat"))
	//	__set_bit(EV_REP, input->evbit);

	//input_set_capability(input, EV_MSC, MSC_SCAN);

	error = sparse_keymap_setup(input, bbq10pmod_keys, NULL);
	if (error)
		return error;

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
