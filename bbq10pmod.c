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

static const struct key_entry bbq10pmod_keys[] = {
	{ KE_KEY, 'a', { KEY_A } },
	{ KE_KEY, 'b', { KEY_B } },
	{ KE_KEY, 'c', { KEY_C } },
	{ KE_KEY, 'd', { KEY_D } },
	{ KE_KEY, 'e', { KEY_E } },
	{ KE_KEY, 'f', { KEY_F } },
	{ KE_KEY, 'g', { KEY_G } },
	{ KE_KEY, 'h', { KEY_H } },
	{ KE_KEY, 'i', { KEY_I } },
	{ KE_KEY, 'j', { KEY_J } },
	{ KE_KEY, 'k', { KEY_K } },
	{ KE_KEY, 'l', { KEY_L } },
	{ KE_KEY, 'm', { KEY_M } },
	{ KE_KEY, 'n', { KEY_N } },
	{ KE_KEY, 'o', { KEY_O } },
	{ KE_KEY, 'p', { KEY_P } },
	{ KE_KEY, 'q', { KEY_Q } },
	{ KE_KEY, 'r', { KEY_R } },
	{ KE_KEY, 's', { KEY_S } },
	{ KE_KEY, 't', { KEY_T } },
	{ KE_KEY, 'u', { KEY_U } },
	{ KE_KEY, 'w', { KEY_W } },
	{ KE_KEY, 'v', { KEY_V } },
	{ KE_KEY, 'x', { KEY_X } },
	{ KE_KEY, 'y', { KEY_Y } },
	{ KE_KEY, 'z', { KEY_Z } },
	{ KE_KEY, '0', { KEY_0 } },
	{ KE_KEY, '1', { KEY_1 } },
	{ KE_KEY, '2', { KEY_2 } },
	{ KE_KEY, '3', { KEY_3 } },
	{ KE_KEY, '4', { KEY_4 } },
	{ KE_KEY, '5', { KEY_5 } },
	{ KE_KEY, '6', { KEY_6 } },
	{ KE_KEY, '7', { KEY_7 } },
	{ KE_KEY, '8', { KEY_8 } },
	{ KE_KEY, '9', { KEY_9 } },
	{ KE_KEY, '-', { KEY_MINUS } },
	//{ KE_KEY, '+', { KEY_PLUS } },
	{ KE_KEY, '=', { KEY_EQUAL } },
	{ KE_KEY, '\b', { KEY_BACKSPACE } },
	{ KE_KEY, ' ', { KEY_SPACE } },
	//{ KE_KEY, '', { KEY_ } },
	//{ KE_KEY, '', { KEY_ } },
	//{ KE_KEY, '', { KEY_ } },
	//{ KE_KEY, '', { KEY_ } },
	//{ KE_KEY, '', { KEY_ } },
	//{ KE_KEY, '', { KEY_ } },
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
		dev_err(&client->dev,
		"%s failed, reg: %d, error: %d\n",
		__func__, reg, error);
		
		return error;
	}
	
	return 0;
}

static int bbq10pmod_write_data(struct bbq10pmod_data *drv_data, u8 reg, u8 *buf, u8 len)
{
	struct i2c_client *client = drv_data->client;
	int error;
	
	struct i2c_msg msgs[] = {
		{ .addr = client->addr | 0xC0, .flags = 0, .len = sizeof(u8), .buf = &reg, },
		{ .addr = client->addr | 0xC0, .flags = 0, .len = len, .buf = buf, },
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
	struct i2c_client *client = drv_data->client;
	int error;
	
	struct i2c_msg msgs[] = {
		{ .addr = client->addr, .flags = 0, .len = sizeof(u8), .buf = &reg, },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = len, .buf = buf }
	};
	
	error = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (error != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev,
		"%s 1 failed, reg: %d, error: %d\n",
		__func__, reg, error);
		
		return error;
	}
	
	msleep(100);
	
	return 0;
}

static void bbq10pmod_read_fifo(struct bbq10pmod_data *drv_data)
{
	struct input_dev *input = drv_data->input;
	u8 data[2];
	u8 count;
	int error;
	const struct key_entry *ke;
	unsigned int keycode;
	
	error = bbq10pmod_read_reg(drv_data, REG_KEY, data, sizeof(u8));
	if (error < 0)
		dev_err(&drv_data->client->dev, "%s: Failed to read KEY: %d\n", __func__, error);
		
	printk("%s: status: 0x%02X\n", __func__, data[0]);
	
	count = (data[0] & KEY_COUNT_MASK);
	
	while (count > 0) {
		error = bbq10pmod_read_reg(drv_data, REG_FIF, data, sizeof(u8) * 2);
		if (error < 0)
			dev_err(&drv_data->client->dev, "%s: Failed to read KEY: %d\n", __func__, error);
		
		printk("%s: key %d/%c, state: %d\n", __func__, data[1], data[1], data[0]);
		
		count -= 1;
		
		if (data[1] != 0 && data[1] != 0xFF && (data[0] == STATE_PRESS || data[0] == STATE_RELEASE)) {
			ke = sparse_keymap_entry_from_scancode(input, data[1]);
			keycode = ke ? ke->keycode : KEY_UNKNOWN;
			printk("input data 0x%04x--> keycode %d\n", data[1], keycode);

			input_report_key(input, keycode, data[0] == STATE_PRESS);
		}
	}
	input_sync(input);
}

static irqreturn_t bbq10pmod_irq_handler(int irq, void *dev_id)
{
	struct bbq10pmod_data *drv_data = dev_id;
	
	printk("%s fired\n", __func__);
	
	// TODO: Read INT reg and see reason
	
	bbq10pmod_read_fifo(drv_data);
	
	// clear INT reg
	
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
	
	//reg = 0;
	//error = bbq10pmod_write_data(drv_data, REG_BKL, &reg, 1);
	//if (error)
	//	return -ENODEV;
	
	error = bbq10pmod_read_reg(drv_data, REG_VER, &reg, sizeof(reg));
	if (error)
		return -ENODEV;
		
	printk("%s: version: 0x%02X\n", __func__, reg);
	
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
