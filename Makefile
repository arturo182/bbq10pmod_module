MODULE_NAME := bbq10pmod

obj-m := $(MODULE_NAME).o

all: modules

modules:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
.PHONY: modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
.PHONY: clean

install:
	@lsmod | grep $(MODULE_NAME) > /dev/null && rmmod $(MODULE_NAME) || true
	insmod $(MODULE_NAME).ko
.PHONY: install

remove:
	rmmod $(MODULE_NAME)
.PHONY: remove

dtbo:
	dtc -I dts -O dtb -o i2c-bbq10pmod.dtbo i2c-bbq10pmod.dts
.PHONY: dtbo
