# SPDX-License-Identifier: GPL-2.0
#
# Out-of-tree build of ti-ads1299 against a configured kernel tree.
#
#   make                       for the running kernel
#   make KDIR=/path/to/kernel  for another one (add ARCH= and CROSS_COMPILE=)
#
# The driver is compiled from a copy in build/, which compat/configure.sh
# adapts to what the kernel lacks and says what it changed. The module is
# build/drivers/iio/adc/ti-ads1299.ko.

KDIR ?= /lib/modules/$(shell uname -r)/build
BUILD := $(CURDIR)/build

all modules: configure
	$(MAKE) -C $(KDIR) M=$(BUILD) modules

configure:
	@MAKE="$(MAKE)" $(CURDIR)/compat/configure.sh $(KDIR) $(BUILD)

modules_install:
	$(MAKE) -C $(KDIR) M=$(BUILD) modules_install

clean:
	rm -rf $(BUILD)

.PHONY: all modules configure modules_install clean
