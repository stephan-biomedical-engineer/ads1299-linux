# SPDX-License-Identifier: GPL-2.0
#
# Out-of-tree build of ti-ads1299 against a configured kernel tree.
#
#   make                       for the running kernel
#   make KDIR=/path/to/kernel  for another one (add ARCH= and CROSS_COMPILE=)
#
# Kbuild stops the build on a kernel without IIO_EV_TYPE_FAULT.

KDIR ?= /lib/modules/$(shell uname -r)/build

all modules:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

modules_install:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

.PHONY: all modules modules_install clean
