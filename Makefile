# SPDX-License-Identifier: GPL-2.0
#
# Out-of-tree build of ti-ads1299 against a configured kernel tree.
#
#   make                       for the running kernel
#   make KDIR=/path/to/kernel  for another one (add ARCH= and CROSS_COMPILE=)

KDIR ?= /lib/modules/$(shell uname -r)/build

# The driver reports open electrodes as IIO_EV_TYPE_FAULT /
# IIO_EV_DIR_FAULT_OPENWIRE. A kernel without that event type cannot build it.
HAS_FAULT_EVENT := $(shell grep -qs IIO_EV_TYPE_FAULT \
	$(KDIR)/include/uapi/linux/iio/types.h && echo y)

all modules: check
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

modules_install: check
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

check:
ifneq ($(HAS_FAULT_EVENT),y)
	$(error $(KDIR) has no IIO_EV_TYPE_FAULT: see README.md, "Kernel requirements")
endif

.PHONY: all modules modules_install clean check
