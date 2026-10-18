# SPDX-License-Identifier: GPL-2.0
obj-m := drivers/iio/adc/ti-ads1299.o

# The driver reports open electrodes as IIO_EV_TYPE_FAULT /
# IIO_EV_DIR_FAULT_OPENWIRE, and a kernel without that event type cannot build
# it. kbuild's srctree is the kernel source whatever the layout of the headers:
# in KDIR, behind an O= output directory, or in a distribution's separate
# package. Only the build checks it; clean does not.
ifneq ($(filter %Makefile.build,$(firstword $(MAKEFILE_LIST))),)
ifneq ($(shell grep -Eqs '^[[:space:]]*IIO_EV_TYPE_FAULT\b' \
	$(srctree)/include/uapi/linux/iio/types.h && echo y),y)
$(error $(abspath $(srctree)) has no IIO_EV_TYPE_FAULT: see README.md, "Kernel requirements")
endif
endif
