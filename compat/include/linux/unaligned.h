/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Kernels before 6.12 keep the unaligned accessors in <asm/unaligned.h>.
 * compat/configure.sh puts this directory on the include path only when the
 * kernel has no <linux/unaligned.h>.
 */
#include <asm/unaligned.h>
