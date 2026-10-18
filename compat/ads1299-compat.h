/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Functions the driver uses that an older kernel lacks, each written over
 * what that kernel has. Included ahead of the driver by build/Kbuild, which
 * compat/configure.sh generates with one ADS1299_NO_* for each feature it
 * found missing in the kernel's headers. Not part of the mainline driver.
 */
#ifndef ADS1299_COMPAT_H
#define ADS1299_COMPAT_H

#ifdef ADS1299_NO_CLAIM_DIRECT
#include <linux/iio/iio.h>

/* The new one returns true when it got the device, the old one 0. */
static inline bool iio_device_claim_direct(struct iio_dev *indio_dev)
{
	return !iio_device_claim_direct_mode(indio_dev);
}

static inline void iio_device_release_direct(struct iio_dev *indio_dev)
{
	iio_device_release_direct_mode(indio_dev);
}
#endif

#ifdef ADS1299_NO_FOR_EACH_ACTIVE_CHANNEL
#include <linux/iio/iio.h>

#define iio_for_each_active_channel(indio_dev, chan) \
	for_each_set_bit((chan), (indio_dev)->active_scan_mask, \
			 (indio_dev)->masklength)
#endif

#ifdef ADS1299_NO_PUSH_WITH_TS
#include <linux/iio/buffer.h>

/* The old one does not check the size of the scan buffer. */
static inline int iio_push_to_buffers_with_ts(struct iio_dev *indio_dev,
					      void *data, size_t data_total_len,
					      s64 timestamp)
{
	return iio_push_to_buffers_with_timestamp(indio_dev, data, timestamp);
}
#endif

#ifdef ADS1299_NO_HRTIMER_SETUP
#include <linux/hrtimer.h>

static inline void hrtimer_setup(struct hrtimer *timer,
				 enum hrtimer_restart (*function)(struct hrtimer *),
				 clockid_t clock_id, enum hrtimer_mode mode)
{
	hrtimer_init(timer, clock_id, mode);
	timer->function = function;
}
#endif

#ifdef ADS1299_NO_FAULT_EVENT
/*
 * configure.sh takes the event off the channels, so these never reach the
 * core: they only let the rest of the driver compile.
 */
#define IIO_EV_TYPE_FAULT		(-1)
#define IIO_EV_DIR_FAULT_OPENWIRE	(-1)
#endif

#endif /* ADS1299_COMPAT_H */
