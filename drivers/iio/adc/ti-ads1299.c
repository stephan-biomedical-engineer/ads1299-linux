// SPDX-License-Identifier: GPL-2.0
/* TI ADS1299 biopotential (EEG) front-end driver
 *
 * Copyright (C) 2023 - 2024 Topic Embedded Products
 *     drivers/iio/adc/ti-ads1298.c, Mike Looijmans <mike.looijmans@topic.nl>,
 *     mainline since Linux 6.9. This file is a derivative of that driver.
 * Copyright (C) 2026 MedPlatform (TCC)
 *
 * Provenance, because it is the point (implementation_plan_iio_afe.md §3):
 * this is NOT software of unknown provenance. It is a derivative of a driver
 * that went through the IIO maintainers' review, whose author, history and
 * review are public and citable, adapted here for the ADS1299 - the EEG
 * sibling of the ADS1298 in the same ADS129x family. The alternative that was
 * rejected was adopting an unmaintained third-party driver, which would have
 * added SOUP in the kernel's trust domain (IEC 62304 §7.1.2-7.1.3).
 *
 * Backport 6.9 -> 6.6, measured rather than assumed: every kernel interface
 * the mainline file uses exists unchanged in both kernels this repository
 * builds (linux-stm32mp 6.6.129 and linux-yocto 6.6.144) - cleanup.h and the
 * spinlock_irqsave guard, REGCACHE_MAPLE, regmap_config's .reg_read/.reg_write,
 * MICROHZ_PER_HZ, get_unaligned_be24, devm_iio_kfifo_buffer_setup. The one
 * release of API drift the plan budgeted for turned out to be zero lines.
 *
 * ADS1298 -> ADS1299: what actually differs, and why each one matters
 * ------------------------------------------------------------------
 *  - PGA gains. ADS1298 {6,1,2,3,4,8,12}; ADS1299 {1,2,4,6,8,12,24}. Same
 *    field, different table - a silent 4x scale error if carried over.
 *  - Reference. ADS1298 selects 2.4 V or 4.0 V with CONFIG3 bit 5; the ADS1299
 *    internal reference is 4.5 V and that bit is reserved. Hardcoding the
 *    ADS1298 choice would put a 12.5% gain error on every trace.
 *  - Data rate. ADS1298 has an HR/LP bit that shifts the divider by 6 or 7;
 *    the ADS1299 has no such bit, the divider shift is always 7, and the rate
 *    set is the discrete {250 .. 16000} SPS the plan's safety envelope names.
 *  - CHnSET out of reset is 0x61 on the ADS1299: gain 24 and MUX = "input
 *    shorted". A driver that does not program the MUX measures nothing and
 *    looks like it works. The ADS1298 driver never touches MUX because its
 *    reset value is the normal input.
 *  - ID register field layout differs (see the masks below).
 *
 * What this file adds beyond the adaptation
 * -----------------------------------------
 *  - A timestamp channel. The mainline driver declares none, and for a
 *    clinical record the instant a sample was converted is not optional. This
 *    is the first candidate to send back upstream.
 *  - An internal hrtimer source for boards where DRDY cannot reach the host as
 *    an interrupt - which is the USB/MCP2210 link, where the bridge offers a
 *    counted GPIO and no asynchronous delivery at all. The timestamp is then
 *    the host's, not the converter's, and the driver says so through
 *    "timestamp_source" so that no record can quietly claim more fidelity than
 *    it has.
 *  - A lost-sample counter, exported, because "samples lost under load" is one
 *    of the measurements docs/RESULTS.md §9 lists as missing.
 *  - A probe-time self test of the analogue path: the internal test generator
 *    routed to every input and its swing measured, then the inputs shorted and
 *    the noise floor measured. A front-end that fails is never registered.
 *    This is the shape of self test IEC 60601-1 §14 expects, and it lives here
 *    because judging the answer needs this part's generator amplitude and
 *    noise floor - facts no layer above the kernel may hold. Running it at
 *    probe, with no ABI of its own, is the IIO convention (adis_self_test() in
 *    drivers/iio/imu/adis.c, ak8974_selftest() in the magnetometer driver).
 *  - An input MUX control, for bench work on the analogue path.
 *
 * DATASHEET VALUES ARE UNVERIFIED. Every constant below marked [SBAS499?] was
 * written from the ADS1299 register map as understood here and has NOT been
 * checked against the datasheet or against silicon. They are collected at the
 * top of the file, in one place, precisely so that checking them is a single
 * pass. Until that pass happens they are debt, in the sense
 * BRINGUP_STM32MP2.md §11 rule 1 gives the word.
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/log2.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/units.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>

#include <asm/unaligned.h>

/* Commands - identical to the ADS1298, the family shares the command set */
#define ADS1299_CMD_WAKEUP	0x02
#define ADS1299_CMD_STANDBY	0x04
#define ADS1299_CMD_RESET	0x06
#define ADS1299_CMD_START	0x08
#define ADS1299_CMD_STOP	0x0a
#define ADS1299_CMD_RDATAC	0x10
#define ADS1299_CMD_SDATAC	0x11
#define ADS1299_CMD_RDATA	0x12
#define ADS1299_CMD_RREG	0x20
#define ADS1299_CMD_WREG	0x40

/* Registers */
#define ADS1299_REG_ID		0x00
/*
 * [SBAS499?] ID is REV_ID[7:5] | 1 | DEV_ID[3:2] | NU_CH[1:0], and the reset
 * value of the 8-channel part is 0x3e. So the part-identifying field is
 * bits [4:2] = 0b111, and the channel count is a TWO bit field - not the three
 * bit field the ADS1298 has, whose bit 2 belongs to DEV_ID here. Using the
 * mainline masks unmodified would decode an ADS1299 as a 16-channel unknown
 * device, which is a good failure (it refuses to probe) but for the wrong
 * reason.
 */
#define ADS1299_MASK_ID_FAMILY			GENMASK(4, 2)
#define ADS1299_MASK_ID_CHANNELS		GENMASK(1, 0)
#define ADS1299_ID_FAMILY_ADS129X		0x7

#define ADS1299_REG_CONFIG1	0x01
/*
 * [SBAS499 Table 13 / Figure 51] CONFIG1, reset 0x96.
 *
 * TWO reserved fields: bit 7 must always be written 1, and bits 4:3 must
 * always be written 2h. 0x90 is the two of them together, and 0x90 | DR is the
 * whole register - 0x96 at the reset data rate.
 *
 * CONFIG1 is only ever touched with regmap_update_bits() on the DR field, so
 * these bits keep the value the part came up with. The full value is spelled
 * out anyway: reaching for BIT(7) alone as a "reserved value", the way CONFIG2
 * and CONFIG3 are written whole, would clear bit 4 on every rate change.
 */
#define ADS1299_MASK_CONFIG1_RESERVED_HI	BIT(7)
#define ADS1299_MASK_CONFIG1_RESERVED_LO	GENMASK(4, 3)
#define ADS1299_CONFIG1_RESERVED_VALUE					\
	(ADS1299_MASK_CONFIG1_RESERVED_HI |				\
	 FIELD_PREP(ADS1299_MASK_CONFIG1_RESERVED_LO, 0x2))
#define ADS1299_MASK_CONFIG1_DAISY_EN		BIT(6)
#define ADS1299_MASK_CONFIG1_CLK_EN		BIT(5)
#define ADS1299_MASK_CONFIG1_DR			GENMASK(2, 0)
/*
 * ODR = f_CLK >> (7 + DR), with the internal 2.048 MHz oscillator giving
 * 16 kSPS at DR=0 and 250 SPS at DR=6. DR=7 is reserved. There is no HR/LP
 * bit: unlike the ADS1298 the shift is not mode dependent.
 */
#define ADS1299_SHIFT_DR			7
#define ADS1299_LOWEST_DR			0x06

#define ADS1299_REG_CONFIG2	0x02
/*
 * [SBAS499 Table 14 / Figure 52] CONFIG2, reset 0xc0.
 *
 * There are TWO reserved fields here and neither is don't-care: bits 7:5 must
 * always be written 6h, and bit 3 must always be written 0. The reserved bits
 * are therefore a VALUE and not a mask of ones - writing GENMASK(7, 5) would
 * put 7h into a field the datasheet says is always 6h.
 */
#define ADS1299_MASK_CONFIG2_RESERVED		GENMASK(7, 5)
#define ADS1299_CONFIG2_RESERVED_VALUE					\
	FIELD_PREP(ADS1299_MASK_CONFIG2_RESERVED, 0x6)
#define ADS1299_MASK_CONFIG2_INT_CAL		BIT(4)
/* Bit 3 is the second reserved field, always 0, so it gets no macro. */
#define ADS1299_MASK_CONFIG2_CAL_AMP		BIT(2)
#define ADS1299_MASK_CONFIG2_CAL_FREQ		GENMASK(1, 0)
#define ADS1299_CAL_FREQ_SLOW			0	/* f_CLK / 2^21, ~1 Hz */
#define ADS1299_CAL_FREQ_FAST			1	/* f_CLK / 2^20, ~2 Hz */
#define ADS1299_CAL_FREQ_DC			3

#define ADS1299_REG_CONFIG3	0x03
/*
 * [SBAS499 Table 15 / Figure 53] CONFIG3, reset 0x60.
 *
 * Every field below sits where the datasheet puts it. Two notes for whoever
 * checks it next.
 *
 * The reserved field 6:5 must always be written 3h, and 3h happens to be "both
 * bits set", so here the mask and the mandated value coincide. That is a
 * coincidence of this register and not a rule - CONFIG2's reserved field is 6h
 * over three bits - so the value is spelled out with FIELD_PREP anyway, and the
 * two registers read the same way.
 *
 * The datasheet calls bit 7 PD_REFBUF, "power-down reference buffer", and then
 * gives it inverted polarity: 0 powers the buffer DOWN, 1 enables it. The macro
 * is named for what setting it does, because "val |= PD_REFBUF" to power
 * something down is a bug waiting to be written. The datasheet name is recorded
 * here so a search for PD_REFBUF finds it.
 */
#define ADS1299_MASK_CONFIG3_PWR_REFBUF		BIT(7)	/* datasheet: PD_REFBUF */
#define ADS1299_MASK_CONFIG3_RESERVED		GENMASK(6, 5)
#define ADS1299_CONFIG3_RESERVED_VALUE					\
	FIELD_PREP(ADS1299_MASK_CONFIG3_RESERVED, 0x3)
#define ADS1299_MASK_CONFIG3_BIAS_MEAS		BIT(4)
#define ADS1299_MASK_CONFIG3_BIASREF_INT	BIT(3)
#define ADS1299_MASK_CONFIG3_PD_BIAS		BIT(2)
#define ADS1299_MASK_CONFIG3_BIAS_LOFF_SENS	BIT(1)
/*
 * Bit 0 is BIAS_STAT, read-only, the lead-off status of the BIAS electrode: 0
 * connected, 1 not connected. It has a macro and no user.
 *
 * Reporting it is not just an attribute: CONFIG3 is cached (REGCACHE_MAPLE, and
 * the volatile table covers only LOFF_STATP..LOFF_STATN), so a read would come
 * back as the value that was written and this bit would report "connected"
 * forever. The register has to join the volatile table in the same change.
 */
#define ADS1299_MASK_CONFIG3_BIAS_STAT		BIT(0)

#define ADS1299_REG_LOFF	0x04
#define ADS1299_REG_CHnSET(n)	(0x05 + (n))
#define ADS1299_MASK_CH_PD		BIT(7)
#define ADS1299_MASK_CH_PGA		GENMASK(6, 4)
#define ADS1299_MASK_CH_SRB2		BIT(3)
#define ADS1299_MASK_CH_MUX		GENMASK(2, 0)

#define ADS1299_REG_BIAS_SENSP	0x0d
#define ADS1299_REG_BIAS_SENSN	0x0e
/*
 * [SBAS499 Tables 20, 21 / Figures 58, 59] One bit per electrode, and this
 * driver never writes either of them.
 *
 * The datasheet is explicit: the LOFF_STATP bits are only valid where the
 * corresponding LOFF_SENSP bit is set, and the same holds for the negative
 * side. Powering the comparators on in CONFIG4 is therefore only half of what
 * lead-off detection needs. With SENSP/SENSN left at 0, LOFF_STATP and
 * LOFF_STATN read 00 00, which is indistinguishable from every electrode being
 * attached.
 *
 * Until detection is enabled per channel, lead_off_status says "disabled"
 * rather than emitting a number the datasheet says to ignore. Enabling it
 * drives the lead-off current into the electrodes, and its magnitude and
 * frequency come from LOFF (0x04), which nothing here programs either.
 */
#define ADS1299_REG_LOFF_SENSP	0x0f
#define ADS1299_REG_LOFF_SENSN	0x10
#define ADS1299_REG_LOFF_FLIP	0x11
#define ADS1299_REG_LOFF_STATP	0x12
#define ADS1299_REG_LOFF_STATN	0x13
#define ADS1299_REG_GPIO	0x14
#define ADS1299_REG_MISC1	0x15
#define ADS1299_REG_MISC2	0x16
#define ADS1299_REG_CONFIG4	0x17
/* [SBAS499?] bit positions in CONFIG4 are the least certain in this file */
#define ADS1299_MASK_CONFIG4_SINGLE_SHOT	BIT(3)
#define ADS1299_MASK_CONFIG4_PD_LOFF_COMP	BIT(1)

/* MUX field of CHnSET - what the channel is actually connected to */
#define ADS1299_MUX_NORMAL		0
#define ADS1299_MUX_SHORTED		1
#define ADS1299_MUX_BIAS_MEAS		2
#define ADS1299_MUX_MVDD		3
#define ADS1299_MUX_TEMPERATURE		4
#define ADS1299_MUX_TEST_SIGNAL		5
#define ADS1299_MUX_BIAS_DRP		6
#define ADS1299_MUX_BIAS_DRN		7

#define ADS1299_MAX_CHANNELS	8
#define ADS1299_BITS_PER_SAMPLE	24
#define ADS1299_CLK_RATE_HZ	2048000
#define ADS1299_CLOCKS_TO_USECS(x) \
		(DIV_ROUND_UP((x) * MICROHZ_PER_HZ, ADS1299_CLK_RATE_HZ))
/*
 * Register access needs 4 clocks to decode, so the transfer speed is limited
 * while doing it rather than inserting inter-byte delays.
 */
#define ADS1299_SPI_BUS_SPEED_SLOW	ADS1299_CLK_RATE_HZ
#define ADS1299_SPI_CMD_BUFFER_SIZE	3
/* Status word plus 'n' 24-bit samples, plus the command byte */
#define ADS1299_SPI_RDATA_BUFFER_SIZE(n)	(((n) + 1) * 3 + 1)
#define ADS1299_SPI_RDATA_BUFFER_SIZE_MAX \
		ADS1299_SPI_RDATA_BUFFER_SIZE(ADS1299_MAX_CHANNELS)

/*
 * Probe-time self test criteria.
 *
 * [SBAS499?] The generator swings +-VREF/2400 at 1x amplitude, so its peak to
 * peak is VREF/1200 - and in CODES that is gain * 2^23 / 1200 whatever the
 * reference is, which is why the check below needs no reference at all.
 */
#define ADS1299_TEST_SIGNAL_PP_DIVISOR		1200
/* +-20%: wider than any reference tolerance, narrower than a dead PGA stage */
#define ADS1299_SELF_TEST_TOLERANCE_PCT		20
/*
 * Shorted-input ceiling, microvolts peak to peak. Two orders of magnitude above
 * the datasheet noise figure: a test of "the analogue path is not broken", not
 * of "the board is quiet". The second needs a bench.
 */
#define ADS1299_SELF_TEST_NOISE_UV_PP		10
/* The ~1 Hz square wave needs about two seconds to show both levels */
#define ADS1299_SELF_TEST_SIGNAL_READINGS	40
#define ADS1299_SELF_TEST_NOISE_READINGS	10
/* Conversions right after a MUX change carry its transient */
#define ADS1299_SELF_TEST_DISCARD		2
#define ADS1299_SELF_TEST_INTERVAL_MS		50

/*
 * The internal reference, in microvolts. Fixed on this part; the ADS1298's
 * 2.4/4.0 V選択 does not exist here. An external reference is still supported
 * through the "vref" regulator, which is why this is only the fallback.
 */
#define ADS1299_INTERNAL_VREF_UV	4500000

enum ads1299_timestamp_source {
	ADS1299_TS_DRDY_IRQ = 0,
	ADS1299_TS_HOST_TIMER,
};

struct ads1299_private {
	struct spi_device *spi;
	struct regulator *reg_avdd;
	struct regulator *reg_vref;
	struct clk *clk;
	struct regmap *regmap;
	struct completion completion;
	struct spi_transfer rdata_xfer;
	struct spi_message rdata_msg;
	spinlock_t irq_busy_lock; /* Handshake between SPI and DRDY irqs */
	/*
	 * rdata_xfer_busy increments when a sample becomes due (DRDY interrupt,
	 * or hrtimer expiry) and decrements when SPI completion is reported:
	 * 0 = idle, 1 = SPI transfer in progress, 2 = one sample became due
	 * during the transfer, >2 = rdata_xfer_busy - 2 samples were lost.
	 */
	unsigned int rdata_xfer_busy;
	/*
	 * Samples the converter produced and the host never collected. Counted
	 * because "sample loss under load" is a number this platform owes
	 * (RESULTS.md §9) and because on the USB link it is the only way to
	 * know: the bridge delivers no interrupt, so a missed conversion is
	 * otherwise indistinguishable from a slower one.
	 */
	u64 lost_samples;

	/* Number of ADC channels the chip reported; excludes the timestamp */
	unsigned int num_adc_channels;

	enum ads1299_timestamp_source ts_source;
	struct hrtimer timer;		/* only used when ts_source is HOST_TIMER */
	ktime_t timer_period;
	bool timer_running;

	/* Demux target. Must stay 8-byte aligned for the timestamp channel. */
	struct {
		u32 chan[ADS1299_MAX_CHANNELS];
		s64 timestamp __aligned(8);
	} scan;

	/* For synchronous SPI exchanges (read/write registers) */
	u8 cmd_buffer[ADS1299_SPI_CMD_BUFFER_SIZE] __aligned(IIO_DMA_MINALIGN);

	u8 rx_buffer[ADS1299_SPI_RDATA_BUFFER_SIZE_MAX];
	/* Contains the RDATA command and zeroes to clock out */
	u8 tx_buffer[ADS1299_SPI_RDATA_BUFFER_SIZE_MAX];
};

/* Three bytes per sample in RX buffer, starting at offset 4 */
#define ADS1299_OFFSET_IN_RX_BUFFER(index)	(3 * (index) + 4)

#define ADS1299_CHAN(index)				\
{							\
	.type = IIO_VOLTAGE,				\
	.indexed = 1,					\
	.channel = index,				\
	.address = ADS1299_OFFSET_IN_RX_BUFFER(index),	\
	.info_mask_separate =				\
		BIT(IIO_CHAN_INFO_RAW) |		\
		BIT(IIO_CHAN_INFO_SCALE),		\
	.info_mask_shared_by_all =			\
		BIT(IIO_CHAN_INFO_SAMP_FREQ) |		\
		BIT(IIO_CHAN_INFO_HARDWAREGAIN),	\
	.info_mask_shared_by_all_available =		\
		BIT(IIO_CHAN_INFO_SAMP_FREQ) |		\
		BIT(IIO_CHAN_INFO_HARDWAREGAIN),	\
	.scan_index = index,				\
	.scan_type = {					\
		.sign = 's',				\
		.realbits = ADS1299_BITS_PER_SAMPLE,	\
		.storagebits = 32,			\
		.endianness = IIO_CPU,			\
	},						\
}

static const struct iio_chan_spec ads1299_channels[ADS1299_MAX_CHANNELS] = {
	ADS1299_CHAN(0),
	ADS1299_CHAN(1),
	ADS1299_CHAN(2),
	ADS1299_CHAN(3),
	ADS1299_CHAN(4),
	ADS1299_CHAN(5),
	ADS1299_CHAN(6),
	ADS1299_CHAN(7),
};

/*
 * The discrete set of output data rates, in the order of the DR field. The
 * converter accepts nothing between them, and exposing that through the IIO
 * ABI is what stops a configuration file asking for 300 SPS and silently
 * getting 250 - the safety envelope in the acquisition service checks set
 * membership for the same reason.
 */
static const int ads1299_samp_freq_avail[] = {
	16000, 8000, 4000, 2000, 1000, 500, 250,
};

/* PGA setting -> gain, indexed by the CHnSET PGA field. [SBAS499?] */
static const int ads1299_pga_gain[] = { 1, 2, 4, 6, 8, 12, 24 };

static const char * const ads1299_mux_names[] = {
	[ADS1299_MUX_NORMAL]		= "normal",
	[ADS1299_MUX_SHORTED]		= "shorted",
	[ADS1299_MUX_BIAS_MEAS]		= "bias_measurement",
	[ADS1299_MUX_MVDD]		= "supply",
	[ADS1299_MUX_TEMPERATURE]	= "temperature",
	[ADS1299_MUX_TEST_SIGNAL]	= "test_signal",
	[ADS1299_MUX_BIAS_DRP]		= "bias_drive_p",
	[ADS1299_MUX_BIAS_DRN]		= "bias_drive_n",
};

static int ads1299_write_cmd(struct ads1299_private *priv, u8 command)
{
	struct spi_transfer xfer = {
		.tx_buf = priv->cmd_buffer,
		.rx_buf = priv->cmd_buffer,
		.len = 1,
		.speed_hz = ADS1299_SPI_BUS_SPEED_SLOW,
		.delay = {
			.value = 2,
			.unit = SPI_DELAY_UNIT_USECS,
		},
	};

	priv->cmd_buffer[0] = command;

	return spi_sync_transfer(priv->spi, &xfer, 1);
}

static int ads1299_get_vref_uv(struct ads1299_private *priv)
{
	int ret;

	if (!priv->reg_vref)
		return ADS1299_INTERNAL_VREF_UV;

	ret = regulator_get_voltage(priv->reg_vref);
	if (ret < 0)
		return ret;

	return ret;
}

static int ads1299_get_gain(struct ads1299_private *priv, unsigned int channel)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_CHnSET(channel), &regval);
	if (ret)
		return ret;

	return ads1299_pga_gain[FIELD_GET(ADS1299_MASK_CH_PGA, regval)];
}

/*
 * Gain is per channel in the hardware and shared by all here on purpose: an
 * EEG montage with different gains per electrode is not a configuration this
 * device offers clinically, and letting the ABI express it would create a
 * state the acquisition service cannot describe in one scale value.
 */
static int ads1299_set_gain(struct ads1299_private *priv, int gain)
{
	unsigned int setting;
	unsigned int i;
	int ret;

	for (setting = 0; setting < ARRAY_SIZE(ads1299_pga_gain); setting++) {
		if (ads1299_pga_gain[setting] == gain)
			break;
	}
	if (setting == ARRAY_SIZE(ads1299_pga_gain))
		return -EINVAL;

	for (i = 0; i < priv->num_adc_channels; i++) {
		ret = regmap_update_bits(priv->regmap, ADS1299_REG_CHnSET(i),
					 ADS1299_MASK_CH_PGA,
					 FIELD_PREP(ADS1299_MASK_CH_PGA, setting));
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * The MUX applies to every channel at once. A state with one electrode on the
 * patient and the rest on the test generator would produce a record that is
 * part measurement and part stimulus, and nothing downstream could tell which
 * sample was which.
 */
static int ads1299_set_mux(struct ads1299_private *priv, unsigned int mux)
{
	unsigned int i;
	int ret;

	for (i = 0; i < priv->num_adc_channels; i++) {
		ret = regmap_update_bits(priv->regmap, ADS1299_REG_CHnSET(i),
					 ADS1299_MASK_CH_MUX,
					 FIELD_PREP(ADS1299_MASK_CH_MUX, mux));
		if (ret)
			return ret;
	}

	return 0;
}

static int ads1299_read_one(struct ads1299_private *priv, int chan_index)
{
	int ret;

	/* Enable the channel */
	ret = regmap_update_bits(priv->regmap, ADS1299_REG_CHnSET(chan_index),
				 ADS1299_MASK_CH_PD, 0);
	if (ret)
		return ret;

	/* Enable single-shot mode, so we don't need to send a STOP */
	ret = regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG4,
				 ADS1299_MASK_CONFIG4_SINGLE_SHOT,
				 ADS1299_MASK_CONFIG4_SINGLE_SHOT);
	if (ret)
		return ret;

	reinit_completion(&priv->completion);

	ret = ads1299_write_cmd(priv, ADS1299_CMD_START);
	if (ret < 0) {
		dev_err(&priv->spi->dev, "CMD_START error: %d\n", ret);
		return ret;
	}

	/*
	 * At the lowest rate a conversion takes 4 ms; on the USB link the
	 * round trip adds a bus frame or two. 100 ms is generous for both and
	 * still short enough that a stuck front-end is reported rather than
	 * hanging a read of a sysfs file.
	 */
	if (priv->ts_source == ADS1299_TS_HOST_TIMER) {
		/*
		 * No DRDY interrupt to complete the transfer for us, so drive
		 * the single conversion from here. A *copy* of the transfer,
		 * never priv->rdata_msg: spi_sync() overwrites a message's
		 * complete/context with its own, which would leave the async
		 * path pointing at a completion on a dead stack frame.
		 */
		struct spi_transfer xfer = priv->rdata_xfer;

		msleep(20);
		return spi_sync_transfer(priv->spi, &xfer, 1);
	}

	ret = wait_for_completion_timeout(&priv->completion, msecs_to_jiffies(100));
	if (!ret)
		return -ETIMEDOUT;

	return 0;
}

static int ads1299_get_samp_freq(struct ads1299_private *priv, int *val)
{
	unsigned long rate;
	unsigned int cfg;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_CONFIG1, &cfg);
	if (ret)
		return ret;

	rate = priv->clk ? clk_get_rate(priv->clk) : ADS1299_CLK_RATE_HZ;
	if (!rate)
		return -EINVAL;

	*val = (int)((rate >> ADS1299_SHIFT_DR) >> (cfg & ADS1299_MASK_CONFIG1_DR));

	return IIO_VAL_INT;
}

static int ads1299_set_samp_freq(struct ads1299_private *priv, int val)
{
	unsigned long rate;
	unsigned int factor;
	unsigned int dr;
	int ret;

	rate = priv->clk ? clk_get_rate(priv->clk) : ADS1299_CLK_RATE_HZ;
	if (!rate || val <= 0)
		return -EINVAL;

	/*
	 * Set membership, not nearest match. Accepting 300 SPS and programming
	 * 250 would be a configured acquisition rate that silently is not the
	 * one recorded in the session metadata.
	 */
	factor = (unsigned int)((rate >> ADS1299_SHIFT_DR) / (unsigned int)val);
	if (!is_power_of_2(factor) ||
	    (rate >> ADS1299_SHIFT_DR) != (unsigned long)val * factor)
		return -EINVAL;

	dr = (unsigned int)ilog2(factor);
	if (dr > ADS1299_LOWEST_DR)
		return -EINVAL;

	ret = regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG1,
				 ADS1299_MASK_CONFIG1_DR, dr);
	if (ret)
		return ret;

	priv->timer_period = ns_to_ktime(div_u64(NSEC_PER_SEC, (u32)val));

	return 0;
}

static int ads1299_get_scale(struct ads1299_private *priv,
			     int channel, int *val, int *val2)
{
	u64 step;
	int vref_uv;
	int gain;

	vref_uv = ads1299_get_vref_uv(priv);
	if (vref_uv < 0)
		return vref_uv;

	gain = ads1299_get_gain(priv, (unsigned int)channel);
	if (gain < 0)
		return gain;

	/*
	 * Full scale is +-VREF/gain, so one LSB is VREF/(gain * 2^23),
	 * reported in millivolts as the IIO ABI requires.
	 *
	 * Computed in 64 bits and returned as INT_PLUS_NANO rather than as the
	 * mainline driver's FRACTIONAL_LOG2 over an integer millivolt full
	 * scale. That form truncates: 4.5 V over gain 24 is 187.5 mV and
	 * becomes 187, a 0.27% gain error on every sample, and the ADS1298
	 * driver has the same shape of loss (4000/24 -> 166). Here the
	 * resolution is 1 pV, so the 22.3517 nV step is carried to 1.3e-5
	 * relative - which is the first thing to offer back upstream after the
	 * timestamp channel.
	 *
	 * This value existing in sysfs is a requirement of the architecture and
	 * not a convenience: nothing in userspace may carry a conversion
	 * constant for a converter it is not allowed to name.
	 */
	step = (u64)vref_uv * MICRO;
	step = div_u64(step, (u64)gain << (ADS1299_BITS_PER_SAMPLE - 1));

	*val = 0;
	*val2 = (int)step;

	return IIO_VAL_INT_PLUS_NANO;
}

static int ads1299_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ads1299_private *priv = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = ads1299_read_one(priv, chan->scan_index);

		iio_device_release_direct_mode(indio_dev);

		if (ret)
			return ret;

		*val = sign_extend32(get_unaligned_be24(priv->rx_buffer + chan->address),
				     ADS1299_BITS_PER_SAMPLE - 1);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		return ads1299_get_scale(priv, chan->channel, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ads1299_get_samp_freq(priv, val);
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = ads1299_get_gain(priv, 0);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ads1299_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = ads1299_samp_freq_avail;
		*length = ARRAY_SIZE(ads1299_samp_freq_avail);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*vals = ads1299_pga_gain;
		*length = ARRAY_SIZE(ads1299_pga_gain);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ads1299_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct ads1299_private *priv = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ads1299_set_samp_freq(priv, val);
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return ads1299_set_gain(priv, val);
	default:
		return -EINVAL;
	}
}

static int ads1299_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ads1299_private *priv = context;
	struct spi_transfer reg_write_xfer = {
		.tx_buf = priv->cmd_buffer,
		.rx_buf = priv->cmd_buffer,
		.len = 3,
		.speed_hz = ADS1299_SPI_BUS_SPEED_SLOW,
		.delay = {
			.value = 2,
			.unit = SPI_DELAY_UNIT_USECS,
		},
	};

	priv->cmd_buffer[0] = ADS1299_CMD_WREG | reg;
	priv->cmd_buffer[1] = 0; /* Number of registers to be written - 1 */
	priv->cmd_buffer[2] = val;

	return spi_sync_transfer(priv->spi, &reg_write_xfer, 1);
}

static int ads1299_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ads1299_private *priv = context;
	struct spi_transfer reg_read_xfer = {
		.tx_buf = priv->cmd_buffer,
		.rx_buf = priv->cmd_buffer,
		.len = 3,
		.speed_hz = ADS1299_SPI_BUS_SPEED_SLOW,
		.delay = {
			.value = 2,
			.unit = SPI_DELAY_UNIT_USECS,
		},
	};
	int ret;

	priv->cmd_buffer[0] = ADS1299_CMD_RREG | reg;
	priv->cmd_buffer[1] = 0; /* Number of registers to be read - 1 */
	priv->cmd_buffer[2] = 0;

	ret = spi_sync_transfer(priv->spi, &reg_read_xfer, 1);
	if (ret)
		return ret;

	*val = priv->cmd_buffer[2];

	return 0;
}

static int ads1299_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval, unsigned int *readval)
{
	struct ads1299_private *priv = iio_priv(indio_dev);

	if (readval)
		return regmap_read(priv->regmap, reg, readval);

	return regmap_write(priv->regmap, reg, writeval);
}

static void ads1299_rdata_unmark_busy(struct ads1299_private *priv)
{
	guard(spinlock_irqsave)(&priv->irq_busy_lock);
	priv->rdata_xfer_busy = 0;
}

static int ads1299_update_scan_mode(struct iio_dev *indio_dev,
				    const unsigned long *scan_mask)
{
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int val;
	unsigned int i;
	int ret;

	/* Make the interrupt routines start with a clean slate */
	ads1299_rdata_unmark_busy(priv);

	/*
	 * Only the ADC channels have a power-down bit; the timestamp channel
	 * sits past them in the scan mask and has no register.
	 */
	for (i = 0; i < priv->num_adc_channels; i++) {
		val = test_bit(i, scan_mask) ? 0 : ADS1299_MASK_CH_PD;
		ret = regmap_update_bits(priv->regmap, ADS1299_REG_CHnSET(i),
					 ADS1299_MASK_CH_PD, val);
		if (ret)
			return ret;
	}

	return 0;
}

/* --------------------------------------------------------------------------
 * Device attributes beyond the IIO ABI
 *
 * IIO has no standard ABI for "connect every channel to the internal test
 * generator" or for "which electrodes are detached". These are custom
 * attributes for bench work and for the record, and the fact that they are
 * custom is one of the things to settle before any of this is offered
 * upstream. The self test does NOT depend on them: it runs at probe, below.
 * --------------------------------------------------------------------------
 */

static ssize_t input_mux_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int regval;
	unsigned int mux;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_CHnSET(0), &regval);
	if (ret)
		return ret;

	mux = FIELD_GET(ADS1299_MASK_CH_MUX, regval);

	return sysfs_emit(buf, "%s\n", ads1299_mux_names[mux]);
}

static ssize_t input_mux_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	int ret;

	ret = sysfs_match_string(ads1299_mux_names, buf);
	if (ret < 0)
		return ret;

	ret = ads1299_set_mux(priv, (unsigned int)ret);
	if (ret)
		return ret;

	return (ssize_t)len;
}

static ssize_t input_mux_available_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	size_t len = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ads1299_mux_names); i++)
		len += sysfs_emit_at(buf, len, "%s%s", i ? " " : "",
				     ads1299_mux_names[i]);

	len += sysfs_emit_at(buf, len, "\n");

	return (ssize_t)len;
}

static ssize_t test_signal_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_CONFIG2, &regval);
	if (ret)
		return ret;

	if (!(regval & ADS1299_MASK_CONFIG2_INT_CAL))
		return sysfs_emit(buf, "off\n");

	return sysfs_emit(buf, "%s_%s\n",
			  regval & ADS1299_MASK_CONFIG2_CAL_AMP ? "2x" : "1x",
			  FIELD_GET(ADS1299_MASK_CONFIG2_CAL_FREQ, regval) ==
				  ADS1299_CAL_FREQ_FAST ? "fast" : "slow");
}

static ssize_t test_signal_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	static const char * const options[] = {
		"off", "1x_slow", "1x_fast", "2x_slow", "2x_fast",
	};
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int val = ADS1299_CONFIG2_RESERVED_VALUE;
	int choice;
	int ret;

	choice = sysfs_match_string(options, buf);
	if (choice < 0)
		return choice;

	if (choice > 0) {
		val |= ADS1299_MASK_CONFIG2_INT_CAL;
		if (choice >= 3)
			val |= ADS1299_MASK_CONFIG2_CAL_AMP;
		if (choice == 2 || choice == 4)
			val |= FIELD_PREP(ADS1299_MASK_CONFIG2_CAL_FREQ,
					  ADS1299_CAL_FREQ_FAST);
	}

	ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG2, val);
	if (ret)
		return ret;

	return (ssize_t)len;
}

static ssize_t lead_off_status_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int sensp;
	unsigned int sensn;
	unsigned int statp;
	unsigned int statn;
	int ret;

	/*
	 * A status bit is only valid where its channel has lead-off sensing
	 * enabled. Reporting the raw registers would print 00 00 - every
	 * electrode attached - on a device where detection was never switched
	 * on.
	 */
	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_SENSP, &sensp);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_SENSN, &sensn);
	if (ret)
		return ret;

	if (!sensp && !sensn)
		return sysfs_emit(buf, "disabled\n");

	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_STATP, &statp);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_STATN, &statn);
	if (ret)
		return ret;

	/* One bit per electrode, positive then negative, valid bits only. */
	return sysfs_emit(buf, "%02x %02x\n", statp & sensp, statn & sensn);
}

static ssize_t lost_samples_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);

	return sysfs_emit(buf, "%llu\n", priv->lost_samples);
}

static ssize_t timestamp_source_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);

	return sysfs_emit(buf, "%s\n",
			  priv->ts_source == ADS1299_TS_DRDY_IRQ ?
				  "drdy_irq" : "host_timer");
}

static IIO_DEVICE_ATTR_RW(input_mux, 0);
static IIO_DEVICE_ATTR_RO(input_mux_available, 0);
static IIO_DEVICE_ATTR_RW(test_signal, 0);
static IIO_DEVICE_ATTR_RO(lead_off_status, 0);
static IIO_DEVICE_ATTR_RO(lost_samples, 0);
static IIO_DEVICE_ATTR_RO(timestamp_source, 0);

static struct attribute *ads1299_attributes[] = {
	&iio_dev_attr_input_mux.dev_attr.attr,
	&iio_dev_attr_input_mux_available.dev_attr.attr,
	&iio_dev_attr_test_signal.dev_attr.attr,
	&iio_dev_attr_lead_off_status.dev_attr.attr,
	&iio_dev_attr_lost_samples.dev_attr.attr,
	&iio_dev_attr_timestamp_source.dev_attr.attr,
	NULL
};

static const struct attribute_group ads1299_attribute_group = {
	.attrs = ads1299_attributes,
};

static const struct iio_info ads1299_info = {
	.read_raw = &ads1299_read_raw,
	.read_avail = &ads1299_read_avail,
	.write_raw = &ads1299_write_raw,
	.update_scan_mode = &ads1299_update_scan_mode,
	.debugfs_reg_access = &ads1299_reg_access,
	.attrs = &ads1299_attribute_group,
};

static void ads1299_rdata_release_busy_or_restart(struct ads1299_private *priv)
{
	guard(spinlock_irqsave)(&priv->irq_busy_lock);

	if (priv->rdata_xfer_busy > 1) {
		/*
		 * A sample became due before the SPI transfer completed. Start
		 * another transfer now to fetch what the chip has not latched
		 * out yet.
		 */
		spi_async(priv->spi, &priv->rdata_msg);
		/*
		 * More than one means samples were lost. Count them - this is
		 * the number RESULTS.md §9 is missing - and reset to 1 so the
		 * handshake resynchronises.
		 */
		if (priv->rdata_xfer_busy > 2)
			priv->lost_samples += priv->rdata_xfer_busy - 2;
		priv->rdata_xfer_busy = 1;
	} else {
		priv->rdata_xfer_busy = 0;
	}
}

/* Called from SPI completion interrupt handler */
static void ads1299_rdata_complete(void *context)
{
	struct iio_dev *indio_dev = context;
	struct ads1299_private *priv = iio_priv(indio_dev);
	int scan_index;
	u32 *bounce = priv->scan.chan;
	s64 timestamp;

	if (!iio_buffer_enabled(indio_dev)) {
		/*
		 * For a single transfer we are kept in direct mode until
		 * completion, avoiding a race with buffered IO.
		 */
		ads1299_rdata_unmark_busy(priv);
		complete(&priv->completion);
		return;
	}

	/*
	 * Taken here, at SPI completion, and not when the frame is handed to
	 * userspace. On the hat link that is one SPI transfer after the DRDY
	 * edge; on the USB link it is one bus frame and one host wakeup after
	 * the conversion, and no arithmetic in this driver can recover the
	 * difference. Which of the two produced a record is declared by
	 * timestamp_source, and carried into the record by the acquisition
	 * service as acquisition.link.
	 */
	timestamp = iio_get_time_ns(indio_dev);

	for_each_set_bit(scan_index, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		const struct iio_chan_spec *scan_chan =
					&indio_dev->channels[scan_index];
		const u8 *data;

		if (scan_chan->type == IIO_TIMESTAMP)
			continue;

		data = priv->rx_buffer + scan_chan->address;
		*bounce++ = get_unaligned_be24(data);
	}

	/* rx_buffer can be overwritten from this point on */
	ads1299_rdata_release_busy_or_restart(priv);

	iio_push_to_buffers_with_timestamp(indio_dev, &priv->scan, timestamp);
}

static void ads1299_sample_due(struct ads1299_private *priv)
{
	unsigned int wasbusy;

	guard(spinlock_irqsave)(&priv->irq_busy_lock);

	wasbusy = priv->rdata_xfer_busy++;
	/* When no SPI transfer is in transit, start one now */
	if (!wasbusy)
		spi_async(priv->spi, &priv->rdata_msg);
}

static irqreturn_t ads1299_interrupt(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;

	ads1299_sample_due(iio_priv(indio_dev));

	return IRQ_HANDLED;
}

/*
 * The host-timed path, for a link that cannot deliver DRDY.
 *
 * This is deliberately NOT dressed up as equivalent to the interrupt path.
 * The converter still decides when a sample exists; this timer only decides
 * when the host asks for one, so the jitter of the whole link lands in the
 * timestamp. That is the measurement the two links exist to compare
 * (implementation_plan_iio_afe.md §9.4), and hiding it here would delete the
 * result.
 */
static enum hrtimer_restart ads1299_timer_fired(struct hrtimer *timer)
{
	struct ads1299_private *priv =
		container_of(timer, struct ads1299_private, timer);

	ads1299_sample_due(priv);
	hrtimer_forward_now(timer, priv->timer_period);

	return priv->timer_running ? HRTIMER_RESTART : HRTIMER_NORESTART;
}

static int ads1299_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ads1299_private *priv = iio_priv(indio_dev);
	int ret;

	priv->lost_samples = 0;

	/* Disable single-shot mode */
	ret = regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG4,
				 ADS1299_MASK_CONFIG4_SINGLE_SHOT, 0);
	if (ret)
		return ret;

	ret = ads1299_write_cmd(priv, ADS1299_CMD_START);
	if (ret)
		return ret;

	if (priv->ts_source == ADS1299_TS_HOST_TIMER) {
		priv->timer_running = true;
		hrtimer_start(&priv->timer, priv->timer_period,
			      HRTIMER_MODE_REL);
	}

	return 0;
}

static int ads1299_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ads1299_private *priv = iio_priv(indio_dev);

	if (priv->ts_source == ADS1299_TS_HOST_TIMER) {
		priv->timer_running = false;
		hrtimer_cancel(&priv->timer);
	}

	return ads1299_write_cmd(priv, ADS1299_CMD_STOP);
}

static const struct iio_buffer_setup_ops ads1299_setup_ops = {
	.postenable = &ads1299_buffer_postenable,
	.predisable = &ads1299_buffer_predisable,
};

static void ads1299_reg_disable(void *reg)
{
	regulator_disable(reg);
}

static const struct regmap_range ads1299_regmap_volatile_range[] = {
	regmap_reg_range(ADS1299_REG_LOFF_STATP, ADS1299_REG_LOFF_STATN),
};

static const struct regmap_access_table ads1299_regmap_volatile = {
	.yes_ranges = ads1299_regmap_volatile_range,
	.n_yes_ranges = ARRAY_SIZE(ads1299_regmap_volatile_range),
};

static const struct regmap_config ads1299_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_read = ads1299_reg_read,
	.reg_write = ads1299_reg_write,
	.max_register = ADS1299_REG_CONFIG4,
	.volatile_table = &ads1299_regmap_volatile,
	.cache_type = REGCACHE_MAPLE,
};

static int ads1299_init(struct iio_dev *indio_dev)
{
	struct ads1299_private *priv = iio_priv(indio_dev);
	struct device *dev = &priv->spi->dev;
	unsigned int val;
	unsigned int i;
	int ret;

	/* The device initialises into RDATAC mode, which we do not want */
	ret = ads1299_write_cmd(priv, ADS1299_CMD_SDATAC);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_ID, &val);
	if (ret)
		return ret;

	if (FIELD_GET(ADS1299_MASK_ID_FAMILY, val) != ADS1299_ID_FAMILY_ADS129X)
		return dev_err_probe(dev, -ENODEV, "Unknown ID: 0x%x\n", val);

	priv->num_adc_channels = 4 + 2 * (val & ADS1299_MASK_ID_CHANNELS);
	if (priv->num_adc_channels > ADS1299_MAX_CHANNELS)
		return dev_err_probe(dev, -ENODEV,
				     "ID reports %u channels\n",
				     priv->num_adc_channels);

	indio_dev->name = devm_kasprintf(dev, GFP_KERNEL, "ads1299-%u",
					 priv->num_adc_channels);
	if (!indio_dev->name)
		return -ENOMEM;

	/* Internal test generator available but not routed to any channel */
	ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG2,
			   ADS1299_CONFIG2_RESERVED_VALUE);
	if (ret)
		return ret;

	/*
	 * Internal reference buffer and the internal bias reference. Without
	 * PWR_REFBUF the converter has no reference at all unless one is wired,
	 * and every channel reads zero.
	 */
	val = ADS1299_CONFIG3_RESERVED_VALUE;
	if (!priv->reg_vref)
		val |= ADS1299_MASK_CONFIG3_PWR_REFBUF |
		       ADS1299_MASK_CONFIG3_BIASREF_INT;
	ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG3, val);
	if (ret)
		return ret;

	/*
	 * CHnSET out of reset is gain 24 with the input SHORTED. Leaving that
	 * alone is how this driver would produce a clean, plausible, entirely
	 * fictional flat trace, so the MUX is programmed explicitly here. The
	 * ADS1298 driver has no equivalent line because its reset value is the
	 * normal input - this is the adaptation most likely to be missed.
	 */
	for (i = 0; i < priv->num_adc_channels; i++) {
		ret = regmap_update_bits(priv->regmap, ADS1299_REG_CHnSET(i),
					 ADS1299_MASK_CH_MUX | ADS1299_MASK_CH_PD,
					 FIELD_PREP(ADS1299_MASK_CH_MUX,
						    ADS1299_MUX_NORMAL));
		if (ret)
			return ret;
	}

	/*
	 * Lead-off comparators powered on. Not the same as lead-off detection
	 * being enabled: that needs LOFF_SENSP/LOFF_SENSN per channel, which
	 * nothing here writes - see the note at their definitions.
	 */
	return regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG4,
				  ADS1299_MASK_CONFIG4_PD_LOFF_COMP,
				  ADS1299_MASK_CONFIG4_PD_LOFF_COMP);
}

/*
 * Take `count` single-shot conversions after discarding the first few, and
 * report the smallest and the largest peak-to-peak swing seen on any channel,
 * in codes. The smallest is what a test signal is judged by - one dead channel
 * is enough to fail - and the largest is what noise is judged by.
 */
static int ads1299_sample_swing(struct ads1299_private *priv,
				unsigned int count, s32 *min_pp, s32 *max_pp)
{
	s32 lo[ADS1299_MAX_CHANNELS];
	s32 hi[ADS1299_MAX_CHANNELS];
	unsigned int reading;
	unsigned int ch;
	int ret;

	for (ch = 0; ch < ADS1299_MAX_CHANNELS; ch++) {
		lo[ch] = S32_MAX;
		hi[ch] = S32_MIN;
	}

	for (reading = 0; reading < count + ADS1299_SELF_TEST_DISCARD; reading++) {
		ret = ads1299_read_one(priv, 0);
		if (ret)
			return ret;

		msleep(ADS1299_SELF_TEST_INTERVAL_MS);

		if (reading < ADS1299_SELF_TEST_DISCARD)
			continue;

		/* One RDATA carries every channel */
		for (ch = 0; ch < priv->num_adc_channels; ch++) {
			s32 v = sign_extend32(get_unaligned_be24(priv->rx_buffer +
						ADS1299_OFFSET_IN_RX_BUFFER(ch)),
					      ADS1299_BITS_PER_SAMPLE - 1);

			lo[ch] = min(lo[ch], v);
			hi[ch] = max(hi[ch], v);
		}
	}

	*min_pp = S32_MAX;
	*max_pp = 0;
	for (ch = 0; ch < priv->num_adc_channels; ch++) {
		*min_pp = min(*min_pp, hi[ch] - lo[ch]);
		*max_pp = max(*max_pp, hi[ch] - lo[ch]);
	}

	return 0;
}

/*
 * Probe-time self test of the analogue path (IEC 60601-1 §14).
 *
 * Step 1 routes the internal generator to every input and requires each
 * channel's swing to be within ADS1299_SELF_TEST_TOLERANCE_PCT of what the
 * gain implies: a dead PGA stage, a stuck gain or a broken reference all fall
 * outside it. Step 2 shorts the inputs and requires the noise floor to be under
 * ADS1299_SELF_TEST_NOISE_UV_PP. Both leave the inputs connected and the
 * generator off, pass or fail.
 *
 * A failure fails the probe. The device is never registered, so no userspace
 * can acquire from a front-end that did not pass - and the numbers that failed
 * are in the kernel log, which is the only place that can say why a front-end
 * is absent.
 */
static int ads1299_self_test(struct ads1299_private *priv)
{
	struct device *dev = &priv->spi->dev;
	s32 min_pp = 0;
	s32 max_pp = 0;
	s32 noise_pp = 0;
	s32 quietest_pp = 0;
	u64 expected;
	u64 window;
	u64 ceiling;
	int vref_uv;
	int gain;
	int restore;
	int ret;

	vref_uv = ads1299_get_vref_uv(priv);
	if (vref_uv <= 0)
		return vref_uv < 0 ? vref_uv : -EINVAL;

	gain = ads1299_get_gain(priv, 0);
	if (gain < 0)
		return gain;

	expected = div_u64((u64)gain << (ADS1299_BITS_PER_SAMPLE - 1),
			   ADS1299_TEST_SIGNAL_PP_DIVISOR);
	window = div_u64(expected * ADS1299_SELF_TEST_TOLERANCE_PCT, 100);
	ceiling = div_u64((u64)ADS1299_SELF_TEST_NOISE_UV_PP * (u64)gain <<
				  (ADS1299_BITS_PER_SAMPLE - 1),
			  (u32)vref_uv);

	/* Step 1 - the internal generator, 1x amplitude, slow */
	ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG2,
			   ADS1299_CONFIG2_RESERVED_VALUE |
			   ADS1299_MASK_CONFIG2_INT_CAL |
			   FIELD_PREP(ADS1299_MASK_CONFIG2_CAL_FREQ,
				      ADS1299_CAL_FREQ_SLOW));
	if (!ret)
		ret = ads1299_set_mux(priv, ADS1299_MUX_TEST_SIGNAL);
	if (!ret)
		ret = ads1299_sample_swing(priv, ADS1299_SELF_TEST_SIGNAL_READINGS,
					   &min_pp, &max_pp);
	if (!ret && ((u64)min_pp + window < expected ||
		     (u64)max_pp > expected + window)) {
		dev_err(dev,
			"self test: test signal swings %d..%d codes peak to peak across channels, expected %llu +-%llu at gain %d\n",
			min_pp, max_pp, expected, window, gain);
		ret = -EIO;
	}

	/* Step 2 - inputs shorted, generator off */
	if (!ret)
		ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG2,
				   ADS1299_CONFIG2_RESERVED_VALUE);
	if (!ret)
		ret = ads1299_set_mux(priv, ADS1299_MUX_SHORTED);
	if (!ret)
		ret = ads1299_sample_swing(priv, ADS1299_SELF_TEST_NOISE_READINGS,
					   &quietest_pp, &noise_pp);
	if (!ret && (u64)noise_pp > ceiling) {
		dev_err(dev,
			"self test: shorted-input noise is %d codes peak to peak, ceiling %llu (%d uV at gain %d)\n",
			noise_pp, ceiling, ADS1299_SELF_TEST_NOISE_UV_PP, gain);
		ret = -EIO;
	}

	/* Pass or fail, the inputs go back to the electrodes */
	restore = regmap_write(priv->regmap, ADS1299_REG_CONFIG2,
			       ADS1299_CONFIG2_RESERVED_VALUE);
	if (!restore)
		restore = ads1299_set_mux(priv, ADS1299_MUX_NORMAL);

	if (ret)
		return ret;
	if (restore)
		return restore;

	dev_info(dev,
		 "self test passed: test signal %d..%d codes peak to peak (expected %llu), shorted-input noise %d codes\n",
		 min_pp, max_pp, expected, noise_pp);

	return 0;
}

static int ads1299_build_channels(struct iio_dev *indio_dev,
				  struct ads1299_private *priv)
{
	struct device *dev = &priv->spi->dev;
	struct iio_chan_spec *channels;
	unsigned int n = priv->num_adc_channels;

	/* n voltage channels plus one timestamp */
	channels = devm_kcalloc(dev, n + 1, sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	memcpy(channels, ads1299_channels, n * sizeof(*channels));
	channels[n] = (struct iio_chan_spec)IIO_CHAN_SOFT_TIMESTAMP(n);

	indio_dev->channels = channels;
	indio_dev->num_channels = (int)(n + 1);

	return 0;
}

static int ads1299_probe(struct spi_device *spi)
{
	struct ads1299_private *priv;
	struct iio_dev *indio_dev;
	struct device *dev = &spi->dev;
	struct gpio_desc *reset_gpio;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);

	/* Reset to be asserted before enabling clock and power */
	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(dev, PTR_ERR(reset_gpio),
				     "Cannot get reset GPIO\n");

	/* VREF can be supplied externally, otherwise use the internal 4.5 V */
	priv->reg_vref = devm_regulator_get_optional(dev, "vref");
	if (IS_ERR(priv->reg_vref)) {
		if (PTR_ERR(priv->reg_vref) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(priv->reg_vref),
					     "Failed to get vref regulator\n");

		priv->reg_vref = NULL;
	} else {
		ret = regulator_enable(priv->reg_vref);
		if (ret)
			return ret;

		ret = devm_add_action_or_reset(dev, ads1299_reg_disable,
					       priv->reg_vref);
		if (ret)
			return ret;
	}

	priv->clk = devm_clk_get_optional_enabled(dev, "clk");
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk), "Failed to get clk\n");

	priv->reg_avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(priv->reg_avdd))
		return dev_err_probe(dev, PTR_ERR(priv->reg_avdd),
				     "Failed to get avdd regulator\n");

	ret = regulator_enable(priv->reg_avdd);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable avdd regulator\n");

	ret = devm_add_action_or_reset(dev, ads1299_reg_disable, priv->reg_avdd);
	if (ret)
		return ret;

	priv->spi = spi;
	init_completion(&priv->completion);
	spin_lock_init(&priv->irq_busy_lock);
	priv->regmap = devm_regmap_init(dev, NULL, priv, &ads1299_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	/*
	 * Which link this is, decided by whether the board could route DRDY to
	 * an interrupt. On the hat the answer is yes and the timestamp is one
	 * SPI transfer from the conversion; behind a USB-SPI bridge the answer
	 * is structurally no - HID delivers no asynchronous interrupt to a host
	 * at all - and the driver falls back to asking on a timer.
	 */
	priv->ts_source = spi->irq > 0 ? ADS1299_TS_DRDY_IRQ
				       : ADS1299_TS_HOST_TIMER;

	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->info = &ads1299_info;

	if (reset_gpio) {
		/*
		 * Deassert reset now that clock and power are active.
		 * Minimum reset pulsewidth is 2 clock cycles.
		 */
		fsleep(ADS1299_CLOCKS_TO_USECS(2));
		gpiod_set_value_cansleep(reset_gpio, 0);
	} else {
		ret = ads1299_write_cmd(priv, ADS1299_CMD_RESET);
		if (ret)
			return dev_err_probe(dev, ret, "RESET failed\n");
	}
	/* Wait 18 clock cycles for the reset command to complete */
	fsleep(ADS1299_CLOCKS_TO_USECS(18));

	ret = ads1299_init(indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Init failed\n");

	ret = ads1299_build_channels(indio_dev, priv);
	if (ret)
		return ret;

	priv->tx_buffer[0] = ADS1299_CMD_RDATA;
	priv->rdata_xfer.tx_buf = priv->tx_buffer;
	priv->rdata_xfer.rx_buf = priv->rx_buffer;
	priv->rdata_xfer.len =
		ADS1299_SPI_RDATA_BUFFER_SIZE(priv->num_adc_channels);
	/* Must keep CS low for 4 clocks */
	priv->rdata_xfer.delay.value = 2;
	priv->rdata_xfer.delay.unit = SPI_DELAY_UNIT_USECS;
	spi_message_init_with_transfers(&priv->rdata_msg, &priv->rdata_xfer, 1);
	priv->rdata_msg.complete = &ads1299_rdata_complete;
	priv->rdata_msg.context = indio_dev;

	if (priv->ts_source == ADS1299_TS_DRDY_IRQ) {
		ret = devm_request_irq(dev, spi->irq, &ads1299_interrupt,
				       IRQF_TRIGGER_FALLING, indio_dev->name,
				       indio_dev);
		if (ret)
			return ret;
	} else {
		int freq;

		hrtimer_init(&priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		priv->timer.function = ads1299_timer_fired;

		if (ads1299_get_samp_freq(priv, &freq) == IIO_VAL_INT && freq > 0)
			priv->timer_period = ns_to_ktime(div_u64(NSEC_PER_SEC, (u32)freq));
		else
			priv->timer_period = ns_to_ktime(div_u64(NSEC_PER_SEC, 250));

		dev_info(dev,
			 "no DRDY interrupt: sampling on a host timer, timestamps are host side\n");
	}

	ret = devm_iio_kfifo_buffer_setup(dev, indio_dev, &ads1299_setup_ops);
	if (ret)
		return ret;

	/*
	 * Last, once conversions can complete on either link and before anything
	 * can see the device. A front-end that fails is not registered at all -
	 * userspace finds no device, and the log says why.
	 */
	ret = ads1299_self_test(priv);
	if (ret)
		return dev_err_probe(dev, ret,
				     "self test failed, front-end not registered\n");

	return devm_iio_device_register(dev, indio_dev);
}

static const struct spi_device_id ads1299_id[] = {
	{ "ads1299" },
	{ "ads1299-4" },
	{ "ads1299-6" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ads1299_id);

static const struct of_device_id ads1299_of_table[] = {
	{ .compatible = "ti,ads1299" },
	{ .compatible = "ti,ads1299-4" },
	{ .compatible = "ti,ads1299-6" },
	{ }
};
MODULE_DEVICE_TABLE(of, ads1299_of_table);

static struct spi_driver ads1299_driver = {
	.driver = {
		.name	= "ads1299",
		.of_match_table = ads1299_of_table,
	},
	.probe		= ads1299_probe,
	.id_table	= ads1299_id,
};
module_spi_driver(ads1299_driver);

MODULE_AUTHOR("Mike Looijmans <mike.looijmans@topic.nl>");
MODULE_AUTHOR("Stephan Costa Barros <stephanbrrs8@gmail.com>");
MODULE_DESCRIPTION("TI ADS1299 biopotential front-end, derived from ti-ads1298");
MODULE_LICENSE("GPL");
