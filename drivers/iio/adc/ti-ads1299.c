// SPDX-License-Identifier: GPL-2.0
/*
 * TI ADS1299 8-channel, 24-bit biopotential ADC
 *
 * Copyright (C) 2023 - 2024 Topic Embedded Products
 * Copyright (C) 2026 Stephan Costa Barros <stephanbrrs8@gmail.com>
 *
 * Based on ti-ads1298.c by Mike Looijmans <mike.looijmans@topic.nl>.
 *
 * Datasheet: https://www.ti.com/lit/ds/symlink/ads1299.pdf
 *
 * Besides the voltage channels, the driver provides:
 *  - a timestamp channel, taken in the DRDY interrupt or, where DRDY cannot
 *    reach the host as an interrupt, from an internal hrtimer; the source in
 *    use is reported by "timestamp_source"
 *  - "lost_samples", the conversions the host did not collect
 *  - a probe-time self test of the analogue path: shorted-input noise and
 *    offset, then the test generator held at DC; a part that fails is not
 *    registered
 *  - input MUX, test signal and bias drive controls, and the lead-off status
 *  - the CLK pin as a clock output, for other converters to run from
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/log2.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/property.h>
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

/* Commands */
#define ADS1299_CMD_WAKEUP			0x02
#define ADS1299_CMD_STANDBY			0x04
#define ADS1299_CMD_RESET			0x06
#define ADS1299_CMD_START			0x08
#define ADS1299_CMD_STOP			0x0a
#define ADS1299_CMD_RDATAC			0x10
#define ADS1299_CMD_SDATAC			0x11
#define ADS1299_CMD_RDATA			0x12
#define ADS1299_CMD_RREG			0x20
#define ADS1299_CMD_WREG			0x40

/*
 * [SBAS499 Table 12] ID: REV_ID[7:5] | 1 | DEV_ID[3:2] | NU_CH[1:0], 0x3e on
 * the 8-channel part. REV_ID may change between revisions, so the byte is
 * masked and never compared whole. NU_CH: 00 = 4, 01 = 6, 10 = 8 channels,
 * 11 undefined.
 */
#define ADS1299_REG_ID				0x00
#define ADS1299_MASK_REV_ID			GENMASK(7, 5)
#define ADS1299_MASK_ID_FAMILY			GENMASK(4, 2)
#define ADS1299_MASK_ID_CHANNELS		GENMASK(1, 0)
#define ADS1299_ID_FAMILY_ADS129X		0x7

/*
 * [SBAS499 Table 13] CONFIG1, reset 0x96. Reserved: bit 7 is always 1 and
 * bits 4:3 always 2h. ODR = f_CLK >> (7 + DR): 16 kSPS at DR = 0, 250 SPS at
 * DR = 6; DR = 7 is reserved.
 */
#define ADS1299_REG_CONFIG1			0x01
#define ADS1299_MASK_CONFIG1_RESERVED_HI	BIT(7)
#define ADS1299_MASK_CONFIG1_DAISY_EN		BIT(6)
#define ADS1299_MASK_CONFIG1_CLK_EN		BIT(5)
#define ADS1299_MASK_CONFIG1_RESERVED_LO	GENMASK(4, 3)
#define ADS1299_MASK_CONFIG1_DR			GENMASK(2, 0)
#define ADS1299_CONFIG1_RESERVED_VALUE					\
	(ADS1299_MASK_CONFIG1_RESERVED_HI |				\
	 FIELD_PREP(ADS1299_MASK_CONFIG1_RESERVED_LO, 0x2))
#define ADS1299_SHIFT_DR			7
#define ADS1299_LOWEST_DR			0x06

/*
 * [SBAS499 Table 14] CONFIG2, reset 0xc0. Reserved: bits 7:5 are always 6h,
 * a value and not a mask of ones, and bit 3 is always 0.
 */
#define ADS1299_REG_CONFIG2			0x02
#define ADS1299_MASK_CONFIG2_RESERVED		GENMASK(7, 5)
#define ADS1299_MASK_CONFIG2_INT_CAL		BIT(4)
#define ADS1299_MASK_CONFIG2_CAL_AMP		BIT(2)
#define ADS1299_MASK_CONFIG2_CAL_FREQ		GENMASK(1, 0)
#define ADS1299_CONFIG2_RESERVED_VALUE					\
	FIELD_PREP(ADS1299_MASK_CONFIG2_RESERVED, 0x6)
#define ADS1299_CAL_FREQ_SLOW			0	/* f_CLK / 2^21, ~1 Hz */
#define ADS1299_CAL_FREQ_FAST			1	/* f_CLK / 2^20, ~2 Hz */
/* 2 is "do not use" */
#define ADS1299_CAL_FREQ_DC			3

/*
 * [SBAS499 Table 15] CONFIG3, reset 0x60. Reserved bits 6:5 are always 3h.
 * PWR_REFBUF is the datasheet's PD_REFBUF, renamed because 1 powers the buffer
 * up. BIAS_STAT is read-only, and CONFIG3 is cached: reading it needs the
 * register in the volatile table.
 */
#define ADS1299_REG_CONFIG3			0x03
#define ADS1299_MASK_CONFIG3_PWR_REFBUF		BIT(7)
#define ADS1299_MASK_CONFIG3_RESERVED		GENMASK(6, 5)
#define ADS1299_MASK_CONFIG3_BIAS_MEAS		BIT(4)
#define ADS1299_MASK_CONFIG3_BIASREF_INT	BIT(3)
#define ADS1299_MASK_CONFIG3_PD_BIAS		BIT(2)
#define ADS1299_MASK_CONFIG3_BIAS_LOFF_SENS	BIT(1)
#define ADS1299_MASK_CONFIG3_BIAS_STAT		BIT(0)
#define ADS1299_CONFIG3_RESERVED_VALUE					\
	FIELD_PREP(ADS1299_MASK_CONFIG3_RESERVED, 0x3)

/*
 * [SBAS499 Table 16] LOFF, reset 0x00. Bit 4 is reserved, always 0. One
 * COMP_TH code sets both comparators: positive % / negative %.
 */
#define ADS1299_REG_LOFF			0x04
#define ADS1299_MASK_LOFF_COMP_TH		GENMASK(7, 5)
#define ADS1299_MASK_LOFF_ILEAD_OFF		GENMASK(3, 2)
#define ADS1299_MASK_LOFF_FLEAD_OFF		GENMASK(1, 0)
#define ADS1299_LOFF_COMP_TH_95_5		0
#define ADS1299_LOFF_COMP_TH_92_5_7_5		1
#define ADS1299_LOFF_COMP_TH_90_10		2
#define ADS1299_LOFF_COMP_TH_87_5_12_5		3
#define ADS1299_LOFF_COMP_TH_85_15		4
#define ADS1299_LOFF_COMP_TH_80_20		5
#define ADS1299_LOFF_COMP_TH_75_25		6
#define ADS1299_LOFF_COMP_TH_70_30		7
#define ADS1299_LOFF_ILEAD_OFF_6NA		0
#define ADS1299_LOFF_ILEAD_OFF_24NA		1
#define ADS1299_LOFF_ILEAD_OFF_6UA		2
#define ADS1299_LOFF_ILEAD_OFF_24UA		3
#define ADS1299_LOFF_FLEAD_OFF_DC		0
#define ADS1299_LOFF_FLEAD_OFF_AC_7_8HZ		1	/* f_CLK / 2^18 */
#define ADS1299_LOFF_FLEAD_OFF_AC_31_2HZ	2	/* f_CLK / 2^16 */
#define ADS1299_LOFF_FLEAD_OFF_AC_FDR_DIV_4	3

/* [SBAS499 Table 17] CHnSET, n = 1 to 8, reset 0x61 */
#define ADS1299_REG_CHnSET(n)			(0x05 + (n))
#define ADS1299_MASK_CH_PD			BIT(7)
#define ADS1299_MASK_CH_PGA			GENMASK(6, 4)
#define ADS1299_MASK_CH_SRB2			BIT(3)
#define ADS1299_MASK_CH_MUX			GENMASK(2, 0)
#define ADS1299_MUX_NORMAL			0
#define ADS1299_MUX_SHORTED			1
#define ADS1299_MUX_BIAS_MEAS			2	/* with CONFIG3.BIAS_MEAS */
#define ADS1299_MUX_MVDD			3
#define ADS1299_MUX_TEMPERATURE			4
#define ADS1299_MUX_TEST_SIGNAL			5
#define ADS1299_MUX_BIAS_DRP			6	/* the positive electrode drives */
#define ADS1299_MUX_BIAS_DRN			7	/* the negative electrode drives */

/*
 * [SBAS499 Tables 18 to 24] One bit per channel. Bits above the part's channel
 * count must be written 0, so masks are derived from NU_CH. A LOFF_STAT bit is
 * valid only where its LOFF_SENS bit is set.
 */
#define ADS1299_REG_BIAS_SENSP			0x0d
#define ADS1299_REG_BIAS_SENSN			0x0e
#define ADS1299_REG_LOFF_SENSP			0x0f
#define ADS1299_REG_LOFF_SENSN			0x10
#define ADS1299_REG_LOFF_FLIP			0x11
#define ADS1299_REG_LOFF_STATP			0x12
#define ADS1299_REG_LOFF_STATN			0x13

/* [SBAS499 Table 25] GPIO, reset 0x0f. GPIOC: 0 output, 1 input */
#define ADS1299_REG_GPIO			0x14
#define ADS1299_MASK_GPIO_DATA			GENMASK(7, 4)
#define ADS1299_MASK_GPIO_CONTROL		GENMASK(3, 0)

/*
 * [SBAS499 Table 26] MISC1, reset 0x00. SRB1 ties the SRB1 pin to every
 * channel's inverting input; the other bits are reserved, always 0.
 */
#define ADS1299_REG_MISC1			0x15
#define ADS1299_MASK_MISC1_SRB1			BIT(5)

/* [SBAS499 Table 27] MISC2, reset 0x00, every bit reserved, always 0 */
#define ADS1299_REG_MISC2			0x16

/*
 * [SBAS499 Table 28] CONFIG4, reset 0x00. PD_LOFF_COMP keeps the datasheet's
 * name although 1 powers the comparators up.
 */
#define ADS1299_REG_CONFIG4			0x17
#define ADS1299_MASK_CONFIG4_SINGLE_SHOT	BIT(3)
#define ADS1299_MASK_CONFIG4_PD_LOFF_COMP	BIT(1)

/*
 * Bias amplifier mode:
 *   off       - powered down, the reset state
 *   reference - drives BIASREF, (AVDD + AVSS) / 2: a DC bias, no feedback
 *   derived   - drives the inverted average of the electrodes' inputs, which
 *               is what rejects common mode
 */
enum ads1299_bias {
	ADS1299_BIAS_OFF,
	ADS1299_BIAS_REFERENCE,
	ADS1299_BIAS_DERIVED,
};

static const char * const ads1299_bias_names[] = {
	[ADS1299_BIAS_OFF]	 = "off",
	[ADS1299_BIAS_REFERENCE] = "reference",
	[ADS1299_BIAS_DERIVED]	 = "derived",
};

#define ADS1299_MAX_CHANNELS			8
#define ADS1299_BITS_PER_SAMPLE			24
#define ADS1299_CLK_RATE_HZ			2048000
#define ADS1299_CLOCKS_TO_USECS(x)					\
	(DIV_ROUND_UP((x) * MICROHZ_PER_HZ, ADS1299_CLK_RATE_HZ))

/* A command takes 4 t_CLK to decode, so register access runs at f_CLK */
#define ADS1299_SPI_BUS_SPEED_SLOW		ADS1299_CLK_RATE_HZ
#define ADS1299_SPI_CMD_BUFFER_SIZE		3
/* Command byte, status word and n 24-bit samples */
#define ADS1299_SPI_RDATA_BUFFER_SIZE(n)	(((n) + 1) * 3 + 1)
#define ADS1299_SPI_RDATA_BUFFER_SIZE_MAX				\
	ADS1299_SPI_RDATA_BUFFER_SIZE(ADS1299_MAX_CHANNELS)

/*
 * Probe-time self test. The test signal is VREF / 2400 peak [SBAS499 Table 14,
 * measured]; held at DC it sits gain * 2^23 / 2400 codes below the
 * shorted-input offset (83886 at gain 24, 83598 measured). DC and not the
 * square wave, because in single-shot mode every START restarts the
 * generator's divider and each reading lands on the same phase.
 */
#define ADS1299_TEST_SIGNAL_DIVISOR		2400
/*
 * +-20% of gain * 2^23 / 2400 codes. VREF cancels out of that ratio, so a
 * wrong VREF is not seen, only a missing one. What is left is the generator
 * and the PGA [SBAS499 7.5: +-2% typical, gain error +-0.5% max], well inside,
 * and a channel stuck at the next gain down from 24 (-50%), well outside.
 */
#define ADS1299_SELF_TEST_TOLERANCE_PCT		20
/* Shorted-input ceiling: catches a broken analogue path, not a noisy board */
#define ADS1299_SELF_TEST_NOISE_UV_PP		10
#define ADS1299_SELF_TEST_READINGS		10
/* Conversions right after a MUX change carry its transient */
#define ADS1299_SELF_TEST_DISCARD		2
#define ADS1299_SELF_TEST_INTERVAL_MS		50

/* Internal reference, used when no "vref" regulator is given */
#define ADS1299_INTERNAL_VREF_UV		4500000

enum ads1299_timestamp_source {
	ADS1299_TS_DRDY_IRQ = 0,
	ADS1299_TS_HOST_TIMER,
};

struct ads1299_private {
	struct spi_device *spi;
	struct regulator *reg_avdd;
	struct regulator *reg_vref;
	struct clk *clk;
	struct clk_hw clk_out;
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
	 * Conversions the host never collected. Without DRDY as an interrupt
	 * this is the only way to tell a missed conversion from a slow one.
	 */
	u64 lost_samples;

	/* Number of ADC channels the chip reported; excludes the timestamp */
	unsigned int num_adc_channels;
	enum ads1299_bias bias;

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

/* Output data rates in DR order; the part has nothing in between */
static const int ads1299_samp_freq_avail[] = {
	16000, 8000, 4000, 2000, 1000, 500, 250,
};

/*
 * [SBAS499 Table 17] PGA field -> gain. 111 is "do not use", hence seven
 * entries.
 */
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

/* One gain for every channel */
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

/* The MUX applies to every channel at once */
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
	 * At the lowest rate a conversion takes 4 ms, and a bus behind a bridge
	 * adds a few more. 100 ms is generous, and short enough that a stuck
	 * part is reported rather than hanging a sysfs read.
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

	/* Exact rates only: 300 SPS is refused, not rounded to 250 */
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
	 * Computed in 64 bits and returned as INT_PLUS_NANO: FRACTIONAL_LOG2
	 * over an integer millivolt full scale would truncate 4.5 V / 24 =
	 * 187.5 mV to 187, a 0.27% gain error.
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
 * generator" or for "which electrodes are detached", so these are custom
 * attributes. The self test does not depend on them.
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

/*
 * Program the bias amplifier. Caller holds nothing; regmap serialises the
 * register access and the three writes are not required to be atomic with
 * respect to each other, because the derivation registers only matter while
 * PD_BIAS is set and that bit is written last on the way up and first on the
 * way down.
 *
 * BIASREF_INT travels with the bias and not with the ADC reference. It used to
 * be written at probe alongside PWR_REFBUF, conditioned on whether an external
 * reference regulator exists - which conflated two different references: bit 7
 * is the ADC's reference buffer and bit 3 is the bias amplifier's reference.
 * An external VREF says nothing about how BIASREF is fed. With the amplifier
 * powered down the bit had no effect, so nothing was wrong on any board; it
 * would have become wrong here.
 */
static int ads1299_set_bias(struct ads1299_private *priv,
			    enum ads1299_bias mode)
{
	unsigned int derivation = 0;
	unsigned int config3 = 0;
	int ret;

	if (mode == ADS1299_BIAS_DERIVED)
		derivation = GENMASK(priv->num_adc_channels - 1, 0);
	if (mode != ADS1299_BIAS_OFF)
		config3 = ADS1299_MASK_CONFIG3_BIASREF_INT |
			  ADS1299_MASK_CONFIG3_PD_BIAS;

	if (mode == ADS1299_BIAS_OFF) {
		ret = regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG3,
					 ADS1299_MASK_CONFIG3_BIASREF_INT |
					 ADS1299_MASK_CONFIG3_PD_BIAS, 0);
		if (ret)
			return ret;
	}

	ret = regmap_write(priv->regmap, ADS1299_REG_BIAS_SENSP, derivation);
	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, ADS1299_REG_BIAS_SENSN, derivation);
	if (ret)
		return ret;

	if (mode != ADS1299_BIAS_OFF) {
		ret = regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG3,
					 ADS1299_MASK_CONFIG3_BIASREF_INT |
					 ADS1299_MASK_CONFIG3_PD_BIAS, config3);
		if (ret)
			return ret;
	}

	priv->bias = mode;

	return 0;
}

static ssize_t bias_drive_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);

	return sysfs_emit(buf, "%s\n", ads1299_bias_names[priv->bias]);
}

static ssize_t bias_drive_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	int mode;
	int ret;

	mode = sysfs_match_string(ads1299_bias_names, buf);
	if (mode < 0)
		return mode;

	/* Not while running: the samples would change meaning mid-stream */
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = ads1299_set_bias(priv, mode);

	iio_device_release_direct_mode(indio_dev);

	return ret ? ret : (ssize_t)len;
}

static ssize_t bias_drive_available_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	size_t len = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ads1299_bias_names); i++)
		len += sysfs_emit_at(buf, len, "%s%s", i ? " " : "",
				     ads1299_bias_names[i]);

	return len + sysfs_emit_at(buf, len, "\n");
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
static IIO_DEVICE_ATTR_RW(bias_drive, 0);
static IIO_DEVICE_ATTR_RO(bias_drive_available, 0);
static IIO_DEVICE_ATTR_RO(lead_off_status, 0);
static IIO_DEVICE_ATTR_RO(lost_samples, 0);
static IIO_DEVICE_ATTR_RO(timestamp_source, 0);

static struct attribute *ads1299_attributes[] = {
	&iio_dev_attr_input_mux.dev_attr.attr,
	&iio_dev_attr_input_mux_available.dev_attr.attr,
	&iio_dev_attr_test_signal.dev_attr.attr,
	&iio_dev_attr_bias_drive.dev_attr.attr,
	&iio_dev_attr_bias_drive_available.dev_attr.attr,
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
		 * More than one means samples were lost. Count them and reset
		 * to 1 so the handshake resynchronises.
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
	 * Taken at SPI completion: one transfer after the DRDY edge, or, on the
	 * host timer, one bus round trip after the conversion. timestamp_source
	 * says which.
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
 * The host-timed path, when DRDY is not an interrupt. The converter still
 * decides when a sample exists; the timer only decides when the host asks for
 * it, so the bus's jitter lands in the timestamp.
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

/*
 * The CLK pin as a clock output [SBAS499 Table 6]: the internal oscillator,
 * driven out while CLK_EN is set. Prepare rather than enable, because the bit
 * is written over SPI, which sleeps.
 */
static unsigned long ads1299_clk_out_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	return ADS1299_CLK_RATE_HZ;
}

static int ads1299_clk_out_prepare(struct clk_hw *hw)
{
	struct ads1299_private *priv =
		container_of(hw, struct ads1299_private, clk_out);

	return regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG1,
				  ADS1299_MASK_CONFIG1_CLK_EN,
				  ADS1299_MASK_CONFIG1_CLK_EN);
}

static void ads1299_clk_out_unprepare(struct clk_hw *hw)
{
	struct ads1299_private *priv =
		container_of(hw, struct ads1299_private, clk_out);

	regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG1,
			   ADS1299_MASK_CONFIG1_CLK_EN, 0);
}

static int ads1299_clk_out_is_prepared(struct clk_hw *hw)
{
	struct ads1299_private *priv =
		container_of(hw, struct ads1299_private, clk_out);
	unsigned int val;

	if (regmap_read(priv->regmap, ADS1299_REG_CONFIG1, &val))
		return 0;

	return !!(val & ADS1299_MASK_CONFIG1_CLK_EN);
}

static const struct clk_ops ads1299_clk_out_ops = {
	.recalc_rate = ads1299_clk_out_recalc_rate,
	.prepare = ads1299_clk_out_prepare,
	.unprepare = ads1299_clk_out_unprepare,
	.is_prepared = ads1299_clk_out_is_prepared,
};

/*
 * Registered only when the node is a clock provider. With an external clock
 * the CLKSEL pin is low and CLK is an input, so there is nothing to drive.
 */
static int ads1299_setup_clk_out(struct ads1299_private *priv)
{
	struct device *dev = &priv->spi->dev;
	struct clk_init_data init = { };
	int ret;

	if (!device_property_present(dev, "#clock-cells"))
		return 0;

	if (priv->clk)
		return dev_err_probe(dev, -EINVAL,
				     "#clock-cells needs the internal oscillator, and clocks names an external one\n");

	init.name = devm_kasprintf(dev, GFP_KERNEL, "%s-clk", dev_name(dev));
	if (!init.name)
		return -ENOMEM;
	device_property_read_string(dev, "clock-output-names", &init.name);
	init.ops = &ads1299_clk_out_ops;
	priv->clk_out.init = &init;

	ret = devm_clk_hw_register(dev, &priv->clk_out);
	if (ret)
		return ret;

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
					   &priv->clk_out);
}

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

	/* CLK high impedance until a consumer prepares the clock output */
	ret = regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG1,
				 ADS1299_MASK_CONFIG1_CLK_EN, 0);
	if (ret)
		return ret;

	/* Internal test generator available but not routed to any channel */
	ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG2,
			   ADS1299_CONFIG2_RESERVED_VALUE);
	if (ret)
		return ret;

	/*
	 * The internal reference buffer. Without PWR_REFBUF the converter has
	 * no reference at all unless one is wired, and every channel reads
	 * zero.
	 *
	 * BIASREF_INT is deliberately NOT set here any more. It is the bias
	 * amplifier's reference, not the ADC's, and tying it to whether an
	 * external VREF regulator exists conflated the two - bit 7 and bit 3 of
	 * the same register answer different questions. It now travels with the
	 * bias configuration, in ads1299_set_bias().
	 */
	val = ADS1299_CONFIG3_RESERVED_VALUE;
	if (!priv->reg_vref)
		val |= ADS1299_MASK_CONFIG3_PWR_REFBUF;
	ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG3, val);
	if (ret)
		return ret;

	/*
	 * Bias off, written rather than assumed. It is the reset state and the
	 * register write above already cleared PD_BIAS, but the derivation
	 * registers are not touched by that write and a part that was left
	 * configured by something else would keep them.
	 */
	ret = ads1299_set_bias(priv, ADS1299_BIAS_OFF);
	if (ret)
		return ret;

	/*
	 * CHnSET out of reset is gain 24 with the input SHORTED, which reads as
	 * a clean, plausible flat trace, so the MUX is programmed explicitly.
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
	 * nothing here writes.
	 */
	return regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG4,
				  ADS1299_MASK_CONFIG4_PD_LOFF_COMP,
				  ADS1299_MASK_CONFIG4_PD_LOFF_COMP);
}

/*
 * Take `count` single-shot conversions after discarding the first few, and
 * report, per channel, the mean and the peak-to-peak spread in codes.
 */
static int ads1299_sample_stats(struct ads1299_private *priv, unsigned int count,
				s32 *mean, s32 *pp)
{
	s64 sum[ADS1299_MAX_CHANNELS] = { };
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

			sum[ch] += v;
			lo[ch] = min(lo[ch], v);
			hi[ch] = max(hi[ch], v);
		}
	}

	for (ch = 0; ch < priv->num_adc_channels; ch++) {
		mean[ch] = (s32)div_s64(sum[ch], count);
		pp[ch] = hi[ch] - lo[ch];
	}

	return 0;
}

/*
 * Probe-time self test of the analogue path.
 *
 * Step 1 shorts the inputs and requires the noise floor to be under
 * ADS1299_SELF_TEST_NOISE_UV_PP; the mean it measures is each channel's
 * offset. Step 2 holds the internal generator at DC on every input and
 * requires each channel's level, less that offset, to be within
 * ADS1299_SELF_TEST_TOLERANCE_PCT of what the gain implies: a dead PGA stage,
 * a stuck gain or a broken reference all fall outside it. Both leave the
 * inputs connected and the generator off, pass or fail.
 *
 * A failure fails the probe, and the numbers that failed go to the kernel
 * log.
 */
static int ads1299_self_test(struct ads1299_private *priv)
{
	struct device *dev = &priv->spi->dev;
	s32 offset[ADS1299_MAX_CHANNELS];
	s32 level[ADS1299_MAX_CHANNELS];
	s32 pp[ADS1299_MAX_CHANNELS];
	u64 min_amp = U64_MAX;
	u64 max_amp = 0;
	s32 noise_pp = 0;
	u64 expected;
	u64 window;
	u64 ceiling;
	unsigned int ch;
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
			   ADS1299_TEST_SIGNAL_DIVISOR);
	window = div_u64(expected * ADS1299_SELF_TEST_TOLERANCE_PCT, 100);
	ceiling = div_u64((u64)ADS1299_SELF_TEST_NOISE_UV_PP * (u64)gain <<
				  (ADS1299_BITS_PER_SAMPLE - 1),
			  (u32)vref_uv);

	/* Step 1 - inputs shorted, generator off: noise and offset */
	ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG2,
			   ADS1299_CONFIG2_RESERVED_VALUE);
	if (!ret)
		ret = ads1299_set_mux(priv, ADS1299_MUX_SHORTED);
	if (!ret)
		ret = ads1299_sample_stats(priv, ADS1299_SELF_TEST_READINGS,
					   offset, pp);
	if (!ret) {
		for (ch = 0; ch < priv->num_adc_channels; ch++)
			noise_pp = max(noise_pp, pp[ch]);
		if ((u64)noise_pp > ceiling) {
			dev_err(dev,
				"self test: shorted-input noise is %d codes peak to peak, ceiling %llu (%d uV at gain %d)\n",
				noise_pp, ceiling, ADS1299_SELF_TEST_NOISE_UV_PP,
				gain);
			ret = -EIO;
		}
	}

	/* Step 2 - the internal generator held at DC, against that offset */
	if (!ret)
		ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG2,
				   ADS1299_CONFIG2_RESERVED_VALUE |
				   ADS1299_MASK_CONFIG2_INT_CAL |
				   FIELD_PREP(ADS1299_MASK_CONFIG2_CAL_FREQ,
					      ADS1299_CAL_FREQ_DC));
	if (!ret)
		ret = ads1299_set_mux(priv, ADS1299_MUX_TEST_SIGNAL);
	if (!ret)
		ret = ads1299_sample_stats(priv, ADS1299_SELF_TEST_READINGS,
					   level, pp);
	if (!ret) {
		for (ch = 0; ch < priv->num_adc_channels; ch++) {
			u64 amp = abs((s64)level[ch] - offset[ch]);

			min_amp = min(min_amp, amp);
			max_amp = max(max_amp, amp);
		}
		if (min_amp + window < expected || max_amp > expected + window) {
			dev_err(dev,
				"self test: test signal DC level is %llu..%llu codes from the shorted-input offset across channels, expected %llu +-%llu at gain %d\n",
				min_amp, max_amp, expected, window, gain);
			ret = -EIO;
		}
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
		 "self test passed: test signal DC level %llu..%llu codes from offset (expected %llu), shorted-input noise %d codes\n",
		 min_amp, max_amp, expected, noise_pp);

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
	 * Without DRDY as an interrupt (behind a USB-SPI bridge, for instance)
	 * the driver asks for each conversion on a timer.
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
	 * Last, once conversions can complete on either path and before anything
	 * can see the device. A front-end that fails is not registered at all -
	 * userspace finds no device, and the log says why.
	 */
	ret = ads1299_self_test(priv);
	if (ret)
		return dev_err_probe(dev, ret,
				     "self test failed, front-end not registered\n");

	/* A part that failed is not a clock source for others either */
	ret = ads1299_setup_clk_out(priv);
	if (ret)
		return ret;

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
MODULE_DESCRIPTION("TI ADS1299 biopotential ADC driver");
MODULE_LICENSE("GPL");
