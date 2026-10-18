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
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/hrtimer.h>
#include <linux/log2.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/units.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>

#include <linux/unaligned.h>

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
 * [SBAS499 Table 12] REV_ID[7:5] | 1 | DEV_ID[3:2] | NU_CH[1:0]. REV_ID may
 * change, so the byte is never compared whole.
 */
#define ADS1299_REG_ID				0x00
#define ADS1299_MASK_REV_ID			GENMASK(7, 5)
#define ADS1299_MASK_ID_FAMILY			GENMASK(4, 2)
#define ADS1299_MASK_ID_CHANNELS		GENMASK(1, 0)
#define ADS1299_ID_FAMILY_ADS129X		0x7

/*
 * [SBAS499 Table 13] CONFIG1. ODR = f_CLK >> (7 + DR); DR = 7 is reserved.
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

/* [SBAS499 Table 14] CONFIG2 */
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
 * [SBAS499 Table 15] CONFIG3. PWR_REFBUF is the datasheet's PD_REFBUF: 1
 * powers the buffer up.
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

/* [SBAS499 Table 16] LOFF */
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
 * [SBAS499 Tables 18 to 24] One bit per channel; bits above the channel count
 * must be written 0.
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

/* [SBAS499 Table 26] MISC1 */
#define ADS1299_REG_MISC1			0x15
#define ADS1299_MASK_MISC1_SRB1			BIT(5)

/* [SBAS499 Table 27] MISC2, reset 0x00, every bit reserved, always 0 */
#define ADS1299_REG_MISC2			0x16

/* [SBAS499 Table 28] CONFIG4. PD_LOFF_COMP = 1 powers the comparators up. */
#define ADS1299_REG_CONFIG4			0x17
#define ADS1299_MASK_CONFIG4_SINGLE_SHOT	BIT(3)
#define ADS1299_MASK_CONFIG4_PD_LOFF_COMP	BIT(1)

/*
 * The bias drive amplifier (PD_BIAS). It drives the inverted average of the
 * inputs selected by bias_sense_p/_n, or BIASREF when none is selected.
 */
static const char * const ads1299_bias_names[] = { "off", "on" };

#define ADS1299_MAX_CHANNELS			8
#define ADS1299_BITS_PER_SAMPLE			24
#define ADS1299_CLK_RATE_HZ			2048000
#define ADS1299_CLOCKS_TO_USECS(x)					\
	(DIV_ROUND_UP((x) * MICROHZ_PER_HZ, ADS1299_CLK_RATE_HZ))

/* A command takes 4 t_CLK to decode, so register access runs at f_CLK */
#define ADS1299_SPI_BUS_SPEED_SLOW		ADS1299_CLK_RATE_HZ
#define ADS1299_SPI_CMD_BUFFER_SIZE		3
/*
 * [SBAS499 9.4.4.2] Status word: 1100b, LOFF_STATP, LOFF_STATN, GPIO[7:4].
 */
#define ADS1299_MASK_STATUS_SYNC		GENMASK(7, 4)
#define ADS1299_STATUS_SYNC			0xc0
#define ADS1299_LEAD_OFF_ADDRESS		1

/* Command byte, status word and n 24-bit samples */
#define ADS1299_SPI_RDATA_BUFFER_SIZE(n)	(((n) + 1) * 3 + 1)
#define ADS1299_SPI_RDATA_BUFFER_SIZE_MAX				\
	ADS1299_SPI_RDATA_BUFFER_SIZE(ADS1299_MAX_CHANNELS)

/*
 * Self test. The test signal is VREF / 2400 [SBAS499 Table 14], held at DC:
 * in single-shot mode every START restarts the generator's divider, so the
 * square wave would always be read at the same phase.
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
#define ADS1299_MUX_DISCARD			2
#define ADS1299_SELF_TEST_INTERVAL_MS		50

/*
 * [SBAS499 9.3.1.1.3, Equation 3] 145300 uV at 25 degC, 490 uV/degC, read at
 * gain 1.
 */
#define ADS1299_TEMP_UV_AT_25C			145300
#define ADS1299_TEMP_UV_PER_C			490

/*
 * [SBAS499 9.3.1.1.4] Supply measurement (MUX = 011) at gain 1: 0.5 x (AVDD +
 * AVSS) on channels 1, 2, 5 to 8, DVDD / 4 on channels 3 and 4.
 */
#define ADS1299_TEMP_CHANNEL			0
#define ADS1299_MVDD_ANALOG_CHANNEL		0
#define ADS1299_MVDD_DIGITAL_CHANNEL		2

/*
 * [SBAS499 Electrical Characteristics] Power-up and reference start-up:
 * 150 ms. Wakeup from standby: 31.25 us plus 4 tCLK.
 */
#define ADS1299_START_UP_MS			150
#define ADS1299_WAKEUP_USECS			32

/* Internal reference, used when no "vref" regulator is given */
#define ADS1299_INTERNAL_VREF_UV		4500000

enum ads1299_timestamp_source {
	ADS1299_TS_DRDY_IRQ = 0,
	ADS1299_TS_HOST_TIMER,
};

struct ads1299_private {
	struct spi_device *spi;
	struct regulator *reg_avdd;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdown_gpio;
	/* Across a system suspend: CONFIG3, volatile, and a running buffer */
	u8 config3_saved;
	bool resume_running;
	struct regulator *reg_vref;
	struct regulator *reg_biasref;
	struct clk *clk;
	struct clk_hw clk_out;
	struct gpio_chip gpio;
	/*
	 * Last value written to GPIO. A read of GPIOD returns the pin levels,
	 * so writes are built from this and never read back.
	 */
	struct mutex gpio_lock;	/* protects gpio_shadow */
	u8 gpio_shadow;
	struct regmap *regmap;
	struct completion completion;
	struct spi_transfer rdata_xfer;
	struct spi_message rdata_msg;
	spinlock_t irq_busy_lock; /* Handshake between SPI and DRDY irqs */
	/*
	 * 0 = idle, 1 = transfer in progress, 2 = a sample became due during
	 * it, n > 2 = n - 2 samples lost.
	 */
	unsigned int rdata_xfer_busy;
	/* Conversions the host never collected. */
	u64 lost_samples;

	/* Number of ADC channels the chip reported; excludes the timestamp */
	unsigned int num_adc_channels;
	/*
	 * Lead-off state used by the SPI completion, which cannot take the
	 * regmap lock.
	 */
	u8 loff_sensp;
	u8 loff_sensn;
	bool loff_ac;
	bool loff_events;
	u8 loff_offp;
	u8 loff_offn;

	enum ads1299_timestamp_source ts_source;
	struct hrtimer timer;		/* only used when ts_source is HOST_TIMER */
	ktime_t timer_period;
	bool timer_running;

	/* Demux target. Must stay 8-byte aligned for the timestamp channel. */
	struct {
		u32 chan[ADS1299_MAX_CHANNELS + 1];	/* + lead-off word */
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

/*
 * The comparators are on while any input or the bias electrode is sensed.
 */
static int ads1299_update_loff_comparators(struct ads1299_private *priv)
{
	unsigned int sensp;
	unsigned int sensn;
	unsigned int config3;
	bool on;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_SENSP, &sensp);
	if (!ret)
		ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_SENSN, &sensn);
	if (!ret)
		ret = regmap_read(priv->regmap, ADS1299_REG_CONFIG3, &config3);
	if (ret)
		return ret;

	on = sensp || sensn || (config3 & ADS1299_MASK_CONFIG3_BIAS_LOFF_SENS);

	return regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG4,
				  ADS1299_MASK_CONFIG4_PD_LOFF_COMP,
				  on ? ADS1299_MASK_CONFIG4_PD_LOFF_COMP : 0);
}

/* The copies of the lead-off configuration the SPI completion reads */
static int ads1299_cache_loff(struct ads1299_private *priv)
{
	unsigned int sensp;
	unsigned int sensn;
	unsigned int loff;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_SENSP, &sensp);
	if (!ret)
		ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_SENSN, &sensn);
	if (!ret)
		ret = regmap_read(priv->regmap, ADS1299_REG_LOFF, &loff);
	if (ret)
		return ret;

	priv->loff_sensp = sensp;
	priv->loff_sensn = sensn;
	priv->loff_ac = FIELD_GET(ADS1299_MASK_LOFF_FLEAD_OFF, loff) !=
			ADS1299_LOFF_FLEAD_OFF_DC;

	return 0;
}

/*
 * Per-channel bits: lead-off sensing and flip [SBAS499 9.3.2.4.3], and the
 * bias derivation [SBAS499 Tables 18, 19].
 */
static const unsigned int ads1299_channel_bit_regs[] = {
	ADS1299_REG_LOFF_SENSP,
	ADS1299_REG_LOFF_SENSN,
	ADS1299_REG_LOFF_FLIP,
	ADS1299_REG_BIAS_SENSP,
	ADS1299_REG_BIAS_SENSN,
};

static ssize_t ads1299_channel_bit_read(struct iio_dev *indio_dev,
					uintptr_t private,
					const struct iio_chan_spec *chan,
					char *buf)
{
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	ret = regmap_read(priv->regmap, ads1299_channel_bit_regs[private],
			  &regval);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", !!(regval & BIT(chan->channel)));
}

static ssize_t ads1299_channel_bit_write(struct iio_dev *indio_dev,
					 uintptr_t private,
					 const struct iio_chan_spec *chan,
					 const char *buf, size_t len)
{
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int reg = ads1299_channel_bit_regs[private];
	bool sense = reg == ADS1299_REG_LOFF_SENSP ||
		     reg == ADS1299_REG_LOFF_SENSN;
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, reg, BIT(chan->channel),
				 on ? BIT(chan->channel) : 0);
	if (!ret && sense)
		ret = ads1299_update_loff_comparators(priv);
	if (!ret && sense)
		ret = ads1299_cache_loff(priv);

	iio_device_release_direct_mode(indio_dev);

	return ret ? ret : (ssize_t)len;
}

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

/* Whether any channel's MUX is at this setting */
static int ads1299_mux_in_use(struct ads1299_private *priv, unsigned int mux)
{
	unsigned int regval;
	unsigned int i;
	int ret;

	for (i = 0; i < priv->num_adc_channels; i++) {
		ret = regmap_read(priv->regmap, ADS1299_REG_CHnSET(i), &regval);
		if (ret)
			return ret;
		if (FIELD_GET(ADS1299_MASK_CH_MUX, regval) == mux)
			return 1;
	}

	return 0;
}

static int ads1299_generator_on(struct ads1299_private *priv)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_CONFIG2, &regval);
	if (ret)
		return ret;

	return !!(regval & ADS1299_MASK_CONFIG2_INT_CAL);
}

/*
 * MUX = 010 needs CONFIG3.BIAS_MEAS [SBAS499 Table 15]: set before a channel
 * moves there, cleared after the last one leaves.
 */
static int ads1299_set_bias_meas(struct ads1299_private *priv, bool on)
{
	return regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG3,
				  ADS1299_MASK_CONFIG3_BIAS_MEAS,
				  on ? ADS1299_MASK_CONFIG3_BIAS_MEAS : 0);
}

/* What one channel is connected to [SBAS499 Table 17] */
static int ads1299_set_mux(struct ads1299_private *priv, unsigned int channel,
			   unsigned int mux)
{
	int ret;

	if (mux == ADS1299_MUX_BIAS_MEAS) {
		ret = ads1299_set_bias_meas(priv, true);
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(priv->regmap, ADS1299_REG_CHnSET(channel),
				 ADS1299_MASK_CH_MUX,
				 FIELD_PREP(ADS1299_MASK_CH_MUX, mux));
	if (ret || mux == ADS1299_MUX_BIAS_MEAS)
		return ret;

	ret = ads1299_mux_in_use(priv, ADS1299_MUX_BIAS_MEAS);
	if (ret < 0)
		return ret;

	return ret ? 0 : ads1299_set_bias_meas(priv, false);
}

/* Every channel to one setting: the self test's shorted and test inputs */
static int ads1299_set_mux_all(struct ads1299_private *priv, unsigned int mux)
{
	unsigned int i;
	int ret;

	for (i = 0; i < priv->num_adc_channels; i++) {
		ret = ads1299_set_mux(priv, i, mux);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * The part has no external test pin, so the test input is refused while the
 * generator is off, and the generator cannot be turned off while in use.
 */
static ssize_t ads1299_mux_read(struct iio_dev *indio_dev, uintptr_t private,
				const struct iio_chan_spec *chan, char *buf)
{
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_CHnSET(chan->channel),
			  &regval);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%s\n",
			  ads1299_mux_names[FIELD_GET(ADS1299_MASK_CH_MUX,
						      regval)]);
}

static ssize_t ads1299_mux_write(struct iio_dev *indio_dev, uintptr_t private,
				 const struct iio_chan_spec *chan,
				 const char *buf, size_t len)
{
	struct ads1299_private *priv = iio_priv(indio_dev);
	int mux;
	int ret;

	mux = sysfs_match_string(ads1299_mux_names, buf);
	if (mux < 0)
		return mux;

	/* Not while running: the channel's samples would change meaning */
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = 0;
	if (mux == ADS1299_MUX_TEST_SIGNAL) {
		ret = ads1299_generator_on(priv);
		if (!ret)
			ret = -EINVAL;
	}
	if (ret >= 0)
		ret = ads1299_set_mux(priv, chan->channel, mux);

	iio_device_release_direct_mode(indio_dev);

	return ret ? ret : (ssize_t)len;
}

static ssize_t ads1299_mux_available(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	size_t len = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ads1299_mux_names); i++)
		len += sysfs_emit_at(buf, len, "%s%s", i ? " " : "",
				     ads1299_mux_names[i]);

	return len + sysfs_emit_at(buf, len, "\n");
}

/* SRB2 [SBAS499 Table 17]: ties the SRB2 pin to the positive input. */
static ssize_t ads1299_srb2_read(struct iio_dev *indio_dev, uintptr_t private,
				 const struct iio_chan_spec *chan, char *buf)
{
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_CHnSET(chan->channel),
			  &regval);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", !!(regval & ADS1299_MASK_CH_SRB2));
}

static ssize_t ads1299_srb2_write(struct iio_dev *indio_dev, uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf, size_t len)
{
	struct ads1299_private *priv = iio_priv(indio_dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	/* Not while running: the channel would measure another difference */
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap,
				 ADS1299_REG_CHnSET(chan->channel),
				 ADS1299_MASK_CH_SRB2,
				 on ? ADS1299_MASK_CH_SRB2 : 0);

	iio_device_release_direct_mode(indio_dev);

	return ret ? ret : (ssize_t)len;
}

static const struct iio_chan_spec_ext_info ads1299_ext_info[] = {
	{
		.name = "lead_off_sense_p",
		.shared = IIO_SEPARATE,
		.read = ads1299_channel_bit_read,
		.write = ads1299_channel_bit_write,
		.private = 0,
	},
	{
		.name = "lead_off_sense_n",
		.shared = IIO_SEPARATE,
		.read = ads1299_channel_bit_read,
		.write = ads1299_channel_bit_write,
		.private = 1,
	},
	{
		.name = "lead_off_flip",
		.shared = IIO_SEPARATE,
		.read = ads1299_channel_bit_read,
		.write = ads1299_channel_bit_write,
		.private = 2,
	},
	{
		.name = "bias_sense_p",
		.shared = IIO_SEPARATE,
		.read = ads1299_channel_bit_read,
		.write = ads1299_channel_bit_write,
		.private = 3,
	},
	{
		.name = "bias_sense_n",
		.shared = IIO_SEPARATE,
		.read = ads1299_channel_bit_read,
		.write = ads1299_channel_bit_write,
		.private = 4,
	},
	{
		.name = "input_mux",
		.shared = IIO_SEPARATE,
		.read = ads1299_mux_read,
		.write = ads1299_mux_write,
	},
	{
		.name = "input_mux_available",
		.shared = IIO_SHARED_BY_TYPE,
		.read = ads1299_mux_available,
	},
	{
		.name = "srb2",
		.shared = IIO_SEPARATE,
		.read = ads1299_srb2_read,
		.write = ads1299_srb2_write,
	},
	{ }
};

/*
 * Lead-off events come from each conversion's status word: buffer only, DC
 * excitation only.
 */
static const struct iio_event_spec ads1299_events[] = {
	{
		.type = IIO_EV_TYPE_FAULT,
		.dir = IIO_EV_DIR_FAULT_OPENWIRE,
		.mask_shared_by_all = BIT(IIO_EV_INFO_ENABLE),
	},
};

#define ADS1299_CHAN(index)						\
{									\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = index,						\
	.address = ADS1299_OFFSET_IN_RX_BUFFER(index),			\
	.info_mask_separate =						\
		BIT(IIO_CHAN_INFO_RAW) |				\
		BIT(IIO_CHAN_INFO_SCALE) |				\
		BIT(IIO_CHAN_INFO_HARDWAREGAIN),			\
	.info_mask_separate_available =					\
		BIT(IIO_CHAN_INFO_HARDWAREGAIN),			\
	.info_mask_shared_by_all =					\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.info_mask_shared_by_all_available =				\
		BIT(IIO_CHAN_INFO_SAMP_FREQ),				\
	.ext_info = ads1299_ext_info,					\
	.event_spec = ads1299_events,					\
	.num_event_specs = ARRAY_SIZE(ads1299_events),			\
	.scan_index = index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = ADS1299_BITS_PER_SAMPLE,			\
		.storagebits = 32,					\
		.endianness = IIO_CPU,					\
	},								\
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

/* [SBAS499 Table 17] PGA field -> gain; 111 is reserved. */
static const int ads1299_pga_gain[] = { 1, 2, 4, 6, 8, 12, 24 };

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

/* The gain of one channel's PGA; its scale follows */
static int ads1299_set_gain(struct ads1299_private *priv, unsigned int channel,
			    int gain)
{
	unsigned int setting;

	for (setting = 0; setting < ARRAY_SIZE(ads1299_pga_gain); setting++) {
		if (ads1299_pga_gain[setting] == gain)
			break;
	}
	if (setting == ARRAY_SIZE(ads1299_pga_gain))
		return -EINVAL;

	return regmap_update_bits(priv->regmap, ADS1299_REG_CHnSET(channel),
				  ADS1299_MASK_CH_PGA,
				  FIELD_PREP(ADS1299_MASK_CH_PGA, setting));
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

	/* A conversion takes 4 ms at the lowest rate, plus bus latency. */
	if (priv->ts_source == ADS1299_TS_HOST_TIMER) {
		/*
		 * No DRDY interrupt: run the transfer here, on a copy, since
		 * spi_sync() overwrites the message's complete/context.
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

/* The 24-bit code at an offset of the last RDATA reply */
static int ads1299_code(const struct ads1299_private *priv,
			unsigned long address)
{
	return sign_extend32(get_unaligned_be24(priv->rx_buffer + address),
			     ADS1299_BITS_PER_SAMPLE - 1);
}

static int ads1299_scale_at_gain(struct ads1299_private *priv, int gain,
				 int *val, int *val2)
{
	u64 step;
	int vref_uv;

	vref_uv = ads1299_get_vref_uv(priv);
	if (vref_uv < 0)
		return vref_uv;

	/*
	 * One LSB is VREF / (gain * 2^23) in mV, in 64 bits to keep the
	 * fraction (4.5 V / 24 = 187.5 mV).
	 */
	step = (u64)vref_uv * MICRO;
	step = div_u64(step, (u64)gain << (ADS1299_BITS_PER_SAMPLE - 1));

	*val = 0;
	*val2 = (int)step;

	return IIO_VAL_INT_PLUS_NANO;
}

static int ads1299_get_scale(struct ads1299_private *priv,
			     int channel, int *val, int *val2)
{
	int gain;

	gain = ads1299_get_gain(priv, (unsigned int)channel);
	if (gain < 0)
		return gain;

	return ads1299_scale_at_gain(priv, gain, val, val2);
}

/*
 * Equation 3 at gain 1, one code = VREF / 2^23:
 *   scale  = VREF * 1000 / (490 * 2^23)		mdegC per code
 *   offset = -(145300 - 25 * 490) * 2^23 / VREF	codes
 */
static int ads1299_temp_scale(struct ads1299_private *priv, int *val, int *val2)
{
	int vref_uv;
	u32 rem;
	u64 q;

	vref_uv = ads1299_get_vref_uv(priv);
	if (vref_uv <= 0)
		return vref_uv < 0 ? vref_uv : -EINVAL;

	q = div64_u64((u64)vref_uv * MILLIDEGREE_PER_DEGREE * NANO,
		      (u64)ADS1299_TEMP_UV_PER_C <<
		      (ADS1299_BITS_PER_SAMPLE - 1));
	*val = (int)div_u64_rem(q, NANO, &rem);
	*val2 = (int)rem;

	return IIO_VAL_INT_PLUS_NANO;
}

static int ads1299_temp_offset(struct ads1299_private *priv, int *val,
			       int *val2)
{
	u64 uv_at_0c = ADS1299_TEMP_UV_AT_25C - 25 * ADS1299_TEMP_UV_PER_C;
	int vref_uv;
	u32 rem;
	u64 q;

	vref_uv = ads1299_get_vref_uv(priv);
	if (vref_uv <= 0)
		return vref_uv < 0 ? vref_uv : -EINVAL;

	q = div_u64((uv_at_0c << (ADS1299_BITS_PER_SAMPLE - 1)) * MICRO,
		    (u32)vref_uv);
	*val = -(int)div_u64_rem(q, MICRO, &rem);
	*val2 = (int)rem;

	return IIO_VAL_INT_PLUS_MICRO;
}

/*
 * Read the temperature or a supply on a borrowed channel, then restore its
 * CHnSET.
 */
static int ads1299_read_internal(struct ads1299_private *priv,
				 const struct iio_chan_spec *chan, int *val)
{
	unsigned int ch = chan->address;
	unsigned int mux = chan->type == IIO_TEMP ? ADS1299_MUX_TEMPERATURE :
						    ADS1299_MUX_MVDD;
	unsigned int saved;
	unsigned int i;
	int restore;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_CHnSET(ch), &saved);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, ADS1299_REG_CHnSET(ch),
			   (saved & ADS1299_MASK_CH_SRB2) |
			   FIELD_PREP(ADS1299_MASK_CH_PGA, 0) |
			   FIELD_PREP(ADS1299_MASK_CH_MUX, mux));
	for (i = 0; !ret && i <= ADS1299_MUX_DISCARD; i++)
		ret = ads1299_read_one(priv, ch);
	if (!ret)
		*val = ads1299_code(priv, ADS1299_OFFSET_IN_RX_BUFFER(ch));

	restore = regmap_write(priv->regmap, ADS1299_REG_CHnSET(ch), saved);

	return ret ? ret : restore;
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

		/* Not in the buffer: the temperature and the supplies */
		if (chan->scan_index < 0) {
			ret = ads1299_read_internal(priv, chan, val);
		} else {
			ret = ads1299_read_one(priv, chan->scan_index);
			if (!ret)
				*val = ads1299_code(priv, chan->address);
		}

		iio_device_release_direct_mode(indio_dev);

		return ret ? ret : IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_TEMP)
			return ads1299_temp_scale(priv, val, val2);
		if (chan->scan_index < 0)
			return ads1299_scale_at_gain(priv, 1, val, val2);
		return ads1299_get_scale(priv, chan->channel, val, val2);
	case IIO_CHAN_INFO_OFFSET:
		return ads1299_temp_offset(priv, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ads1299_get_samp_freq(priv, val);
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = ads1299_get_gain(priv, chan->channel);
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
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ads1299_set_samp_freq(priv, val);
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = ads1299_set_gain(priv, chan->channel, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	iio_device_release_direct_mode(indio_dev);

	return ret;
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

	/* The timestamp channel has no power-down bit. */
	for (i = 0; i < priv->num_adc_channels; i++) {
		val = test_bit(i, scan_mask) ? 0 : ADS1299_MASK_CH_PD;
		ret = regmap_update_bits(priv->regmap, ADS1299_REG_CHnSET(i),
					 ADS1299_MASK_CH_PD, val);
		if (ret)
			return ret;
	}

	return 0;
}

/* Device attributes beyond the IIO ABI */

/*
 * BIASREF_INT goes with the bias amplifier: BIASREF is (AVDD + AVSS) / 2
 * unless the board feeds the pin.
 */
static int ads1299_set_bias(struct ads1299_private *priv, bool on)
{
	unsigned int config3 = 0;

	if (on)
		config3 = ADS1299_MASK_CONFIG3_PD_BIAS |
			  (priv->reg_biasref ? 0 : ADS1299_MASK_CONFIG3_BIASREF_INT);

	return regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG3,
				  ADS1299_MASK_CONFIG3_BIASREF_INT |
				  ADS1299_MASK_CONFIG3_PD_BIAS, config3);
}

static ssize_t bias_drive_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int regval;
	bool on;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_CONFIG3, &regval);
	if (ret)
		return ret;

	on = regval & ADS1299_MASK_CONFIG3_PD_BIAS;

	return sysfs_emit(buf, "%s\n", ads1299_bias_names[on]);
}

static ssize_t bias_drive_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	int on;
	int ret;

	on = sysfs_match_string(ads1299_bias_names, buf);
	if (on < 0)
		return on;

	/* Not while running: the samples would change meaning mid-stream */
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = ads1299_set_bias(priv, on);

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

/*
 * Test generator settings. INT_CAL = 0 leaves the test input undriven, as
 * there is no external test pin; CAL_FREQ = 10 is reserved.
 */
static const struct {
	const char *name;
	u8 config2;
} ads1299_test_signals[] = {
	{ "off", 0 },
	{ "1x_slow", ADS1299_MASK_CONFIG2_INT_CAL |
		     FIELD_PREP_CONST(ADS1299_MASK_CONFIG2_CAL_FREQ,
				      ADS1299_CAL_FREQ_SLOW) },
	{ "1x_fast", ADS1299_MASK_CONFIG2_INT_CAL |
		     FIELD_PREP_CONST(ADS1299_MASK_CONFIG2_CAL_FREQ,
				      ADS1299_CAL_FREQ_FAST) },
	{ "1x_dc", ADS1299_MASK_CONFIG2_INT_CAL |
		   FIELD_PREP_CONST(ADS1299_MASK_CONFIG2_CAL_FREQ,
				    ADS1299_CAL_FREQ_DC) },
	{ "2x_slow", ADS1299_MASK_CONFIG2_INT_CAL |
		     ADS1299_MASK_CONFIG2_CAL_AMP |
		     FIELD_PREP_CONST(ADS1299_MASK_CONFIG2_CAL_FREQ,
				      ADS1299_CAL_FREQ_SLOW) },
	{ "2x_fast", ADS1299_MASK_CONFIG2_INT_CAL |
		     ADS1299_MASK_CONFIG2_CAL_AMP |
		     FIELD_PREP_CONST(ADS1299_MASK_CONFIG2_CAL_FREQ,
				      ADS1299_CAL_FREQ_FAST) },
	{ "2x_dc", ADS1299_MASK_CONFIG2_INT_CAL |
		   ADS1299_MASK_CONFIG2_CAL_AMP |
		   FIELD_PREP_CONST(ADS1299_MASK_CONFIG2_CAL_FREQ,
				    ADS1299_CAL_FREQ_DC) },
};

#define ADS1299_MASK_CONFIG2_TEST					\
	(ADS1299_MASK_CONFIG2_INT_CAL | ADS1299_MASK_CONFIG2_CAL_AMP |	\
	 ADS1299_MASK_CONFIG2_CAL_FREQ)

static ssize_t test_signal_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int regval;
	unsigned int i;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_CONFIG2, &regval);
	if (ret)
		return ret;

	/* With the generator off, its amplitude and frequency mean nothing */
	if (!(regval & ADS1299_MASK_CONFIG2_INT_CAL))
		return sysfs_emit(buf, "off\n");

	for (i = 0; i < ARRAY_SIZE(ads1299_test_signals); i++)
		if ((regval & ADS1299_MASK_CONFIG2_TEST) ==
		    ads1299_test_signals[i].config2)
			return sysfs_emit(buf, "%s\n",
					  ads1299_test_signals[i].name);

	/* CAL_FREQ = 10, which only a debugfs register write can set */
	return -EINVAL;
}

static ssize_t test_signal_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ads1299_test_signals); i++)
		if (sysfs_streq(buf, ads1299_test_signals[i].name))
			break;
	if (i == ARRAY_SIZE(ads1299_test_signals))
		return -EINVAL;

	/* Not while running: a test channel's samples would change meaning */
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = 0;
	if (!(ads1299_test_signals[i].config2 & ADS1299_MASK_CONFIG2_INT_CAL)) {
		ret = ads1299_mux_in_use(priv, ADS1299_MUX_TEST_SIGNAL);
		if (ret > 0)
			ret = -EINVAL;
	}
	if (!ret)
		ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG2,
				   ADS1299_CONFIG2_RESERVED_VALUE |
				   ads1299_test_signals[i].config2);

	iio_device_release_direct_mode(indio_dev);

	return ret ? ret : (ssize_t)len;
}

static ssize_t test_signal_available_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	size_t len = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ads1299_test_signals); i++)
		len += sysfs_emit_at(buf, len, "%s%s", i ? " " : "",
				     ads1299_test_signals[i].name);

	return len + sysfs_emit_at(buf, len, "\n");
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
	unsigned int loff;
	int ret;

	/* A status bit is valid only where its input is sensed. */
	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_SENSP, &sensp);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_SENSN, &sensn);
	if (ret)
		return ret;

	if (!sensp && !sensn)
		return sysfs_emit(buf, "disabled\n");

	/* The comparators are the DC method [SBAS499 9.3.2.4.3.1]. */
	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF, &loff);
	if (ret)
		return ret;

	if (FIELD_GET(ADS1299_MASK_LOFF_FLEAD_OFF, loff) !=
	    ADS1299_LOFF_FLEAD_OFF_DC)
		return sysfs_emit(buf, "ac\n");

	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_STATP, &statp);
	if (ret)
		return ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF_STATN, &statn);
	if (ret)
		return ret;

	/* One bit per electrode, positive then negative, valid bits only. */
	return sysfs_emit(buf, "%02x %02x\n", statp & sensp, statn & sensn);
}

/*
 * Bias electrode lead-off [SBAS499 9.3.2.4.4]: with the drive off the current
 * goes into the electrode; with it on, BIAS_STAT compares the amplifier's
 * output against COMP_TH.
 */
static const char * const ads1299_bias_lead_off_names[] = { "off", "on" };

static ssize_t bias_lead_off_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int regval;
	bool on;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_CONFIG3, &regval);
	if (ret)
		return ret;

	on = regval & ADS1299_MASK_CONFIG3_BIAS_LOFF_SENS;

	return sysfs_emit(buf, "%s\n", ads1299_bias_lead_off_names[on]);
}

static ssize_t bias_lead_off_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	int on;
	int ret;

	on = sysfs_match_string(ads1299_bias_lead_off_names, buf);
	if (on < 0)
		return on;

	/* Not while running: the lead-off current changes what is measured */
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG3,
				 ADS1299_MASK_CONFIG3_BIAS_LOFF_SENS,
				 on ? ADS1299_MASK_CONFIG3_BIAS_LOFF_SENS : 0);
	/* BIAS_STAT is one of the comparators' outputs */
	if (!ret)
		ret = ads1299_update_loff_comparators(priv);

	iio_device_release_direct_mode(indio_dev);

	return ret ? ret : (ssize_t)len;
}

static ssize_t bias_lead_off_status_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	/* CONFIG3 is volatile: this reaches the part */
	ret = regmap_read(priv->regmap, ADS1299_REG_CONFIG3, &regval);
	if (ret)
		return ret;

	if (!(regval & ADS1299_MASK_CONFIG3_BIAS_LOFF_SENS))
		return sysfs_emit(buf, "disabled\n");

	return sysfs_emit(buf, "%s\n",
			  regval & ADS1299_MASK_CONFIG3_BIAS_STAT ?
				  "disconnected" : "connected");
}

/*
 * [SBAS499 Table 16] LOFF fields by code. COMP_TH is the positive threshold
 * in percent (the negative is its complement), current in nA, AC frequency in
 * Hz for the 2.048 MHz internal clock.
 */
static const char * const ads1299_loff_threshold_names[] = {
	"95", "92.5", "90", "87.5", "85", "80", "75", "70",
};

static const char * const ads1299_loff_current_names[] = {
	"6", "24", "6000", "24000",
};

static const char * const ads1299_loff_frequency_names[] = {
	"dc", "ac_7.8", "ac_31.2", "ac_fdr_div_4",
};

enum ads1299_loff_field {
	ADS1299_LOFF_THRESHOLD,
	ADS1299_LOFF_CURRENT,
	ADS1299_LOFF_FREQUENCY,
};

static const struct {
	u8 mask;
	const char * const *names;
	unsigned int count;
} ads1299_loff_fields[] = {
	[ADS1299_LOFF_THRESHOLD] = {
		ADS1299_MASK_LOFF_COMP_TH, ads1299_loff_threshold_names,
		ARRAY_SIZE(ads1299_loff_threshold_names),
	},
	[ADS1299_LOFF_CURRENT] = {
		ADS1299_MASK_LOFF_ILEAD_OFF, ads1299_loff_current_names,
		ARRAY_SIZE(ads1299_loff_current_names),
	},
	[ADS1299_LOFF_FREQUENCY] = {
		ADS1299_MASK_LOFF_FLEAD_OFF, ads1299_loff_frequency_names,
		ARRAY_SIZE(ads1299_loff_frequency_names),
	},
};

static ssize_t ads1299_loff_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int field = to_iio_dev_attr(attr)->address;
	u8 mask = ads1299_loff_fields[field].mask;
	unsigned int regval;
	unsigned int code;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_LOFF, &regval);
	if (ret)
		return ret;

	code = (regval & mask) >> __ffs(mask);

	return sysfs_emit(buf, "%s\n", ads1299_loff_fields[field].names[code]);
}

static ssize_t ads1299_loff_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int field = to_iio_dev_attr(attr)->address;
	u8 mask = ads1299_loff_fields[field].mask;
	int code;
	int ret;

	code = __sysfs_match_string(ads1299_loff_fields[field].names,
				    ads1299_loff_fields[field].count, buf);
	if (code < 0)
		return code;

	/* Not while running: current and thresholds change what is measured */
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, ADS1299_REG_LOFF, mask,
				 ((unsigned int)code << __ffs(mask)) & mask);
	if (!ret)
		ret = ads1299_cache_loff(priv);

	iio_device_release_direct_mode(indio_dev);

	return ret ? ret : (ssize_t)len;
}

static ssize_t ads1299_loff_available_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	unsigned int field = to_iio_dev_attr(attr)->address;
	size_t len = 0;
	unsigned int i;

	for (i = 0; i < ads1299_loff_fields[field].count; i++)
		len += sysfs_emit_at(buf, len, "%s%s", i ? " " : "",
				     ads1299_loff_fields[field].names[i]);

	return len + sysfs_emit_at(buf, len, "\n");
}

/*
 * SRB1 [SBAS499 Table 26]: ties SRB1 to the negative input of every channel
 * on the electrodes.
 */
static const char * const ads1299_montage_names[] = {
	"bipolar", "referential",
};

static ssize_t montage_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int regval;
	bool referential;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_MISC1, &regval);
	if (ret)
		return ret;

	referential = regval & ADS1299_MASK_MISC1_SRB1;

	return sysfs_emit(buf, "%s\n", ads1299_montage_names[referential]);
}

static ssize_t montage_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	int referential;
	int ret;

	referential = sysfs_match_string(ads1299_montage_names, buf);
	if (referential < 0)
		return referential;

	/* Not while running: every sample would measure another difference */
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, ADS1299_REG_MISC1,
				 ADS1299_MASK_MISC1_SRB1,
				 referential ? ADS1299_MASK_MISC1_SRB1 : 0);

	iio_device_release_direct_mode(indio_dev);

	return ret ? ret : (ssize_t)len;
}

static ssize_t montage_available_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	size_t len = 0;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ads1299_montage_names); i++)
		len += sysfs_emit_at(buf, len, "%s%s", i ? " " : "",
				     ads1299_montage_names[i]);

	return len + sysfs_emit_at(buf, len, "\n");
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

static IIO_DEVICE_ATTR_RW(test_signal, 0);
static IIO_DEVICE_ATTR_RO(test_signal_available, 0);
static IIO_DEVICE_ATTR_RW(bias_drive, 0);
static IIO_DEVICE_ATTR_RO(bias_drive_available, 0);
static IIO_DEVICE_ATTR_RO(lead_off_status, 0);
static IIO_DEVICE_ATTR_RW(bias_lead_off, 0);
static IIO_DEVICE_ATTR_RO(bias_lead_off_status, 0);
static IIO_DEVICE_ATTR_RW(montage, 0);
static IIO_DEVICE_ATTR_RO(montage_available, 0);
static IIO_DEVICE_ATTR(lead_off_threshold_percent, 0644, ads1299_loff_show,
		       ads1299_loff_store, ADS1299_LOFF_THRESHOLD);
static IIO_DEVICE_ATTR(lead_off_threshold_percent_available, 0444,
		       ads1299_loff_available_show, NULL, ADS1299_LOFF_THRESHOLD);
static IIO_DEVICE_ATTR(lead_off_current_nanoamp, 0644, ads1299_loff_show,
		       ads1299_loff_store, ADS1299_LOFF_CURRENT);
static IIO_DEVICE_ATTR(lead_off_current_nanoamp_available, 0444,
		       ads1299_loff_available_show, NULL, ADS1299_LOFF_CURRENT);
static IIO_DEVICE_ATTR(lead_off_frequency, 0644, ads1299_loff_show,
		       ads1299_loff_store, ADS1299_LOFF_FREQUENCY);
static IIO_DEVICE_ATTR(lead_off_frequency_available, 0444,
		       ads1299_loff_available_show, NULL, ADS1299_LOFF_FREQUENCY);
static IIO_DEVICE_ATTR_RO(lost_samples, 0);
static IIO_DEVICE_ATTR_RO(timestamp_source, 0);

static struct attribute *ads1299_attributes[] = {
	&iio_dev_attr_test_signal.dev_attr.attr,
	&iio_dev_attr_test_signal_available.dev_attr.attr,
	&iio_dev_attr_bias_drive.dev_attr.attr,
	&iio_dev_attr_bias_drive_available.dev_attr.attr,
	&iio_dev_attr_lead_off_status.dev_attr.attr,
	&iio_dev_attr_bias_lead_off.dev_attr.attr,
	&iio_dev_attr_bias_lead_off_status.dev_attr.attr,
	&iio_dev_attr_montage.dev_attr.attr,
	&iio_dev_attr_montage_available.dev_attr.attr,
	&iio_dev_attr_lead_off_threshold_percent.dev_attr.attr,
	&iio_dev_attr_lead_off_threshold_percent_available.dev_attr.attr,
	&iio_dev_attr_lead_off_current_nanoamp.dev_attr.attr,
	&iio_dev_attr_lead_off_current_nanoamp_available.dev_attr.attr,
	&iio_dev_attr_lead_off_frequency.dev_attr.attr,
	&iio_dev_attr_lead_off_frequency_available.dev_attr.attr,
	&iio_dev_attr_lost_samples.dev_attr.attr,
	&iio_dev_attr_timestamp_source.dev_attr.attr,
	NULL
};

static const struct attribute_group ads1299_attribute_group = {
	.attrs = ads1299_attributes,
};

static int ads1299_read_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir)
{
	struct ads1299_private *priv = iio_priv(indio_dev);

	if (type != IIO_EV_TYPE_FAULT)
		return -EINVAL;

	return priv->loff_events;
}

static int ads1299_write_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir, int state)
{
	struct ads1299_private *priv = iio_priv(indio_dev);

	if (type != IIO_EV_TYPE_FAULT)
		return -EINVAL;

	priv->loff_events = state;

	return 0;
}

static const struct iio_info ads1299_info = {
	.read_raw = &ads1299_read_raw,
	.read_avail = &ads1299_read_avail,
	.write_raw = &ads1299_write_raw,
	.update_scan_mode = &ads1299_update_scan_mode,
	.read_event_config = &ads1299_read_event_config,
	.write_event_config = &ads1299_write_event_config,
	.debugfs_reg_access = &ads1299_reg_access,
	.attrs = &ads1299_attribute_group,
};

static void ads1299_rdata_release_busy_or_restart(struct ads1299_private *priv)
{
	guard(spinlock_irqsave)(&priv->irq_busy_lock);

	if (priv->rdata_xfer_busy > 1) {
		/*
		 * A sample became due during the transfer: fetch it now.
		 */
		spi_async(priv->spi, &priv->rdata_msg);
		/* More than one: count the losses and resynchronise. */
		if (priv->rdata_xfer_busy > 2)
			priv->lost_samples += priv->rdata_xfer_busy - 2;
		priv->rdata_xfer_busy = 1;
	} else {
		priv->rdata_xfer_busy = 0;
	}
}

/*
 * Inputs off at this conversion; bits of unsensed inputs, or with AC
 * excitation, are cleared.
 */
static void ads1299_lead_off_state(const struct ads1299_private *priv,
				   u8 *offp, u8 *offn)
{
	const u8 *status = priv->rx_buffer + ADS1299_LEAD_OFF_ADDRESS;

	if (priv->loff_ac) {
		*offp = 0;
		*offn = 0;
		return;
	}

	*offp = (u8)((status[0] << 4) | (status[1] >> 4)) & priv->loff_sensp;
	*offn = (u8)((status[1] << 4) | (status[2] >> 4)) & priv->loff_sensn;
}

/* An event for each channel with an input newly off */
static void ads1299_lead_off_events(struct iio_dev *indio_dev, u8 offp, u8 offn,
				    s64 timestamp)
{
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned long newly = (offp & ~priv->loff_offp) |
			      (offn & ~priv->loff_offn);
	unsigned int bit;

	priv->loff_offp = offp;
	priv->loff_offn = offn;

	if (!priv->loff_events)
		return;

	for_each_set_bit(bit, &newly, priv->num_adc_channels)
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, bit,
						    IIO_EV_TYPE_FAULT,
						    IIO_EV_DIR_FAULT_OPENWIRE),
			       timestamp);
}

/* Called from SPI completion interrupt handler */
static void ads1299_rdata_complete(void *context)
{
	struct iio_dev *indio_dev = context;
	struct ads1299_private *priv = iio_priv(indio_dev);
	int scan_index;
	u32 *bounce = priv->scan.chan;
	s64 timestamp;
	u8 offp;
	u8 offn;

	if (!iio_buffer_enabled(indio_dev)) {
		/*
		 * For a single transfer we are kept in direct mode until
		 * completion, avoiding a race with buffered IO.
		 */
		ads1299_rdata_unmark_busy(priv);
		complete(&priv->completion);
		return;
	}

	/* Taken at SPI completion; see timestamp_source. */
	timestamp = iio_get_time_ns(indio_dev);

	ads1299_lead_off_state(priv, &offp, &offn);
	ads1299_lead_off_events(indio_dev, offp, offn, timestamp);

	for_each_set_bit(scan_index, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		const struct iio_chan_spec *scan_chan =
					&indio_dev->channels[scan_index];
		const u8 *data;

		if (scan_chan->type == IIO_TIMESTAMP)
			continue;

		/* (positive << 8) | negative, bit n for channel n + 1 */
		if (scan_chan->address == ADS1299_LEAD_OFF_ADDRESS) {
			*bounce++ = ((u32)offp << 8) | offn;
			continue;
		}

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
 * Without a DRDY interrupt a timer polls for each conversion, so bus jitter
 * lands in the timestamp.
 */
static enum hrtimer_restart ads1299_timer_fired(struct hrtimer *timer)
{
	struct ads1299_private *priv =
		container_of(timer, struct ads1299_private, timer);

	ads1299_sample_due(priv);
	hrtimer_forward_now(timer, priv->timer_period);

	return priv->timer_running ? HRTIMER_RESTART : HRTIMER_NORESTART;
}

/* Continuous conversions, on DRDY or on the host timer */
static int ads1299_start_conversions(struct ads1299_private *priv)
{
	int ret;

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

static int ads1299_stop_conversions(struct ads1299_private *priv)
{
	if (priv->ts_source == ADS1299_TS_HOST_TIMER) {
		priv->timer_running = false;
		hrtimer_cancel(&priv->timer);
	}

	return ads1299_write_cmd(priv, ADS1299_CMD_STOP);
}

static int ads1299_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ads1299_private *priv = iio_priv(indio_dev);

	priv->lost_samples = 0;
	/* An electrode already off raises its event at the first conversion */
	priv->loff_offp = 0;
	priv->loff_offn = 0;

	return ads1299_start_conversions(priv);
}

static int ads1299_buffer_predisable(struct iio_dev *indio_dev)
{
	return ads1299_stop_conversions(iio_priv(indio_dev));
}

static const struct iio_buffer_setup_ops ads1299_setup_ops = {
	.postenable = &ads1299_buffer_postenable,
	.predisable = &ads1299_buffer_predisable,
};

static void ads1299_reg_disable(void *reg)
{
	regulator_disable(reg);
}

/* An optional supply, enabled for the life of the device; NULL when absent */
static int ads1299_get_optional_supply(struct device *dev, const char *id,
				       struct regulator **reg)
{
	int ret;

	*reg = devm_regulator_get_optional(dev, id);
	if (IS_ERR(*reg)) {
		if (PTR_ERR(*reg) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(*reg),
					     "Failed to get %s regulator\n", id);
		*reg = NULL;
		return 0;
	}

	ret = regulator_enable(*reg);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, ads1299_reg_disable, *reg);
}

/*
 * CONFIG3 for BIAS_STAT; GPIO because GPIOD reads the pins.
 */
static const struct regmap_range ads1299_regmap_volatile_range[] = {
	regmap_reg_range(ADS1299_REG_CONFIG3, ADS1299_REG_CONFIG3),
	regmap_reg_range(ADS1299_REG_LOFF_STATP, ADS1299_REG_LOFF_STATN),
	regmap_reg_range(ADS1299_REG_GPIO, ADS1299_REG_GPIO),
};

static const struct regmap_access_table ads1299_regmap_volatile = {
	.yes_ranges = ads1299_regmap_volatile_range,
	.n_yes_ranges = ARRAY_SIZE(ads1299_regmap_volatile_range),
};

/* Read only: what a regcache_sync() must not write back */
static const struct regmap_range ads1299_regmap_read_only_range[] = {
	regmap_reg_range(ADS1299_REG_ID, ADS1299_REG_ID),
	regmap_reg_range(ADS1299_REG_LOFF_STATP, ADS1299_REG_LOFF_STATN),
};

static const struct regmap_access_table ads1299_regmap_writeable = {
	.no_ranges = ads1299_regmap_read_only_range,
	.n_no_ranges = ARRAY_SIZE(ads1299_regmap_read_only_range),
};

static const struct regmap_config ads1299_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_read = ads1299_reg_read,
	.reg_write = ads1299_reg_write,
	.max_register = ADS1299_REG_CONFIG4,
	.wr_table = &ads1299_regmap_writeable,
	.volatile_table = &ads1299_regmap_volatile,
	.cache_type = REGCACHE_MAPLE,
};

/*
 * CLK output [SBAS499 Table 6]: CLK_EN is written over SPI, which sleeps, so
 * the work is done in prepare.
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

/* With an external clock CLK is an input. */
/*
 * GPIO1 to GPIO4 [SBAS499 Table 25]. GPIOC bit n set = input; GPIOD reads the
 * pins, so writes come from gpio_shadow.
 */
#define ADS1299_NUM_GPIOS			4

static int ads1299_gpio_write(struct ads1299_private *priv)
{
	return regmap_write(priv->regmap, ADS1299_REG_GPIO, priv->gpio_shadow);
}

static int ads1299_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct ads1299_private *priv = gpiochip_get_data(gc);

	guard(mutex)(&priv->gpio_lock);

	return priv->gpio_shadow & BIT(offset) ? GPIO_LINE_DIRECTION_IN :
						 GPIO_LINE_DIRECTION_OUT;
}

static int ads1299_gpio_direction_input(struct gpio_chip *gc,
					unsigned int offset)
{
	struct ads1299_private *priv = gpiochip_get_data(gc);

	guard(mutex)(&priv->gpio_lock);

	priv->gpio_shadow |= BIT(offset);

	return ads1299_gpio_write(priv);
}

static void ads1299_gpio_set_level(struct ads1299_private *priv,
				   unsigned int offset, int value)
{
	if (value)
		priv->gpio_shadow |= BIT(offset + ADS1299_NUM_GPIOS);
	else
		priv->gpio_shadow &= ~BIT(offset + ADS1299_NUM_GPIOS);
}

static int ads1299_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct ads1299_private *priv = gpiochip_get_data(gc);
	unsigned int regval;
	int ret;

	ret = regmap_read(priv->regmap, ADS1299_REG_GPIO, &regval);
	if (ret)
		return ret;

	return !!(regval & BIT(offset + ADS1299_NUM_GPIOS));
}

static void ads1299_gpio_set(struct gpio_chip *gc, unsigned int offset,
			     int value)
{
	struct ads1299_private *priv = gpiochip_get_data(gc);

	guard(mutex)(&priv->gpio_lock);

	ads1299_gpio_set_level(priv, offset, value);
	ads1299_gpio_write(priv);
}

/*
 * Whether turning a pin into an output latches its level is unspecified, so
 * write the register twice.
 */
static int ads1299_gpio_direction_output(struct gpio_chip *gc,
					 unsigned int offset, int value)
{
	struct ads1299_private *priv = gpiochip_get_data(gc);
	int ret;

	guard(mutex)(&priv->gpio_lock);

	ads1299_gpio_set_level(priv, offset, value);
	priv->gpio_shadow &= ~BIT(offset);

	ret = ads1299_gpio_write(priv);
	if (ret)
		return ret;

	return ads1299_gpio_write(priv);
}

/*
 * Unused pins are tied to DGND, so only register the gpiochip when the board
 * says they are wired.
 */
static int ads1299_setup_gpio(struct ads1299_private *priv)
{
	struct device *dev = &priv->spi->dev;

	if (!device_property_present(dev, "gpio-controller"))
		return 0;

	priv->gpio.label = dev_name(dev);
	priv->gpio.parent = dev;
	priv->gpio.owner = THIS_MODULE;
	priv->gpio.base = -1;
	priv->gpio.ngpio = ADS1299_NUM_GPIOS;
	priv->gpio.can_sleep = true;
	priv->gpio.get_direction = ads1299_gpio_get_direction;
	priv->gpio.direction_input = ads1299_gpio_direction_input;
	priv->gpio.direction_output = ads1299_gpio_direction_output;
	priv->gpio.get = ads1299_gpio_get;
	priv->gpio.set = ads1299_gpio_set;

	return devm_gpiochip_add_data(dev, &priv->gpio, priv);
}

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

/* Reset by pin or command, then 18 tCLK [SBAS499 Figure 76]. */
static int ads1299_reset(struct ads1299_private *priv)
{
	int ret;

	if (priv->reset_gpio) {
		gpiod_set_value_cansleep(priv->reset_gpio, 1);
		fsleep(ADS1299_CLOCKS_TO_USECS(2));
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
	} else {
		ret = ads1299_write_cmd(priv, ADS1299_CMD_RESET);
		if (ret)
			return ret;
	}
	fsleep(ADS1299_CLOCKS_TO_USECS(18));

	return 0;
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

	/*
	 * DAISY_EN = 0 is daisy-chain mode [SBAS499 Table 13], which reads
	 * the same on a single device. CLK stays off until prepared.
	 */
	ret = regmap_update_bits(priv->regmap, ADS1299_REG_CONFIG1,
				 ADS1299_MASK_CONFIG1_DAISY_EN |
				 ADS1299_MASK_CONFIG1_CLK_EN, 0);
	if (ret)
		return ret;

	/* Every GPIO an input, as after reset, written */
	ret = devm_mutex_init(dev, &priv->gpio_lock);
	if (ret)
		return ret;
	priv->gpio_shadow = ADS1299_MASK_GPIO_CONTROL;
	ret = ads1299_gpio_write(priv);
	if (ret)
		return ret;

	/* SRB1 open, written; the rest of MISC1 is reserved and always 0 */
	ret = regmap_write(priv->regmap, ADS1299_REG_MISC1, 0);
	if (ret)
		return ret;

	/* Lead-off at its reset value, written: 95% / 5%, 6 nA, DC */
	ret = regmap_write(priv->regmap, ADS1299_REG_LOFF,
			   FIELD_PREP(ADS1299_MASK_LOFF_COMP_TH,
				      ADS1299_LOFF_COMP_TH_95_5) |
			   FIELD_PREP(ADS1299_MASK_LOFF_ILEAD_OFF,
				      ADS1299_LOFF_ILEAD_OFF_6NA) |
			   FIELD_PREP(ADS1299_MASK_LOFF_FLEAD_OFF,
				      ADS1299_LOFF_FLEAD_OFF_DC));
	if (ret)
		return ret;

	/* Internal test generator available but not routed to any channel */
	ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG2,
			   ADS1299_CONFIG2_RESERVED_VALUE);
	if (ret)
		return ret;

	/* Without the reference buffer every channel reads zero. */
	val = ADS1299_CONFIG3_RESERVED_VALUE;
	if (!priv->reg_vref)
		val |= ADS1299_MASK_CONFIG3_PWR_REFBUF;
	ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG3, val);
	if (ret)
		return ret;

	/* Bias off. */
	ret = ads1299_set_bias(priv, false);
	if (ret)
		return ret;

	/*
	 * Out of reset the inputs are shorted, which looks like a clean
	 * trace, so MUX is written explicitly.
	 */
	for (i = 0; i < priv->num_adc_channels; i++) {
		ret = regmap_update_bits(priv->regmap, ADS1299_REG_CHnSET(i),
					 ADS1299_MASK_CH_MUX |
					 ADS1299_MASK_CH_PD |
					 ADS1299_MASK_CH_SRB2,
					 FIELD_PREP(ADS1299_MASK_CH_MUX,
						    ADS1299_MUX_NORMAL));
		if (ret)
			return ret;
	}

	/* No lead-off sensing, flip or bias derivation. */
	for (i = 0; i < ARRAY_SIZE(ads1299_channel_bit_regs); i++) {
		ret = regmap_write(priv->regmap,
				   ads1299_channel_bit_regs[i], 0);
		if (ret)
			return ret;
	}

	ret = ads1299_update_loff_comparators(priv);
	if (ret)
		return ret;

	return ads1299_cache_loff(priv);
}

/*
 * Per channel mean and peak-to-peak, in codes, of `count` conversions after a
 * few discarded.
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

	for (reading = 0; reading < count + ADS1299_MUX_DISCARD; reading++) {
		ret = ads1299_read_one(priv, 0);
		if (ret)
			return ret;

		msleep(ADS1299_SELF_TEST_INTERVAL_MS);

		/* Not 1100b: the reply is out of step. */
		if ((priv->rx_buffer[1] & ADS1299_MASK_STATUS_SYNC) !=
		    ADS1299_STATUS_SYNC) {
			dev_err(&priv->spi->dev,
				"status word opens with 0x%02x, not 0x%02x\n",
				priv->rx_buffer[1], ADS1299_STATUS_SYNC);
			return -EIO;
		}

		if (reading < ADS1299_MUX_DISCARD)
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
 * Probe-time self test: shorted inputs for noise and offset, then the test
 * generator at DC against the level the gain implies. A failure fails the
 * probe.
 */
static int ads1299_self_test(struct ads1299_private *priv)
{
	struct device *dev = &priv->spi->dev;
	s32 offset[ADS1299_MAX_CHANNELS];
	s32 level[ADS1299_MAX_CHANNELS];
	s32 pp[ADS1299_MAX_CHANNELS];
	int gain[ADS1299_MAX_CHANNELS];
	unsigned int mux[ADS1299_MAX_CHANNELS];
	unsigned int regval;
	s32 noise_pp = 0;
	u64 expected;
	u64 window;
	u64 ceiling;
	u64 amp;
	unsigned int ch;
	int vref_uv;
	int restore;
	int ret;

	vref_uv = ads1299_get_vref_uv(priv);
	if (vref_uv <= 0)
		return vref_uv < 0 ? vref_uv : -EINVAL;

	/* Each channel is judged at its own gain, and goes back to its input */
	for (ch = 0; ch < priv->num_adc_channels; ch++) {
		gain[ch] = ads1299_get_gain(priv, ch);
		if (gain[ch] < 0)
			return gain[ch];
		ret = regmap_read(priv->regmap, ADS1299_REG_CHnSET(ch),
				  &regval);
		if (ret)
			return ret;
		mux[ch] = FIELD_GET(ADS1299_MASK_CH_MUX, regval);
	}

	/* Step 1 - inputs shorted, generator off: noise and offset */
	ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG2,
			   ADS1299_CONFIG2_RESERVED_VALUE);
	if (!ret)
		ret = ads1299_set_mux_all(priv, ADS1299_MUX_SHORTED);
	if (!ret)
		ret = ads1299_sample_stats(priv, ADS1299_SELF_TEST_READINGS,
					   offset, pp);
	for (ch = 0; !ret && ch < priv->num_adc_channels; ch++) {
		ceiling = (u64)ADS1299_SELF_TEST_NOISE_UV_PP * (u64)gain[ch];
		ceiling = div_u64(ceiling << (ADS1299_BITS_PER_SAMPLE - 1),
				  (u32)vref_uv);
		noise_pp = max(noise_pp, pp[ch]);
		if ((u64)pp[ch] > ceiling) {
			dev_err(dev,
				"self test: channel %u shorted-input noise is %d codes peak to peak, ceiling %llu (%d uV at gain %d)\n",
				ch, pp[ch], ceiling,
				ADS1299_SELF_TEST_NOISE_UV_PP, gain[ch]);
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
		ret = ads1299_set_mux_all(priv, ADS1299_MUX_TEST_SIGNAL);
	if (!ret)
		ret = ads1299_sample_stats(priv, ADS1299_SELF_TEST_READINGS,
					   level, pp);
	for (ch = 0; !ret && ch < priv->num_adc_channels; ch++) {
		expected = (u64)gain[ch] << (ADS1299_BITS_PER_SAMPLE - 1);
		expected = div_u64(expected, ADS1299_TEST_SIGNAL_DIVISOR);
		window = div_u64(expected * ADS1299_SELF_TEST_TOLERANCE_PCT,
				 100);
		amp = abs((s64)level[ch] - offset[ch]);
		if (amp + window < expected || amp > expected + window) {
			dev_err(dev,
				"self test: channel %u test signal DC level is %llu codes from the shorted-input offset, expected %llu +-%llu at gain %d\n",
				ch, amp, expected, window, gain[ch]);
			ret = -EIO;
		}
	}

	/* Pass or fail, every channel goes back to what it was connected to */
	restore = 0;
	for (ch = 0; !restore && ch < priv->num_adc_channels; ch++)
		restore = ads1299_set_mux(priv, ch, mux[ch]);
	if (!restore)
		restore = regmap_write(priv->regmap, ADS1299_REG_CONFIG2,
				       ADS1299_CONFIG2_RESERVED_VALUE);

	if (ret)
		return ret;
	if (restore)
		return restore;

	dev_info(dev,
		 "self test passed: %u channels at their gains, shorted-input noise up to %d codes\n",
		 priv->num_adc_channels, noise_pp);

	return 0;
}

static int ads1299_build_channels(struct iio_dev *indio_dev,
				  struct ads1299_private *priv)
{
	struct device *dev = &priv->spi->dev;
	struct iio_chan_spec *channels;
	unsigned int n = priv->num_adc_channels;

	/*
	 * Voltage channels, lead-off word, timestamp, then temperature and two
	 * supplies, which are not buffered.
	 */
	channels = devm_kcalloc(dev, n + 5, sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	memcpy(channels, ads1299_channels, n * sizeof(*channels));
	/*
	 * Lead-off state per conversion, (positive << 8) | negative. No _raw
	 * or _scale: it is not a voltage.
	 */
	channels[n] = (struct iio_chan_spec){
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = (int)n,
		.extend_name = "lead_off",
		.address = ADS1299_LEAD_OFF_ADDRESS,
		.scan_index = (int)n,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 32,
			.endianness = IIO_CPU,
		},
	};
	channels[n + 1] = (struct iio_chan_spec)IIO_CHAN_SOFT_TIMESTAMP(n + 1);
	channels[n + 2] = (struct iio_chan_spec){
		.type = IIO_TEMP,
		.address = ADS1299_TEMP_CHANNEL,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.scan_index = -1,
	};
	channels[n + 3] = (struct iio_chan_spec){
		.type = IIO_VOLTAGE,
		.extend_name = "mvdd_analog",
		.address = ADS1299_MVDD_ANALOG_CHANNEL,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = -1,
	};
	channels[n + 4] = (struct iio_chan_spec){
		.type = IIO_VOLTAGE,
		.extend_name = "mvdd_digital",
		.address = ADS1299_MVDD_DIGITAL_CHANNEL,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = -1,
	};

	indio_dev->channels = channels;
	indio_dev->num_channels = (int)(n + 5);

	return 0;
}

static int ads1299_probe(struct spi_device *spi)
{
	struct ads1299_private *priv;
	struct iio_dev *indio_dev;
	struct device *dev = &spi->dev;
	u32 chain = 1;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);

	/* Reset to be asserted before enabling clock and power */
	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "Cannot get reset GPIO\n");

	/* Out of power-down, if the board can put it there */
	priv->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown",
						       GPIOD_OUT_LOW);
	if (IS_ERR(priv->powerdown_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->powerdown_gpio),
				     "Cannot get powerdown GPIO\n");

	/* VREF can be supplied externally, otherwise use the internal 4.5 V */
	ret = ads1299_get_optional_supply(dev, "vref", &priv->reg_vref);
	if (ret)
		return ret;

	/* So can BIASREF, otherwise the part makes (AVDD + AVSS) / 2 */
	ret = ads1299_get_optional_supply(dev, "biasref", &priv->reg_biasref);
	if (ret)
		return ret;

	priv->clk = devm_clk_get_optional_enabled(dev, "clk");
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk), "Failed to get clk\n");

	/* A daisy chain would be read as its first device alone. */
	device_property_read_u32(dev, "#daisy-chained-devices", &chain);
	if (chain > 1)
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "a daisy chain of %u devices is not supported\n",
				     chain);

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

	ret = devm_regulator_get_enable(dev, "dvdd");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable dvdd regulator\n");

	spi_set_drvdata(spi, indio_dev);
	priv->spi = spi;
	init_completion(&priv->completion);
	spin_lock_init(&priv->irq_busy_lock);
	priv->regmap = devm_regmap_init(dev, NULL, priv, &ads1299_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	/* Without a DRDY interrupt, poll each conversion with a timer. */
	priv->ts_source = spi->irq > 0 ? ADS1299_TS_DRDY_IRQ
				       : ADS1299_TS_HOST_TIMER;

	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->info = &ads1299_info;

	/* Just out of power-down: the oscillator and reference wake first */
	if (priv->powerdown_gpio)
		msleep(ADS1299_START_UP_MS);

	ret = ads1299_reset(priv);
	if (ret)
		return dev_err_probe(dev, ret, "RESET failed\n");

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

	/* A front-end that fails is not registered. */
	ret = ads1299_self_test(priv);
	if (ret)
		return dev_err_probe(dev, ret,
				     "self test failed, front-end not registered\n");

	/* A part that failed is not a clock source for others either */
	ret = ads1299_setup_clk_out(priv);
	if (ret)
		return ret;

	ret = ads1299_setup_gpio(priv);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

/*
 * With a PWDN pin the part is powered down, and resume assumes its registers
 * are lost; without one, standby keeps them. CONFIG3 and GPIO are volatile and
 * written back by hand.
 *
 * No runtime PM: it would drop the bias drive and the clock output.
 */
static int ads1299_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	unsigned int config3;
	int ret;

	priv->resume_running = iio_buffer_enabled(indio_dev);
	if (priv->resume_running) {
		ret = ads1299_stop_conversions(priv);
		if (ret)
			return ret;
	}

	ret = regmap_read(priv->regmap, ADS1299_REG_CONFIG3, &config3);
	if (ret)
		return ret;
	priv->config3_saved = config3 & ~ADS1299_MASK_CONFIG3_BIAS_STAT;

	if (!priv->powerdown_gpio)
		return ads1299_write_cmd(priv, ADS1299_CMD_STANDBY);

	gpiod_set_value_cansleep(priv->powerdown_gpio, 1);
	regcache_mark_dirty(priv->regmap);

	return 0;
}

static int ads1299_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ads1299_private *priv = iio_priv(indio_dev);
	int ret;

	if (priv->powerdown_gpio) {
		gpiod_set_value_cansleep(priv->powerdown_gpio, 0);
		msleep(ADS1299_START_UP_MS);

		ret = ads1299_reset(priv);
		if (!ret)
			ret = ads1299_write_cmd(priv, ADS1299_CMD_SDATAC);
		if (!ret)
			ret = regcache_sync(priv->regmap);
		if (!ret)
			ret = regmap_write(priv->regmap, ADS1299_REG_CONFIG3,
					   priv->config3_saved);
		/* Twice, as direction_output() does: the pins came back inputs */
		if (!ret) {
			guard(mutex)(&priv->gpio_lock);
			ret = ads1299_gpio_write(priv);
			if (!ret)
				ret = ads1299_gpio_write(priv);
		}
	} else {
		ret = ads1299_write_cmd(priv, ADS1299_CMD_WAKEUP);
		fsleep(ADS1299_WAKEUP_USECS + ADS1299_CLOCKS_TO_USECS(4));
	}
	if (ret)
		return ret;

	return priv->resume_running ? ads1299_start_conversions(priv) : 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ads1299_pm_ops, ads1299_suspend,
				ads1299_resume);

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
		.pm	= pm_sleep_ptr(&ads1299_pm_ops),
	},
	.probe		= ads1299_probe,
	.id_table	= ads1299_id,
};
module_spi_driver(ads1299_driver);

MODULE_AUTHOR("Stephan Costa Barros <stephanbrrs8@gmail.com>");
MODULE_DESCRIPTION("TI ADS1299 biopotential ADC driver");
MODULE_LICENSE("GPL");
