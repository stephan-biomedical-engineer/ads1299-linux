# ti-ads1299

Linux IIO driver for the Texas Instruments **ADS1299**, ADS1299-6 and ADS1299-4: low-noise,
24-bit, 8/6/4-channel analogue front-ends for EEG and other biopotential measurements, on SPI.

The driver is derived from mainline's `ti-ads1298.c` (Mike Looijmans, Topic Embedded Products),
whose copyright it keeps, and it is being prepared for submission to the mainline kernel. This
repository holds only that work, with its development history: the driver, its devicetree
binding and its sysfs ABI document, laid out at the paths they have in the kernel tree.

## What it supports

- All seven data rates (250 to 16000 SPS) and all seven PGA gains (1 to 24), per channel.
- The input multiplexer per channel: normal, shorted, test signal, supply measurement,
  temperature sensor, bias measurement and bias drive.
- The internal test signal (1x/2x, slow/fast/DC), SRB1 (`montage`) and SRB2 per channel.
- Bias drive derived per channel, an external BIASREF (`biasref-supply`) and bias lead-off.
- Lead-off detection: threshold, current, DC/AC excitation, sense and flip per channel; status as
  an IIO `fault`/`openwire` event and as a lead-off word in every buffered sample.
- The temperature sensor and both supply monitors as IIO channels.
- A kfifo buffer, fed from DRDY when it is wired and from an hrtimer when it is not.
- GPIO1 to GPIO4 as a `gpiochip`, the CLK pin as a clock provider, PWDN, and system suspend.
- A probe-time self test of the analogue path (test-signal level and shorted-input noise); a
  front-end that fails it is not registered.

Daisy-chained converters are described by the binding and refused by this driver; their support
is kept apart for a separate series.

The attributes are documented in `Documentation/ABI/testing/sysfs-bus-iio-adc-ti-ads1299`, and the
devicetree properties in `Documentation/devicetree/bindings/iio/adc/ti,ads1299.yaml`.

## Kernel requirements

**Today the driver builds against Linux 6.6 only**, with one patch backported. It was developed
on STMicroelectronics' `linux-stm32mp` 6.6 for an STM32MP257 board, and the port to current
mainline is the next step before submission:

- It needs `IIO_EV_TYPE_FAULT` / `IIO_EV_DIR_FAULT_OPENWIRE`, which mainline gained in 2025
  (<https://patch.msgid.link/20250127-ad4111_openwire-v5-1-ef2db05c384f@baylibre.com>). On 6.6
  that patch is backported. Against a kernel without it the `Makefile` stops with a message
  rather than failing in the compiler.
- It still includes `<asm/unaligned.h>` and calls `hrtimer_init()`, as 6.6 does. Mainline moved
  the first to `<linux/unaligned.h>` in 6.12 and replaces the second with `hrtimer_setup()`, so
  no mainline release builds it unchanged yet.

The kernel also needs `CONFIG_SPI`, `CONFIG_COMMON_CLK`, `CONFIG_GPIOLIB`, `CONFIG_IIO`,
`CONFIG_IIO_BUFFER`, `CONFIG_IIO_KFIFO_BUF` and `CONFIG_REGMAP`.

## Building out of tree

```sh
make                                   # against the running kernel
make KDIR=/path/to/kernel ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
sudo make modules_install
```

The module is `drivers/iio/adc/ti-ads1299.ko`. With Secure Boot on, an unsigned module is refused
at load; sign it with a key enrolled in the machine (MOK).

## Building in tree

Copy the three files to the same paths in a kernel tree, then add to `drivers/iio/adc/Kconfig`:

```
config TI_ADS1299
	tristate "Texas Instruments ADS1299 biopotential AFE driver"
	depends on SPI
	depends on COMMON_CLK
	depends on GPIOLIB
	select IIO_BUFFER
	select IIO_KFIFO_BUF
	select REGMAP
	help
	  Say yes here to build support for Texas Instruments ADS1299,
	  ADS1299-6 and ADS1299-4 low-noise 24-bit biopotential (EEG)
	  analog front-ends.

	  This driver can also be built as a module. If so, the module will be
	  called ti-ads1299.
```

and to `drivers/iio/adc/Makefile`:

```
obj-$(CONFIG_TI_ADS1299) += ti-ads1299.o
```

## What has been tested

On one ADS1299 (`ID = 0x3e`, `REV_ID = 1`), on a PC through a USB-to-SPI bridge, with no DRDY
interrupt, at room temperature, on 2026-10-17:

- The ABI behaves as documented: 42 checks, including 12 refusals of invalid or ill-timed requests.
- Every setting writes its field at the datasheet's bit position and nothing else: the 24
  registers, read back past the driver by an independent SPI client, matched the datasheet in nine
  patterns that cover every code of every field (216 of 216), and the driver's register cache
  matched the silicon after each.
- The analogue values agree with the datasheet (SBAS499C): shorted-input noise at all seven gains
  (−19 % to +8 % of the typical RMS), test signal plus gain within ±2.5 % (−0.06 % to −0.42 %),
  both supply monitors within 0.2 % of a multimeter.
- The self-test limits were measured, not assumed: ±20 % on the test signal is 8 times the worst
  in-spec case, and 10 µV on the noise is 6.25 times the datasheet's guaranteed maximum.

**Not tested yet**: the DRDY interrupt and continuous read (`RDATAC`) up to 16 kSPS, the
`openwire` event (the PC's kernel lacked the event type), PWDN, the clock output and the
`gpiochip`, long acquisitions, and any other part or temperature. They need a native SPI host
with a devicetree that declares them.

## License

GPL-2.0, see `LICENSE`. Each source file carries its SPDX identifier.
