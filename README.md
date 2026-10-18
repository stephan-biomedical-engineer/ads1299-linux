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

The driver is written against the current mainline API. Built on 2026-10-18:

| Kernel | Result |
|---|---|
| 7.2.11 (arm64, x86-64) | builds, no errors and no warnings, also with `W=1` |
| 6.18.57 (arm64, x86-64) | builds, no errors and no warnings, also with `W=1` |
| 6.18.50+rpt-rpi-v8 (Raspberry Pi OS, on the Pi) | builds, no errors and no warnings; loaded and run, see below. Built at `97a1774`, before the switch to `iio_for_each_active_channel()` |
| 6.12.113 (arm64, x86-64) | does not build |
| 6.8 (Ubuntu 24.04 headers) | does not build |

6.12 and 6.8 lack what the driver uses (6.13 to 6.17 were not tried):

- `IIO_EV_TYPE_FAULT` / `IIO_EV_DIR_FAULT_OPENWIRE`, which mainline gained in 2025
  (<https://patch.msgid.link/20250127-ad4111_openwire-v5-1-ef2db05c384f@baylibre.com>). Without
  it the build stops at once with a message naming the kernel source it looked in, rather than
  failing in the compiler.
- `iio_device_claim_direct()`, `hrtimer_setup()`, `iio_push_to_buffers_with_ts()`, the `bool`
  state of `write_event_config()` and the `int` return of a `gpio_chip`'s `set()`, all missing
  from 6.12; 6.8 also lacks `<linux/unaligned.h>` and `iio_for_each_active_channel()`.

A kernel with the event backported but the older API, such as a 6.12, would pass the first
check and fail in the compiler; that case was not tried. No compatibility layer for older kernels
exists yet.

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

### With DKMS

DKMS rebuilds the module for every kernel the machine installs:

```sh
sudo cp -r . /usr/src/ads1299-linux-0.1
sudo dkms install ads1299-linux/0.1
```

The version in that path has to match `PACKAGE_VERSION` in `dkms.conf`. A kernel without
`IIO_EV_TYPE_FAULT` fails the DKMS build with the message above. Not yet exercised with `dkms`.

## Describing the hardware

The driver binds to a devicetree node; `Documentation/devicetree/bindings/iio/adc/ti,ads1299.yaml`
lists the properties and gives examples. `dts/` holds an overlay for the Raspberry Pi, with the
chip select, DRDY, RESET, PWDN and SPI clock as `config.txt` parameters, and says how to compile
and enable it.

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

On one ADS1299 (`ID = 0x3e`, `REV_ID = 1`), at room temperature, with nothing connected to the
electrode inputs.

On a Raspberry Pi 4B, on its own SPI with DRDY as an interrupt, Raspberry Pi OS with Linux
6.18.50, through `dts/ads1299-overlay.dts`, on 2026-10-18:

- The self test passed at boot and on 10 module reloads; shorted-input noise up to 35 codes,
  against a limit of 447.
- Buffered acquisition from the DRDY interrupt, 10 s at 250 SPS: 2497 conversions due, 2497
  delivered, none lost.
- The test signal's level on each channel repeated the PC's measurement below within 0.004
  percentage points.
- The `openwire` event: with every channel's positive input sensed, one event per open input,
  timestamped with the conversion that showed it; none for an input held by SRB2, none with the
  event disabled, none with AC excitation. The inputs were already open when the buffer started:
  an electrode coming off during an acquisition was not tried.
- 41 of the PC's 42 ABI checks below; the other expects the timer, and the Pi reports
  `drdy_irq`.

On a PC through a USB-to-SPI bridge, with no DRDY interrupt, on 2026-10-17, built from the
same source before its port to the current API:

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

**Not tested yet**: data rates above 250 SPS and continuous read (`RDATAC`), an electrode coming
off during an acquisition, whether PWDN acts (the board pulls it up, so the probe would pass
without it), the clock output and the `gpiochip`, suspend, long acquisitions, and any other part
or temperature.

## Upstream status

**Not submitted yet.** The series (binding, driver, ABI document) will be posted to the
`linux-iio` and `devicetree` mailing lists. This section will then link it on
[lore.kernel.org](https://lore.kernel.org/linux-iio/), together with each revision and its
review.

## License

GPL-2.0, see `LICENSE`. Each source file carries its SPDX identifier.
