# Devicetree overlays

## `ads1299-overlay.dts`: Raspberry Pi

An ADS1299 on SPI0. The default wiring, by BCM GPIO number with the 40-pin header pin in brackets:

| Signal | GPIO |
|---|---|
| MOSI | 10 [19] |
| MISO | 9 [21] |
| SCLK | 11 [23] |
| CS | CE0, 8 [24] |
| DRDY | 25 [22] |
| RESET, PWDN | not connected |

Compile it with plain `dtc` (no kernel headers needed) and install it:

```sh
dtc -@ -Wno-unit_address_vs_reg -I dts -O dtb -o ads1299.dtbo ads1299-overlay.dts
sudo cp ads1299.dtbo /boot/firmware/overlays/    # /boot/overlays/ on older releases
```

Each [release](https://github.com/stephan-biomedical-engineer/ads1299-linux/releases) carries
`ads1299.dtbo` compiled by that command, so that only the copy is needed.

Then enable it in `config.txt` (`/boot/firmware/config.txt`, or `/boot/config.txt` on older
releases) and reboot:

```
dtparam=spi=on
dtoverlay=ads1299
```

Parameters, appended as `dtoverlay=ads1299,<param>=<value>`:

| Parameter | Effect |
|---|---|
| `cs1` | use CE1 (GPIO7 [26]) instead of CE0 |
| `drdy=<gpio>` | DRDY on another GPIO |
| `reset=<gpio>` | RESET driven by the driver |
| `pwdn=<gpio>` | PWDN driven by the driver |
| `speed=<hz>` | SPI clock, default 4000000 |

For example, `dtoverlay=ads1299,cs1,drdy=17,reset=22,pwdn=27`.

The analogue supply is declared as a fixed 5 V regulator, the usual AVDD - AVSS for this part.
A board with another analogue supply needs `regulator-min-microvolt` and
`regulator-max-microvolt` changed in the overlay.

**Status**: compiled with `dtc` and applied with Raspberry Pi's `dtmerge` to the official
`bcm2711-rpi-4-b.dtb`, without parameters and with all of them: the converter lands on `spi0` at
the chosen chip select, the matching `spidev` is disabled, and DRDY, RESET, PWDN and the clock
take the given values. Booted on a Raspberry Pi 4B (Raspberry Pi OS, Linux 6.18.50) as
`dtoverlay=ads1299,drdy=6,pwdn=5` on 2026-10-18: the firmware applied it, the driver bound and
sampled from the DRDY interrupt (top-level `README.md`, "What has been tested"). The `cs1`,
`reset` and `speed` parameters were not booted, nor was the default DRDY pin. The driver's own
kernel requirements apply.

## Other boards

The binding, `Documentation/devicetree/bindings/iio/adc/ti,ads1299.yaml`, lists every property
and has examples. What a board has to say is the SPI bus and chip select, `spi-cpha`, the analogue
supply (`avdd-supply`), and DRDY as an interrupt when it is wired. Without DRDY the driver falls
back to a timer.
