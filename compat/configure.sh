#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# Prepare build/ for the kernel in KDIR: a copy of the driver adapted to what
# that kernel has, and the Kbuild that compiles it. The driver in
# drivers/iio/adc is never changed. Run by the Makefile:
#
#   compat/configure.sh KDIR BUILD
#
# Features are detected, not inferred from the version, because distributions
# backport. Each is looked up in the kernel source that kbuild itself uses (its
# srctree), which differs between Ubuntu's headers, an O= output directory and
# Debian's or Raspberry Pi OS's split packages. A header that cannot be read
# stops the build instead of reading as "missing".
#
# Each change to the copy is checked by count: a pattern that does not appear
# exactly as often as expected stops the build, rather than compiling
# something other than intended.

set -eu

KDIR=$1
BUILD=$2
REPO=$(cd "$(dirname "$0")/.." && pwd)
SRC=drivers/iio/adc/ti-ads1299.c
MAKE=${MAKE:-make}

say() { echo "ads1299: $*"; }
die() { echo "ads1299: $*" >&2; exit 1; }

mkdir -p "$BUILD"

# --- where kbuild finds the kernel source -----------------------------------

probe=$BUILD/.probe
rm -rf "$probe"
mkdir -p "$probe"
# shellcheck disable=SC2016
printf '%s\n' '$(info ADS1299_SRCTREE=$(abspath $(srctree)))' 'obj-m :=' >"$probe/Kbuild"
out=$($MAKE -C "$KDIR" M="$probe" KBUILD_MODPOST_WARN=1 modules 2>&1) || true
rm -rf "$probe"
srctree=$(printf '%s\n' "$out" | sed -n 's/^ADS1299_SRCTREE=//p' | head -1)
if [ -z "$srctree" ]; then
	printf '%s\n' "$out" | tail -5 >&2
	die "kbuild in $KDIR did not say where its source is (is KDIR a configured kernel?)"
fi
say "kernel source: $srctree"

hdr() {
	[ -r "$srctree/$1" ] || die "cannot read $srctree/$1"
	echo "$srctree/$1"
}

IIO_H=$(hdr include/linux/iio/iio.h)
BUFFER_H=$(hdr include/linux/iio/buffer.h)
HRTIMER_H=$(hdr include/linux/hrtimer.h)
GPIO_H=$(hdr include/linux/gpio/driver.h)
TYPES_H=$(hdr include/uapi/linux/iio/types.h)

# --- detection ----------------------------------------------------------------

has() { grep -Eqs "$1" "$2"; }

# The text of a struct member's prototype, from its line to the closing ");".
proto() {
	awk -v pat="$1" '
		!on && index($0, pat) { on = 1 }
		on { text = text " " $0; if (index($0, ");")) { print text; exit } }
	' "$2"
}

flags=()
adapted=()
lost=()

need() {	# need FLAG NAME: the kernel lacks NAME
	flags+=("-D$1")
	adapted+=("$2")
}

has 'bool[[:space:]]+iio_device_claim_direct[[:space:]]*\(' "$IIO_H" ||
	need ADS1299_NO_CLAIM_DIRECT "iio_device_claim_direct()"
has '#[[:space:]]*define[[:space:]]+iio_for_each_active_channel\b' "$IIO_H" ||
	need ADS1299_NO_FOR_EACH_ACTIVE_CHANNEL "iio_for_each_active_channel()"
if ! has 'int[[:space:]]+iio_push_to_buffers_with_ts[[:space:]]*\(' "$BUFFER_H"; then
	need ADS1299_NO_PUSH_WITH_TS "iio_push_to_buffers_with_ts()"
	lost+=("the check of the scan buffer's size")
fi
has '\bhrtimer_setup[[:space:]]*\(' "$HRTIMER_H" ||
	need ADS1299_NO_HRTIMER_SETUP "hrtimer_setup()"

unaligned=y
if [ ! -e "$srctree/include/linux/unaligned.h" ]; then
	unaligned=n
	adapted+=("<linux/unaligned.h>")
fi

wec=$(proto '(*write_event_config)' "$IIO_H")
case $wec in
*bool*state*) wec_bool=y ;;
*int*state*) wec_bool=n; adapted+=("write_event_config() taking an int") ;;
*) die "cannot tell the type of write_event_config()'s state in $IIO_H" ;;
esac

if has 'int[[:space:]]+\(\*set\)[[:space:]]*\(' "$GPIO_H"; then
	gpio_int=y
elif has 'void[[:space:]]+\(\*set\)[[:space:]]*\(' "$GPIO_H"; then
	gpio_int=n
	adapted+=("gpio_chip set() returning void")
	lost+=("the error of a GPIO write, which that kernel cannot take")
else
	die "cannot tell what gpio_chip's set() returns in $GPIO_H"
fi

ev_type=n ev_dir=n
has '^[[:space:]]*IIO_EV_TYPE_FAULT\b' "$TYPES_H" && ev_type=y
has '^[[:space:]]*IIO_EV_DIR_FAULT_OPENWIRE\b' "$TYPES_H" && ev_dir=y
[ $ev_type = $ev_dir ] ||
	die "$TYPES_H has only one of IIO_EV_TYPE_FAULT and IIO_EV_DIR_FAULT_OPENWIRE"
if [ $ev_type = n ]; then
	flags+=("-DADS1299_NO_FAULT_EVENT")
	adapted+=("IIO_EV_TYPE_FAULT")
	lost+=("the openwire event (lead-off stays in every buffered sample)")
fi

# --- the adapted copy ---------------------------------------------------------

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cp "$REPO/$SRC" "$tmp/src.c"

# count PATTERN: lines of the copy that contain PATTERN, literally
count() {
	PAT=$1 awk 'index($0, ENVIRON["PAT"]) { n++ } END { print n + 0 }' "$tmp/src.c"
}

expect_one() {
	local n
	n=$(count "$1")
	[ "$n" = 1 ] ||
		die "'$1' is in $SRC $n times, expected once: the driver changed, update compat/configure.sh"
}

# replace OLD NEW: in the one line that contains OLD
replace() {
	expect_one "$1"
	OLD=$1 NEW=$2 awk '{
		i = index($0, ENVIRON["OLD"])
		if (i) $0 = substr($0, 1, i - 1) ENVIRON["NEW"] substr($0, i + length(ENVIRON["OLD"]))
		print
	}' "$tmp/src.c" >"$tmp/next.c"
	mv "$tmp/next.c" "$tmp/src.c"
}

# delete PATTERN: the one line that contains it
delete() {
	expect_one "$1"
	PAT=$1 awk '!index($0, ENVIRON["PAT"])' "$tmp/src.c" >"$tmp/next.c"
	mv "$tmp/next.c" "$tmp/src.c"
}

# insert_before ANCHOR TEXT: TEXT on its own lines before the line with ANCHOR
insert_before() {
	expect_one "$1"
	PAT=$1 TEXT=$2 awk 'index($0, ENVIRON["PAT"]) { print ENVIRON["TEXT"] } { print }' \
		"$tmp/src.c" >"$tmp/next.c"
	mv "$tmp/next.c" "$tmp/src.c"
}

if [ $wec_bool = n ]; then
	insert_before 'static const struct iio_info ads1299_info = {' \
'/* compat: this kernel passes the state as an int */
static int ads1299_write_event_config_compat(struct iio_dev *indio_dev,
					     const struct iio_chan_spec *chan,
					     enum iio_event_type type,
					     enum iio_event_direction dir,
					     int state)
{
	return ads1299_write_event_config(indio_dev, chan, type, dir, !!state);
}
'
	replace '.write_event_config = &ads1299_write_event_config,' \
		'.write_event_config = &ads1299_write_event_config_compat,'
fi

if [ $gpio_int = n ]; then
	insert_before 'static int ads1299_setup_gpio(' \
'/* compat: this kernel cannot take the error of a write */
static void ads1299_gpio_set_compat(struct gpio_chip *gc, unsigned int offset,
				    int value)
{
	ads1299_gpio_set(gc, offset, value);
}
'
	replace 'priv->gpio.set = ads1299_gpio_set;' \
		'priv->gpio.set = ads1299_gpio_set_compat;'
fi

if [ $ev_type = n ]; then
	delete '.event_spec = ads1299_events,'
	delete '.num_event_specs = ARRAY_SIZE(ads1299_events),'
	replace 'static const struct iio_event_spec ads1299_events[] = {' \
		'static const struct iio_event_spec ads1299_events[] __maybe_unused = {'
fi

# --- build/ -------------------------------------------------------------------

ccflags="-include $BUILD/compat/ads1299-compat.h"
[ ${#flags[@]} -eq 0 ] || ccflags="$ccflags ${flags[*]}"
[ $unaligned = y ] || ccflags="$ccflags -I$BUILD/compat/include"

{
	echo "# Generated by compat/configure.sh for $KDIR; do not edit."
	echo "obj-m := ${SRC%.c}.o"
	echo "ccflags-y += $ccflags"
} >"$tmp/Kbuild"

# Only what changed is rewritten, so that make does not rebuild for nothing.
install_file() {
	mkdir -p "$(dirname "$2")"
	cmp -s "$1" "$2" || cp "$1" "$2"
}
install_file "$tmp/src.c" "$BUILD/$SRC"
install_file "$tmp/Kbuild" "$BUILD/Kbuild"
install_file "$REPO/compat/ads1299-compat.h" "$BUILD/compat/ads1299-compat.h"
install_file "$REPO/compat/include/linux/unaligned.h" "$BUILD/compat/include/linux/unaligned.h"

if [ ${#adapted[@]} -eq 0 ]; then
	say "no adaptation needed: build/ compiles the driver unchanged"
else
	say "adapted for this kernel, which lacks:"
	for a in "${adapted[@]}"; do say "  $a"; done
	if [ ${#lost[@]} -gt 0 ]; then
		say "the adapted driver loses:"
		for l in "${lost[@]}"; do say "  $l"; done
	fi
fi
