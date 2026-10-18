#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# Build a git revision of the driver against every test kernel, each in its
# own copy, and print one line per kernel.
#
#   tools/build-matrix.sh [-r REV] [-p PATCH]... [-W] [-o DIR] [FILTER]...
#
#   -r REV    revision to build (default HEAD)
#   -p PATCH  apply PATCH to each copy before building (repeatable)
#   -W        build with W=1
#   -o DIR    keep copies and logs in DIR (default: a new temporary directory)
#   FILTER    only targets whose name contains one of these strings
#
# Targets: the host's own headers (/lib/modules/$(uname -r)/build, native),
# then every $KERNELS/build-<arch>-<version> output directory
# (KERNELS defaults to ~/Documents/TCC/kernels).
#
# Columns:
#   FAULT   IIO_EV_TYPE_FAULT in the kernel's uapi types.h, looked up in
#           KDIR and in KDIR/source (an O= output directory links the source there)
#   gate    what the copy's own Makefile decides: "make check": pass or stop
#   build   the module built with kbuild directly (make -C KDIR M=copy),
#           bypassing the Makefile, with KBUILD_MODPOST_WARN=1
#   err/warn  compiler "error:"/"warning:" lines, plus modpost ERROR/WARNING.
#           Not counted: modpost's undefined-symbol warnings and "Module.symvers
#           is missing", because modules_prepare writes no Module.symvers and
#           every external symbol is then reported (hence KBUILD_MODPOST_WARN)
#   lines   source lines of the errors
#
# A compiler stops at a missing header, which hides every other error. When
# that happens a second pass supplies the header (asm/unaligned.h as a wrapper
# of linux/unaligned.h, anything else empty) and reports what was behind it,
# marked "+hdr".
#
# Exit status: 0 when every target built, 1 otherwise.

set -u

die() { echo "build-matrix: $*" >&2; exit 2; }

REV=HEAD
PATCHES=()
W=
OUT=
while getopts "r:p:Wo:h" opt; do
	case $opt in
	r) REV=$OPTARG ;;
	p) PATCHES+=("$(realpath "$OPTARG")") || exit 2 ;;
	W) W=1 ;;
	o) OUT=$OPTARG ;;
	h) sed -n '3,/^$/s/^# \{0,1\}//p' "$0"; exit 0 ;;
	*) exit 2 ;;
	esac
done
shift $((OPTIND - 1))
FILTERS=("$@")

REPO=$(git -C "$(dirname "$0")" rev-parse --show-toplevel) || die "not in a git tree"
SHA=$(git -C "$REPO" rev-parse --short "$REV^{commit}") || die "bad revision $REV"
KERNELS=${KERNELS:-$HOME/Documents/TCC/kernels}

if [ -n "$OUT" ]; then
	mkdir -p "$OUT" || exit 2
else
	OUT=$(mktemp -d "${TMPDIR:-/tmp}/ads1299-matrix.XXXXXX") || exit 2
fi
OUT=$(realpath "$OUT")

# name|KDIR|make arguments for the architecture
TARGETS=()
host=/lib/modules/$(uname -r)/build
if [ -d "$host" ]; then
	TARGETS+=("$(uname -r | cut -d- -f1)-host/$(uname -m)|$host|")
fi
for k in "$KERNELS"/build-*-*; do
	[ -d "$k" ] || continue
	b=${k##*/build-}
	arch=${b%%-*}
	ver=${b#*-}
	args="ARCH=$arch"
	[ "$arch" = arm64 ] && args="$args CROSS_COMPILE=aarch64-linux-gnu-"
	TARGETS+=("$ver/$arch|$k|$args")
done
# Sort by version, then architecture.
mapfile -t TARGETS < <(printf '%s\n' "${TARGETS[@]}" | sort -t'|' -k1,1V)

selected() {
	[ ${#FILTERS[@]} -eq 0 ] && return 0
	local f
	for f in "${FILTERS[@]}"; do
		[[ $1 == *"$f"* ]] && return 0
	done
	return 1
}

has_fault() {
	local f
	for f in "$1/include/uapi/linux/iio/types.h" \
		 "$1/source/include/uapi/linux/iio/types.h"; do
		grep -Eqs '^[[:space:]]*IIO_EV_TYPE_FAULT\b' "$f" && return 0
	done
	return 1
}

# count LOG -> "errors warnings lines"
count() {
	local e w l
	e=$(grep -Ec ': (fatal )?error:|^ERROR:' "$1")
	w=$(grep -E ': warning:|^WARNING:' "$1" |
	    grep -Evc '^WARNING: modpost: ("[^"]+" \[.*\] undefined!|suppressed [0-9]+ unresolved)|^WARNING: (.*/)?Module\.symvers is missing')
	l=$(grep -Eo 'ti-ads1299\.c:[0-9]+:[0-9]+: (fatal )?error:' "$1" |
	    cut -d: -f2 | sort -nu | paste -sd, -)
	echo "$e $w ${l:--}"
}

# kbuild KDIR SRC LOG ARGS [KCFLAGS]
kbuild() {
	# shellcheck disable=SC2086
	make -C "$1" M="$2" KBUILD_MODPOST_WARN=1 ${W:+W=1} $4 \
		${5:+KCFLAGS="$5"} modules >"$3" 2>&1
}

printf 'rev %s, logs in %s\n\n' "$SHA" "$OUT"
FMT='%-28s %-5s %-5s %-6s %4s %5s  %s\n'
printf "$FMT" kernel FAULT gate build err warn lines
failed=0
for t in "${TARGETS[@]}"; do
	IFS='|' read -r name kdir args <<<"$t"
	selected "$name" || continue
	dir=$OUT/${name//\//-}
	rm -rf "$dir"
	mkdir -p "$dir/src"
	git -C "$REPO" archive "$REV" | tar x -C "$dir/src" || die "archive failed"
	for p in "${PATCHES[@]}"; do
		patch -s -d "$dir/src" -p1 <"$p" || die "$p does not apply"
	done

	if has_fault "$kdir"; then fault=yes; else fault=no; fi

	# shellcheck disable=SC2086
	if make -s -C "$dir/src" KDIR="$kdir" $args check >"$dir/check.log" 2>&1; then
		gate=pass
	else
		gate=stop
	fi

	if kbuild "$kdir" "$dir/src" "$dir/build.log" "$args"; then
		res=ok
	else
		res=FAIL
		failed=1
	fi
	read -r e w l < <(count "$dir/build.log")
	printf "$FMT" "$name" "$fault" "$gate" "$res" "$e" "$w" "$l"

	# Errors hidden behind a missing header.
	log=$dir/build.log
	shim=$dir/shim
	pass=1
	while hdr=$(grep -Eo 'fatal error: [^:]+: No such file' "$log" |
		    head -1 | sed 's/fatal error: //; s/: No such file//') &&
	      [ -n "$hdr" ] && [ $pass -le 3 ]; do
		[ -e "$shim/$hdr" ] && break
		mkdir -p "$shim/$(dirname "$hdr")"
		case $hdr in
		asm/unaligned.h) echo '#include <linux/unaligned.h>' ;;
		*) : ;;
		esac >"$shim/$hdr"
		pass=$((pass + 1))
		log=$dir/build-hdr$pass.log
		kbuild "$kdir" "$dir/src" "$log" "$args" "-I$shim" && r=ok || r=FAIL
		read -r e w l < <(count "$log")
		printf "$FMT" "  +hdr $hdr" "" "" "$r" "$e" "$w" "$l"
	done
done
exit $failed
