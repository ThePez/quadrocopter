#!/usr/bin/env bash
#
# Checks that the board plugged in is the one a firmware image is meant for.
#
# usage: check_device.sh <drone|remote|bridge> [port] [force]
#
# Reads the connected chip's MAC address with esptool and compares it to the
# MAC that libs/espnow-comm/espnow_comm.c lists for that role, so the addresses
# are only ever defined in one place. A chip-type mismatch (ESP32 vs ESP32-S3)
# always fails; a MAC mismatch fails unless the third argument is "force".
#
# Set CHECK_DEVICE_HINT to change the command suggested in the "force" message.
#
# On success the serial port to use is printed on stdout (so the caller can
# flash to exactly the board that was checked); everything else goes to stderr.

set -euo pipefail

role="${1:-}"
port="${2:-}"
force="${3:-}"

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_file="$root/libs/espnow-comm/espnow_comm.c"

case "$role" in
    drone | remote) want_s3=0 ;;
    bridge) want_s3=1 ;;
    *)
        echo "Unknown target: '$role' (expected drone, remote, or bridge)" >&2
        exit 1
        ;;
esac

# Prints a role's MAC as lower case aa:bb:cc:dd:ee:ff, taken from the C array
# `const uint8_t <role>_mac[6] = {0x.., ...};` (works across line breaks).
mac_of() {
    tr '\n' ' ' <"$source_file" \
        | grep -oE "$1_mac\[6\] *= *\{[^}]*\}" \
        | sed -E 's/.*\{([^}]*)\}.*/\1/; s/0[xX]//g; s/[ ,]+/:/g; s/^:+//; s/:+$//' \
        | tr 'A-F' 'a-f'
}

expected="$(mac_of "$role" || true)"
if [ -z "$expected" ]; then
    echo "Could not find ${role}_mac in $source_file" >&2
    exit 1
fi

py="$(command -v python || command -v python3 || true)"
if [ -z "$py" ]; then
    echo "python not found on PATH. Run 'espidf' in this terminal first." >&2
    exit 1
fi

# No port given: use the one USB serial device that is plugged in. (esptool's own
# auto-detect probes every /dev/ttyS* port in turn, which is slow, noisy, and hard
# to read the chosen port back out of reliably.)
if [ -z "$port" ]; then
    shopt -s nullglob
    candidates=(/dev/ttyUSB* /dev/ttyACM* /dev/cu.usbserial* /dev/cu.usbmodem* /dev/cu.SLAB_USBtoUART*)
    shopt -u nullglob
    case "${#candidates[@]}" in
        0)
            echo "ERROR: no USB serial device found. Is the board plugged in?" >&2
            exit 1
            ;;
        1) port="${candidates[0]}" ;;
        *)
            echo "ERROR: several serial devices found: ${candidates[*]}" >&2
            echo "       Say which one: just flash $role <port>" >&2
            exit 1
            ;;
    esac
fi

port_args=(--port "$port")

# Reading the MAC makes the board reset afterwards, which is harmless
if ! out="$("$py" -m esptool ${port_args[@]+"${port_args[@]}"} read_mac 2>&1)"; then
    echo "$out" >&2
    echo "" >&2
    if echo "$out" | grep -qi 'No module named'; then
        echo "ERROR: esptool isn't available to this python. Run 'espidf' in this terminal first." >&2
    else
        echo "ERROR: could not read the connected device. Check the cable and port," >&2
        echo "       and that nothing else (a serial monitor?) is using the port." >&2
    fi
    exit 1
fi

found="$(echo "$out" | grep -i 'MAC' | grep -oiE '([0-9a-f]{2}:){5}[0-9a-f]{2}' | head -1 | tr 'A-F' 'a-f' || true)"
if [ -z "$found" ]; then
    echo "$out" >&2
    echo "ERROR: could not find a MAC address in esptool's output." >&2
    exit 1
fi

# Chip type: ESP32-S3 for the bridge, plain ESP32 for the drone and remote
is_s3=0
if echo "$out" | grep -qi 'ESP32-S3'; then
    is_s3=1
fi

if [ "$is_s3" != "$want_s3" ]; then
    if [ "$is_s3" = 1 ]; then
        echo "ERROR: the connected board is an ESP32-S3 (the bridge's chip), but '$role' is for a plain ESP32." >&2
    else
        echo "ERROR: the connected board is a plain ESP32 (drone or remote), but 'bridge' is for an ESP32-S3." >&2
    fi
    echo "       Refusing to flash." >&2
    exit 1
fi

if [ "$found" = "$expected" ]; then
    echo "Device check OK: $port is the $role ($found)" >&2
    echo "$port"
    exit 0
fi

# Wrong board for this role: say which one it is, if we know
actual="an unknown board (its MAC is not in espnow_comm.c)"
for r in drone remote bridge; do
    if [ "$(mac_of "$r" || true)" = "$found" ]; then
        actual="the $r"
    fi
done

echo "ERROR: the connected board is $actual ($found)," >&2
echo "       but you are flashing '$role' (expected $expected)." >&2

if [ "$force" = "force" ]; then
    echo "       'force' given, continuing anyway." >&2
    echo "$port"
    exit 0
fi

echo "       Refusing to flash. If this really is the right board, add 'force':" >&2
echo "       ${CHECK_DEVICE_HINT:-just flash $role} $port force" >&2
exit 1
