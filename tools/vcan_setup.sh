#!/usr/bin/env bash
#
# Bring up a virtual CAN interface (vcan0) for the loopback example and any
# on-bus tests. Linux only; needs privileges (run with sudo, or as root).
#
#   sudo tools/vcan_setup.sh          # create + bring up vcan0
#   sudo tools/vcan_setup.sh down     # tear it down
#
set -euo pipefail

IFACE="${IFACE:-vcan0}"

if [[ "${1:-up}" == "down" ]]; then
    ip link set down "$IFACE" 2>/dev/null || true
    ip link delete "$IFACE" 2>/dev/null || true
    echo "$IFACE removed."
    exit 0
fi

modprobe vcan
if ! ip link show "$IFACE" >/dev/null 2>&1; then
    ip link add dev "$IFACE" type vcan
fi
ip link set up "$IFACE"

echo "$IFACE is up:"
ip -details -brief link show "$IFACE"
echo
echo "Now run:  cargo run -p canopen-host --example vcan_loopback"
