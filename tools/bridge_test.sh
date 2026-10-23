#!/bin/sh
# Traffic test through the bridge. IFACE is the Mac interface wired to the
# bridge's RJ45 port; traffic is bound to it so it can't take another route.
# Usage: tools/bridge_test.sh [IFACE] [GATEWAY]
IFACE=${1:-en20}
GW=${2:-$(route -n get default | awk '/gateway/ {print $2}')}

ifconfig "$IFACE" | grep -E "media|inet "
echo "== ping $GW x100 =="
ping -b "$IFACE" -c 100 -i 0.1 "$GW" | tail -2
echo "== download 5 MB =="
curl -sS --interface "$IFACE" -o /dev/null --max-time 60 -w "%{speed_download} B/s in %{time_total}s\n" \
    "https://speed.cloudflare.com/__down?bytes=5000000"
echo "== upload 3 MB =="
head -c 3000000 /dev/zero | curl -sS --interface "$IFACE" -o /dev/null --max-time 60 \
    -w "%{speed_upload} B/s in %{time_total}s\n" --data-binary @- https://speed.cloudflare.com/__up
