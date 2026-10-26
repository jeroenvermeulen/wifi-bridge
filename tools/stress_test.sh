#!/bin/sh
# Longer traffic test through the bridge, with the board's log alongside.
# bridge_test.sh is too short to catch faults that happen once per 10-20 MB.
# Runs N downloads and N uploads of SIZE bytes, counts curl failures, and
# compares the frames the Mac sent/received on IFACE with the bridge's
# counters from the stats log. Starts the log reader itself, since opening
# a USB-serial port resets the board (see serial_log.py).
# Usage: tools/stress_test.sh [IFACE] [PORT] [N] [SIZE] [LOGFILE]
IFACE=${1:-en20}
PORT=${2:-$(ls /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null | head -1)}
N=${3:-16}
SIZE=${4:-5000000}
LOG=${5:-/tmp/bridge_stress_$(date +%H%M%S).log}
PY=${PY:-$HOME/.platformio/penv/bin/python}
cd "$(dirname "$0")/.." || exit 1

if lsof "$PORT" >/dev/null 2>&1; then
    echo "$PORT is already open by another program; two readers garble the log" >&2
    exit 1
fi
"$PY" tools/serial_log.py "$PORT" 1200 >"$LOG" 2>&1 &
LOGPID=$!
trap 'kill $LOGPID 2>/dev/null' EXIT

echo "== waiting for the board (log: $LOG)"
i=0
until grep -q "WiFi connected" "$LOG"; do
    sleep 1
    i=$((i + 1))
    if [ $i -ge 90 ]; then echo "board did not connect to WiFi"; exit 1; fi
done
sleep 5  # let the block-ack sessions come up
ifconfig "$IFACE" | grep -E "media"
grep -E "AP .* rssi" "$LOG" | tail -1

pkts() { netstat -I "$IFACE" -b | awk 'NR==2 {print $5, $8}'; }
set -- $(pkts)
IN0=$1
OUT0=$2
T0=$(date +%s)

run() {  # run DIR N
    total=0
    fail=0
    i=0
    while [ $i -lt "$2" ]; do
        if [ "$1" = down ]; then
            out=$(curl -sS --interface "$IFACE" -o /dev/null --max-time 120 -w "%{speed_download}" \
                "https://speed.cloudflare.com/__down?bytes=$SIZE" 2>&1)
        else
            out=$(head -c "$SIZE" /dev/urandom | curl -sS --interface "$IFACE" -o /dev/null --max-time 120 \
                -w "%{speed_upload}" --data-binary @- https://speed.cloudflare.com/__up 2>&1)
        fi
        case "$out" in
        *curl*) fail=$((fail + 1)); echo "  $1 $i: $out" ;;
        *) total=$(echo "$total + $out" | bc) ;;
        esac
        i=$((i + 1))
    done
    ok=$(($2 - fail))
    [ $ok -gt 0 ] && avg=$(echo "$total / $ok / 1000" | bc) || avg=0
    echo "== $1: $ok of $2 ok, avg $avg KB/s, $fail failed"
}

echo "== $N downloads of $SIZE bytes"
run down "$N"
echo "== $N uploads of $SIZE bytes"
run up "$N"

set -- $(pkts)
echo "== $IFACE in $(($1 - IN0)) frames, out $(($2 - OUT0)) frames, $(($(date +%s) - T0)) s"
sleep 12  # one more stats line
kill $LOGPID 2>/dev/null
echo "== board log: warnings and last stats"
grep -E "^(W|E) \(" "$LOG" | grep -v "other source" | sort | uniq -c | sort -rn | head -20
grep -E "bridge: (wifi=|internal heap|AP )" "$LOG" | tail -3
