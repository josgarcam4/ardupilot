#!/bin/bash
# Interactive detector demo - see spoofing detection in real-time

set -e

SITL_BIN="./build/sitl/bin/arducopter"
HOME="-35.362938,149.165085,584.1,0"

if [ ! -f "$SITL_BIN" ]; then
    echo "ERROR: SITL binary not found"
    exit 1
fi

export PYTHONPATH="/home/joseantonio.garcia@sener.es/.local/lib/python3.10/site-packages:$PYTHONPATH"

echo "========================================"
echo "GNSS Spoof Detector - Interactive Demo"
echo "========================================"
echo ""
echo "This script will:"
echo "  1. Launch SITL in background"
echo "  2. Open MAVProxy for interactive commands"
echo "  3. Show GSPF detector messages in real-time"
echo ""

# Start SITL
echo "[1/3] Starting SITL..."
SITL_LOG="/tmp/sitl_demo.log"
$SITL_BIN -S -I0 --home=$HOME --model=quad --speedup=1 > $SITL_LOG 2>&1 &
SITL_PID=$!
echo "      SITL PID: $SITL_PID"

sleep 2

# Start log monitor in background
echo "[2/3] Starting real-time detector message monitor..."
(
    tail -f $SITL_LOG | grep --line-buffered -E "GSPF|Ready|EKF|GPS" &
    MONITOR_PID=$!
    trap "kill $MONITOR_PID 2>/dev/null" EXIT
    wait
) &
MONITOR_PID=$!

sleep 1

# Launch MAVProxy
echo "[3/3] Launching MAVProxy terminal..."
echo ""
echo "=========================================="
echo "INTERACTIVE COMMANDS TO RUN IN MAVPROXY:"
echo "=========================================="
echo ""
echo "1. Wait for heartbeat, then:"
echo "   arm throttle"
echo ""
echo "2. Take off to 15m:"
echo "   takeoff 15"
echo ""
echo "3. Switch to LOITER mode:"
echo "   mode LOITER"
echo ""
echo "4. Wait ~20 seconds for system to stabilize"
echo "   (watch for GSPF messages above)"
echo ""
echo "5. INJECT SPOOFING - set velocity errors:"
echo "   param set SIM_GPS1_VERR_X 1.5"
echo "   param set SIM_GPS1_VERR_Y 0.75"
echo ""
echo "6. Wait ~30 seconds and watch for:"
echo "   - 'GSPF: SUSPICIOUS' message (within 15 seconds)"
echo "   - 'GSPF: CONFIRMED SPOOFING' message (within 60 seconds)"
echo ""
echo "7. REMOVE SPOOFING:"
echo "   param set SIM_GPS1_VERR_X 0"
echo "   param set SIM_GPS1_VERR_Y 0"
echo ""
echo "8. Land and exit:"
echo "   land"
echo "   sleep 30"
echo "   quit"
echo ""
echo "=========================================="
echo ""

python3 /home/joseantonio.garcia@sener.es/.local/bin/mavproxy.py \
    --master=127.0.0.1:14550 \
    --logfile=/tmp/mavproxy_demo.log

# Cleanup
echo ""
echo "Cleaning up..."
kill $MONITOR_PID 2>/dev/null || true
kill $SITL_PID 2>/dev/null || true
wait $SITL_PID 2>/dev/null || true

echo ""
echo "========================================"
echo "Demo Complete"
echo "========================================"
echo "Log files saved:"
echo "  SITL:    $SITL_LOG"
echo "  MAVProxy: /tmp/mavproxy_demo.log"
echo ""
echo "To review detector messages:"
echo "  grep GSPF $SITL_LOG"
echo ""
