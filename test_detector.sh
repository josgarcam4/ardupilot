#!/bin/bash
# Test detector manually using MAVProxy

set -e

SITL_BIN="./build/sitl/bin/arducopter"
HOME="-35.362938,149.165085,584.1,0"

if [ ! -f "$SITL_BIN" ]; then
    echo "ERROR: SITL binary not found at $SITL_BIN"
    exit 1
fi

echo "========================================"
echo "Starting SITL..."
echo "========================================"
$SITL_BIN -S -I0 --home=$HOME --model=quad --speedup=1 > /tmp/sitl.log 2>&1 &
SITL_PID=$!
echo "SITL PID: $SITL_PID"

echo "Waiting for SITL to initialize (this takes ~15-20 seconds)..."
for i in {1..30}; do
    if grep -q "Ready to fly\|ArduPilot Ready" /tmp/sitl.log; then
        echo "✓ SITL Ready!"
        break
    fi
    echo -n "."
    sleep 1
done
echo ""

echo ""
echo "========================================"
echo "Starting MAVProxy terminal..."
echo "========================================"
echo "Run these commands in MAVProxy:"
echo "  1. arm throttle"
echo "  2. takeoff 15"
echo "  3. mode LOITER"
echo "  4. wait 20 seconds (for EKF to stabilize)"
echo "  5. param set SIM_GPS1_VERR_X 1.5"
echo "  6. param set SIM_GPS1_VERR_Y 0.75"
echo "  7. watch STATUSTEXT (to see GSPF messages)"
echo "  8. wait 30 seconds for SUSPICIOUS/CONFIRMED"
echo "  9. param set SIM_GPS1_VERR_X 0"
echo " 10. param set SIM_GPS1_VERR_Y 0"
echo " 11. land"
echo ""
echo "Look for messages like:"
echo "  AP: GSPF: SUSPICIOUS score=X.XX"
echo "  AP: GSPF: CONFIRMED SPOOFING score=X.XX"
echo ""
echo "========================================"
echo "Press Ctrl+C to stop"
echo "========================================"

python3 -m MAVProxy.mavproxy --master=127.0.0.1:14550 --logfile=/tmp/mavproxy.log

# Cleanup
echo ""
echo "Stopping SITL..."
kill $SITL_PID 2>/dev/null || true
wait $SITL_PID 2>/dev/null || true

echo "SITL log saved to: /tmp/sitl.log"
echo "MAVProxy log saved to: /tmp/mavproxy.log"
echo ""
echo "Check SITL log for GSPF messages:"
echo "  grep GSPF /tmp/sitl.log"
