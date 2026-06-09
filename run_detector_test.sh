#!/bin/bash
# Fully automated detector test - just run and watch

set -e

# Ensure Python can find user-installed packages
export PYTHONPATH="/home/joseantonio.garcia@sener.es/.local/lib/python3.10/site-packages:$PYTHONPATH"

SITL_BIN="./build/sitl/bin/arducopter"
HOME="-35.362938,149.165085,584.1,0"

if [ ! -f "$SITL_BIN" ]; then
    echo "ERROR: SITL binary not found"
    exit 1
fi

echo "========================================"
echo "GNSS Spoof Detector - Automated Test"
echo "========================================"

# Start SITL
echo ""
echo "[1/5] Starting SITL simulator..."
$SITL_BIN -S -I0 --home=$HOME --model=quad --speedup=1 > /tmp/sitl_test.log 2>&1 &
SITL_PID=$!

# Give SITL a moment to start
sleep 2

# Now connect with MAVProxy and run commands
echo "[2/5] Connecting to SITL with MAVProxy..."
echo "      (SITL will initialize once MAVProxy connects)"
sleep 1

# Wait a bit for SITL to be ready
sleep 5

# Build command string for MAVProxy
MAVPROXY_CMD="arm throttle; takeoff 15; mode LOITER; sleep 25; param set SIM_GPS1_VERR_X 1.5; param set SIM_GPS1_VERR_Y 0.75; sleep 35; param set SIM_GPS1_VERR_X 0; param set SIM_GPS1_VERR_Y 0; land; sleep 30; quit"

# Run test with MAVProxy
echo "[3/4] Running spoofing injection test..."
timeout 180 python3 /home/joseantonio.garcia@sener.es/.local/bin/mavproxy.py --master=127.0.0.1:14550 \
    --cmd="$MAVPROXY_CMD" \
    --logfile=/tmp/mavproxy_test.log \
    --non-interactive > /tmp/mavproxy_output.log 2>&1 || true

sleep 2

# Kill SITL
echo ""
echo "[4/4] Cleaning up..."
kill $SITL_PID 2>/dev/null || true
wait $SITL_PID 2>/dev/null || true

# Analyze results
echo ""
echo "========================================"
echo "TEST RESULTS"
echo "========================================"

echo ""
echo "Looking for detector messages..."
GSPF_MESSAGES=$(grep -c "GSPF" /tmp/sitl_test.log || true)

if [ $GSPF_MESSAGES -gt 0 ]; then
    echo "✓ Found $GSPF_MESSAGES GSPF messages"
    echo ""
    echo "Detector messages:"
    grep "GSPF" /tmp/sitl_test.log | head -20

    if grep -q "SUSPICIOUS" /tmp/sitl_test.log; then
        echo "  ✓ SUSPICIOUS state detected"
    else
        echo "  ✗ SUSPICIOUS state NOT detected"
    fi

    if grep -q "CONFIRMED" /tmp/sitl_test.log; then
        echo "  ✓ CONFIRMED state detected"
    else
        echo "  ✗ CONFIRMED state NOT detected"
    fi
else
    echo "✗ No GSPF messages found"
    echo ""
    echo "Checking SITL initialization..."
    grep "Ready\|EKF\|GPS" /tmp/sitl_test.log | tail -20
fi

echo ""
echo "========================================"
echo "Log files:"
echo "  SITL:      /tmp/sitl_test.log"
echo "  MAVProxy:  /tmp/mavproxy_test.log"
echo "  Output:    /tmp/mavproxy_output.log"
echo "========================================"
echo ""
echo "To view detector messages:"
echo "  grep GSPF /tmp/sitl_test.log | head -30"
echo ""
