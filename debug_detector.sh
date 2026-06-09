#!/bin/bash
# Debug script to check detector initialization

SITL_BIN="./build/sitl/bin/arducopter"
HOME="-35.362938,149.165085,584.1,0"

echo "========================================"
echo "DETECTOR DEBUG SCRIPT"
echo "========================================"

# 1. Check binary
echo ""
echo "[1] Checking SITL binary..."
if [ -f "$SITL_BIN" ]; then
    echo "SITL binary found"
    SIZE=$(stat -f%z "$SITL_BIN" 2>/dev/null || stat -c%s "$SITL_BIN")
    echo "  Size: $SIZE bytes"
else
    echo "SITL binary NOT found at $SITL_BIN"
    exit 1
fi

# 2. Check for detector symbols
echo ""
echo "[2] Checking detector symbols in binary..."
if nm "$SITL_BIN" | grep -q "AP_GpsSpoofDetect"; then
    echo "Detector symbols found"
    nm "$SITL_BIN" | grep "AP_GpsSpoofDetect" | wc -l | xargs echo "  Count:"
else
    echo "Detector symbols NOT found"
    exit 1
fi

# 3. Launch SITL and check logs
echo ""
echo "[3] Launching SITL and checking initialization..."
echo "  Starting SITL (this takes ~15 seconds to fully initialize)..."
$SITL_BIN -S -I0 --home=$HOME --model=quad --speedup=1 > /tmp/sitl_init.log 2>&1 &
SITL_PID=$!

# Wait for SITL to fully initialize
echo "  Waiting for initialization..."
for i in {1..20}; do
    if grep -q "Ready to fly\|ArduPilot Ready\|EKF3.*active" /tmp/sitl_init.log; then
        echo "  SITL initialized after $((i*1)) seconds"
        break
    fi
    sleep 1
done

# 4. Check for GPS fix
echo ""
echo "[4] Checking SITL messages..."
if grep -q "GPS 1: detected" /tmp/sitl_init.log; then
    echo "GPS detected"
else
    echo "GPS not detected"
fi

if grep -q "EKF3" /tmp/sitl_init.log; then
    echo "EKF3 active"
else
    echo "EKF3 not active"
fi

if grep -q "AHRS" /tmp/sitl_init.log; then
    echo "AHRS ready"
else
    echo "AHRS not ready"
fi

if grep -q "ready\|Ready" /tmp/sitl_init.log; then
    echo "System ready message detected"
else
    echo "Ready message not found"
fi

# 5. Check for detector initialization
echo ""
echo "[5] Checking detector initialization..."
if grep -q "GSPF" /tmp/sitl_init.log; then
    echo "Detector messages found in SITL log"
    grep "GSPF" /tmp/sitl_init.log | head -5
else
    echo "No GSPF messages in early startup (expected - needs spoofing)"
fi

# Kill SITL
kill $SITL_PID 2>/dev/null || true
wait $SITL_PID 2>/dev/null || true

echo ""
echo "========================================"
echo "FULL SITL LOG: /tmp/sitl_init.log"
echo "========================================"
echo ""
echo "To continue testing manually, run:"
echo "  ./test_detector.sh"
echo ""
