#!/bin/bash
# Lanza SITL y MAVProxy configurados para QGroundControl

set -e

SITL_BIN="./build/sitl/bin/arducopter"
HOME="-35.362938,149.165085,584.1,0"

if [ ! -f "$SITL_BIN" ]; then
    echo "❌ ERROR: SITL binary no encontrado en $SITL_BIN"
    exit 1
fi

export PYTHONPATH="/home/joseantonio.garcia@sener.es/.local/lib/python3.10/site-packages:$PYTHONPATH"

# Kill any previous processes
pkill -9 arducopter 2>/dev/null || true
pkill -9 mavproxy 2>/dev/null || true
sleep 1

echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║                                                           ║"
echo "║      LANZANDO SITL PARA QGROUNDCONTROL                    ║"
echo "║                                                           ║"
echo "║  Pasos:                                                  ║"
echo "║  1. Se lanzará SITL (simulador)                          ║"
echo "║  2. Se lanzará MAVProxy (que desbloquea SITL)            ║"
echo "║  3. QGroundControl se conectará automáticamente          ║"
echo "║                                                           ║"
echo "║  Verás el UAV en el mapa de QGroundControl               ║"
echo "║                                                           ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Start SITL
echo " [1/2] Lanzando SITL..."
SITL_LOG="/tmp/sitl_qgc.log"
$SITL_BIN -S -I0 --home=$HOME --model=quad --speedup=1 > $SITL_LOG 2>&1 &
SITL_PID=$!
echo "       SITL PID: $SITL_PID"
echo "       Log: $SITL_LOG"

sleep 2

# Start MAVProxy (bridge to unlock SITL and expose to QGC)
echo ""
echo " [2/2] Lanzando MAVProxy (esto desbloquea SITL)..."
python3 /home/joseantonio.garcia@sener.es/.local/bin/mavproxy.py \
    --master=127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    --logfile=/tmp/mavproxy_qgc.log \
    --non-interactive > /tmp/mavproxy_qgc_output.log 2>&1 &
MAVPROXY_PID=$!
echo "       MAVProxy PID: $MAVPROXY_PID"

sleep 2

echo ""
echo "═══════════════════════════════════════════════════════════"
echo ""
echo " SITL y MAVProxy están corriendo"
echo ""
echo " QGroundControl debería conectarse automáticamente"
echo "   Si no, abre QGroundControl manualmente."
echo ""
echo " Mantén esta terminal abierta..."
echo "   Presiona Ctrl+C para detener"
echo ""
echo "═══════════════════════════════════════════════════════════"
echo ""

# Wait for interruption
trap 'echo ""; echo "Deteniendo..."; kill $MAVPROXY_PID 2>/dev/null; kill $SITL_PID 2>/dev/null; wait 2>/dev/null; echo "Detenido"; exit 0' SIGINT SIGTERM

wait

echo ""
echo "Logs guardados en:"
echo "  SITL:     $SITL_LOG"
echo "  MAVProxy: /tmp/mavproxy_qgc.log"
echo ""
