#!/bin/bash
# Demostración simple y clara
# Lanza SITL, MAVProxy con visualización interactiva, despegue automático y spoofing

set -e

SITL_BIN="./build/sitl/bin/arducopter"
HOME="-35.362938,149.165085,584.1,0"

if [ ! -f "$SITL_BIN" ]; then
    echo "❌ ERROR: SITL binary no encontrado"
    exit 1
fi

export PYTHONPATH="/home/joseantonio.garcia@sener.es/.local/lib/python3.10/site-packages:$PYTHONPATH"

echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║                                                           ║"
echo "║    DEMOSTRACIÓN: VUELO CON DETECTOR DE SPOOFING          ║"
echo "║                                                           ║"
echo "║  Se lanzará:                                             ║"
echo "║  1. SITL (simulador)                                     ║"
echo "║  2. MAVProxy (interfaz de control)                       ║"
echo "║                                                           ║"
echo "║  Luego ejecutarás comandos manualmente en MAVProxy:      ║"
echo "║  - arm throttle                                          ║"
echo "║  - takeoff 15                                            ║"
echo "║  - mode LOITER                                           ║"
echo "║  - esperar 20 segundos                                   ║"
echo "║  - param set SIM_GPS1_VERR_X 1.5  (INYECTAR SPOOFING)   ║"
echo "║  - param set SIM_GPS1_VERR_Y 0.75                        ║"
echo "║  - OBSERVAR mensajes GSPF en otra terminal               ║"
echo "║  - param set SIM_GPS1_VERR_X 0 (remover spoofing)       ║"
echo "║  - param set SIM_GPS1_VERR_Y 0                           ║"
echo "║  - land                                                  ║"
echo "║                                                           ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# Kill any previous processes
pkill -9 arducopter 2>/dev/null || true
pkill -9 mavproxy 2>/dev/null || true
sleep 1

# Start SITL in background
echo "🚀 [1/3] Lanzando SITL..."
SITL_LOG="/tmp/sitl_demo.log"
$SITL_BIN -S -I0 --home=$HOME --model=quad --speedup=1 > $SITL_LOG 2>&1 &
SITL_PID=$!
echo "       SITL PID: $SITL_PID"

sleep 2

# Start log monitor in background to show GSPF messages
echo "📊 [2/3] Iniciando monitor de mensajes del detector..."
(
    tail -f $SITL_LOG | grep --line-buffered "GSPF\|Ready\|EKF" &
    MONITOR_PID=$!
    trap "kill $MONITOR_PID 2>/dev/null" EXIT
    wait
) &
MONITOR_PID=$!

sleep 1

# Launch MAVProxy
echo "📡 [3/3] Lanzando MAVProxy (terminal interactiva)..."
echo ""
echo "═══════════════════════════════════════════════════════════"
echo ""

python3 /home/joseantonio.garcia@sener.es/.local/bin/mavproxy.py \
    --master=127.0.0.1:14550 \
    --logfile=/tmp/mavproxy.log

# Cleanup
echo ""
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "🛑 Deteniendo simulación..."

kill $MONITOR_PID 2>/dev/null || true
kill $SITL_PID 2>/dev/null || true
wait $SITL_PID 2>/dev/null || true

echo "✅ Demo completada"
echo ""
echo "Archivos de log:"
echo "  SITL:     $SITL_LOG"
echo "  MAVProxy: /tmp/mavproxy.log"
echo ""
echo "Para revisar mensajes del detector:"
echo "  grep GSPF $SITL_LOG"
echo ""
