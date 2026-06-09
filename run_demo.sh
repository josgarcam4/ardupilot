#!/bin/bash
# Demostración correcta: sim_vehicle.py + QGroundControl

echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║                                                           ║"
echo "║    DEMOSTRACIÓN DETECTOR DE SPOOFING - FORMA CORRECTA     ║"
echo "║                                                           ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

cd ArduCopter

echo "🚀 Lanzando SITL con sim_vehicle.py..."
echo ""
echo "Esto abrirá:"
echo "  ✓ SITL (simulador)"
echo "  ✓ MAVProxy (controlador)"
echo "  ✓ Consola de MAVProxy"
echo "  ✓ Mapa (si QGroundControl está abierto)"
echo ""
echo "QGroundControl debería conectarse automáticamente."
echo ""
echo "═══════════════════════════════════════════════════════════"
echo ""

../Tools/autotest/sim_vehicle.py --map --console

echo ""
echo "Demo finalizada"
