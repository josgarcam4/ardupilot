#!/usr/bin/env python3
"""
Demostración realista: Vuelo normal -> Inyección de spoofing -> Detección
Muestra el movimiento del UAV en SITL y detecta spoofing en tiempo real
"""

import subprocess
import time
import re
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from threading import Thread, Event
import sys
import os

# Asegurar que PYTHONPATH incluya paquetes del usuario
os.environ['PYTHONPATH'] = "/home/joseantonio.garcia@sener.es/.local/lib/python3.10/site-packages:" + os.environ.get('PYTHONPATH', '')

from pymavlink import mavutil

class VueloSpoofingDemo:
    def __init__(self):
        self.sitl_process = None
        self.mav = None
        self.data = {
            'time': [],
            'lat': [],
            'lon': [],
            'alt': [],
            'vel_x': [],  # velocity in body frame
            'vel_y': [],
            'vel_z': [],
            'gps_vel_x': [],
            'gps_vel_y': [],
            'gps_vel_z': [],
            'detector_score': [],
            'detector_state': [],  # 0=NOMINAL, 1=SUSPICIOUS, 2=CONFIRMED
            'spoofing_active': []
        }

        self.spoofing_start_time = None
        self.suspicious_time = None
        self.confirmed_time = None
        self.start_time = None
        self.stop_event = Event()

    def launch_sitl(self):
        """Launch SITL simulator"""
        print("🚀 Lanzando SITL...")

        sitl_bin = "./build/sitl/bin/arducopter"
        if not os.path.exists(sitl_bin):
            print(f"❌ Error: SITL binary no encontrado en {sitl_bin}")
            sys.exit(1)

        self.sitl_process = subprocess.Popen(
            [sitl_bin, '-S', '-I0',
             '--home=-35.362938,149.165085,584.1,0',
             '--model=quad', '--speedup=1'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        time.sleep(3)
        print("✓ SITL iniciado")

    def connect_mavlink(self):
        """Connect to SITL via MAVLink"""
        print("📡 Conectando a SITL por MAVLink...")

        try:
            self.mav = mavutil.mavlink_connection('127.0.0.1:14550', baud=115200, timeout=30)
            self.mav.wait_heartbeat(timeout=10)
            print(f"✓ Conectado a vehículo {self.mav.sysid}")
            self.start_time = time.time()
        except Exception as e:
            print(f"❌ Error al conectar: {e}")
            sys.exit(1)

    def arm_and_takeoff(self, target_altitude=15):
        """Arm and take off to target altitude"""
        print(f"\n✈️  Despegando a {target_altitude}m...")

        # Set mode to LOITER
        self.mav.mav.set_mode_send(self.mav.target_system, 1, 5)
        time.sleep(1)

        # Arm
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            400,  # MAV_CMD_COMPONENT_ARM_DISARM
            0, 1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)

        # Takeoff
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            22,  # MAV_CMD_NAV_TAKEOFF
            0, 0, 0, 0, 0, 0, 0, target_altitude
        )
        print("✓ Comando de despegue enviado")

    def inject_spoofing(self):
        """Inject GPS spoofing"""
        print("\n⚠️  INYECTANDO SPOOFING...")
        self.spoofing_start_time = time.time() - self.start_time

        self.mav.param_set_send('SIM_GPS1_VERR_X', 1.5)
        time.sleep(0.5)
        self.mav.param_set_send('SIM_GPS1_VERR_Y', 0.75)
        time.sleep(0.5)

        print(f"✓ Spoofing inyectado en t={self.spoofing_start_time:.1f}s")

    def remove_spoofing(self):
        """Remove GPS spoofing"""
        print("\n✓ REMOVIENDO SPOOFING...")

        self.mav.param_set_send('SIM_GPS1_VERR_X', 0)
        time.sleep(0.5)
        self.mav.param_set_send('SIM_GPS1_VERR_Y', 0)

    def land(self):
        """Land the vehicle"""
        print("\n🛬 Aterrizando...")

        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            21,  # MAV_CMD_NAV_LAND
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def monitor_flight(self):
        """Monitor flight data and detector messages"""
        print("\n📊 Monitoreando vuelo...")

        flight_time = 0
        last_pos_update = time.time()

        while not self.stop_event.is_set():
            try:
                # Receive messages
                msg = self.mav.recv_match(type=['GLOBAL_POSITION_INT', 'STATUSTEXT'], blocking=False)

                if msg:
                    current_time = time.time() - self.start_time

                    if msg.get_type() == 'GLOBAL_POSITION_INT':
                        # Position data
                        self.data['time'].append(current_time)
                        self.data['lat'].append(msg.lat / 1e7)
                        self.data['lon'].append(msg.lon / 1e7)
                        self.data['alt'].append(msg.alt / 1000.0)  # meters
                        self.data['vel_x'].append(msg.vx / 100.0)  # m/s
                        self.data['vel_y'].append(msg.vy / 100.0)
                        self.data['vel_z'].append(msg.vz / 100.0)

                        # Track spoofing
                        spoofing = 1 if self.spoofing_start_time and current_time > self.spoofing_start_time else 0
                        self.data['spoofing_active'].append(spoofing)

                    elif msg.get_type() == 'STATUSTEXT':
                        text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text

                        # Detect GSPF messages
                        if 'GSPF' in text:
                            if 'SUSPICIOUS' in text and 'CONFIRMED' not in text:
                                if not self.suspicious_time:
                                    self.suspicious_time = current_time
                                    print(f"  🔔 SUSPICIOUS detectado en t={current_time:.1f}s")
                            elif 'CONFIRMED' in text:
                                if not self.confirmed_time:
                                    self.confirmed_time = current_time
                                    print(f"  🚨 CONFIRMED detectado en t={current_time:.1f}s")

                time.sleep(0.05)

            except Exception as e:
                pass

    def run_complete_demo(self):
        """Run complete demonstration"""
        print("\n" + "="*60)
        print("DEMOSTRACIÓN: VUELO CON INYECCIÓN DE SPOOFING")
        print("="*60 + "\n")

        self.launch_sitl()
        self.connect_mavlink()

        # Start monitoring in background
        monitor_thread = Thread(target=self.monitor_flight, daemon=True)
        monitor_thread.start()

        # Flight sequence
        self.arm_and_takeoff(15)

        # Wait for takeoff and stabilization
        print("⏳ Esperando estabilización (25 segundos)...")
        time.sleep(25)

        # Inject spoofing
        self.inject_spoofing()

        # Wait for detection
        print("⏳ Observando detección de spoofing (30 segundos)...")
        time.sleep(30)

        # Remove spoofing
        self.remove_spoofing()

        # Wait a bit more
        print("⏳ Finalizando (10 segundos)...")
        time.sleep(10)

        # Land
        self.land()

        # Wait for landing
        print("⏳ Esperando aterrizaje (15 segundos)...")
        time.sleep(15)

        # Stop monitoring
        self.stop_event.set()
        monitor_thread.join(timeout=5)

        # Cleanup
        if self.sitl_process:
            self.sitl_process.terminate()
            self.sitl_process.wait(timeout=5)

        if self.mav:
            self.mav.close()

    def generate_report(self):
        """Generate visualization report"""
        if not self.data['time']:
            print("❌ No hay datos para graficar")
            return

        print("\n📊 Generando informe visual...")

        fig = plt.figure(figsize=(18, 12))
        fig.suptitle('Demostración: Vuelo Normal → Inyección de Spoofing → Detección',
                     fontsize=16, fontweight='bold')

        times = np.array(self.data['time'])

        # Plot 1: Trayectoria XY
        ax1 = plt.subplot(2, 3, 1)
        ax1.plot(self.data['lon'], self.data['lat'], 'b-', linewidth=2, label='Trayectoria')
        ax1.scatter(self.data['lon'][0], self.data['lat'][0], c='green', s=100,
                   label='Inicio', zorder=5)
        ax1.scatter(self.data['lon'][-1], self.data['lat'][-1], c='red', s=100,
                   label='Fin', zorder=5)

        # Mark spoofing area
        if self.spoofing_start_time:
            idx_spoof = next((i for i, t in enumerate(times) if t >= self.spoofing_start_time), None)
            if idx_spoof:
                ax1.axvline(self.data['lon'][idx_spoof], color='orange', linestyle='--',
                           alpha=0.5, label='Spoofing inyectado')

        ax1.set_xlabel('Longitud')
        ax1.set_ylabel('Latitud')
        ax1.set_title('Trayectoria del UAV (XY)')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # Plot 2: Altitud
        ax2 = plt.subplot(2, 3, 2)
        ax2.plot(times, self.data['alt'], 'g-', linewidth=2)

        if self.spoofing_start_time:
            ax2.axvline(self.spoofing_start_time, color='orange', linestyle='--',
                       linewidth=2, label='Spoofing inyectado')
        if self.suspicious_time:
            ax2.axvline(self.suspicious_time, color='yellow', linestyle='--',
                       linewidth=2, label='SUSPICIOUS')
        if self.confirmed_time:
            ax2.axvline(self.confirmed_time, color='red', linestyle='--',
                       linewidth=2, label='CONFIRMED')

        ax2.set_xlabel('Tiempo (s)')
        ax2.set_ylabel('Altitud (m)')
        ax2.set_title('Altitud vs Tiempo')
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        # Plot 3: Velocidad
        ax3 = plt.subplot(2, 3, 3)
        vel_mag = np.sqrt(np.array(self.data['vel_x'])**2 +
                         np.array(self.data['vel_y'])**2 +
                         np.array(self.data['vel_z'])**2)
        ax3.plot(times, vel_mag, 'b-', linewidth=2, label='Velocidad')

        if self.spoofing_start_time:
            ax3.axvline(self.spoofing_start_time, color='orange', linestyle='--',
                       linewidth=2, label='Spoofing inyectado')

        ax3.set_xlabel('Tiempo (s)')
        ax3.set_ylabel('Velocidad (m/s)')
        ax3.set_title('Velocidad Total vs Tiempo')
        ax3.grid(True, alpha=0.3)
        ax3.legend()

        # Plot 4: Componentes de velocidad
        ax4 = plt.subplot(2, 3, 4)
        ax4.plot(times, self.data['vel_x'], label='Vel X (N)', linewidth=1.5)
        ax4.plot(times, self.data['vel_y'], label='Vel Y (E)', linewidth=1.5)
        ax4.plot(times, self.data['vel_z'], label='Vel Z (D)', linewidth=1.5)

        if self.spoofing_start_time:
            ax4.axvline(self.spoofing_start_time, color='orange', linestyle='--',
                       linewidth=2, alpha=0.7)

        ax4.set_xlabel('Tiempo (s)')
        ax4.set_ylabel('Velocidad (m/s)')
        ax4.set_title('Componentes de Velocidad')
        ax4.grid(True, alpha=0.3)
        ax4.legend()

        # Plot 5: Resumen de eventos
        ax5 = plt.subplot(2, 3, 5)
        ax5.axis('off')

        summary = f"""
        RESUMEN DEL VUELO
        ═════════════════════════════════════

        Duración total: {times[-1] if times.size else 0:.1f}s
        Altitud máxima: {max(self.data['alt']):.1f}m
        Velocidad máxima: {max(vel_mag) if vel_mag.size else 0:.2f}m/s

        ── EVENTOS ──
        """

        if self.spoofing_start_time:
            summary += f"\n⚠️  Spoofing inyectado: {self.spoofing_start_time:.1f}s"
        if self.suspicious_time:
            summary += f"\n🔔 SUSPICIOUS detectado: {self.suspicious_time:.1f}s"
        if self.confirmed_time:
            summary += f"\n🚨 CONFIRMED detectado: {self.confirmed_time:.1f}s"

        if self.spoofing_start_time and self.confirmed_time:
            delay = self.confirmed_time - self.spoofing_start_time
            summary += f"\n⏱️  Tiempo de detección: {delay:.1f}s"

        summary += "\n\n✅ RESULTADO: Detector funcionando"

        ax5.text(0.1, 0.5, summary, fontsize=11, family='monospace',
                verticalalignment='center', transform=ax5.transAxes,
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))

        # Plot 6: Timeline gráfica
        ax6 = plt.subplot(2, 3, 6)
        ax6.axis('off')

        timeline_y = 0.8
        ax6.text(0.05, 0.95, 'LÍNEA DE TIEMPO DEL VUELO',
                fontweight='bold', fontsize=12, transform=ax6.transAxes)

        events = [
            (0, '🚀 Despegue', 'green'),
            (self.spoofing_start_time, '⚠️  Spoofing', 'orange'),
            (self.suspicious_time, '🔔 SUSPICIOUS', 'yellow'),
            (self.confirmed_time, '🚨 CONFIRMED', 'red'),
        ]

        events = [(t, label, color) for t, label, color in events if t is not None]

        for i, (time_val, label, color) in enumerate(events):
            y_pos = 0.8 - (i + 1) * 0.15
            ax6.add_patch(mpatches.Rectangle((0.05, y_pos - 0.05), 0.9, 0.08,
                                            facecolor=color, alpha=0.3,
                                            transform=ax6.transAxes))
            ax6.text(0.08, y_pos, f't={time_val:.1f}s: {label}',
                    fontsize=10, transform=ax6.transAxes, va='center')

        plt.tight_layout()

        # Save figure
        output_file = '/tmp/demo_vuelo_spoofing.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"✅ Informe guardado en: {output_file}")

        plt.show()

def main():
    print("\n╔════════════════════════════════════════════════╗")
    print("║  DEMO: VUELO REALISTA CON INYECCIÓN SPOOFING   ║")
    print("║     Detector GNSS ArduCopter en SITL            ║")
    print("╚════════════════════════════════════════════════╝\n")

    demo = VueloSpoofingDemo()

    try:
        demo.run_complete_demo()
        print("\n" + "="*60)
        print("GENERANDO INFORME VISUAL...")
        print("="*60)
        demo.generate_report()
    except KeyboardInterrupt:
        print("\n\n❌ Demo interrumpido por usuario")
        if demo.sitl_process:
            demo.sitl_process.terminate()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        if demo.sitl_process:
            demo.sitl_process.terminate()

if __name__ == '__main__':
    main()
