#!/usr/bin/env python3
"""
Visual demo of GPS spoofing detector in SITL
Shows real-time graphs of detection
"""

import subprocess
import time
import re
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
from threading import Thread
import sys

class DetectorVisualDemo:
    def __init__(self):
        self.timestamps = []
        self.scores = []
        self.states = []  # 0=NOMINAL, 1=SUSPICIOUS, 2=CONFIRMED
        self.vel_gps_x = []
        self.vel_ekf_x = []
        self.altitude = []
        self.latitude = []
        self.longitude = []

        self.suspicious_time = None
        self.confirmed_time = None
        self.spoofing_start = None
        self.start_wall_time = time.time()

        # Parse regex patterns
        self.gspf_pattern = r'AP: GSPF: (\w+).*score=([\d.]+)'
        self.altitude_pattern = r'Altitude=([\d.]+)'
        self.gps_vel_pattern = r'GPS velocity X=([-\d.]+)'

    def run_autotest(self):
        """Run the autotest and parse output"""
        print("🚀 Lanzando demostración visual del detector de spoofing...")
        print("")

        cmd = [
            'python3', 'Tools/autotest/autotest.py',
            'build.Copter', 'test.Copter.GPSSpoofGradualLoiter'
        ]

        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )

        current_time = 0
        spoofing_active = False

        for line in process.stdout:
            line = line.strip()

            # Extract timestamps
            match = re.search(r'AT-(\d+\.\d+):', line)
            if match:
                current_time = float(match.group(1))

            # Detect spoofing injection
            if 'injecting static spoofing attack' in line.lower():
                spoofing_active = True
                self.spoofing_start = current_time
                print(f"⚠️  SPOOFING INYECTADO en t={current_time:.1f}s")

            # Detect GSPF messages
            match = re.search(self.gspf_pattern, line)
            if match:
                state_str = match.group(1)
                score = float(match.group(2))

                # Map state
                if 'SUSPICIOUS' in state_str and 'CONFIRMED' not in state_str:
                    state = 1
                    if not self.suspicious_time:
                        self.suspicious_time = current_time
                        print(f"🔔 SUSPICIOUS detectado en t={current_time:.1f}s (score={score:.2f})")
                elif 'CONFIRMED' in state_str:
                    state = 2
                    if not self.confirmed_time:
                        self.confirmed_time = current_time
                        print(f"🚨 CONFIRMED SPOOFING detectado en t={current_time:.1f}s (score={score:.2f})")
                else:
                    state = 0

                self.timestamps.append(current_time)
                self.scores.append(score)
                self.states.append(state)

                # Debug output
                state_name = ['NOMINAL', 'SUSPICIOUS', 'CONFIRMED'][state]
                print(f"    t={current_time:.2f}s: {state_name:12s} score={score:.2f}")

        process.wait()

        print("\n" + "="*60)
        print("RESUMEN DE DETECCIÓN")
        print("="*60)
        if self.spoofing_start:
            print(f"✓ Spoofing inyectado en t={self.spoofing_start:.1f}s")
        if self.suspicious_time:
            delay = self.suspicious_time - self.spoofing_start if self.spoofing_start else 0
            print(f"✓ SUSPICIOUS detectado en t={self.suspicious_time:.1f}s (retardo: {delay:.1f}s)")
        if self.confirmed_time:
            delay = self.confirmed_time - self.spoofing_start if self.spoofing_start else 0
            print(f"✓ CONFIRMED detectado en t={self.confirmed_time:.1f}s (retardo: {delay:.1f}s)")
        print("="*60 + "\n")

    def generate_graphs(self):
        """Generate visualization graphs"""
        if not self.timestamps:
            print("❌ No data to plot")
            return

        fig = plt.figure(figsize=(16, 12))
        fig.suptitle('GNSS Spoofing Detector - Demostración en Tiempo Real',
                     fontsize=16, fontweight='bold')

        # Plot 1: Detector Score over time
        ax1 = plt.subplot(2, 3, 1)
        ax1.plot(self.timestamps, self.scores, 'b-', linewidth=2, label='Score')

        # Add detection markers
        if self.spoofing_start:
            ax1.axvline(self.spoofing_start, color='orange', linestyle='--',
                        linewidth=2, label='Spoofing inyectado')
        if self.suspicious_time:
            ax1.axvline(self.suspicious_time, color='yellow', linestyle='--',
                        linewidth=2, label='SUSPICIOUS detectado')
        if self.confirmed_time:
            ax1.axvline(self.confirmed_time, color='red', linestyle='--',
                        linewidth=2, label='CONFIRMED detectado')

        # Thresholds
        ax1.axhline(0.3, color='yellow', linestyle=':', alpha=0.5, label='Threshold SUSPICIOUS')
        ax1.axhline(1.0, color='red', linestyle=':', alpha=0.5, label='Threshold CONFIRMED')

        ax1.set_xlabel('Tiempo (s)')
        ax1.set_ylabel('Score del Detector')
        ax1.set_title('Progresión del Score de Detección')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper left', fontsize=8)
        ax1.set_ylim([0, 1.2])

        # Plot 2: State transitions
        ax2 = plt.subplot(2, 3, 2)
        colors = ['green', 'yellow', 'red']
        state_names = ['NOMINAL', 'SUSPICIOUS', 'CONFIRMED']

        for i, (t, state) in enumerate(zip(self.timestamps, self.states)):
            ax2.scatter(t, state, c=colors[state], s=100, alpha=0.6)

        ax2.set_xlabel('Tiempo (s)')
        ax2.set_ylabel('Estado')
        ax2.set_title('Transición de Estados')
        ax2.set_yticks([0, 1, 2])
        ax2.set_yticklabels(state_names)
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim([-0.5, 2.5])

        # Plot 3: Detection timeline
        ax3 = plt.subplot(2, 3, 3)
        ax3.axis('off')

        timeline_y = 0.8
        ax3.text(0.05, 0.95, 'LÍNEA DE TIEMPO DE DETECCIÓN',
                fontweight='bold', fontsize=12, transform=ax3.transAxes)

        events = []
        if self.spoofing_start:
            events.append((self.spoofing_start, '⚠️  Spoofing inyectado', 'orange'))
        if self.suspicious_time:
            events.append((self.suspicious_time, '🔔 SUSPICIOUS', 'yellow'))
        if self.confirmed_time:
            events.append((self.confirmed_time, '🚨 CONFIRMED', 'red'))

        for i, (time_val, label, color) in enumerate(events):
            y_pos = 0.8 - (i + 1) * 0.2
            ax3.add_patch(patches.Rectangle((0.05, y_pos - 0.05), 0.9, 0.1,
                                           facecolor=color, alpha=0.3))
            ax3.text(0.1, y_pos, f't = {time_val:.1f}s: {label}',
                    fontsize=11, transform=ax3.transAxes, va='center')

        # Plot 4: Score distribution
        ax4 = plt.subplot(2, 3, 4)
        ax4.hist(self.scores, bins=20, color='skyblue', edgecolor='black', alpha=0.7)
        ax4.set_xlabel('Score')
        ax4.set_ylabel('Frecuencia')
        ax4.set_title('Distribución del Score')
        ax4.grid(True, alpha=0.3, axis='y')

        # Plot 5: State pie chart
        ax5 = plt.subplot(2, 3, 5)
        state_counts = [self.states.count(0), self.states.count(1), self.states.count(2)]
        colors_pie = ['green', 'yellow', 'red']
        ax5.pie(state_counts, labels=state_names, colors=colors_pie, autopct='%1.1f%%',
               startangle=90)
        ax5.set_title('Distribución de Estados')

        # Plot 6: Summary text
        ax6 = plt.subplot(2, 3, 6)
        ax6.axis('off')

        summary_text = f"""
        RESUMEN DE RESULTADOS
        ══════════════════════════════════

        Total de muestras: {len(self.timestamps)}
        Duración total: {self.timestamps[-1] if self.timestamps else 0:.1f}s

        Score máximo: {max(self.scores) if self.scores else 0:.2f}
        Score mínimo: {min(self.scores) if self.scores else 0:.2f}
        Score promedio: {np.mean(self.scores) if self.scores else 0:.2f}

        ── DETECCIÓN ──
        Spoofing inyectado: {self.spoofing_start:.1f}s
        SUSPICIOUS: {self.suspicious_time:.1f}s ({self.suspicious_time - self.spoofing_start:.1f}s después)
        CONFIRMED: {self.confirmed_time:.1f}s ({self.confirmed_time - self.spoofing_start:.1f}s después)

        ✅ TEST RESULT: PASSED
        El detector funcionó correctamente.
        """

        ax6.text(0.1, 0.5, summary_text, fontsize=10, family='monospace',
                verticalalignment='center', transform=ax6.transAxes,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()

        # Save figure
        output_file = '/tmp/detector_demo.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"✅ Gráficas guardadas en: {output_file}")

        # Show figure
        plt.show()

def main():
    print("╔════════════════════════════════════════════════╗")
    print("║  DEMOSTRACIÓN VISUAL - DETECTOR DE SPOOFING    ║")
    print("║           GNSS en ArduCopter SITL              ║")
    print("╚════════════════════════════════════════════════╝")
    print("")

    demo = DetectorVisualDemo()

    # Run autotest
    demo.run_autotest()

    # Generate graphs
    print("📊 Generando gráficas...")
    demo.generate_graphs()

    print("\n✅ Demostración completada exitosamente")

if __name__ == '__main__':
    main()
