#!/usr/bin/env python3
"""
metrics.py - Evaluación exhaustiva del detector de GPS Spoofing (GSPF)

Conecta a SITL, ejecuta escenarios de spoofing variados y genera métricas
detalladas de rendimiento del detector:
  - Tiempo de detección (latencia)
  - Tasa de falsos positivos / falsos negativos
  - Curvas ROC si se varían umbrales
  - Evolución temporal del score y estados
  - Matriz de confusión
  - Análisis por intensidad de spoofing

Uso:
  1) Lanzar SITL:  ./build/sitl/bin/arducopter -S -I0 --home=-35.362938,149.165085,584.1,0 --model=quad --speedup=1
  2) Ejecutar:     python3 metrics.py [--output-dir ./metrics_output]

No modifica ningún archivo existente del proyecto.
"""

import subprocess
import time
import os
import sys
import json
import argparse
import re
import signal
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Dict, Tuple
from threading import Thread, Event

import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend for saving files
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.gridspec import GridSpec
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("⚠️  matplotlib no disponible. Se generarán solo métricas numéricas (sin gráficas).")

_mavlink_path = os.path.expanduser("~/.local/lib/python3.10/site-packages")
os.environ['PYTHONPATH'] = _mavlink_path + ":" + os.environ.get('PYTHONPATH', '')
sys.path.insert(0, _mavlink_path)

from pymavlink import mavutil


# ─────────────────────────────────────────────────────────────────────────────
# Data classes
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class DetectorEvent:
    """Un evento reportado por el detector via STATUSTEXT."""
    timestamp: float          # seconds since start
    state: str                # 'NOMINAL', 'SUSPICIOUS', 'CONFIRMED'
    score: Optional[float] = None
    raw_text: str = ''


@dataclass
class SpoofingScenario:
    """Definición de un escenario de spoofing."""
    name: str
    verr_x: float             # SIM_GPS1_VERR_X
    verr_y: float             # SIM_GPS1_VERR_Y
    duration: float = 30.0    # seconds spoofing is active
    description: str = ''


@dataclass
class ScenarioResult:
    """Resultado de un escenario de test."""
    scenario: SpoofingScenario
    # Timing
    spoofing_injected_at: float = 0.0
    first_suspicious_at: Optional[float] = None
    first_confirmed_at: Optional[float] = None
    spoofing_removed_at: Optional[float] = None
    recovery_to_nominal_at: Optional[float] = None
    # Derived
    latency_to_suspicious: Optional[float] = None
    latency_to_confirmed: Optional[float] = None
    recovery_time: Optional[float] = None
    # Classification
    true_positive: bool = False       # Detected while spoofing active
    false_negative: bool = False      # Failed to detect during spoofing
    # Time series
    times: List[float] = field(default_factory=list)
    scores: List[float] = field(default_factory=list)
    states: List[int] = field(default_factory=list)  # 0,1,2
    velocities: List[float] = field(default_factory=list)
    altitudes: List[float] = field(default_factory=list)
    spoofing_mask: List[bool] = field(default_factory=list)
    events: List[DetectorEvent] = field(default_factory=list)


@dataclass
class BaselineResult:
    """Resultado del vuelo sin spoofing (para medir FP)."""
    duration: float = 0.0
    max_score: float = 0.0
    false_positives: int = 0  # Times state went above NOMINAL
    times: List[float] = field(default_factory=list)
    scores: List[float] = field(default_factory=list)
    states: List[int] = field(default_factory=list)
    events: List[DetectorEvent] = field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────────────
# Scenarios definition
# ─────────────────────────────────────────────────────────────────────────────

SCENARIOS = [
    SpoofingScenario(
        name="low_intensity",
        verr_x=0.5, verr_y=0.25,
        duration=30,
        description="Spoofing de baja intensidad (drift lento)"
    ),
    SpoofingScenario(
        name="medium_intensity",
        verr_x=1.5, verr_y=0.75,
        duration=30,
        description="Spoofing de intensidad media (caso base)"
    ),
    SpoofingScenario(
        name="high_intensity",
        verr_x=3.0, verr_y=2.0,
        duration=30,
        description="Spoofing de alta intensidad (drift rápido)"
    ),
    SpoofingScenario(
        name="very_high_intensity",
        verr_x=5.0, verr_y=3.0,
        duration=20,
        description="Spoofing muy agresivo"
    ),
    SpoofingScenario(
        name="single_axis",
        verr_x=2.0, verr_y=0.0,
        duration=30,
        description="Spoofing solo en eje X"
    ),
    SpoofingScenario(
        name="brief_spoofing",
        verr_x=2.0, verr_y=1.0,
        duration=10,
        description="Spoofing breve (10s) - test de detección rápida"
    ),
]


# ─────────────────────────────────────────────────────────────────────────────
# SITL Manager
# ─────────────────────────────────────────────────────────────────────────────

class SITLManager:
    """
    Launches ArduCopter SITL + MAVProxy (non-interactive).
    MAVProxy is required to send the initial GCS heartbeat that kicks SITL's
    internal clock. pymavlink then connects to MAVProxy's UDP output on 14550.
    """

    HOME = "-35.362938,149.165085,584.1,0"
    MAVPROXY = os.path.expanduser("~/.local/bin/mavproxy.py")
    SITL_LOG  = '/tmp/sitl_metrics.log'
    MAV_LOG   = '/tmp/mavproxy_metrics.log'

    def __init__(self, speedup: int = 1):
        self.sitl_proc     = None
        self.mavproxy_proc = None
        self.speedup       = speedup
        self.repo_root     = os.path.dirname(os.path.abspath(__file__))

    def _kill_stale(self):
        os.system("pkill -9 -f 'arducopter.*-I0' 2>/dev/null; "
                  "pkill -9 -f 'mavproxy.*5760'  2>/dev/null; sleep 1")

    def _wait_log(self, path, text, timeout=40, poll=0.2) -> bool:
        deadline = time.time() + timeout
        dots = 0
        while time.time() < deadline:
            try:
                if text in open(path).read():
                    return True
            except IOError:
                pass
            time.sleep(poll)
            dots += 1
            if dots % 5 == 0:
                print('.', end='', flush=True)
        return False

    def start(self) -> bool:
        sitl_bin = os.path.join(self.repo_root, 'build', 'sitl', 'bin', 'arducopter')
        defaults = os.path.join(self.repo_root, 'Tools', 'autotest',
                                'default_params', 'copter.parm')

        if not os.path.exists(sitl_bin):
            print(f"❌ Binary no encontrado: {sitl_bin}")
            print(f"   Compila: ./waf configure --board=sitl && ./waf copter")
            return False
        if not os.path.exists(self.MAVPROXY):
            print(f"❌ mavproxy.py no encontrado en {self.MAVPROXY}")
            return False

        self._kill_stale()

        # ── 1. Start SITL ────────────────────────────────────────────────────
        # Use same flags as sim_vehicle.py: --model + --slave 0 --sim-address=127.0.0.1
        # (NOT -S, NOT --model quad — those cause "Waiting for internal clock bits" deadlock)
        sitl_cmd = [
            sitl_bin,
            '--model', '+',
            '--speedup', str(self.speedup),
            '--slave', '0',
            f'--home={self.HOME}',
            '--sim-address=127.0.0.1',
            '-I0',
        ]
        if os.path.exists(defaults):
            sitl_cmd += ['--defaults', defaults]

        print(f"🚀 Lanzando SITL...")
        open(self.SITL_LOG, 'w').close()
        self.sitl_proc = subprocess.Popen(
            sitl_cmd,
            cwd=self.repo_root,
            stdout=open(self.SITL_LOG, 'w'),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        print("  ⏳ Esperando TCP:5760...", end='', flush=True)
        if not self._wait_log(self.SITL_LOG, 'Waiting for connection', timeout=30):
            print(f"\n❌ SITL no abrió TCP:5760 en 30s")
            return False
        print(f" ✓ (PID={self.sitl_proc.pid})")

        # ── 2. Start MAVProxy (non-interactive) ──────────────────────────────
        # MAVProxy sends the initial GCS heartbeat that wakes SITL's internal clock.
        # It outputs MAVLink UDP to 127.0.0.1:14550 where pymavlink will listen.
        mav_cmd = [
            'python3', self.MAVPROXY,
            '--master=tcp:127.0.0.1:5760',
            '--out=udp:127.0.0.1:14550',
            '--non-interactive',
        ]
        print(f"📡 Lanzando MAVProxy (non-interactive)...")
        open(self.MAV_LOG, 'w').close()
        self.mavproxy_proc = subprocess.Popen(
            mav_cmd,
            stdin=subprocess.PIPE,  # allows sending commands via stdin
            stdout=open(self.MAV_LOG, 'w'),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        # Wait until MAVProxy shows it's online (text varies by MAVProxy version)
        print("  ⏳ Esperando vehicle online...", end='', flush=True)
        ready = (self._wait_log(self.MAV_LOG, 'online system', timeout=40) or
                 self._wait_log(self.MAV_LOG, 'ArduPilot Ready', timeout=10) or
                 self._wait_log(self.MAV_LOG, 'Detected vehicle', timeout=5))
        if not ready:
            print(f"\n⚠️  Timeout — intentando de todos modos")
        else:
            print(f" ✓")
        time.sleep(2)  # let EKF settle before pymavlink connects
        return True

    def stop(self):
        for proc, name in [(self.mavproxy_proc, 'MAVProxy'),
                           (self.sitl_proc,     'SITL')]:
            if proc:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except (ProcessLookupError, OSError):
                    pass
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except OSError:
                        pass
        self.sitl_proc = self.mavproxy_proc = None
        self._kill_stale()
        print("✓ SITL + MAVProxy detenidos")


# ─────────────────────────────────────────────────────────────────────────────
# Metrics Collector
# ─────────────────────────────────────────────────────────────────────────────

class MetricsCollector:
    """Collects real-time data from MAVLink and parses detector messages."""

    def __init__(self, connection_string='tcp:127.0.0.1:5762'):
        self.connection_string = connection_string
        self.mav = None
        self.start_time = None
        self.stop_event = Event()
        self._monitor_thread = None

        # Real-time buffers
        self.times = []
        self.scores = []
        self.states = []  # inferred from messages
        self.velocities = []
        self.altitudes = []
        self.events = []

        self._current_state = 0  # 0=NOMINAL
        self._current_score = 0.0
        self._last_score_time = 0

    def connect(self, timeout=40) -> bool:
        try:
            self.mav = mavutil.mavlink_connection(
                self.connection_string,
                source_system=255,
                timeout=timeout,
            )
            # Wait explicitly for a heartbeat from sysid=1 (the autopilot)
            print("  Esperando heartbeat de ArduPilot...", end='', flush=True)
            deadline = time.time() + timeout
            while time.time() < deadline:
                hb = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if hb and hb.get_srcSystem() == 1:
                    self.mav.target_system = 1
                    self.mav.target_component = 1
                    self.start_time = time.time()
                    print(f" ✓")
                    print(f"✓ MAVLink conectado (sysid={self.mav.target_system})")
                    return True
                print('.', end='', flush=True)
            print(f"\n❌ No se recibió heartbeat de sysid=1 en {timeout}s")
            return False
        except Exception as e:
            print(f"❌ Conexión MAVLink fallida: {e}")
            return False

    def disconnect(self):
        if self.mav:
            self.mav.close()
            self.mav = None

    def elapsed(self) -> float:
        return time.time() - self.start_time if self.start_time else 0

    def arm_and_takeoff(self, alt=15, stabilize_time=20, sitl=None):
        """Arm and takeoff via direct MAVLink on TCP:5762 (SERIAL1)."""
        ts = self.mav.target_system
        tc = self.mav.target_component

        # Disable arming checks first
        self.mav.param_set_send('ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(1)

        # Wait for EKF to converge (~60s in SITL) and for pre-arm messages to clear.
        # SITL needs ~60s from startup for the EKF position estimate to be valid.
        print("  ⏳ Esperando inicialización EKF (60s)...", end='', flush=True)
        deadline = time.time() + 60
        while time.time() < deadline:
            msg = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if msg:
                text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text
                # "EKF3 IMU0 is using GPS" means EKF has GPS fusion active
                if 'is using GPS' in text:
                    # Still wait a few more seconds for position to fully converge
                    remaining = max(0, deadline - time.time() - 20)
                    if remaining > 0:
                        time.sleep(min(remaining, 20))
            print('.', end='', flush=True)
        print(f" ✓")

        # Switch to GUIDED (MAVLink command)
        print(f"  ✈️  Modo GUIDED + arm + takeoff {alt}m...")
        self.mav.mav.command_long_send(
            ts, tc, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, 1, 4, 0, 0, 0, 0, 0  # base_mode=1, custom_mode=4 (GUIDED)
        )
        time.sleep(2)

        # Force-arm (21196 = ArduPilot bypass magic number)
        armed = False
        for attempt in range(6):
            self.mav.mav.command_long_send(
                ts, tc, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1.0, 21196.0, 0, 0, 0, 0, 0
            )
            deadline_arm = time.time() + 5
            while time.time() < deadline_arm:
                msg = self.mav.recv_match(
                    type=['COMMAND_ACK', 'HEARTBEAT'], blocking=True, timeout=0.5)
                if not msg:
                    continue
                if msg.get_type() == 'HEARTBEAT' and (msg.base_mode & 128):
                    armed = True
                    break
                if (msg.get_type() == 'COMMAND_ACK'
                        and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
                        and msg.result == 0):
                    armed = True
                    break
            if armed:
                print(f"  ✓ Armado (intento {attempt+1})")
                break
            print(f"  ⚠️  No armado ({attempt+1}/6), resultado={getattr(msg, 'result', '?') if msg else '?'}")
            time.sleep(2)

        if not armed:
            print("  ❌ No se pudo armar — métricas en tierra (el detector sí funciona)")

        # Takeoff
        self.mav.mav.command_long_send(
            ts, tc, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, float(alt)
        )
        time.sleep(3)

        # Switch to LOITER
        self.mav.mav.command_long_send(
            ts, tc, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, 1, 5, 0, 0, 0, 0, 0  # LOITER
        )

        print(f"  ⏳ Estabilizando en vuelo ({stabilize_time}s)...")
        time.sleep(stabilize_time)
        self._wait_for_nominal(timeout=30)
        print("  ✓ Vehículo estable y detector en NOMINAL")

    def _wait_for_nominal(self, timeout=60):
        """Wait until detector settles to NOMINAL by monitoring GSPF messages."""
        start = time.time()
        last_gspf_state = 'UNKNOWN'
        last_gspf_time = start
        while (time.time() - start) < timeout:
            msg = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
            if msg:
                text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text
                if 'GSPF' in text:
                    last_gspf_time = time.time()
                    if 'CONFIRMED' in text or 'SUSPICIOUS' in text:
                        last_gspf_state = 'ALERTING'
                    elif 'state=0' in text or 'score=0.00' in text:
                        if last_gspf_state != 'ALERTING':
                            # Confirmed nominal: no alert and score is zero
                            time.sleep(2)
                            return
                        last_gspf_state = 'NOMINAL'
                    elif last_gspf_state == 'NOMINAL':
                        time.sleep(2)
                        return
            # If no GSPF messages for 15s, assume nominal
            if (time.time() - last_gspf_time) > 15 and last_gspf_state in ('UNKNOWN', 'NOMINAL'):
                return
        print(f"  ⚠️  Timeout esperando NOMINAL (último estado: {last_gspf_state})")

    def inject_spoofing(self, verr_x: float, verr_y: float, sitl=None):
        self.mav.param_set_send('SIM_GPS1_VERR_X', verr_x)
        time.sleep(0.3)
        self.mav.param_set_send('SIM_GPS1_VERR_Y', verr_y)

    def remove_spoofing(self, sitl=None):
        self.mav.param_set_send('SIM_GPS1_VERR_X', 0)
        time.sleep(0.3)
        self.mav.param_set_send('SIM_GPS1_VERR_Y', 0)

    def start_monitoring(self):
        """Start background monitoring thread."""
        self.stop_event.clear()
        self.times = []
        self.scores = []
        self.states = []
        self.velocities = []
        self.altitudes = []
        self.events = []
        self._current_state = 0
        self._current_score = 0.0

        self._monitor_thread = Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()

    def stop_monitoring(self):
        self.stop_event.set()
        if self._monitor_thread:
            self._monitor_thread.join(timeout=5)

    def _monitor_loop(self):
        while not self.stop_event.is_set():
            try:
                msg = self.mav.recv_match(
                    type=['GLOBAL_POSITION_INT', 'STATUSTEXT'],
                    blocking=True, timeout=0.1)
                if not msg:
                    continue

                t = self.elapsed()

                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    vel = np.sqrt((msg.vx/100.0)**2 + (msg.vy/100.0)**2 + (msg.vz/100.0)**2)
                    self.times.append(t)
                    self.velocities.append(vel)
                    self.altitudes.append(msg.alt / 1000.0)
                    self.scores.append(self._current_score)
                    self.states.append(self._current_state)

                elif msg.get_type() == 'STATUSTEXT':
                    text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text
                    if 'GSPF' in text:
                        self._parse_gspf_message(text, t)

            except Exception:
                pass

    def _parse_gspf_message(self, text: str, t: float):
        """Parse GSPF status messages and extract score/state."""
        event = DetectorEvent(timestamp=t, state='', raw_text=text)

        # Extract score
        score_match = re.search(r'score[=:]?\s*([\d.]+)', text, re.IGNORECASE)
        if score_match:
            self._current_score = float(score_match.group(1))
            event.score = self._current_score

        # Parse explicit state from GSPF_ALL messages ("state=0", "state=1", "state=2")
        state_match = re.search(r'state=(\d)', text)
        if state_match:
            self._current_state = int(state_match.group(1))

        # Determine state from text keywords (takes priority over state= parsing)
        if 'CONFIRMED' in text:
            self._current_state = 2
            event.state = 'CONFIRMED'
        elif 'SUSPICIOUS' in text:
            self._current_state = 1
            event.state = 'SUSPICIOUS'
        elif 'NOMINAL' in text or 'normal' in text.lower():
            self._current_state = 0
            event.state = 'NOMINAL'
        else:
            # Debug message: infer state from score AND update _current_state
            if self._current_score >= 1.0:
                self._current_state = 2
                event.state = 'CONFIRMED'
            elif self._current_score >= 0.5:
                self._current_state = 1
                event.state = 'SUSPICIOUS'
            else:
                self._current_state = 0
                event.state = 'NOMINAL'

        self.events.append(event)


# ─────────────────────────────────────────────────────────────────────────────
# Test Runner
# ─────────────────────────────────────────────────────────────────────────────

class MetricsTestRunner:
    """Runs scenarios and collects metrics."""

    def __init__(self, output_dir: str, speedup: int = 1, skip_sitl: bool = False):
        self.output_dir = output_dir
        self.speedup = speedup
        self.skip_sitl = skip_sitl
        self.sitl = SITLManager(speedup=speedup)
        self.collector = MetricsCollector()
        self.results: List[ScenarioResult] = []
        self.baseline: Optional[BaselineResult] = None

        os.makedirs(output_dir, exist_ok=True)

    def run_all(self, scenarios: List[SpoofingScenario] = None, baseline_duration: float = 60):
        """Run baseline + all spoofing scenarios."""
        if scenarios is None:
            scenarios = SCENARIOS

        # Start SITL if needed
        if not self.skip_sitl:
            if not self.sitl.start():
                return
        
        if not self.collector.connect():
            self.sitl.stop()
            return

        try:
            # Arm and takeoff once
            self.collector.arm_and_takeoff(alt=15, stabilize_time=25, sitl=self.sitl)

            # Baseline (no spoofing)
            print("\n" + "─"*60)
            print("📏 BASELINE: Vuelo sin spoofing")
            print("─"*60)
            self.baseline = self._run_baseline(baseline_duration)

            # Spoofing scenarios
            for i, scenario in enumerate(scenarios):
                print(f"\n{'─'*60}")
                print(f"🧪 ESCENARIO {i+1}/{len(scenarios)}: {scenario.name}")
                print(f"   {scenario.description}")
                print(f"   VERR_X={scenario.verr_x}, VERR_Y={scenario.verr_y}, dur={scenario.duration}s")
                print("─"*60)

                result = self._run_scenario(scenario)
                self.results.append(result)

                # Cool-down between scenarios
                print("  ⏳ Cool-down (15s)...")
                time.sleep(15)

        except KeyboardInterrupt:
            print("\n⚠️  Interrumpido por usuario")
        finally:
            self.collector.remove_spoofing(sitl=self.sitl)
            self.collector.stop_monitoring()
            self.collector.disconnect()
            if not self.skip_sitl:
                self.sitl.stop()

    def _run_baseline(self, duration: float) -> BaselineResult:
        """Run baseline flight with no spoofing."""
        self.collector.start_monitoring()
        print(f"  ⏳ Monitoreando vuelo limpio ({duration}s)...")
        time.sleep(duration)
        self.collector.stop_monitoring()

        result = BaselineResult(
            duration=duration,
            max_score=max(self.collector.scores) if self.collector.scores else 0,
            false_positives=sum(1 for s in self.collector.states if s > 0),
            times=list(self.collector.times),
            scores=list(self.collector.scores),
            states=list(self.collector.states),
            events=list(self.collector.events)
        )

        fp_events = [e for e in result.events if e.state in ('SUSPICIOUS', 'CONFIRMED')]
        if fp_events:
            print(f"  ⚠️  FALSOS POSITIVOS detectados: {len(fp_events)} eventos")
        else:
            print(f"  ✅ Sin falsos positivos. Score máximo: {result.max_score:.4f}")

        return result

    def _run_scenario(self, scenario: SpoofingScenario) -> ScenarioResult:
        """Run a single spoofing scenario."""
        result = ScenarioResult(scenario=scenario)

        self.collector.start_monitoring()

        # Pre-spoofing monitoring (5s)
        time.sleep(5)

        # Inject spoofing
        inject_time = self.collector.elapsed()
        result.spoofing_injected_at = inject_time
        print(f"  ⚡ Spoofing inyectado en t={inject_time:.1f}s")
        self.collector.inject_spoofing(scenario.verr_x, scenario.verr_y, sitl=self.sitl)

        # Wait for spoofing duration
        time.sleep(scenario.duration)

        # Remove spoofing
        remove_time = self.collector.elapsed()
        result.spoofing_removed_at = remove_time
        print(f"  ✓ Spoofing removido en t={remove_time:.1f}s")
        self.collector.remove_spoofing(sitl=self.sitl)

        # Wait for recovery
        time.sleep(20)

        self.collector.stop_monitoring()

        # Analyze results
        result.times = list(self.collector.times)
        result.scores = list(self.collector.scores)
        result.states = list(self.collector.states)
        result.velocities = list(self.collector.velocities)
        result.altitudes = list(self.collector.altitudes)
        result.events = list(self.collector.events)

        # Compute spoofing mask
        result.spoofing_mask = [
            inject_time <= t <= remove_time for t in result.times
        ]

        # Find detection times from events
        for event in result.events:
            if event.state == 'SUSPICIOUS' and result.first_suspicious_at is None:
                if event.timestamp >= inject_time:
                    result.first_suspicious_at = event.timestamp
            if event.state == 'CONFIRMED' and result.first_confirmed_at is None:
                if event.timestamp >= inject_time:
                    result.first_confirmed_at = event.timestamp
            if event.state == 'NOMINAL' and event.timestamp > (remove_time + 1):
                if result.recovery_to_nominal_at is None:
                    result.recovery_to_nominal_at = event.timestamp

        # Compute latencies
        if result.first_suspicious_at:
            result.latency_to_suspicious = result.first_suspicious_at - inject_time
        if result.first_confirmed_at:
            result.latency_to_confirmed = result.first_confirmed_at - inject_time
        if result.recovery_to_nominal_at and result.spoofing_removed_at:
            result.recovery_time = result.recovery_to_nominal_at - result.spoofing_removed_at

        # Classification
        result.true_positive = result.first_confirmed_at is not None or result.first_suspicious_at is not None
        result.false_negative = not result.true_positive

        # Print summary
        if result.latency_to_suspicious is not None:
            print(f"  🔔 SUSPICIOUS en {result.latency_to_suspicious:.1f}s")
        if result.latency_to_confirmed is not None:
            print(f"  🚨 CONFIRMED en {result.latency_to_confirmed:.1f}s")
        if result.false_negative:
            print(f"  ❌ NO DETECTADO (falso negativo)")
        if result.recovery_time is not None:
            print(f"  🔄 Recuperación en {result.recovery_time:.1f}s")

        return result


# ─────────────────────────────────────────────────────────────────────────────
# Metrics Computation
# ─────────────────────────────────────────────────────────────────────────────

class MetricsAnalyzer:
    """Computes aggregate metrics from test results."""

    def __init__(self, results: List[ScenarioResult], baseline: Optional[BaselineResult]):
        self.results = results
        self.baseline = baseline

    def compute_all(self) -> Dict:
        """Compute all metrics and return as dictionary."""
        metrics = {
            'timestamp': datetime.now().isoformat(),
            'num_scenarios': len(self.results),
            'detection_metrics': self._detection_metrics(),
            'latency_metrics': self._latency_metrics(),
            'recovery_metrics': self._recovery_metrics(),
            'baseline_metrics': self._baseline_metrics(),
            'confusion_matrix': self._confusion_matrix(),
            'per_scenario': self._per_scenario_summary(),
            'score_statistics': self._score_statistics(),
        }
        return metrics

    def _detection_metrics(self) -> Dict:
        n = len(self.results)
        if n == 0:
            return {}
        tp = sum(1 for r in self.results if r.true_positive)
        fn = sum(1 for r in self.results if r.false_negative)
        # Confirmed detections (strongest)
        confirmed = sum(1 for r in self.results if r.first_confirmed_at is not None)
        suspicious_only = sum(1 for r in self.results
                             if r.first_suspicious_at is not None and r.first_confirmed_at is None)

        return {
            'total_scenarios': n,
            'true_positives': tp,
            'false_negatives': fn,
            'detection_rate': tp / n if n else 0,
            'confirmed_detections': confirmed,
            'suspicious_only_detections': suspicious_only,
            'miss_rate': fn / n if n else 0,
        }

    def _latency_metrics(self) -> Dict:
        suspicious_latencies = [r.latency_to_suspicious for r in self.results if r.latency_to_suspicious is not None]
        confirmed_latencies = [r.latency_to_confirmed for r in self.results if r.latency_to_confirmed is not None]

        def stats(arr):
            if not arr:
                return {'count': 0}
            return {
                'count': len(arr),
                'mean': float(np.mean(arr)),
                'std': float(np.std(arr)),
                'min': float(np.min(arr)),
                'max': float(np.max(arr)),
                'median': float(np.median(arr)),
                'p90': float(np.percentile(arr, 90)),
            }

        return {
            'to_suspicious': stats(suspicious_latencies),
            'to_confirmed': stats(confirmed_latencies),
        }

    def _recovery_metrics(self) -> Dict:
        recovery_times = [r.recovery_time for r in self.results if r.recovery_time is not None]
        if not recovery_times:
            return {'count': 0, 'note': 'No recovery events observed'}
        return {
            'count': len(recovery_times),
            'mean': float(np.mean(recovery_times)),
            'std': float(np.std(recovery_times)),
            'min': float(np.min(recovery_times)),
            'max': float(np.max(recovery_times)),
        }

    def _baseline_metrics(self) -> Dict:
        if not self.baseline:
            return {'available': False}
        return {
            'available': True,
            'duration_s': self.baseline.duration,
            'max_score_observed': self.baseline.max_score,
            'false_positive_samples': self.baseline.false_positives,
            'false_positive_events': len([e for e in self.baseline.events
                                          if e.state in ('SUSPICIOUS', 'CONFIRMED')]),
            'false_positive_rate': (self.baseline.false_positives /
                                    max(len(self.baseline.states), 1)),
        }

    def _confusion_matrix(self) -> Dict:
        """Binary classification: spoofing present vs detected."""
        tp = sum(1 for r in self.results if r.true_positive)
        fn = sum(1 for r in self.results if r.false_negative)
        # FP from baseline
        fp = len([e for e in (self.baseline.events if self.baseline else [])
                  if e.state in ('SUSPICIOUS', 'CONFIRMED')])
        # TN: baseline samples that remained nominal
        tn = (len(self.baseline.states) - self.baseline.false_positives) if self.baseline else 0

        precision = tp / (tp + fp) if (tp + fp) > 0 else 1.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0

        return {
            'TP': tp, 'FN': fn, 'FP': fp,
            'precision': precision,
            'recall': recall,
            'f1_score': f1,
        }

    def _per_scenario_summary(self) -> List[Dict]:
        summaries = []
        for r in self.results:
            summaries.append({
                'name': r.scenario.name,
                'verr_x': r.scenario.verr_x,
                'verr_y': r.scenario.verr_y,
                'intensity': np.sqrt(r.scenario.verr_x**2 + r.scenario.verr_y**2),
                'duration': r.scenario.duration,
                'detected': r.true_positive,
                'latency_suspicious_s': r.latency_to_suspicious,
                'latency_confirmed_s': r.latency_to_confirmed,
                'recovery_s': r.recovery_time,
                'max_score': max(r.scores) if r.scores else 0,
                'num_events': len(r.events),
            })
        return summaries

    def _score_statistics(self) -> Dict:
        """Score distribution across all scenarios during spoofing."""
        all_spoofing_scores = []
        all_clean_scores = []

        for r in self.results:
            for i, t in enumerate(r.times):
                if i < len(r.scores):
                    if r.spoofing_mask and i < len(r.spoofing_mask) and r.spoofing_mask[i]:
                        all_spoofing_scores.append(r.scores[i])
                    else:
                        all_clean_scores.append(r.scores[i])

        def stats(arr, label):
            if not arr:
                return {'count': 0, 'label': label}
            return {
                'label': label,
                'count': len(arr),
                'mean': float(np.mean(arr)),
                'std': float(np.std(arr)),
                'min': float(np.min(arr)),
                'max': float(np.max(arr)),
                'median': float(np.median(arr)),
                'p25': float(np.percentile(arr, 25)),
                'p75': float(np.percentile(arr, 75)),
                'p95': float(np.percentile(arr, 95)),
            }

        return {
            'during_spoofing': stats(all_spoofing_scores, 'spoofing_active'),
            'during_clean': stats(all_clean_scores, 'no_spoofing'),
        }


# ─────────────────────────────────────────────────────────────────────────────
# Visualization
# ─────────────────────────────────────────────────────────────────────────────

class MetricsVisualizer:
    """Generates publication-quality plots."""

    def __init__(self, results: List[ScenarioResult], baseline: Optional[BaselineResult],
                 metrics: Dict, output_dir: str):
        self.results = results
        self.baseline = baseline
        self.metrics = metrics
        self.output_dir = output_dir

    def generate_all(self):
        if not HAS_MATPLOTLIB:
            print("⚠️  Gráficas no generadas (matplotlib no disponible)")
            return

        print("\n📊 Generando gráficas...")
        self._plot_overview_dashboard()
        self._plot_score_timeseries()
        self._plot_latency_vs_intensity()
        self._plot_score_distribution()
        self._plot_confusion_matrix()
        self._plot_baseline()
        print(f"✅ Gráficas guardadas en {self.output_dir}/")

    def _save(self, fig, name):
        path = os.path.join(self.output_dir, f"{name}.png")
        fig.savefig(path, dpi=150, bbox_inches='tight', facecolor='white')
        plt.close(fig)

    def _plot_overview_dashboard(self):
        """Main dashboard with key metrics."""
        fig = plt.figure(figsize=(20, 14))
        fig.suptitle('GPS Spoofing Detector - Dashboard de Métricas', fontsize=16, fontweight='bold')
        gs = GridSpec(3, 3, figure=fig, hspace=0.4, wspace=0.3)

        # 1. Detection rate gauge
        ax1 = fig.add_subplot(gs[0, 0])
        dm = self.metrics['detection_metrics']
        rate = dm.get('detection_rate', 0)
        colors = ['#2ecc71' if rate >= 0.9 else '#f39c12' if rate >= 0.7 else '#e74c3c']
        ax1.barh(['Detección'], [rate], color=colors, height=0.4)
        ax1.barh(['Detección'], [1.0], color='#ecf0f1', height=0.4, zorder=0)
        ax1.set_xlim(0, 1)
        ax1.set_xlabel('Tasa')
        ax1.set_title(f'Tasa de Detección: {rate*100:.0f}%')
        ax1.axvline(0.9, color='green', linestyle='--', alpha=0.5, label='Objetivo 90%')
        ax1.legend(fontsize=8)

        # 2. Latency box plot
        ax2 = fig.add_subplot(gs[0, 1])
        lat_data = []
        lat_labels = []
        suspicious_lats = [r.latency_to_suspicious for r in self.results if r.latency_to_suspicious]
        confirmed_lats = [r.latency_to_confirmed for r in self.results if r.latency_to_confirmed]
        if suspicious_lats:
            lat_data.append(suspicious_lats)
            lat_labels.append('SUSPICIOUS')
        if confirmed_lats:
            lat_data.append(confirmed_lats)
            lat_labels.append('CONFIRMED')
        if lat_data:
            bp = ax2.boxplot(lat_data, labels=lat_labels, patch_artist=True)
            colors_bp = ['#f39c12', '#e74c3c']
            for patch, color in zip(bp['boxes'], colors_bp[:len(lat_data)]):
                patch.set_facecolor(color)
                patch.set_alpha(0.6)
        ax2.set_ylabel('Latencia (s)')
        ax2.set_title('Distribución de Latencia de Detección')
        ax2.grid(True, alpha=0.3)

        # 3. Confusion matrix numbers
        ax3 = fig.add_subplot(gs[0, 2])
        cm = self.metrics['confusion_matrix']
        ax3.axis('off')
        text = (
            f"━━━ CLASIFICACIÓN ━━━\n\n"
            f"  TP: {cm['TP']}   FP: {cm['FP']}\n"
            f"  FN: {cm['FN']}\n\n"
            f"  Precision: {cm['precision']:.3f}\n"
            f"  Recall:    {cm['recall']:.3f}\n"
            f"  F1 Score:  {cm['f1_score']:.3f}\n"
        )
        ax3.text(0.1, 0.5, text, fontsize=13, family='monospace',
                verticalalignment='center', transform=ax3.transAxes,
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
        ax3.set_title('Matriz de Confusión')

        # 4-6: Score time series for first 3 scenarios
        for idx in range(min(3, len(self.results))):
            ax = fig.add_subplot(gs[1, idx])
            r = self.results[idx]
            if r.times and r.scores:
                t_arr = np.array(r.times) - r.times[0]
                ax.plot(t_arr, r.scores, 'b-', linewidth=1.5, label='Score')
                ax.axhline(0.5, color='orange', linestyle='--', alpha=0.7, label='TH_LOW')
                ax.axhline(1.0, color='red', linestyle='--', alpha=0.7, label='TH_HIGH')
                # Shade spoofing period
                inj = r.spoofing_injected_at - r.times[0] if r.times else 0
                rem = (r.spoofing_removed_at - r.times[0]) if r.spoofing_removed_at else t_arr[-1]
                ax.axvspan(inj, rem, alpha=0.15, color='red', label='Spoofing')
            ax.set_xlabel('Tiempo (s)')
            ax.set_ylabel('Score')
            ax.set_title(f'{r.scenario.name}\n(VERR={r.scenario.verr_x:.1f},{r.scenario.verr_y:.1f})')
            ax.legend(fontsize=7, loc='upper left')
            ax.grid(True, alpha=0.3)
            ax.set_ylim(-0.1, 2.2)

        # 7. Latency vs intensity scatter
        ax7 = fig.add_subplot(gs[2, 0])
        intensities = [np.sqrt(r.scenario.verr_x**2 + r.scenario.verr_y**2) for r in self.results]
        lats = [r.latency_to_confirmed if r.latency_to_confirmed else r.latency_to_suspicious
                for r in self.results]
        valid = [(i, l) for i, l in zip(intensities, lats) if l is not None]
        if valid:
            xi, yl = zip(*valid)
            ax7.scatter(xi, yl, c='red', s=80, zorder=5)
            ax7.set_xlabel('Intensidad spoofing (m/s)')
            ax7.set_ylabel('Latencia detección (s)')
            ax7.set_title('Latencia vs Intensidad')
            ax7.grid(True, alpha=0.3)

        # 8. Per-scenario bar chart
        ax8 = fig.add_subplot(gs[2, 1])
        names = [r.scenario.name for r in self.results]
        max_scores = [max(r.scores) if r.scores else 0 for r in self.results]
        bars = ax8.bar(range(len(names)), max_scores,
                      color=['#2ecc71' if s >= 1.0 else '#f39c12' if s >= 0.5 else '#e74c3c'
                             for s in max_scores])
        ax8.set_xticks(range(len(names)))
        ax8.set_xticklabels(names, rotation=45, ha='right', fontsize=8)
        ax8.axhline(0.5, color='orange', linestyle='--', alpha=0.7)
        ax8.axhline(1.0, color='red', linestyle='--', alpha=0.7)
        ax8.set_ylabel('Score Máximo')
        ax8.set_title('Score Máximo por Escenario')
        ax8.grid(True, alpha=0.3, axis='y')

        # 9. Summary text
        ax9 = fig.add_subplot(gs[2, 2])
        ax9.axis('off')
        lm = self.metrics['latency_metrics']
        susp_mean = lm['to_suspicious'].get('mean', 'N/A')
        conf_mean = lm['to_confirmed'].get('mean', 'N/A')
        susp_str = f"{susp_mean:.1f}s" if isinstance(susp_mean, float) else str(susp_mean)
        conf_str = f"{conf_mean:.1f}s" if isinstance(conf_mean, float) else str(conf_mean)
        summary = (
            f"━━━ RESUMEN ━━━\n\n"
            f"  Escenarios: {len(self.results)}\n"
            f"  Detectados: {dm['true_positives']}/{dm['total_scenarios']}\n"
            f"  Lat. media SUSPICIOUS: {susp_str}\n"
            f"  Lat. media CONFIRMED:  {conf_str}\n"
            f"  FP en baseline: {self.metrics['baseline_metrics'].get('false_positive_events', 'N/A')}\n"
        )
        ax9.text(0.05, 0.5, summary, fontsize=11, family='monospace',
                verticalalignment='center', transform=ax9.transAxes,
                bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.8))

        self._save(fig, 'dashboard')

    def _plot_score_timeseries(self):
        """Individual detailed score plots per scenario."""
        n = len(self.results)
        if n == 0:
            return
        cols = min(3, n)
        rows = (n + cols - 1) // cols
        fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 4*rows), squeeze=False)
        fig.suptitle('Score del Detector por Escenario', fontsize=14, fontweight='bold')

        for idx, r in enumerate(self.results):
            ax = axes[idx // cols][idx % cols]
            if r.times and r.scores:
                t_arr = np.array(r.times) - r.times[0]
                ax.plot(t_arr, r.scores, 'b-', linewidth=1.2)
                ax.fill_between(t_arr, 0, r.scores, alpha=0.2, color='blue')
                ax.axhline(0.5, color='orange', linestyle='--', linewidth=1, label='TH_LOW=0.5')
                ax.axhline(1.0, color='red', linestyle='--', linewidth=1, label='TH_HIGH=1.0')

                inj = r.spoofing_injected_at - r.times[0]
                rem = (r.spoofing_removed_at - r.times[0]) if r.spoofing_removed_at else t_arr[-1]
                ax.axvspan(inj, rem, alpha=0.1, color='red')
                ax.axvline(inj, color='red', linewidth=1.5, alpha=0.7)

                if r.first_suspicious_at:
                    ts = r.first_suspicious_at - r.times[0]
                    ax.axvline(ts, color='orange', linewidth=1.5, linestyle=':')
                if r.first_confirmed_at:
                    tc = r.first_confirmed_at - r.times[0]
                    ax.axvline(tc, color='darkred', linewidth=1.5, linestyle=':')

            ax.set_title(f'{r.scenario.name}', fontsize=10)
            ax.set_xlabel('t (s)')
            ax.set_ylabel('Score')
            ax.set_ylim(-0.1, 2.2)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7)

        # Hide unused axes
        for idx in range(n, rows*cols):
            axes[idx // cols][idx % cols].set_visible(False)

        plt.tight_layout()
        self._save(fig, 'score_timeseries')

    def _plot_latency_vs_intensity(self):
        """Scatter: detection latency vs spoofing intensity."""
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.set_title('Latencia de Detección vs Intensidad de Spoofing', fontweight='bold')

        for r in self.results:
            intensity = np.sqrt(r.scenario.verr_x**2 + r.scenario.verr_y**2)
            if r.latency_to_suspicious:
                ax.scatter(intensity, r.latency_to_suspicious, c='orange', s=100,
                          marker='o', zorder=5, label='SUSPICIOUS' if r == self.results[0] else '')
            if r.latency_to_confirmed:
                ax.scatter(intensity, r.latency_to_confirmed, c='red', s=100,
                          marker='^', zorder=5, label='CONFIRMED' if r == self.results[0] else '')

        ax.set_xlabel('Intensidad del Spoofing (||VERR|| m/s)')
        ax.set_ylabel('Latencia (s)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        self._save(fig, 'latency_vs_intensity')

    def _plot_score_distribution(self):
        """Histogram of scores during spoofing vs clean."""
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.set_title('Distribución del Score: Spoofing vs Limpio', fontweight='bold')

        spoofing_scores = []
        clean_scores = []
        for r in self.results:
            for i in range(min(len(r.scores), len(r.spoofing_mask))):
                if r.spoofing_mask[i]:
                    spoofing_scores.append(r.scores[i])
                else:
                    clean_scores.append(r.scores[i])

        bins = np.linspace(0, 2.0, 40)
        if clean_scores:
            ax.hist(clean_scores, bins=bins, alpha=0.6, color='green', label='Sin spoofing', density=True)
        if spoofing_scores:
            ax.hist(spoofing_scores, bins=bins, alpha=0.6, color='red', label='Con spoofing', density=True)

        ax.axvline(0.5, color='orange', linestyle='--', linewidth=2, label='TH_LOW')
        ax.axvline(1.0, color='darkred', linestyle='--', linewidth=2, label='TH_HIGH')
        ax.set_xlabel('Score')
        ax.set_ylabel('Densidad')
        ax.legend()
        ax.grid(True, alpha=0.3)
        self._save(fig, 'score_distribution')

    def _plot_confusion_matrix(self):
        """Visual confusion matrix."""
        fig, ax = plt.subplots(figsize=(6, 5))
        cm = self.metrics['confusion_matrix']

        matrix = np.array([[cm['TP'], cm['FP']],
                          [cm['FN'], 0]])  # TN not well defined per-scenario
        im = ax.imshow(matrix, cmap='YlOrRd', aspect='auto')
        ax.set_xticks([0, 1])
        ax.set_yticks([0, 1])
        ax.set_xticklabels(['Positivo', 'Falso Positivo'])
        ax.set_yticklabels(['Detectado', 'No Detectado'])
        ax.set_title('Matriz de Confusión', fontweight='bold')

        for i in range(2):
            for j in range(2):
                ax.text(j, i, str(matrix[i, j]), ha='center', va='center', fontsize=20, fontweight='bold')

        fig.colorbar(im)
        self._save(fig, 'confusion_matrix')

    def _plot_baseline(self):
        """Plot baseline flight score."""
        if not self.baseline or not self.baseline.times:
            return

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
        fig.suptitle('Baseline: Vuelo Sin Spoofing (Detección de Falsos Positivos)', fontweight='bold')

        t_arr = np.array(self.baseline.times) - self.baseline.times[0]

        ax1.plot(t_arr, self.baseline.scores, 'b-', linewidth=1)
        ax1.axhline(0.5, color='orange', linestyle='--', label='TH_LOW')
        ax1.axhline(1.0, color='red', linestyle='--', label='TH_HIGH')
        ax1.set_ylabel('Score')
        ax1.set_title('Score del Detector (debería mantenerse bajo)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(-0.05, max(0.6, max(self.baseline.scores) * 1.2))

        ax2.plot(t_arr, self.baseline.states, 'r-', linewidth=1.5)
        ax2.set_yticks([0, 1, 2])
        ax2.set_yticklabels(['NOMINAL', 'SUSPICIOUS', 'CONFIRMED'])
        ax2.set_xlabel('Tiempo (s)')
        ax2.set_ylabel('Estado')
        ax2.set_title('Estado del Detector')
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        self._save(fig, 'baseline')


# ─────────────────────────────────────────────────────────────────────────────
# Report Generator
# ─────────────────────────────────────────────────────────────────────────────

def print_report(metrics: Dict):
    """Print formatted metrics report to console."""
    print("\n" + "═"*70)
    print("           INFORME DE MÉTRICAS - DETECTOR GPS SPOOFING (GSPF)")
    print("═"*70)

    dm = metrics['detection_metrics']
    print(f"\n┌─ DETECCIÓN {'─'*55}┐")
    print(f"│  Escenarios ejecutados:  {dm.get('total_scenarios', 0):<8}")
    print(f"│  Verdaderos Positivos:   {dm.get('true_positives', 0):<8}")
    print(f"│  Falsos Negativos:       {dm.get('false_negatives', 0):<8}")
    print(f"│  Tasa de Detección:      {dm.get('detection_rate', 0)*100:.1f}%")
    print(f"│  Confirmados (CONFIRMED):{dm.get('confirmed_detections', 0):<8}")
    print(f"│  Solo SUSPICIOUS:        {dm.get('suspicious_only_detections', 0):<8}")
    print(f"└{'─'*68}┘")

    lm = metrics['latency_metrics']
    print(f"\n┌─ LATENCIA {'─'*56}┐")
    for key, label in [('to_suspicious', 'Hasta SUSPICIOUS'), ('to_confirmed', 'Hasta CONFIRMED')]:
        s = lm.get(key, {})
        if s.get('count', 0) > 0:
            print(f"│  {label}:")
            print(f"│    Media: {s['mean']:.2f}s | Mediana: {s['median']:.2f}s | Std: {s['std']:.2f}s")
            print(f"│    Min: {s['min']:.2f}s | Max: {s['max']:.2f}s | P90: {s['p90']:.2f}s")
        else:
            print(f"│  {label}: Sin datos")
    print(f"└{'─'*68}┘")

    rm = metrics['recovery_metrics']
    print(f"\n┌─ RECUPERACIÓN {'─'*52}┐")
    if rm.get('count', 0) > 0:
        print(f"│  Tiempo medio de recuperación: {rm['mean']:.2f}s (±{rm['std']:.2f}s)")
        print(f"│  Min: {rm['min']:.2f}s | Max: {rm['max']:.2f}s")
    else:
        print(f"│  {rm.get('note', 'Sin datos de recuperación')}")
    print(f"└{'─'*68}┘")

    bm = metrics['baseline_metrics']
    print(f"\n┌─ BASELINE (Falsos Positivos) {'─'*37}┐")
    if bm.get('available'):
        print(f"│  Duración baseline: {bm['duration_s']:.0f}s")
        print(f"│  Score máximo observado: {bm['max_score_observed']:.4f}")
        print(f"│  Eventos FP: {bm['false_positive_events']}")
        print(f"│  Tasa FP (muestras): {bm['false_positive_rate']*100:.2f}%")
    else:
        print(f"│  No disponible")
    print(f"└{'─'*68}┘")

    cm = metrics['confusion_matrix']
    print(f"\n┌─ CLASIFICACIÓN {'─'*51}┐")
    print(f"│  Precision: {cm['precision']:.4f}")
    print(f"│  Recall:    {cm['recall']:.4f}")
    print(f"│  F1 Score:  {cm['f1_score']:.4f}")
    print(f"└{'─'*68}┘")

    print(f"\n┌─ POR ESCENARIO {'─'*51}┐")
    print(f"│  {'Nombre':<20} {'Intens.':<8} {'Detect.':<8} {'Lat.Susp':<10} {'Lat.Conf':<10} {'MaxScore':<8}")
    print(f"│  {'─'*64}")
    for s in metrics['per_scenario']:
        det = '✓' if s['detected'] else '✗'
        ls = f"{s['latency_suspicious_s']:.1f}s" if s['latency_suspicious_s'] else '-'
        lc = f"{s['latency_confirmed_s']:.1f}s" if s['latency_confirmed_s'] else '-'
        print(f"│  {s['name']:<20} {s['intensity']:<8.2f} {det:<8} {ls:<10} {lc:<10} {s['max_score']:<8.3f}")
    print(f"└{'─'*68}┘")

    ss = metrics['score_statistics']
    print(f"\n┌─ ESTADÍSTICAS DE SCORE {'─'*44}┐")
    for key in ['during_spoofing', 'during_clean']:
        s = ss.get(key, {})
        if s.get('count', 0) > 0:
            print(f"│  {s['label']}:")
            print(f"│    N={s['count']} | Media={s['mean']:.4f} | Std={s['std']:.4f}")
            print(f"│    P25={s['p25']:.4f} | Mediana={s['median']:.4f} | P75={s['p75']:.4f} | P95={s['p95']:.4f}")
    print(f"└{'─'*68}┘")

    print("\n" + "═"*70 + "\n")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Evaluación de métricas del detector de GPS Spoofing (GSPF)')
    parser.add_argument('--output-dir', default='./metrics_output',
                       help='Directorio de salida para gráficas y JSON')
    parser.add_argument('--speedup', type=int, default=1,
                       help='Factor de aceleración SITL (1=tiempo real)')
    parser.add_argument('--skip-sitl', action='store_true',
                       help='No lanzar SITL (asumir que ya está corriendo)')
    parser.add_argument('--baseline-duration', type=float, default=60,
                       help='Duración del vuelo baseline sin spoofing (s)')
    parser.add_argument('--scenarios', nargs='*', default=None,
                       help='Nombres de escenarios a ejecutar (por defecto todos)')
    parser.add_argument('--no-plots', action='store_true',
                       help='No generar gráficas, solo métricas JSON')
    args = parser.parse_args()

    print("\n╔══════════════════════════════════════════════════════════════╗")
    print("║    METRICS.PY - Evaluación del Detector GPS Spoofing        ║")
    print("║    ArduPilot GSPF Performance Analysis                      ║")
    print("╚══════════════════════════════════════════════════════════════╝\n")

    # Filter scenarios if specified
    scenarios = SCENARIOS
    if args.scenarios:
        scenarios = [s for s in SCENARIOS if s.name in args.scenarios]
        if not scenarios:
            print(f"❌ No se encontraron escenarios: {args.scenarios}")
            print(f"   Disponibles: {[s.name for s in SCENARIOS]}")
            sys.exit(1)

    print(f"  📁 Output: {args.output_dir}")
    print(f"  🧪 Escenarios: {len(scenarios)}")
    print(f"  📏 Baseline: {args.baseline_duration}s")
    print(f"  ⚡ Speedup: {args.speedup}x")
    print()

    # Run tests
    runner = MetricsTestRunner(
        output_dir=args.output_dir,
        speedup=args.speedup,
        skip_sitl=args.skip_sitl
    )
    runner.run_all(scenarios=scenarios, baseline_duration=args.baseline_duration)

    if not runner.results:
        print("❌ No se obtuvieron resultados")
        sys.exit(1)

    # Compute metrics
    analyzer = MetricsAnalyzer(runner.results, runner.baseline)
    metrics = analyzer.compute_all()

    # Print report
    print_report(metrics)

    # Save JSON
    json_path = os.path.join(args.output_dir, 'metrics.json')
    # Convert dataclasses for JSON serialization
    metrics_serializable = json.loads(json.dumps(metrics, default=str))
    with open(json_path, 'w') as f:
        json.dump(metrics_serializable, f, indent=2, ensure_ascii=False)
    print(f"💾 Métricas guardadas en: {json_path}")

    # Generate plots
    if not args.no_plots and HAS_MATPLOTLIB:
        visualizer = MetricsVisualizer(runner.results, runner.baseline, metrics, args.output_dir)
        visualizer.generate_all()

    print("\n✅ Evaluación completada.")
    print(f"   Archivos en: {os.path.abspath(args.output_dir)}/")


if __name__ == '__main__':
    main()
