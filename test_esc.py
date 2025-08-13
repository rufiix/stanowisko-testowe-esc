import time
import random
import threading
import logging
from collections import deque
from typing import Optional

import pigpio
from nicegui import ui, app

# Wyciszenie logów NiceGUI
logging.getLogger('nicegui').setLevel(logging.WARNING)
logging.getLogger('nicegui.binding').setLevel(logging.WARNING)

# -------------------- KONFIG --------------------
GPIO_PIN_ESC = 17
APP_PORT = 8081

# PWM (analog) domyślne
PWM_FREQUENCY = 490
PWM_RANGE = 255
DEFAULT_IDLE_PWM = 1050
DEFAULT_DURATION = 60
MIN_PWM_LIMIT = 900
MAX_PWM_LIMIT = 2500

# DShot300 (cyfrowy)
# Bit time ≈ 3.33 µs, ale używamy 3 µs (integer µs) => wymagane pigpiod -s 1
DSHOT_BIT_TOTAL_US = 3
DSHOT1_HIGH_US = 2
DSHOT1_LOW_US = DSHOT_BIT_TOTAL_US - DSHOT1_HIGH_US  # 1 us
DSHOT0_HIGH_US = 1
DSHOT0_LOW_US = DSHOT_BIT_TOTAL_US - DSHOT0_HIGH_US  # 2 us
# ~1 kHz ramek: 16 bitów * 3us = 48us => gap ≈ 952us
DSHOT_FRAME_GAP_US = max(10, 1000 - 16 * DSHOT_BIT_TOTAL_US)
DSHOT_MIN_THROTTLE = 48   # <48 to komendy; >=48 to gaz
DSHOT_MAX_THROTTLE = 2047
DSHOT_DEFAULT_IDLE = 0    # BEZPIECZNIE: brak obrotów po ARM

# -------------------- DSHOT TX --------------------
class DShotTransmitter:
    """Nadajnik DShot oparty o pigpio wave z bezpiecznym przełączaniem fal."""
    def __init__(self, pi: pigpio.pi, gpio: int):
        self.pi = pi
        self.gpio = gpio
        self._current_wid: Optional[int] = None
        self._last_throttle: Optional[int] = None
        self._lock = threading.Lock()
        self._pending_delete = deque()  # (wid, safe_after_time)

    @staticmethod
    def _make_packet(throttle: int, telemetry: int = 0) -> int:
        t = max(0, min(2047, int(throttle)))
        payload = ((t << 1) | (telemetry & 0x1)) & 0x0FFF  # 12 bitów
        csum = 0
        for i in range(3):
            csum ^= (payload >> (i * 4)) & 0xF
        csum &= 0xF
        return (payload << 4) | csum  # 16-bitowy pakiet

    def _build_wave(self, throttle: int) -> int:
        packet = self._make_packet(throttle, telemetry=0)

        pulses = []
        on_mask = 1 << self.gpio
        off_mask = on_mask

        # MSB -> LSB, 16 bitów
        for i in range(15, -1, -1):
            bit = (packet >> i) & 1
            if bit:
                pulses.append(pigpio.pulse(on_mask, 0, DSHOT1_HIGH_US))
                pulses.append(pigpio.pulse(0, off_mask, DSHOT1_LOW_US))
            else:
                pulses.append(pigpio.pulse(on_mask, 0, DSHOT0_HIGH_US))
                pulses.append(pigpio.pulse(0, off_mask, DSHOT0_LOW_US))

        # przerwa między ramkami (linia niska)
        pulses.append(pigpio.pulse(0, off_mask, DSHOT_FRAME_GAP_US))

        # Poprawnie: wave_add_new przygotowuje bufor pulsów do nowej fali (nie kasuje istniejących WID)
        self.pi.wave_add_new()
        self.pi.wave_add_generic(pulses)
        wid = self.pi.wave_create()
        if wid < 0:
            raise RuntimeError("pigpio wave_create failed")
        return wid

    def _process_pending_deletes(self):
        """Kasuje opóźnione wave'y po upływie safe_after_time."""
        now = time.time()
        while self._pending_delete and self._pending_delete[0][1] <= now:
            wid, _ = self._pending_delete.popleft()
            try:
                self.pi.wave_delete(wid)
            except Exception:
                pass

    def set_throttle(self, throttle: int):
        """Ustawia i powtarza falę dla danej wartości; bezpiecznie przełącza z poprzedniej."""
        throttle = max(0, min(DSHOT_MAX_THROTTLE, int(throttle)))
        with self._lock:
            if throttle == self._last_throttle and self._current_wid is not None:
                self._process_pending_deletes()
                return

            new_wid = self._build_wave(throttle)
            old_wid = self._current_wid

            # Start powtarzania nowej fali (preferuj tryb SYNC jeśli dostępny)
            wave_send_using_mode = getattr(self.pi, 'wave_send_using_mode', None)
            if callable(wave_send_using_mode):
                mode = getattr(pigpio, 'WAVE_MODE_REPEAT_SYNC', 3)
                self.pi.wave_send_using_mode(new_wid, mode)
            else:
                self.pi.wave_send_repeat(new_wid)

            # Bezpieczne, opóźnione kasowanie starej fali
            if old_wid is not None:
                self._pending_delete.append((old_wid, time.time() + 0.005))  # ok. 5 ramek DShot

            self._current_wid = new_wid
            self._last_throttle = throttle

            # Sprzątanie zaległych
            self._process_pending_deletes()

    def stop(self):
        with self._lock:
            try:
                self.pi.wave_tx_stop()
            except Exception:
                pass
            if self._current_wid is not None:
                try:
                    self.pi.wave_delete(self._current_wid)
                except Exception:
                    pass
                self._current_wid = None
            while self._pending_delete:
                wid, _ = self._pending_delete.popleft()
                try:
                    self.pi.wave_delete(wid)
                except Exception:
                    pass
            try:
                self.pi.wave_clear()
            except Exception:
                pass
            self._last_throttle = None

# -------------------- POMOCNICZE --------------------
def ramp(current: float, target: float, up_rate: float, down_rate: float, dt: float) -> float:
    if current < target:
        return min(current + up_rate * dt, target)
    else:
        return max(current - down_rate * dt, target)

def target_when_close(current: float, target: float, min_v: float, max_v: float, eps: float = 2.0) -> float:
    return random.uniform(min_v, max_v) if abs(current - target) < eps else target

# -------------------- LOGIKA --------------------
class ESCTestStand:
    def __init__(self, gpio_pin: int):
        self.gpio_pin = gpio_pin

        # Ustawienia testu (globalne)
        self.duration_seconds = DEFAULT_DURATION

        # PWM (analog)
        self.idle_pwm = DEFAULT_IDLE_PWM
        self.min_pwm = 1100
        self.max_pwm = 2000
        self.ramp_up_per_s = 500
        self.ramp_down_per_s = 800

        # DShot (cyfrowy)
        self.dshot_idle = DSHOT_DEFAULT_IDLE
        self.dshot_min = 300
        self.dshot_max = 1200
        self.dshot_ramp_up_per_s = 500
        self.dshot_ramp_down_per_s = 800

        # Protokół i parametry PWM
        self._protocol = 'PWM490'  # 'PWM50' | 'PWM490' | 'DSHOT300'
        self._pwm_frequency = PWM_FREQUENCY
        self._pwm_range = PWM_RANGE
        self._pu = 1_000_000 / self._pwm_frequency  # okres w µs

        # Stany bieżące
        self._current_pwm = 0.0
        self._target_pwm = 0.0
        self._current_dshot = 0.0
        self._target_dshot = 0.0
        self._end_time = 0.0
        self._time_left_at_pause = 0.0
        self._finishing = False
        self._last_update_time = time.time()

        # Synchronizacja
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._running_event = threading.Event()
        self._armed = False

        # Kolejka powiadomień do UI
        self._ui_notifications = deque()

        # pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.pi.stop()
            raise IOError("pigpio daemon not running! Uruchom: sudo pigpiod -s 1")

        # Ostrzeżenie o -s 1 (nie ma prostego sprawdzenia z API)
        self._notify_bg("Upewnij się, że pigpiod działa z -s 1 (1µs) dla DShot.", 'warning')

        try:
            self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(self.gpio_pin, self._pwm_frequency)
            self.pi.set_PWM_range(self.gpio_pin, self._pwm_range)
        except Exception as err:
            self.pi.stop()
            raise IOError(f"pigpio initialization error: {err}")

        # Nadajnik DShot
        self._dshot_tx = DShotTransmitter(self.pi, self.gpio_pin)

        # Wątek sterujący
        self._pwm_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._pwm_thread.start()

    # ---------- Powiadomienia ----------
    def _notify_bg(self, msg: str, type_: str = 'info'):
        with self._lock:
            self._ui_notifications.append((msg, type_))

    def pop_notifications(self):
        out = []
        with self._lock:
            while self._ui_notifications:
                out.append(self._ui_notifications.popleft())
        return out

    # ---------- Zarządzanie cyklem życia ----------
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()

    def cleanup(self):
        self._stop_event.set()
        self._pwm_thread.join(timeout=1.0)
        if self._pwm_thread.is_alive():
            print("Warning: PWM thread did not terminate properly")
        try:
            self._dshot_tx.stop()
        except Exception:
            pass
        if self.pi.connected:
            try:
                if self._protocol == 'DSHOT300':
                    self.pi.write(self.gpio_pin, 0)
                else:
                    self.pi.set_PWM_dutycycle(self.gpio_pin, 0)
                self.pi.stop()
            except Exception:
                pass

    # ---------- Protokół ----------
    @property
    def protocol(self) -> str:
        return self._protocol

    @property
    def can_change_protocol(self) -> bool:
        return (not self._armed) and (not self.is_running)

    @property
    def pwm50_button_enabled(self) -> bool:
        return self.can_change_protocol and self._protocol != 'PWM50'

    @property
    def pwm490_button_enabled(self) -> bool:
        return self.can_change_protocol and self._protocol != 'PWM490'

    @property
    def dshot_button_enabled(self) -> bool:
        return self.can_change_protocol and self._protocol != 'DSHOT300'

    def set_protocol(self, protocol: str):
        if self._armed or self.is_running:
            ui.notify("Zmiana protokołu możliwa tylko, gdy DISARMED i TEST STOPPED", type='negative')
            return

        protocol = protocol.upper()
        with self._lock:
            if protocol == self._protocol:
                return

            if protocol == 'DSHOT300':
                self._protocol = 'DSHOT300'
                ui.notify("Protokół: DShot300 (UWAGA: wymagane pigpiod -s 1)", type='info')
            elif protocol in ('PWM50', 'PWM490'):
                freq = 50 if protocol == 'PWM50' else 490
                try:
                    self.pi.set_PWM_frequency(self.gpio_pin, freq)
                    self.pi.set_PWM_range(self.gpio_pin, self._pwm_range)
                except Exception as err:
                    ui.notify(f"Frequency error: {err}", type='negative')
                    return

                self._pwm_frequency = freq
                self._pu = 1_000_000 / freq
                self._protocol = protocol
                ui.notify(f"Protokół: PWM {freq}Hz", type='info')
            else:
                ui.notify("Nieobsługiwany protokół", type='negative')

    # ---------- Pętla sterująca ----------
    def _run_loop(self):
        while not self._stop_event.is_set():
            loop_start = time.time()

            # Jeden lock dla całej logiki decyzyjnej
            with self._lock:
                now = time.time()
                dt = now - self._last_update_time
                self._last_update_time = now

                running = self._running_event.is_set()
                paused = self._pause_event.is_set()
                armed = self._armed
                proto = self._protocol

                # Decyzje i aktualizacja stanów
                if running and not paused:
                    if now >= self._end_time and not self._finishing:
                        self._finishing = True
                        if proto == 'DSHOT300':
                            self._target_dshot = float(self.dshot_idle)
                        else:
                            self._target_pwm = float(self.idle_pwm)

                    if proto == 'DSHOT300':
                        self._target_dshot = target_when_close(
                            self._current_dshot, self._target_dshot,
                            self.dshot_min, self.dshot_max
                        )
                        self._current_dshot = ramp(
                            self._current_dshot, self._target_dshot,
                            self.dshot_ramp_up_per_s, self.dshot_ramp_down_per_s, dt
                        )
                        if self._finishing and abs(self._current_dshot - self.dshot_idle) < 1.0:
                            self._running_event.clear()
                            self._pause_event.clear()
                            self._finishing = False
                            self._end_time = 0.0
                            self._time_left_at_pause = 0.0
                    else:
                        self._target_pwm = target_when_close(
                            self._current_pwm, self._target_pwm,
                            self.min_pwm, self.max_pwm
                        )
                        self._current_pwm = ramp(
                            self._current_pwm, self._target_pwm,
                            self.ramp_up_per_s, self.ramp_down_per_s, dt
                        )
                        if self._finishing and abs(self._current_pwm - self.idle_pwm) < 1.0:
                            self._running_event.clear()
                            self._pause_event.clear()
                            self._finishing = False
                            self._end_time = 0.0
                            self._time_left_at_pause = 0.0

                elif armed:
                    if proto == 'DSHOT300':
                        self._current_dshot = float(self.dshot_idle)
                        self._target_dshot = float(self.dshot_idle)
                    else:
                        self._current_pwm = float(self.idle_pwm)
                        self._target_pwm = float(self.idle_pwm)
                else:
                    self._current_pwm = 0.0
                    self._target_pwm = 0.0
                    self._current_dshot = 0.0
                    self._target_dshot = 0.0

                # Przygotowanie wyjścia (poza lockiem)
                if proto == 'DSHOT300':
                    out = ('DSHOT', int(max(0, min(DSHOT_MAX_THROTTLE, round(self._current_dshot)))))
                else:
                    duty = int((self._current_pwm / self._pu) * self._pwm_range)
                    duty = max(0, min(self._pwm_range, duty))
                    out = ('PWM', duty)

            # Wyślij sygnał (poza lockiem)
            try:
                if out[0] == 'DSHOT':
                    self._dshot_tx.set_throttle(out[1])
                else:
                    self.pi.set_PWM_dutycycle(self.gpio_pin, out[1])
            except Exception as e:
                self._notify_bg(f"Output error: {str(e)}", 'negative')
                with self._lock:
                    armed_now = self._armed
                if armed_now:
                    # Rozbrajamy bezpiecznie (z powiadomieniem w kolejce)
                    self.disarm_system()

            # Tempo pętli
            elapsed = time.time() - loop_start
            if out[0] == 'DSHOT':
                time.sleep(max(0, 0.001 - elapsed))  # ~1 kHz
            else:
                time.sleep(max(0, 0.02 - elapsed))   # ~50 Hz

    # ---------- Sterowanie testem ----------
    def arm_system(self):
        with self._lock:
            self._pause_event.clear()
            self._running_event.clear()
            self._time_left_at_pause = 0.0
            self._finishing = False
            self._armed = True
            if self._protocol == 'DSHOT300':
                self._current_dshot = float(self.dshot_idle)
                self._target_dshot = float(self.dshot_idle)
            else:
                self._current_pwm = float(self.idle_pwm)
                self._target_pwm = float(self.idle_pwm)
            self._last_update_time = time.time()

        try:
            if self._protocol == 'DSHOT300':
                self._dshot_tx.set_throttle(int(self.dshot_idle))
            else:
                duty = int((self.idle_pwm / self._pu) * self._pwm_range)
                self.pi.set_PWM_dutycycle(self.gpio_pin, duty)
        except Exception as e:
            self._notify_bg(f"Output error: {str(e)}", type_='negative')
            self.disarm_system()
            return

        self._notify_bg("SYSTEM ARMED", type_='warning')

    def disarm_system(self):
        with self._lock:
            self._pause_event.clear()
            self._running_event.clear()
            self._time_left_at_pause = 0.0
            self._finishing = False
            self._armed = False
            # Reset wszystkich stanów
            self._current_pwm = 0.0
            self._target_pwm = 0.0
            self._current_dshot = 0.0
            self._target_dshot = 0.0
            self._end_time = 0.0
            self._last_update_time = time.time()

        try:
            if self._protocol == 'DSHOT300':
                self._dshot_tx.stop()
                self.pi.write(self.gpio_pin, 0)
            else:
                self.pi.set_PWM_dutycycle(self.gpio_pin, 0)
        except Exception as e:
            self._notify_bg(f"Output error: {str(e)}", type_='negative')

        self._notify_bg("SYSTEM DISARMED", type_='positive')

    def start_test(self):
        with self._lock:
            if not self._armed or self._running_event.is_set():
                return

            if self._protocol == 'DSHOT300':
                if self.dshot_min >= self.dshot_max:
                    self._notify_bg("DShot: MIN musi być < MAX", type_='negative')
                    return
            else:
                if self.min_pwm >= self.max_pwm:
                    self._notify_bg("PWM: MIN musi być < MAX", type_='negative')
                    return

            now = time.time()
            self._running_event.set()
            self._pause_event.clear()
            self._finishing = False
            self._end_time = now + self.duration_seconds
            self._time_left_at_pause = 0.0
            self._last_update_time = now

            if self._protocol == 'DSHOT300':
                self._target_dshot = random.uniform(self.dshot_min, self.dshot_max)
            else:
                self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)

        self._notify_bg("TEST STARTED", type_='positive')

    def stop_test(self, notify: bool = True, hard: bool = False):
        was_dshot = False
        with self._lock:
            if not self._running_event.is_set():
                return

            if hard:
                was_dshot = (self._protocol == 'DSHOT300')
                self._running_event.clear()
                self._pause_event.clear()
                self._finishing = False
                # PEŁNY RESET STANÓW
                self._current_pwm = 0.0
                self._target_pwm = 0.0
                self._current_dshot = 0.0
                self._target_dshot = 0.0
                self._end_time = 0.0
                self._time_left_at_pause = 0.0
                self._last_update_time = time.time()
            else:
                # miękki stop: ramp do idle
                self._pause_event.clear()
                self._finishing = True
                if self._protocol == 'DSHOT300':
                    self._target_dshot = float(self.dshot_idle)
                else:
                    self._target_pwm = float(self.idle_pwm)

        # Natychmiastowe odcięcie sprzętowe dla HARD STOP
        if hard:
            try:
                if was_dshot:
                    self._dshot_tx.stop()
                    self.pi.write(self.gpio_pin, 0)
                else:
                    self.pi.set_PWM_dutycycle(self.gpio_pin, 0)
            except Exception as e:
                self._notify_bg(f"Output error: {str(e)}", type_='negative')

        if notify:
            self._notify_bg("TEST STOPPED" + (" (HARD)" if hard else " (SMOOTH)"), type_='info')

    def toggle_pause(self):
        with self._lock:
            if not self._running_event.is_set():
                return
            if self._finishing:
                # Nie pozwalamy pauzować w trakcie FINISHING
                self._notify_bg("Nie można PAUSE/RESUME w trakcie FINISHING", type_='warning')
                return

            if self._pause_event.is_set():
                # RESUME
                self._pause_event.clear()
                self._end_time = time.time() + self._time_left_at_pause
                self._last_update_time = time.time()
                msg = "TEST RESUMED"
            else:
                # PAUSE
                self._pause_event.set()
                self._time_left_at_pause = max(0.0, self._end_time - time.time())
                msg = "TEST PAUSED"

        self._notify_bg(msg, type_='info')

    # ---------- Właściwości / pomocnicze ----------
    @property
    def pwm_frequency(self) -> int:
        return self._pwm_frequency

    @property
    def is_armed(self) -> bool:
        return self._armed

    @property
    def is_running(self) -> bool:
        return self._running_event.is_set()

    @property
    def is_paused(self) -> bool:
        return self._pause_event.is_set()

    @property
    def can_start(self) -> bool:
        return self._armed and not self._running_event.is_set()

    @property
    def status_text(self) -> str:
        if not self._armed:
            return "DISARMED"
        if self.is_paused:
            return f"PAUSED | Time left: {int(self.time_left)}s"
        if self._finishing:
            return "FINISHING (ramp to idle)"
        if self.is_running:
            return f"RUNNING | Time left: {int(self.time_left)}s"
        if self._protocol == 'DSHOT300':
            return f"ARMED (IDLE: {int(self.dshot_idle)} [DShot])"
        return f"ARMED (IDLE: {self.idle_pwm}µs)"

    @property
    def time_left(self) -> float:
        if not self.is_running:
            return float(self.duration_seconds)
        if self.is_paused:
            return self._time_left_at_pause
        if self._finishing:
            return 0.0
        return max(0.0, self._end_time - time.time())

    @property
    def elapsed_time(self) -> float:
        if not self.is_running:
            return 0.0
        return self.duration_seconds - self.time_left

    @property
    def progress(self) -> float:
        if not self.is_running or self.duration_seconds <= 0:
            return 0.0
        if self._finishing:
            return 1.0
        return min(1.0, self.elapsed_time / self.duration_seconds)

    @property
    def current_value(self) -> str:
        if self._protocol == 'DSHOT300':
            return f"DShot: {int(self._current_dshot)}"
        return f"PWM: {self._current_pwm:.1f}µs"


# -------------------- INICJALIZACJA --------------------
try:
    esc_test_stand = ESCTestStand(GPIO_PIN_ESC)
    init_error = None
    app.on_shutdown(esc_test_stand.cleanup)
except Exception as err:
    esc_test_stand = None
    init_error = err

# -------------------- UI --------------------
def create_controls_section():
    ui.label('ESC TEST STAND').classes('text-h3 text-bold q-my-md text-center w-full')

    with ui.card().classes('w-full max-w-2xl mx-auto'):
        ui.label('STATUS').classes('text-h5')
        ui.label().classes('text-xl').bind_text_from(esc_test_stand, 'status_text')

        # Aktualny protokół
        ui.label().classes('text-subtitle2 text-grey-7').bind_text_from(
            esc_test_stand, 'protocol', lambda p: f"Protocol: {p}"
        )

        # Aktualna wartość (PWM lub DShot)
        ui.label().classes('text-lg font-mono').bind_text_from(
            esc_test_stand, 'current_value'
        )

        # Czas i progres
        time_label = ui.label().classes('text-lg')
        progress_label = ui.label().classes('absolute-center text-black text-bold text-h6')

        with ui.linear_progress(show_value=False) \
                .bind_value_from(esc_test_stand, 'progress') \
                .style('height:35px;border-radius:8px;position:relative;') \
                .props('instant-feedback'):
            pass

        def update_display():
            # Powiadomienia z wątku sterującego
            for msg, t in esc_test_stand.pop_notifications():
                ui.notify(msg, type=t)

            t_left = esc_test_stand.time_left
            time_label.set_text(
                f"Time remaining: {int(t_left//60):02d}:{int(t_left%60):02d}"
            )
            elapsed = esc_test_stand.elapsed_time if esc_test_stand.is_running else 0
            progress_label.set_text(
                f"{esc_test_stand.current_value} | Elapsed: {int(elapsed//60):02d}:{int(elapsed%60):02d}"
            )

        ui.timer(0.1, update_display)

def create_buttons_section():
    ui.separator().classes('q-my-md')

    with ui.row().classes('w-full justify-around items-center'):
        # Arm/Disarm
        ui.button('ARM', on_click=esc_test_stand.arm_system, color='warning') \
            .props('icon=lock_open') \
            .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a)

        ui.button('DISARM', on_click=esc_test_stand.disarm_system, color='positive') \
            .props('icon=lock') \
            .bind_enabled_from(esc_test_stand, 'is_armed')

        # Start/Pause/Stop
        ui.button('START', on_click=esc_test_stand.start_test, color='negative') \
            .props('icon=play_arrow') \
            .bind_enabled_from(esc_test_stand, 'can_start')

        pause_button = ui.button('PAUSE', on_click=esc_test_stand.toggle_pause, color='info') \
            .props('icon=pause') \
            .bind_enabled_from(esc_test_stand, 'is_running')

        # Smooth stop
        ui.button('STOP', on_click=esc_test_stand.stop_test, color='negative') \
            .props('icon=stop') \
            .bind_enabled_from(esc_test_stand, 'is_running')

        # Hard stop (awaryjny)
        ui.button('EMERGENCY STOP', on_click=lambda: esc_test_stand.stop_test(hard=True), color='negative') \
            .props('icon=bolt') \
            .bind_enabled_from(esc_test_stand, 'is_running')

        # Wybór protokołu
        ui.button('PWM 50Hz', on_click=lambda: esc_test_stand.set_protocol('PWM50'), color='primary') \
            .bind_enabled_from(esc_test_stand, 'pwm50_button_enabled')

        ui.button('PWM 490Hz', on_click=lambda: esc_test_stand.set_protocol('PWM490'), color='primary') \
            .bind_enabled_from(esc_test_stand, 'pwm490_button_enabled')

        ui.button('DShot300', on_click=lambda: esc_test_stand.set_protocol('DSHOT300'), color='secondary') \
            .bind_enabled_from(esc_test_stand, 'dshot_button_enabled')

        def update_pause_button():
            if esc_test_stand.is_paused:
                pause_button.set_text('RESUME')
                pause_button.set_icon('play_arrow')
            else:
                pause_button.set_text('PAUSE')
                pause_button.set_icon('pause')

        update_pause_button()
        ui.timer(0.1, update_pause_button)

def create_settings_section():
    with ui.card().classes('w-full max-w-2xl mx-auto q-mt-md'):
        ui.label('SETTINGS').classes('text-h5')

        # Walidacja zakresów w UI
        def ensure_pwm_min(val):
            try:
                v = int(val)
            except Exception:
                return
            if v >= esc_test_stand.max_pwm:
                esc_test_stand.min_pwm = max(MIN_PWM_LIMIT, esc_test_stand.max_pwm - 1)
                ui.notify('MIN PWM musi być < MAX PWM; skorygowano.', type='warning')

        def ensure_pwm_max(val):
            try:
                v = int(val)
            except Exception:
                return
            if v <= esc_test_stand.min_pwm:
                esc_test_stand.max_pwm = min(MAX_PWM_LIMIT, esc_test_stand.min_pwm + 1)
                ui.notify('MAX PWM musi być > MIN PWM; skorygowano.', type='warning')

        def ensure_dshot_min(val):
            try:
                v = int(val)
            except Exception:
                return
            if v >= esc_test_stand.dshot_max:
                esc_test_stand.dshot_min = max(DSHOT_MIN_THROTTLE, esc_test_stand.dshot_max - 1)
                ui.notify('DShot MIN musi być < MAX; skorygowano.', type='warning')

        def ensure_dshot_max(val):
            try:
                v = int(val)
            except Exception:
                return
            if v <= esc_test_stand.dshot_min:
                esc_test_stand.dshot_max = min(DSHOT_MAX_THROTTLE, esc_test_stand.dshot_min + 1)
                ui.notify('DShot MAX musi być > MIN; skorygowano.', type='warning')

        # Globalny czas trwania testu
        ui.number('TEST DURATION (s)',
                  min=1,
                  max=86400,
                  step=1) \
            .bind_value(esc_test_stand, 'duration_seconds') \
            .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

        # Zakładki: PWM vs DShot
        with ui.tabs() as tabs:
            pwm_tab = ui.tab('PWM Settings')
            dshot_tab = ui.tab('DShot Settings')

        with ui.tab_panels(tabs, value=pwm_tab):
            # PWM
            with ui.tab_panel(pwm_tab):
                with ui.grid(columns=2).classes('w-full gap-4'):
                    ui.number('IDLE PWM (µs)',
                              min=MIN_PWM_LIMIT,
                              max=MAX_PWM_LIMIT,
                              step=1) \
                        .bind_value(esc_test_stand, 'idle_pwm') \
                        .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a)

                    ui.number('MIN PWM (µs)',
                              min=MIN_PWM_LIMIT,
                              max=MAX_PWM_LIMIT,
                              step=10,
                              on_change=lambda e: ensure_pwm_min(e.value)) \
                        .bind_value(esc_test_stand, 'min_pwm') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('MAX PWM (µs)',
                              min=MIN_PWM_LIMIT,
                              max=MAX_PWM_LIMIT,
                              step=10,
                              on_change=lambda e: ensure_pwm_max(e.value)) \
                        .bind_value(esc_test_stand, 'max_pwm') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('RAMP UP RATE (µs/s)',
                              min=1,
                              max=1_000_000,
                              step=100) \
                        .bind_value(esc_test_stand, 'ramp_up_per_s') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('RAMP DOWN RATE (µs/s)',
                              min=1,
                              max=1_000_000,
                              step=100) \
                        .bind_value(esc_test_stand, 'ramp_down_per_s') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

            # DShot
            with ui.tab_panel(dshot_tab):
                with ui.grid(columns=2).classes('w-full gap-4'):
                    ui.number('IDLE THROTTLE',
                              min=0,  # pozwalamy na 0 dla pełnego stopu
                              max=DSHOT_MAX_THROTTLE,
                              step=1) \
                        .bind_value(esc_test_stand, 'dshot_idle') \
                        .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a)

                    ui.number('MIN THROTTLE',
                              min=DSHOT_MIN_THROTTLE,
                              max=DSHOT_MAX_THROTTLE,
                              step=10,
                              on_change=lambda e: ensure_dshot_min(e.value)) \
                        .bind_value(esc_test_stand, 'dshot_min') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('MAX THROTTLE',
                              min=DSHOT_MIN_THROTTLE,
                              max=DSHOT_MAX_THROTTLE,
                              step=10,
                              on_change=lambda e: ensure_dshot_max(e.value)) \
                        .bind_value(esc_test_stand, 'dshot_max') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('RAMP UP (units/s)',
                              min=1,
                              max=5000,
                              step=50) \
                        .bind_value(esc_test_stand, 'dshot_ramp_up_per_s') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('RAMP DOWN (units/s)',
                              min=1,
                              max=5000,
                              step=50) \
                        .bind_value(esc_test_stand, 'dshot_ramp_down_per_s') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

@ui.page('/')
def main_page():
    if init_error:
        ui.label("INITIALIZATION ERROR").classes("text-h4 text-negative text-center w-full")
        ui.html(f"<p>pigpio error:</p><pre>{init_error}</pre>") \
            .classes("text-body1 text-center w-full")
        ui.label("Upewnij się, że pigpio działa z próbkowaniem 1us: sudo pigpiod -s 1").classes("text-center w-full")
    else:
        create_controls_section()
        create_buttons_section()
        create_settings_section()

ui.run(
    title='ESC Test Stand',
    port=APP_PORT,
    host='0.0.0.0',
    show=False,
    reload=False,
    favicon='🚀'
)
