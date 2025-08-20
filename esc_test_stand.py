import time
import random
import threading
import logging
from collections import deque

import pigpio

from pigpio_worker import PigpioWorker
from dshot import (
    DShotTransmitter,
    DSHOT_MIN_THROTTLE,
    DSHOT_MAX_THROTTLE,
    DSHOT_DEFAULT_IDLE,
)

# -------------------- KONFIG --------------------
# PWM (analog) domyślne
PWM_FREQUENCY = 490
PWM_RANGE = 255
DEFAULT_IDLE_PWM = 1050
DEFAULT_DURATION = 60
MIN_PWM_LIMIT = 900
MAX_PWM_LIMIT = 2500

# Finishing: maksymalny czas rampy do idle (asekuracyjnie)
FINISHING_MAX_S = 3.0


# -------------------- POMOCNICZE --------------------
def ramp(current: float, target: float, up_rate: float, down_rate: float, dt: float) -> float:
    up_rate = max(1e-6, float(up_rate))
    down_rate = max(1e-6, float(down_rate))
    if current < target:
        return min(current + up_rate * dt, target)
    else:
        return max(current - down_rate * dt, target)


def target_when_close(current: float, target: float, min_v: float, max_v: float, eps: float = 2.0) -> float:
    return random.uniform(min_v, max_v) if abs(current - target) < eps else target


def fmt_mmss(seconds: float) -> str:
    s = int(max(0.0, float(seconds)))
    return f"{s//60:02d}:{s%60:02d}"


# -------------------- LOGIKA --------------------
class ESCTestStand:
    def __init__(self, gpio_pin: int):
        self.gpio_pin = gpio_pin

        # Logger dla błędów
        self._logger = logging.getLogger('ESCTestStand')
        self._logger.setLevel(logging.INFO)
        if not self._logger.handlers:
            fh = logging.FileHandler('esc_test_errors.log')
            fh.setLevel(logging.ERROR)
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            fh.setFormatter(formatter)
            self._logger.addHandler(fh)

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
        self._pu = 1_000_000 / self._pwm_frequency  # okres w µs dla PWM490

        # Stany bieżące
        self._current_pwm = 0.0
        self._target_pwm = 0.0
        self._current_dshot = 0.0
        self._target_dshot = 0.0

        self._end_time = 0.0
        self._start_time = 0.0
        self._time_left_at_pause = 0.0

        self._finishing = False
        self._finishing_deadline = 0.0
        self._auto_finish = False
        self._stopping_soft = False
        self._last_update_time = time.time()

        # Snapshoty do UI
        self._progress_snapshot = 0.0
        self._elapsed_snapshot = 0.0
        self._time_left_snapshot = float(self.duration_seconds)

        # Synchronizacja
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._wake_event = threading.Event()
        self._pause_event = threading.Event()
        self._running_event = threading.Event()
        self._armed = False

        # Kolejka powiadomień do UI
        self._ui_notifications = deque()

        # pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            try:
                self.pi.stop()
            except Exception as e:
                self._log_error("pigpio stop during init", e)
            raise IOError("pigpio daemon not running! Uruchom: sudo pigpiod -s 1")

        # Worker do pigpio
        self._pw = PigpioWorker(self.pi)

        self._notify_bg("Upewnij się, że pigpiod działa z -s 1 (1µs) dla DShot.", 'warning')

        try:
            self._safe_pigpio_call('set_mode', self.gpio_pin, pigpio.OUTPUT, timeout=1.0)
            r1 = self._safe_pigpio_call('set_PWM_frequency', self.gpio_pin, self._pwm_frequency, timeout=1.0)
            r2 = self._safe_pigpio_call('set_PWM_range', self.gpio_pin, self._pwm_range, timeout=1.0)
            if isinstance(r1, int) and r1 < 0:
                raise RuntimeError(f"set_PWM_frequency failed: {r1}")
            if isinstance(r2, int) and r2 < 0:
                raise RuntimeError(f"set_PWM_range failed: {r2}")
        except Exception as err:
            self._pw.stop()
            try:
                self.pi.stop()
            except Exception as e:
                self._log_error("pigpio stop after init error", e)
            raise IOError(f"pigpio initialization error: {err}")

        # Nadajnik DShot
        self._dshot_tx = DShotTransmitter(self._pw, self.gpio_pin)

        # Jeśli domyślny protokół to DSHOT300, wystartuj od razu transmisję 0
        if self._protocol == 'DSHOT300':
            try:
                self._dshot_tx.set_throttle(0)
            except Exception as e:
                self._log_error("Initial DShot throttle=0 failed", e)

        # Wątek sterujący
        self._pwm_thread = threading.Thread(target=self._run_loop, daemon=True, name='esc-run-loop')
        self._pwm_thread.start()

    # ---------- Logowanie / bezpieczne wywołania ----------
    def _log_error(self, context: str, error: Exception):
        self._logger.error(f"{context}: {type(error).__name__}: {str(error)}", exc_info=True)

    def _safe_pigpio_call(self, method: str, *args, **kwargs):
        """Bezpieczne wywołanie metody pigpio z obsługą rozłączenia."""
        if not self.pi.connected:
            self._handle_pigpio_disconnected()
            raise ConnectionError("pigpio disconnected")

        try:
            return self._pw.call(method, *args, **kwargs)
        except ConnectionError:
            # Rozłączenie w trakcie
            self._handle_pigpio_disconnected()
            raise
        except Exception as e:
            # Loguj inne błędy
            self._log_error(f"pigpio call failed: {method}", e)
            raise

    def _handle_pigpio_disconnected(self):
        """Obsłuż rozłączenie z pigpio."""
        with self._lock:
            was_armed = self._armed
            # Natychmiastowe zatrzymanie
            self._armed = False
            self._running_event.clear()
            self._finishing = False

        if was_armed:
            self._notify_bg("PIGPIO DISCONNECTED - EMERGENCY STOP", 'negative')

        # Spróbuj ponownego połączenia
        self._attempt_reconnect()

    def _attempt_reconnect(self):
        """Próba ponownego połączenia z pigpio."""
        try:
            if self.pi.connected:
                return

            # Zamknij stare połączenie
            try:
                self.pi.stop()
            except Exception as e:
                self._log_error("pi.stop during reconnect", e)

            # Nowe połączenie
            self.pi = pigpio.pi()
            if self.pi.connected:
                # Reinicjalizacja workera i DShot
                old_worker = self._pw
                self._pw = PigpioWorker(self.pi)
                try:
                    old_worker.stop()
                except Exception as e:
                    self._log_error("old_worker.stop during reconnect", e)

                self._dshot_tx = DShotTransmitter(self._pw, self.gpio_pin)

                # Podstawowa re-konfiguracja GPIO
                try:
                    self._safe_pigpio_call('set_mode', self.gpio_pin, pigpio.OUTPUT, timeout=1.0)
                    if self._protocol == 'PWM490':
                        self._safe_pigpio_call('set_PWM_frequency', self.gpio_pin, self._pwm_frequency, timeout=1.0)
                        self._safe_pigpio_call('set_PWM_range', self.gpio_pin, self._pwm_range, timeout=1.0)
                    elif self._protocol == 'PWM50':
                        self._safe_pigpio_call('set_servo_pulsewidth', self.gpio_pin, 0, timeout=1.0)
                    elif self._protocol == 'DSHOT300':
                        try:
                            self._dshot_tx.set_throttle(0)
                        except Exception as e:
                            self._log_error("DShot set_throttle(0) after reconnect", e)
                except Exception as e:
                    self._log_error("GPIO reconfiguration after reconnect", e)

                self._notify_bg("Reconnected to pigpio", 'positive')
            else:
                self._notify_bg("Failed to reconnect to pigpio", 'negative')

        except Exception as e:
            self._log_error("Reconnection failed", e)

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

    def _handle_pigpio_error(self, e: Exception):
        self._log_error("pigpio error", e)
        self._notify_bg(f"pigpio error: {e}", 'negative')
        try:
            self.disarm_system()
        except Exception as ex:
            self._log_error("disarm_system after pigpio error", ex)

    def cleanup(self):
        self._stop_event.set()
        self._wake_event.set()
        self._pwm_thread.join(timeout=2.0)
        if self._pwm_thread.is_alive():
            print("Warning: PWM thread did not terminate properly")

        try:
            self._dshot_tx.stop()
        except Exception as e:
            self._log_error("Failed to stop DShot transmitter", e)

        try:
            if self._protocol == 'DSHOT300':
                try:
                    self._safe_pigpio_call('write', self.gpio_pin, 0, timeout=1.0)
                except Exception as e:
                    self._log_error("Failed to write 0 on DSHOT300 during cleanup", e)
            elif self._protocol == 'PWM50':
                try:
                    self._safe_pigpio_call('set_servo_pulsewidth', self.gpio_pin, 0, timeout=1.0)
                except Exception as e:
                    self._log_error("Failed to set_servo_pulsewidth 0 during cleanup", e)
            else:
                try:
                    self._safe_pigpio_call('set_PWM_dutycycle', self.gpio_pin, 0, timeout=1.0)
                except Exception as e:
                    self._log_error("Failed to set_PWM_dutycycle 0 during cleanup", e)
        except Exception as e:
            self._log_error("Failed to reset GPIO in cleanup", e)

        try:
            self._pw.stop()
        except Exception as e:
            self._log_error("PigpioWorker.stop in cleanup", e)
        try:
            if self.pi.connected:
                self.pi.stop()
        except Exception as e:
            self._log_error("pigpio.stop in cleanup", e)

    # ---------- Protokół ----------
    @property
    def protocol(self) -> str:
        return self._protocol

    @property
    def can_change_protocol(self) -> bool:
        return (not self._armed) and (not self.is_running) and (not self._finishing)

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
        if self._armed or self.is_running or self._finishing:
            self._notify_bg("Zmiana protokołu możliwa tylko, gdy DISARMED i TEST STOPPED", 'negative')
            return

        protocol = protocol.upper()
        with self._lock:
            if protocol == self._protocol:
                return

            if protocol == 'DSHOT300':
                # Wyłącz poprzednie tryby i ustaw niski poziom na pinie
                try:
                    self._safe_pigpio_call('set_PWM_dutycycle', self.gpio_pin, 0, timeout=1.0)
                except Exception:
                    pass
                try:
                    self._safe_pigpio_call('set_servo_pulsewidth', self.gpio_pin, 0, timeout=1.0)
                except Exception:
                    pass
                try:
                    self._safe_pigpio_call('write', self.gpio_pin, 0, timeout=1.0)
                except Exception:
                    pass
                self._protocol = 'DSHOT300'
                self._notify_bg("Protokół: DShot300 (UWAGA: wymagane pigpiod -s 1)", 'info')
                # WAŻNE: od razu start transmisji z 0
                try:
                    self._dshot_tx.set_throttle(0)
                except Exception as e:
                    self._log_error("DShot set_throttle(0) on set_protocol", e)

            elif protocol == 'PWM50':
                try:
                    self._safe_pigpio_call('set_PWM_dutycycle', self.gpio_pin, 0, timeout=1.0)
                except Exception:
                    pass
                self._protocol = 'PWM50'
                self._notify_bg("Protokół: PWM 50Hz (servo pulses)", 'info')

            elif protocol == 'PWM490':
                try:
                    self._safe_pigpio_call('set_servo_pulsewidth', self.gpio_pin, 0, timeout=1.0)
                except Exception:
                    pass
                freq = 490
                try:
                    r1 = self._safe_pigpio_call('set_PWM_frequency', self.gpio_pin, freq, timeout=1.0)
                    r2 = self._safe_pigpio_call('set_PWM_range', self.gpio_pin, self._pwm_range, timeout=1.0)
                    if isinstance(r1, int) and r1 < 0:
                        raise RuntimeError(f"set_PWM_frequency failed: {r1}")
                    if isinstance(r2, int) and r2 < 0:
                        raise RuntimeError(f"set_PWM_range failed: {r2}")
                except Exception as err:
                    self._notify_bg(f"Frequency error: {err}", 'negative')
                    return
                self._pwm_frequency = freq
                self._pu = 1_000_000 / freq
                self._protocol = 'PWM490'
                self._notify_bg("Protokół: PWM 490Hz", 'info')
            else:
                self._notify_bg("Nieobsługiwany protokół", 'negative')

        self._wake_event.set()

    # ---------- Pętla sterująca ----------
    def _run_loop(self):
        while not self._stop_event.is_set():
            loop_start = time.time()

            with self._lock:
                now = time.time()
                dt = now - self._last_update_time
                self._last_update_time = now

                running = self._running_event.is_set()
                paused = self._pause_event.is_set()
                armed = self._armed
                proto = self._protocol

                # Domyślne wyjście (ZAWSZE zdefiniowane)
                if proto == 'DSHOT300':
                    out = ('DSHOT', 0)
                elif proto == 'PWM50':
                    out = ('SERVO', 0)
                else:
                    out = ('PWM', 0)

                if self._finishing:
                    if proto == 'DSHOT300':
                        target = float(max(DSHOT_MIN_THROTTLE, min(DSHOT_MAX_THROTTLE, self.dshot_idle)))
                        self._target_dshot = target
                        self._current_dshot = ramp(
                            self._current_dshot, target,
                            self.dshot_ramp_up_per_s, self.dshot_ramp_down_per_s, dt
                        )
                        out = ('DSHOT', int(max(0, min(DSHOT_MAX_THROTTLE, round(self._current_dshot)))))
                    elif proto == 'PWM50':
                        target = float(self.idle_pwm)
                        self._target_pwm = target
                        self._current_pwm = ramp(self._current_pwm, target, self.ramp_up_per_s, self.ramp_down_per_s, dt)
                        out = ('SERVO', int(self._current_pwm))
                    else:  # PWM490
                        target = float(self.idle_pwm)
                        self._target_pwm = target
                        self._current_pwm = ramp(self._current_pwm, target, self.ramp_up_per_s, self.ramp_down_per_s, dt)
                        duty = int((self._current_pwm / self._pu) * self._pwm_range)
                        out = ('PWM', max(0, min(self._pwm_range, duty)))

                    close_enough = (abs((self._current_dshot if proto == 'DSHOT300' else self._current_pwm) - target) < 1.0)
                    timeout_reached = (now >= self._finishing_deadline) if self._finishing_deadline > 0 else False
                    if close_enough or timeout_reached:
                        self._finishing = False
                        self._auto_finish = False
                        self._stopping_soft = False
                        self._finishing_deadline = 0.0
                        self._time_left_at_pause = 0.0

                elif running and not paused:
                    if now >= self._end_time:
                        self._running_event.clear()
                        self._finishing = True
                        self._auto_finish = True
                        self._stopping_soft = False
                        self._finishing_deadline = now + FINISHING_MAX_S
                        self._progress_snapshot = 1.0
                        self._elapsed_snapshot = float(self.duration_seconds)
                        self._time_left_snapshot = 0.0

                        if proto == 'DSHOT300':
                            target = float(max(DSHOT_MIN_THROTTLE, min(DSHOT_MAX_THROTTLE, self.dshot_idle)))
                            self._target_dshot = target
                            self._current_dshot = ramp(
                                self._current_dshot, target,
                                self.dshot_ramp_up_per_s, self.dshot_ramp_down_per_s, dt
                            )
                            out = ('DSHOT', int(max(0, min(DSHOT_MAX_THROTTLE, round(self._current_dshot)))))
                        elif proto == 'PWM50':
                            target = float(self.idle_pwm)
                            self._target_pwm = target
                            self._current_pwm = ramp(self._current_pwm, target, self.ramp_up_per_s, self.ramp_down_per_s, dt)
                            out = ('SERVO', int(self._current_pwm))
                        else:
                            target = float(self.idle_pwm)
                            self._target_pwm = target
                            self._current_pwm = ramp(self._current_pwm, target, self.ramp_up_per_s, self.ramp_down_per_s, dt)
                            duty = int((self._current_pwm / self._pu) * self._pwm_range)
                            out = ('PWM', max(0, min(self._pwm_range, duty)))

                    else:
                        if proto == 'DSHOT300':
                            self._target_dshot = target_when_close(self._current_dshot, self._target_dshot, self.dshot_min, self.dshot_max)
                            self._current_dshot = ramp(self._current_dshot, self._target_dshot,
                                                       self.dshot_ramp_up_per_s, self.dshot_ramp_down_per_s, dt)
                            out = ('DSHOT', int(max(0, min(DSHOT_MAX_THROTTLE, round(self._current_dshot)))))
                        elif proto == 'PWM50':
                            self._target_pwm = target_when_close(self._current_pwm, self._target_pwm, self.min_pwm, self.max_pwm)
                            self._current_pwm = ramp(self._current_pwm, self._target_pwm,
                                                     self.ramp_up_per_s, self.ramp_down_per_s, dt)
                            out = ('SERVO', int(self._current_pwm))
                        else:
                            self._target_pwm = target_when_close(self._current_pwm, self._target_pwm, self.min_pwm, self.max_pwm)
                            self._current_pwm = ramp(self._current_pwm, self._target_pwm,
                                                     self.ramp_up_per_s, self.ramp_down_per_s, dt)
                            duty = int((self._current_pwm / self._pu) * self._pwm_range)
                            out = ('PWM', max(0, min(self._pwm_range, duty)))

                        if self.duration_seconds > 0:
                            elapsed = min(self.duration_seconds, max(0.0, now - self._start_time))
                            self._elapsed_snapshot = elapsed
                            self._progress_snapshot = min(1.0, elapsed / self.duration_seconds)
                            self._time_left_snapshot = max(0.0, self._end_time - now)

                elif armed:
                    if proto == 'DSHOT300':
                        self._current_dshot = float(max(DSHOT_MIN_THROTTLE, min(DSHOT_MAX_THROTTLE, self.dshot_idle)))
                        self._target_dshot = self._current_dshot
                        out = ('DSHOT', int(self._current_dshot))
                    elif proto == 'PWM50':
                        self._current_pwm = float(self.idle_pwm)
                        self._target_pwm = float(self.idle_pwm)
                        out = ('SERVO', int(self._current_pwm))
                    else:
                        self._current_pwm = float(self.idle_pwm)
                        self._target_pwm = float(self.idle_pwm)
                        duty = int((self._current_pwm / self._pu) * self._pwm_range)
                        out = ('PWM', max(0, min(self._pwm_range, duty)))
                else:
                    # DISARMED: ciągła transmisja 0
                    if proto == 'DSHOT300':
                        out = ('DSHOT', 0)
                    elif proto == 'PWM50':
                        out = ('SERVO', 0)
                    else:
                        out = ('PWM', 0)

            # Wyślij sygnał
            try:
                if not self._pw.connected:
                    raise ConnectionError("pigpio disconnected")
                if out[0] == 'DSHOT':
                    self._dshot_tx.set_throttle(out[1])
                elif out[0] == 'SERVO':
                    self._safe_pigpio_call('set_servo_pulsewidth', self.gpio_pin, int(out[1]), retries=3, timeout=1.0)
                else:
                    self._safe_pigpio_call('set_PWM_dutycycle', self.gpio_pin, out[1], retries=3, timeout=1.0)
            except Exception as e:
                self._handle_pigpio_error(e)

            # Tempo pętli
            elapsed = time.time() - loop_start
            if out[0] == 'DSHOT':
                timeout = max(0.0, 0.001 - elapsed)  # ~1 kHz
            else:
                timeout = max(0.0, 0.01 - elapsed)   # ~100 Hz
            self._wake_event.wait(timeout=timeout)
            self._wake_event.clear()

    # ---------- Sterowanie testem ----------
    def arm_system(self):
        with self._lock:
            self._pause_event.clear()
            self._running_event.clear()
            self._time_left_at_pause = 0.0
            self._finishing = False
            self._auto_finish = False
            self._stopping_soft = False
            self._finishing_deadline = 0.0
            self._armed = True
            self._progress_snapshot = 0.0
            self._elapsed_snapshot = 0.0
            self._time_left_snapshot = float(self.duration_seconds)

            if self._protocol == 'DSHOT300':
                # start od 0 (krótko), potem idle
                self._current_dshot = 0.0
                self._target_dshot = 0.0
            else:
                self._current_pwm = float(self.idle_pwm)
                self._target_pwm = float(self.idle_pwm)
            self._last_update_time = time.time()

        try:
            if self._protocol == 'DSHOT300':
                # Wyślij 0 teraz, po 200ms przejdź na idle
                self._dshot_tx.set_throttle(0)

                def set_idle():
                    with self._lock:
                        if self._armed and self._protocol == 'DSHOT300':
                            idle_val = int(max(DSHOT_MIN_THROTTLE, min(DSHOT_MAX_THROTTLE, self.dshot_idle)))
                            self._current_dshot = float(idle_val)
                            self._target_dshot = float(idle_val)
                            try:
                                self._dshot_tx.set_throttle(idle_val)
                            except Exception as e:
                                self._log_error("DShot set_idle after arm", e)

                threading.Timer(0.2, set_idle).start()
            elif self._protocol == 'PWM50':
                self._safe_pigpio_call('set_servo_pulsewidth', self.gpio_pin, int(self.idle_pwm), retries=3, timeout=1.0)
            else:
                duty = int((self.idle_pwm / self._pu) * self._pwm_range)
                self._safe_pigpio_call('set_PWM_dutycycle', self.gpio_pin, duty, retries=3, timeout=1.0)
        except Exception as e:
            self._notify_bg(f"Output error: {str(e)}", 'negative')
            try:
                self.disarm_system()
            except Exception as ex:
                self._log_error("disarm_system after arm error", ex)
            return

        self._notify_bg("SYSTEM ARMED", type_='warning')
        self._wake_event.set()

    def disarm_system(self):
        with self._lock:
            self._pause_event.clear()
            self._running_event.clear()
            self._time_left_at_pause = 0.0
            self._finishing = False
            self._auto_finish = False
            self._stopping_soft = False
            self._finishing_deadline = 0.0
            self._armed = False
            # Reset stanów
            self._current_pwm = 0.0
            self._target_pwm = 0.0
            self._current_dshot = 0.0
            self._target_dshot = 0.0
            self._end_time = 0.0
            self._start_time = 0.0
            self._last_update_time = time.time()
            self._progress_snapshot = 0.0
            self._elapsed_snapshot = 0.0
            self._time_left_snapshot = float(self.duration_seconds)

        try:
            if self._protocol == 'DSHOT300':
                # dalej transmitujemy 0 (ciągłość ramek)
                try:
                    self._dshot_tx.set_throttle(0)
                except Exception as e:
                    self._log_error("DShot set_throttle(0) on disarm", e)
            elif self._protocol == 'PWM50':
                try:
                    self._safe_pigpio_call('set_servo_pulsewidth', self.gpio_pin, 0, timeout=1.0)
                except Exception as e:
                    self._log_error("SERVO 0 on disarm", e)
            else:
                try:
                    self._safe_pigpio_call('set_PWM_dutycycle', self.gpio_pin, 0, timeout=1.0)
                except Exception as e:
                    self._log_error("PWM 0 on disarm", e)
        except Exception as e:
            self._notify_bg(f"Output error: {str(e)}", 'negative')

        self._notify_bg("SYSTEM DISARMED", type_='positive')
        self._wake_event.set()

    def start_test(self):
        with self._lock:
            if not self._armed or self._running_event.is_set() or self._finishing:
                return

            if self._protocol == 'DSHOT300':
                if self.dshot_min >= self.dshot_max:
                    self._notify_bg("DShot: MIN musi być < MAX", type_='negative')
                    return
                self.dshot_idle = int(max(DSHOT_MIN_THROTTLE, min(DSHOT_MAX_THROTTLE, self.dshot_idle)))
            else:
                if self.min_pwm >= self.max_pwm:
                    self._notify_bg("PWM: MIN musi być < MAX", type_='negative')
                    return

            now = time.time()
            self._running_event.set()
            self._pause_event.clear()
            self._finishing = False
            self._auto_finish = False
            self._stopping_soft = False
            self._finishing_deadline = 0.0
            self._start_time = now
            self._end_time = now + self.duration_seconds
            self._time_left_at_pause = 0.0
            self._progress_snapshot = 0.0
            self._elapsed_snapshot = 0.0
            self._time_left_snapshot = float(self.duration_seconds)
            self._last_update_time = now

            if self._protocol == 'DSHOT300':
                self._target_dshot = random.uniform(self.dshot_min, self.dshot_max)
            else:
                self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)

        self._notify_bg("TEST STARTED", type_='positive')
        self._wake_event.set()

    def stop_test(self, notify: bool = True, hard: bool = False):
        with self._lock:
            proto = self._protocol
            running_now = self._running_event.is_set()
            finishing_now = self._finishing

            if hard:
                self._running_event.clear()
                self._pause_event.clear()
                self._finishing = False
                self._auto_finish = False
                self._stopping_soft = False
                self._finishing_deadline = 0.0
                self._current_pwm = 0.0
                self._target_pwm = 0.0
                self._current_dshot = 0.0
                self._target_dshot = 0.0
                self._end_time = 0.0
                self._time_left_at_pause = 0.0
                self._last_update_time = time.time()
            else:
                if not finishing_now:
                    self._running_event.clear()
                    self._pause_event.clear()
                    self._stopping_soft = True
                    self._finishing = True
                    self._auto_finish = False
                    self._finishing_deadline = time.time() + FINISHING_MAX_S
                    if proto == 'DSHOT300':
                        self._target_dshot = float(max(DSHOT_MIN_THROTTLE, min(DSHOT_MAX_THROTTLE, self.dshot_idle)))
                    else:
                        self._target_pwm = float(self.idle_pwm)

        if hard:
            try:
                if proto == 'DSHOT300':
                    # utrzymujemy transmisję, ustawiamy 0
                    self._dshot_tx.set_throttle(0)
                elif proto == 'PWM50':
                    try:
                        self._safe_pigpio_call('set_servo_pulsewidth', self.gpio_pin, 0, timeout=1.0)
                    except Exception as e:
                        self._log_error("SERVO 0 on hard stop", e)
                else:
                    try:
                        self._safe_pigpio_call('set_PWM_dutycycle', self.gpio_pin, 0, timeout=1.0)
                    except Exception as e:
                        self._log_error("PWM 0 on hard stop", e)
            except Exception as e:
                self._notify_bg(f"Output error: {str(e)}", type_='negative')

        if notify:
            if hard:
                self._notify_bg("TEST STOPPED (HARD)", type_='info')
            else:
                if running_now and not finishing_now:
                    self._notify_bg("TEST STOPPED (SMOOTH)", type_='info')
                elif finishing_now:
                    self._notify_bg("Already stopping…", type_='warning')

        self._wake_event.set()

    def toggle_pause(self):
        with self._lock:
            if not self._running_event.is_set() or self._finishing:
                self._notify_bg("Nie można PAUSE/RESUME (brak RUN lub trwa FINISHING)", type_='warning')
                return

            now = time.time()
            if self._pause_event.is_set():
                # RESUME
                self._pause_event.clear()
                self._end_time = now + self._time_left_at_pause
                self._start_time = now - self._elapsed_snapshot
                self._last_update_time = now
                msg = "TEST RESUMED"
            else:
                # PAUSE
                self._pause_event.set()
                self._time_left_at_pause = max(0.0, self._end_time - now)
                self._time_left_snapshot = self._time_left_at_pause
                msg = "TEST PAUSED"

        self._notify_bg(msg, type_='info')
        self._wake_event.set()

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
    def is_finishing(self) -> bool:
        return self._finishing

    @property
    def can_start(self) -> bool:
        return self._armed and (not self._running_event.is_set()) and (not self._finishing)

    @property
    def can_emergency_stop(self) -> bool:
        return self._running_event.is_set() or self._finishing

    @property
    def status_text(self) -> str:
        if not self._armed:
            return "DISARMED"
        if self._finishing:
            return "FINISHING (ramp to idle)"
        if self.is_paused:
            return f"PAUSED | Time left: {fmt_mmss(self.time_left)}"
        if self.is_running:
            return f"RUNNING | Time left: {fmt_mmss(self.time_left)}"
        if self._protocol == 'DSHOT300':
            return f"ARMED (IDLE: {int(max(DSHOT_MIN_THROTTLE, min(DSHOT_MAX_THROTTLE, self.dshot_idle)))} [DShot])"
        return f"ARMED (IDLE: {self.idle_pwm}µs)"

    @property
    def time_left(self) -> float:
        if self.is_running and not self.is_paused:
            return max(0.0, self._end_time - time.time())
        if self.is_paused:
            return self._time_left_at_pause
        if self._finishing:
            return self._time_left_snapshot
        return float(self.duration_seconds)

    @property
    def time_left_text(self) -> str:
        if self._finishing:
            return "Stopping…"
        return fmt_mmss(self.time_left)

    @property
    def elapsed_time(self) -> float:
        return float(self._elapsed_snapshot)

    @property
    def progress(self) -> float:
        return float(self._progress_snapshot)

    @property
    def current_value(self) -> str:
        if self._protocol == 'DSHOT300':
            return f"DShot: {int(self._current_dshot)}"
        return f"PWM: {self._current_pwm:.1f}µs"
