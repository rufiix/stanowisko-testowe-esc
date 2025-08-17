import time
import random
import threading
import logging
import queue
from collections import deque
from typing import Optional, Any
from concurrent.futures import Future, CancelledError, InvalidStateError

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

# DShot (cyfrowy): limity wartości
DSHOT_MIN_THROTTLE = 48   # <48 to komendy; >=48 to gaz
DSHOT_MAX_THROTTLE = 2047
DSHOT_DEFAULT_IDLE = DSHOT_MIN_THROTTLE  # bezpieczny idle (nie komendy)

# DShot: bezpieczne opóźnienie przed kasowaniem poprzedniej fali
DSHOT_WAVE_DELETE_DELAY_S = 0.02  # 20 ms

# Finishing: maksymalny czas rampy do idle (asekuracyjnie)
FINISHING_MAX_S = 3.0


# -------------------- PIGPIO WORKER --------------------
class PigpioWorker:
    """Wątek do seriowania wszystkich wywołań pigpio (thread-safe).
       call(): synchronicznie z timeoutem; submit(): asynchronicznie (Future).
       Reentrancy: call() w wątku worker'a wykonuje inline (bez kolejki)."""

    def __init__(self, pi: pigpio.pi, name: str = 'pigpio-worker'):
        self.pi = pi
        self._q: "queue.Queue[Optional[tuple]]" = queue.Queue()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name=name)
        self._thread_id: Optional[int] = None  # ID wątku workera (dla reentrancy)
        self._thread.start()
        # Poczekaj aż wątek się uruchomi (max ~5 s)
        for _ in range(50):
            if self._thread_id is not None:
                break
            time.sleep(0.1)

    def _loop(self):
        # Zapisz ID wątku (do reentrancy detection)
        self._thread_id = threading.get_ident()

        while not self._stop_event.is_set():
            try:
                item = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            func_desc, args, kwargs, retries, retry_delay, future = item
            if future.cancelled():
                continue
            try:
                res = self._execute(func_desc, args, kwargs, retries, retry_delay)
                try:
                    future.set_result(res)
                except (InvalidStateError, CancelledError):
                    pass
            except Exception as e:
                try:
                    future.set_exception(e)
                except (InvalidStateError, CancelledError):
                    pass

    def _execute(self, func_desc: Any, args: tuple, kwargs: dict, retries: int, retry_delay: float):
        last_exc = None
        for attempt in range(retries):
            if not self.pi.connected:
                raise ConnectionError("pigpio disconnected")
            try:
                if isinstance(func_desc, str):
                    fn = getattr(self.pi, func_desc)
                    return fn(*args, **kwargs)
                elif callable(func_desc):
                    return func_desc(self.pi, *args, **kwargs)
                else:
                    raise ValueError("Invalid func descriptor for PigpioWorker.call")
            except AttributeError as e:
                last_exc = e
                if attempt >= retries - 1:
                    raise
                time.sleep(retry_delay)
            except Exception as e:
                last_exc = e
                if attempt >= retries - 1:
                    raise
                time.sleep(retry_delay)
        if last_exc:
            raise last_exc

    def call(self, func: Any, *args, retries: int = 3, retry_delay: float = 0.02,
             timeout: Optional[float] = 5.0, **kwargs):
        if self._stop_event.is_set():
            raise RuntimeError("PigpioWorker is stopped")

        # Wykonaj inline, jeżeli jesteśmy w wątku workera (zapobiega deadlockom)
        if threading.get_ident() == self._thread_id:
            return self._execute(func, args, kwargs, retries, retry_delay)

        # Inny wątek: użyj kolejki i Future
        fut: Future = Future()
        self._q.put((func, args, kwargs, retries, retry_delay, fut))
        try:
            return fut.result(timeout=timeout)
        except Exception:
            try:
                fut.cancel()
            except Exception:
                pass
            raise

    def submit(self, func: Any, *args, retries: int = 3, retry_delay: float = 0.02, **kwargs) -> Future:
        if self._stop_event.is_set():
            raise RuntimeError("PigpioWorker is stopped")
        fut: Future = Future()
        self._q.put((func, args, kwargs, retries, retry_delay, fut))
        return fut

    def stop(self):
        self._stop_event.set()
        self._q.put(None)
        self._thread.join(timeout=2.0)

    @property
    def connected(self) -> bool:
        return bool(self.pi.connected) and (not self._stop_event.is_set())


# -------------------- DSHOT TX --------------------
class DShotTransmitter:
    """Nadajnik DShot oparty o pigpio wave z bezpiecznym przełączaniem i kasowaniem (via PigpioWorker).
       Bit time konfigurowalny (3 µs lub 4 µs; wymagane pigpiod -s 1 dla sensownej pracy)."""

    MAX_PENDING_DELETES = 100  # Limit kolejki oczekujących usunięć

    def __init__(self, worker: PigpioWorker, gpio: int, bit_time_us: int = 3):
        if gpio >= 32:
            raise ValueError("DShot via pigpio wave wspiera GPIO 0-31 (bank 0). Użyj pinu < 32.")
        self.worker = worker
        self.gpio = gpio
        self._current_wid: Optional[int] = None
        self._last_throttle: Optional[int] = None
        self._lock = threading.Lock()
        self._tx_lock = threading.Lock()  # serializacja wave*
        self._pending_delete = deque()  # (wid, safe_after_time)
        self._pending_ids = set()       # deduplikacja WID w kolejce
        self._req_counter = 0

        # Timing DShot
        self._tbit_us = 3
        self._h1 = 2
        self._l1 = 1
        self._h0 = 1
        self._l0 = 2
        self._gap_us = 1000 - 16 * self._tbit_us  # ok. 1 kHz
        self.set_bit_time_us(bit_time_us)

    def set_bit_time_us(self, bit_us: int):
        """Ustaw czas bitu DShot (µs). Zalecane 3 µs. 4 µs bywa akceptowalne."""
        with self._tx_lock:
            bit_us = int(max(1, min(10, bit_us)))
            self._tbit_us = bit_us
            # 1 ≈ 1/3-1/2 całego bitu wysoko
            if bit_us == 3:
                self._h1, self._l1 = 2, 1
                self._h0, self._l0 = 1, 2
            elif bit_us == 4:
                self._h1, self._l1 = 3, 1
                self._h0, self._l0 = 1, 3
            else:
                # fallback: proporcje 2/3 i 1/3
                self._h1, self._l1 = max(1, bit_us * 2 // 3), max(1, bit_us - max(1, bit_us * 2 // 3))
                self._h0, self._l0 = max(1, bit_us // 3), max(1, bit_us - max(1, bit_us // 3))
            self._gap_us = max(10, 1000 - 16 * self._tbit_us)

    @staticmethod
    def _make_packet(throttle: int, telemetry: int = 0) -> int:
        t = max(0, min(2047, int(throttle)))
        payload = ((t << 1) | (telemetry & 0x1)) & 0x0FFF
        csum = 0
        for i in range(3):
            csum ^= (payload >> (i * 4)) & 0xF
        csum &= 0xF
        return (payload << 4) | csum

    def _cleanup_old_waves(self):
        """Usuń stare i nieużywane fale (gdy zasoby są wysoko użyte)."""
        tx_at = self._get_tx_at()
        now = time.time()
        to_delete = []
        with self._lock:
            remaining = deque()
            while self._pending_delete:
                wid, safe_time = self._pending_delete.popleft()
                if (safe_time <= now) and (wid != tx_at) and (wid != self._current_wid):
                    to_delete.append(wid)
                    self._pending_ids.discard(wid)
                else:
                    remaining.append((wid, safe_time))
            self._pending_delete = remaining

        for wid in to_delete:
            try:
                self.worker.call('wave_delete', wid, timeout=0.5)
            except Exception:
                pass

    def _emergency_cleanup(self):
        """Usuń tylko nieaktywne fale (awaryjnie, bez wave_clear())."""
        with self._lock:
            tx_at = self._get_tx_at()
            to_delete = []

            # Zbierz fale do usunięcia (oprócz aktualnie transmitowanej i bieżącej)
            for wid, _ in list(self._pending_delete):
                if wid != tx_at and wid != self._current_wid:
                    to_delete.append(wid)

            # Oczyść kolejkę z pozycji do usunięcia
            self._pending_delete = deque(
                (w, t) for w, t in self._pending_delete
                if w == tx_at or w == self._current_wid
            )

        # Usuń fale poza lockiem
        for wid in to_delete:
            try:
                self.worker.call('wave_delete', wid, timeout=0.5)
                with self._lock:
                    self._pending_ids.discard(wid)
            except Exception:
                pass

    def _create_wave_atomic(self, pulses):
        """Funkcja wykonywana w wątku worker, atomowo dodająca i tworząca falę."""
        def _inner(pi: pigpio.pi, pulses_):
            pi.wave_add_new()
            pi.wave_add_generic(pulses_)
            return pi.wave_create()
        return self.worker.call(_inner, pulses, timeout=2.0)

    def _build_wave(self, throttle: int) -> int:
        if not self.worker.connected:
            raise ConnectionError("pigpio disconnected")

        packet = self._make_packet(throttle, telemetry=0)

        pulses = []
        on_mask = 1 << self.gpio
        off_mask = on_mask

        h1, l1, h0, l0, gap_us = self._h1, self._l1, self._h0, self._l0, self._gap_us

        # MSB -> LSB
        for i in range(15, -1, -1):
            bit = (packet >> i) & 1
            if bit:
                pulses.append(pigpio.pulse(on_mask, 0, h1))
                pulses.append(pigpio.pulse(0, off_mask, l1))
            else:
                pulses.append(pigpio.pulse(on_mask, 0, h0))
                pulses.append(pigpio.pulse(0, off_mask, l0))
        # przerwa między ramkami (≈ 1 kHz)
        pulses.append(pigpio.pulse(0, off_mask, gap_us))

        # Sprawdź zasoby i ewentualnie posprzątaj stare fale
        try:
            max_cbs = self.worker.call('wave_get_max_cbs', timeout=1.0)
            cur_cbs = self.worker.call('wave_get_cbs', timeout=1.0)
            # Przybliż, ile "fal" zajmują CB (dzielnik ~25 jako heurystyka)
            max_waves = int(max_cbs) // 25 if isinstance(max_cbs, int) else 0
            current_waves = int(cur_cbs) // 25 if isinstance(cur_cbs, int) else 0
            if max_waves > 0 and current_waves > max_waves * 0.8:
                self._cleanup_old_waves()
        except Exception:
            pass

        wid = self._create_wave_atomic(pulses)
        if not isinstance(wid, int) or wid < 0:
            # Zamiast wave_clear() — sprzątnij nieaktywne fale i spróbuj ponownie
            self._emergency_cleanup()
            wid = self._create_wave_atomic(pulses)
            if not isinstance(wid, int) or wid < 0:
                raise RuntimeError(f"pigpio wave_create failed after cleanup: {wid}")
        return wid

    def _get_tx_at(self) -> int:
        try:
            return int(self.worker.call('wave_tx_at', timeout=1.0))
        except AttributeError:
            return -999
        except Exception:
            return -999

    def _process_pending_deletes(self):
        with self._tx_lock:
            now = time.time()
            to_delete = []
            with self._lock:
                while self._pending_delete and self._pending_delete[0][1] <= now:
                    wid, _ = self._pending_delete.popleft()
                    to_delete.append(wid)

            if not to_delete:
                return

            tx_at = self._get_tx_at()
            requeue = []
            for wid in to_delete:
                if wid == tx_at:
                    requeue.append(wid)
                else:
                    try:
                        self.worker.call('wave_delete', wid, timeout=1.0)
                    except Exception:
                        pass
                    finally:
                        with self._lock:
                            self._pending_ids.discard(wid)

            if requeue:
                with self._lock:
                    for wid in requeue:
                        if wid not in self._pending_ids:
                            self._pending_ids.add(wid)
                        self._pending_delete.append((wid, time.time() + DSHOT_WAVE_DELETE_DELAY_S))

    def set_throttle(self, throttle: int):
        throttle = max(0, min(DSHOT_MAX_THROTTLE, int(throttle)))

        with self._tx_lock:
            with self._lock:
                if throttle == self._last_throttle and self._current_wid is not None:
                    return
                self._req_counter += 1
                req_id = self._req_counter
                prev_wid = self._current_wid
                self._last_throttle = throttle

            new_wid = None
            try:
                new_wid = self._build_wave(throttle)

                with self._lock:
                    if req_id != self._req_counter or self._last_throttle != throttle:
                        try:
                            self.worker.submit('wave_delete', new_wid, retries=1)
                        except Exception:
                            pass
                        return

                    self._current_wid = new_wid

                # Wyślij falę (po ustawieniu _current_wid)
                try:
                    mode = getattr(pigpio, 'WAVE_MODE_REPEAT_SYNC', 3)
                    self.worker.call('wave_send_using_mode', new_wid, mode, timeout=1.0)
                except AttributeError:
                    self.worker.call('wave_send_repeat', new_wid, timeout=1.0)

                # Zaplanuj usunięcie poprzedniej i ogranicz rozmiar kolejki pending_delete
                with self._lock:
                    now = time.time()
                    # Staraj się usuwać tylko te, które są bezpieczne
                    while len(self._pending_delete) > self.MAX_PENDING_DELETES:
                        oldest_wid, safe_time = self._pending_delete[0]
                        tx_at = self._get_tx_at()
                        if (safe_time <= now) and (oldest_wid != tx_at) and (oldest_wid != self._current_wid):
                            self._pending_delete.popleft()
                            self._pending_ids.discard(oldest_wid)
                            try:
                                self.worker.call('wave_delete', oldest_wid, timeout=0.2)
                            except Exception:
                                pass
                        else:
                            break

                    if prev_wid is not None and prev_wid not in self._pending_ids:
                        self._pending_ids.add(prev_wid)
                        self._pending_delete.append((prev_wid, time.time() + DSHOT_WAVE_DELETE_DELAY_S))

            except Exception:
                with self._lock:
                    if self._current_wid == new_wid:
                        self._current_wid = prev_wid  # Rollback
                if new_wid is not None:
                    try:
                        self.worker.call('wave_delete', new_wid, timeout=1.0)
                    except Exception:
                        pass
                raise

        self._process_pending_deletes()

    def stop(self):
        with self._tx_lock:
            try:
                self.worker.call('wave_tx_stop', timeout=1.0)
            except Exception:
                pass

            with self._lock:
                wids = list(self._pending_ids)
                if self._current_wid is not None:
                    wids.append(self._current_wid)
                self._pending_delete.clear()
                self._pending_ids.clear()
                self._current_wid = None
                self._last_throttle = None

            for wid in wids:
                try:
                    self.worker.call('wave_delete', wid, timeout=1.0)
                except Exception:
                    pass
            try:
                self.worker.call('wave_clear', timeout=1.0)
            except Exception:
                pass


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

        # Logger
        self._logger = logging.getLogger('ESCTestStand')
        self._logger.setLevel(logging.INFO)
        if not self._logger.handlers:
            fh = logging.FileHandler('esc_test.log')
            fh.setLevel(logging.INFO)
            formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
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
        self.dshot_bit_us = 3  # 3 µs zalecane (aproksymacja DShot300)

        # Protokół i parametry PWM
        self._protocol = 'PWM490'  # 'PWM50' | 'PWM490' | 'DSHOT300' (UI: "DShot")
        self._pwm_frequency = PWM_FREQUENCY
        self._pwm_range = PWM_RANGE
        self._pu = 1_000_000 / self._pwm_frequency  # okres w µs dla PWM490

        # Stany bieżące
        self._current_pwm = 0.0
        self._target_pwm = 0.0
        self._current_dshot = 0.0
        self._target_dshot = 0.0

        # Czas (monotoniczny, odporny na zmiany zegara)
        self._end_time_mono = 0.0
        self._start_time_mono = 0.0
        self._time_left_at_pause_mono = 0.0

        self._finishing = False
        self._finishing_deadline_mono = 0.0
        self._auto_finish = False
        self._stopping_soft = False
        self._last_update_mono = time.monotonic()

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
        self._dshot_tx = DShotTransmitter(self._pw, self.gpio_pin, bit_time_us=self.dshot_bit_us)

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
    def _log_info(self, msg: str):
        self._logger.info(msg)

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
            self._log_info("EMERGENCY STOP due to pigpio disconnect")

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

                self._dshot_tx = DShotTransmitter(self._pw, self.gpio_pin, bit_time_us=self.dshot_bit_us)

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
                self._log_info("Reconnected to pigpio")
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
                self._notify_bg("Protokół: DShot (bit=3–4µs; wymaga pigpiod -s 1)", 'info')
                # WAŻNE: od razu start transmisji z 0
                try:
                    self._dshot_tx.set_bit_time_us(self.dshot_bit_us)
                    self._dshot_tx.set_throttle(0)
                except Exception as e:
                    self._log_error("DShot set_throttle(0) on set_protocol", e)
                self._log_info("Protocol changed to DSHOT")

            elif protocol == 'PWM50':
                try:
                    self._safe_pigpio_call('set_PWM_dutycycle', self.gpio_pin, 0, timeout=1.0)
                except Exception:
                    pass
                self._protocol = 'PWM50'
                self._notify_bg("Protokół: PWM 50Hz (servo pulses)", 'info')
                self._log_info("Protocol changed to PWM50")

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
                self._log_info("Protocol changed to PWM490")
            else:
                self._notify_bg("Nieobsługiwany protokół", 'negative')

        self._wake_event.set()

    def set_dshot_bit_time(self, bit_us: int):
        """Zmiana czasu bitu DShot w locie (gdy system DISARMED)."""
        if self._armed or self.is_running or self._finishing:
            self._notify_bg("Zmiana DShot Bit Time możliwa tylko, gdy DISARMED i TEST STOPPED", 'warning')
            return
        bit_us = int(bit_us)
        self.dshot_bit_us = bit_us
        try:
            self._dshot_tx.set_bit_time_us(bit_us)
        except Exception as e:
            self._log_error("Failed to set DShot bit time", e)
        self._notify_bg(f"DShot Bit Time ustawiony na {bit_us} µs", 'info')
        self._log_info(f"DShot bit time set to {bit_us}us")

    # ---------- Pętla sterująca ----------
    def _run_loop(self):
        while not self._stop_event.is_set():
            loop_start = time.monotonic()

            with self._lock:
                now = time.monotonic()
                dt = now - self._last_update_mono
                self._last_update_mono = now

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
                    timeout_reached = (now >= self._finishing_deadline_mono) if self._finishing_deadline_mono > 0 else False
                    if close_enough or timeout_reached:
                        self._finishing = False
                        self._auto_finish = False
                        self._stopping_soft = False
                        self._finishing_deadline_mono = 0.0
                        self._time_left_at_pause_mono = 0.0

                elif running and not paused:
                    if now >= self._end_time_mono:
                        self._running_event.clear()
                        self._finishing = True
                        self._auto_finish = True
                        self._stopping_soft = False
                        self._finishing_deadline_mono = now + FINISHING_MAX_S
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
                            elapsed = min(self.duration_seconds, max(0.0, now - self._start_time_mono))
                            self._elapsed_snapshot = elapsed
                            self._progress_snapshot = min(1.0, elapsed / self.duration_seconds)
                            self._time_left_snapshot = max(0.0, self._end_time_mono - now)

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

            # Wyślij sygnał (sprawdź protokół tuż przed IO, by uniknąć rzadkiego „starego” IO po zmianie)
            try:
                if not self._pw.connected:
                    raise ConnectionError("pigpio disconnected")
                current_proto = self._protocol
                if out[0] == 'DSHOT' and current_proto == 'DSHOT300':
                    self._dshot_tx.set_throttle(out[1])
                elif out[0] == 'SERVO' and current_proto == 'PWM50':
                    self._safe_pigpio_call('set_servo_pulsewidth', self.gpio_pin, int(out[1]), retries=3, timeout=1.0)
                elif out[0] == 'PWM' and current_proto == 'PWM490':
                    self._safe_pigpio_call('set_PWM_dutycycle', self.gpio_pin, out[1], retries=3, timeout=1.0)
            except Exception as e:
                self._handle_pigpio_error(e)

            # Tempo pętli
            elapsed = time.monotonic() - loop_start
            if self._protocol == 'DSHOT300':
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
            self._time_left_at_pause_mono = 0.0
            self._finishing = False
            self._auto_finish = False
            self._stopping_soft = False
            self._finishing_deadline_mono = 0.0
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
            self._last_update_mono = time.monotonic()

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
            self._notify_bg(f"Output error: {str(e)}", type_='negative')
            try:
                self.disarm_system()
            except Exception as ex:
                self._log_error("disarm_system after arm error", ex)
            return

        self._notify_bg("SYSTEM ARMED", type_='warning')
        self._log_info("System ARMED")
        self._wake_event.set()

    def disarm_system(self):
        with self._lock:
            self._pause_event.clear()
            self._running_event.clear()
            self._time_left_at_pause_mono = 0.0
            self._finishing = False
            self._auto_finish = False
            self._stopping_soft = False
            self._finishing_deadline_mono = 0.0
            self._armed = False
            # Reset stanów
            self._current_pwm = 0.0
            self._target_pwm = 0.0
            self._current_dshot = 0.0
            self._target_dshot = 0.0
            self._end_time_mono = 0.0
            self._start_time_mono = 0.0
            self._last_update_mono = time.monotonic()
            self._progress_snapshot = 0.0
            self._elapsed_snapshot = 0.0
            self._time_left_snapshot = float(self.duration_seconds)

        try:
            if self._protocol == 'DSHOT300':
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
            self._notify_bg(f"Output error: {str(e)}", type_='negative')

        self._notify_bg("SYSTEM DISARMED", type_='positive')
        self._log_info("System DISARMED")
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

            now = time.monotonic()
            self._running_event.set()
            self._pause_event.clear()
            self._finishing = False
            self._auto_finish = False
            self._stopping_soft = False
            self._finishing_deadline_mono = 0.0
            self._start_time_mono = now
            self._end_time_mono = now + self.duration_seconds
            self._time_left_at_pause_mono = 0.0
            self._progress_snapshot = 0.0
            self._elapsed_snapshot = 0.0
            self._time_left_snapshot = float(self.duration_seconds)
            self._last_update_mono = now

            if self._protocol == 'DSHOT300':
                self._target_dshot = random.uniform(self.dshot_min, self.dshot_max)
            else:
                self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)

        self._notify_bg("TEST STARTED", type_='positive')
        self._log_info("Test STARTED")
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
                self._finishing_deadline_mono = 0.0
                self._current_pwm = 0.0
                self._target_pwm = 0.0
                self._current_dshot = 0.0
                self._target_dshot = 0.0
                self._end_time_mono = 0.0
                self._time_left_at_pause_mono = 0.0
                self._last_update_mono = time.monotonic()
            else:
                if not finishing_now:
                    self._running_event.clear()
                    self._pause_event.clear()
                    self._stopping_soft = True
                    self._finishing = True
                    self._auto_finish = False
                    self._finishing_deadline_mono = time.monotonic() + FINISHING_MAX_S
                    if proto == 'DSHOT300':
                        self._target_dshot = float(max(DSHOT_MIN_THROTTLE, min(DSHOT_MAX_THROTTLE, self.dshot_idle)))
                    else:
                        self._target_pwm = float(self.idle_pwm)

        if hard:
            try:
                if proto == 'DSHOT300':
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
                self._log_info("Test STOPPED (HARD)")
            else:
                if running_now and not finishing_now:
                    self._notify_bg("TEST STOPPED (SMOOTH)", type_='info')
                    self._log_info("Test STOPPED (SMOOTH)")
                elif finishing_now:
                    self._notify_bg("Already stopping…", type_='warning')

        self._wake_event.set()

    def toggle_pause(self):
        with self._lock:
            if not self._running_event.is_set() or self._finishing:
                self._notify_bg("Nie można PAUSE/RESUME (brak RUN lub trwa FINISHING)", type_='warning')
                return

            now = time.monotonic()
            if self._pause_event.is_set():
                # RESUME
                self._pause_event.clear()
                self._end_time_mono = now + self._time_left_at_pause_mono
                self._start_time_mono = now - self._elapsed_snapshot
                self._last_update_mono = now
                msg = "TEST RESUMED"
                self._log_info("Test RESUMED")
            else:
                # PAUSE
                self._pause_event.set()
                self._time_left_at_pause_mono = max(0.0, self._end_time_mono - now)
                self._time_left_snapshot = self._time_left_at_pause_mono
                msg = "TEST PAUSED"
                self._log_info("Test PAUSED")

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
            return max(0.0, self._end_time_mono - time.monotonic())
        if self.is_paused:
            return self._time_left_at_pause_mono
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

        ui.label().classes('text-subtitle2 text-grey-7').bind_text_from(
            esc_test_stand, 'protocol', lambda p: f"Protocol: {p} (DShot bit={esc_test_stand.dshot_bit_us}µs)"
        )

        ui.label().classes('text-lg font-mono').bind_text_from(
            esc_test_stand, 'current_value'
        )

        time_label = ui.label().classes('text-lg')

        # Pasek postępu z nakładką (overlay)
        with ui.element('div').style('position:relative;width:100%'):
            ui.linear_progress(show_value=False) \
                .bind_value_from(esc_test_stand, 'progress') \
                .style('height:35px;border-radius:8px;') \
                .props('instant-feedback')
            progress_label = ui.label().classes('absolute-center text-black text-bold text-h6')

        def update_display():
            for msg, t in esc_test_stand.pop_notifications():
                ui.notify(msg, type=t)
            time_label.set_text(f"Time: {esc_test_stand.time_left_text}")
            elapsed = esc_test_stand.elapsed_time
            progress_label.set_text(
                f"{esc_test_stand.current_value} | Elapsed: {fmt_mmss(elapsed)}"
            )

        ui.timer(0.1, update_display)

def create_buttons_section():
    ui.separator().classes('q-my-md')

    with ui.row().classes('w-full justify-around items-center'):
        ui.button('ARM', on_click=esc_test_stand.arm_system, color='warning') \
            .props('icon=lock_open') \
            .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a)

        ui.button('DISARM', on_click=esc_test_stand.disarm_system, color='positive') \
            .props('icon=lock') \
            .bind_enabled_from(esc_test_stand, 'is_armed')

        ui.button('START', on_click=esc_test_stand.start_test, color='negative') \
            .props('icon=play_arrow') \
            .bind_enabled_from(esc_test_stand, 'can_start')

        pause_button = ui.button('PAUSE', on_click=esc_test_stand.toggle_pause, color='info') \
            .props('icon=pause') \
            .bind_enabled_from(esc_test_stand, 'is_running')

        ui.button('STOP', on_click=esc_test_stand.stop_test, color='negative') \
            .props('icon=stop') \
            .bind_enabled_from(esc_test_stand, 'is_running')

        ui.button('EMERGENCY STOP', on_click=lambda: esc_test_stand.stop_test(hard=True), color='negative') \
            .props('icon=bolt') \
            .bind_enabled_from(esc_test_stand, 'can_emergency_stop')

        ui.button('PWM 50Hz', on_click=lambda: esc_test_stand.set_protocol('PWM50'), color='primary') \
            .bind_enabled_from(esc_test_stand, 'pwm50_button_enabled')

        ui.button('PWM 490Hz', on_click=lambda: esc_test_stand.set_protocol('PWM490'), color='primary') \
            .bind_enabled_from(esc_test_stand, 'pwm490_button_enabled')

        ui.button('DShot', on_click=lambda: esc_test_stand.set_protocol('DSHOT300'), color='secondary') \
            .bind_enabled_from(esc_test_stand, 'dshot_button_enabled') \
            .tooltip('DShot via pigpio waves (bit 3–4 µs, pigpiod -s 1 required)')

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

        ui.number('TEST DURATION (s)', min=1, max=86400, step=1) \
            .bind_value(esc_test_stand, 'duration_seconds') \
            .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

        with ui.tabs() as tabs:
            pwm_tab = ui.tab('PWM Settings')
            dshot_tab = ui.tab('DShot Settings')

        with ui.tab_panels(tabs, value=pwm_tab):
            with ui.tab_panel(pwm_tab):
                with ui.grid(columns=2).classes('w-full gap-4'):
                    ui.number('IDLE PWM (µs)', min=MIN_PWM_LIMIT, max=MAX_PWM_LIMIT, step=1) \
                        .bind_value(esc_test_stand, 'idle_pwm') \
                        .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a)

                    ui.number('MIN PWM (µs)', min=MIN_PWM_LIMIT, max=MAX_PWM_LIMIT, step=10,
                              on_change=lambda e: ensure_pwm_min(e.value)) \
                        .bind_value(esc_test_stand, 'min_pwm') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('MAX PWM (µs)', min=MIN_PWM_LIMIT, max=MAX_PWM_LIMIT, step=10,
                              on_change=lambda e: ensure_pwm_max(e.value)) \
                        .bind_value(esc_test_stand, 'max_pwm') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('RAMP UP RATE (µs/s)', min=1, max=1_000_000, step=100) \
                        .bind_value(esc_test_stand, 'ramp_up_per_s') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('RAMP DOWN RATE (µs/s)', min=1, max=1_000_000, step=100) \
                        .bind_value(esc_test_stand, 'ramp_down_per_s') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

            with ui.tab_panel(dshot_tab):
                with ui.grid(columns=2).classes('w-full gap-4'):
                    ui.number('IDLE THROTTLE', min=DSHOT_MIN_THROTTLE, max=DSHOT_MAX_THROTTLE, step=1) \
                        .bind_value(esc_test_stand, 'dshot_idle') \
                        .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a)

                    ui.number('MIN THROTTLE', min=DSHOT_MIN_THROTTLE, max=DSHOT_MAX_THROTTLE, step=10,
                              on_change=lambda e: ensure_dshot_min(e.value)) \
                        .bind_value(esc_test_stand, 'dshot_min') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('MAX THROTTLE', min=DSHOT_MIN_THROTTLE, max=DSHOT_MAX_THROTTLE, step=10,
                              on_change=lambda e: ensure_dshot_max(e.value)) \
                        .bind_value(esc_test_stand, 'dshot_max') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('RAMP UP (units/s)', min=1, max=5000, step=50) \
                        .bind_value(esc_test_stand, 'dshot_ramp_up_per_s') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('RAMP DOWN (units/s)', min=1, max=5000, step=50) \
                        .bind_value(esc_test_stand, 'dshot_ramp_down_per_s') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.select(
                        options=[('3 µs (zalecane)', 3), ('4 µs (eksperymentalnie)', 4)],
                        label='DShot Bit Time'
                    ).bind_value(esc_test_stand, 'dshot_bit_us') \
                        .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a) \
                        .on('change', lambda e: esc_test_stand.set_dshot_bit_time(e.value))

@ui.page('/')
def main_page():
    if init_error:
        ui.label("INITIALIZATION ERROR").classes("text-h4 text-negative text-center w-full")
        ui.html(f"<p>pigpio error:</p><pre style='white-space:pre-wrap;overflow-wrap:anywhere'>{init_error}</pre>") \
            .classes("text-body1 text-center w-full")
        ui.label("Upewnij się, że pigpio działa z próbkowaniem 1µs: sudo pigpiod -s 1").classes("text-center w-full")
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
    favicon=None,  # ustaw plik .ico/.png jeśli chcesz własną ikonę
)
