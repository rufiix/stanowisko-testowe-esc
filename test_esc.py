import time
import random
import threading
import logging
from typing import Optional

import pigpio
from nicegui import ui, app

# Minimalizacja logów NiceGUI (ukrywa np. "binding propagation")
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
DSHOT300_BIT_US = 1_000_000 / 300_000.0  # ~3.333 us
# W praktyce użyjemy zaokrągleń 2/1 us dla '1' i '0' (wymaga pigpiod -s 1)
DSHOT1_HIGH_US = 2
DSHOT1_LOW_US = 1
DSHOT0_HIGH_US = 1
DSHOT0_LOW_US = 2
DSHOT_MIN_THROTTLE = 48     # wg spec: <48 to komendy, >=48 to gaz
DSHOT_MAX_THROTTLE = 2047
DSHOT_DEFAULT_IDLE = 48

# -------------------- LOGIKA --------------------
class ESCTestStand:
    def __init__(self, gpio_pin: int):
        self.gpio_pin = gpio_pin

        # Ustawienia testu
        self.duration_seconds = DEFAULT_DURATION

        # PWM (analog)
        self.idle_pwm = DEFAULT_IDLE_PWM
        self.min_pwm = 1100
        self.max_pwm = 2000
        self.ramp_up_per_s = 500
        self.ramp_down_per_s = 800

        # DShot (cyfrowy)
        self.dshot_enabled = False
        self.dshot_idle = DSHOT_DEFAULT_IDLE
        self.dshot_min = 300
        self.dshot_max = 1200
        self.dshot_ramp_up_per_s = 500
        self.dshot_ramp_down_per_s = 800

        # Protokół i parametry PWM
        self._protocol = 'PWM490'  # 'PWM50' | 'PWM490' | 'DSHOT300'
        self._pwm_frequency = PWM_FREQUENCY
        self._pwm_range = PWM_RANGE
        self._pu = 1_000_000 / self._pwm_frequency  # okres w us (dla przeliczeń duty)

        # Stany bieżące
        self._current_pwm = 0.0
        self._target_pwm = 0.0
        self._current_dshot = 0.0
        self._target_dshot = 0.0
        self._end_time = 0.0
        self._time_left_at_pause = 0.0
        self._last_update_time = time.time()

        # Synchronizacja
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._running_event = threading.Event()
        self._armed = False

        # pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.pi.stop()
            raise IOError("pigpio daemon not running! Uruchom: sudo pigpiod -s 1")

        try:
            self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(self.gpio_pin, self._pwm_frequency)
            self.pi.set_PWM_range(self.gpio_pin, self._pwm_range)
        except Exception as err:
            self.pi.stop()
            raise IOError(f"pigpio initialization error: {err}")

        # Wątek sterujący
        self._pwm_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._pwm_thread.start()

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
            self.pi.wave_tx_stop()
            self.pi.wave_clear()
        except Exception:
            pass
        if self.pi.connected:
            try:
                self.pi.set_PWM_dutycycle(self.gpio_pin, 0)
                self.pi.stop()
            except Exception:
                pass

    # ---------- Protokół ----------
    @property
    def protocol(self) -> str:
        return self._protocol

    def set_protocol(self, protocol: str):
        """Ustaw protokół: 'PWM50' | 'PWM490' | 'DSHOT300' (tylko gdy DISARMED i STOPPED)."""
        if self._armed or self.is_running:
            ui.notify("Zmiana protokołu możliwa tylko, gdy DISARMED i TEST STOPPED", type='negative')
            return

        protocol = protocol.upper()
        with self._lock:
            if protocol == self._protocol:
                return

            if protocol == 'DSHOT300':
                self.dshot_enabled = True
                self._protocol = 'DSHOT300'
                ui.notify("Protokół: DShot300", type='info')
            elif protocol in ('PWM50', 'PWM490'):
                # Przełącz na PWM
                freq = 50 if protocol == 'PWM50' else 490
                try:
                    self.pi.set_PWM_frequency(self.gpio_pin, freq)
                    self.pi.set_PWM_range(self.gpio_pin, self._pwm_range)
                except Exception as err:
                    ui.notify(f"Frequency error: {err}", type='negative')
                    return

                self._pwm_frequency = freq
                self._pu = 1_000_000 / freq
                self.dshot_enabled = False
                self._protocol = protocol
                ui.notify(f"Protokół: PWM {freq}Hz", type='info')
            else:
                ui.notify("Nieobsługiwany protokół", type='negative')

    # ---------- DShot ----------
    @staticmethod
    def _dshot_packet(throttle: int, telemetry: int = 0) -> int:
        """Buduje 16-bitowy pakiet DShot: 11b throttle, 1b telemetry, 4b CRC."""
        t = max(0, min(2047, int(throttle)))
        payload = ((t << 1) | (telemetry & 0x1)) & 0x0FFF  # 12 bit
        csum = 0
        csum_data = payload
        for i in range(3):
            csum ^= (csum_data >> (i * 4)) & 0xF
        csum &= 0xF
        return (payload << 4) | csum

    def _send_dshot_frame(self, throttle: int):
        """Wysyła pojedynczą ramkę DShot300 za pomocą pigpio wave."""
        packet = self._dshot_packet(throttle, telemetry=0)  # bez telemetrii

        # Budujemy przebieg: MSB -> LSB
        pulses = []
        on_mask = 1 << self.gpio_pin
        off_mask = on_mask

        for i in range(15, -1, -1):
            bit = (packet >> i) & 1
            if bit:
                # '1' => ~0.75T high, reszta low
                pulses.append(pigpio.pulse(on_mask, 0, DSHOT1_HIGH_US))
                pulses.append(pigpio.pulse(0, off_mask, DSHOT1_LOW_US))
            else:
                # '0' => ~0.375T high, reszta low
                pulses.append(pigpio.pulse(on_mask, 0, DSHOT0_HIGH_US))
                pulses.append(pigpio.pulse(0, off_mask, DSHOT0_LOW_US))

        # Ramka przerwy (niewielka przerwa przed kolejną ramką)
        pulses.append(pigpio.pulse(0, off_mask, 5))  # ~5us przerwy

        try:
            self.pi.wave_add_new()
            self.pi.wave_add_generic(pulses)
            wid = self.pi.wave_create()
            if wid >= 0:
                self.pi.wave_send_once(wid)
                while self.pi.wave_tx_busy():
                    time.sleep(0.0001)
                self.pi.wave_delete(wid)
        except Exception as e:
            raise e

    # ---------- Pętla sterująca ----------
    def _run_loop(self):
        while not self._stop_event.is_set():
            loop_start = time.time()

            now = time.time()
            dt = now - self._last_update_time
            self._last_update_time = now

            with self._lock:
                if self._running_event.is_set() and not self._pause_event.is_set():
                    if self.dshot_enabled:
                        # Ramping DShot (wartości całkowite)
                        if abs(self._current_dshot - self._target_dshot) < 2:
                            self._target_dshot = random.uniform(self.dshot_min, self.dshot_max)
                        if self._current_dshot < self._target_dshot:
                            self._current_dshot = min(
                                self._current_dshot + self.dshot_ramp_up_per_s * dt,
                                self._target_dshot
                            )
                        else:
                            self._current_dshot = max(
                                self._current_dshot - self.dshot_ramp_down_per_s * dt,
                                self._target_dshot
                            )
                    else:
                        # Ramping PWM (µs)
                        if abs(self._current_pwm - self._target_pwm) < 2:
                            self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)
                        if self._current_pwm < self._target_pwm:
                            self._current_pwm = min(
                                self._current_pwm + self.ramp_up_per_s * dt,
                                self._target_pwm
                            )
                        else:
                            self._current_pwm = max(
                                self._current_pwm - self.ramp_down_per_s * dt,
                                self._target_pwm
                            )

                    # Koniec testu
                    if now >= self._end_time:
                        self._running_event.clear()
                        self._pause_event.clear()
                elif self._armed:
                    # Uzbrojony, ale test nie działa
                    if self.dshot_enabled:
                        self._current_dshot = float(self.dshot_idle)
                    else:
                        self._current_pwm = float(self.idle_pwm)
                else:
                    # Rozbrojony
                    self._current_pwm = 0.0
                    self._current_dshot = 0.0

            # Wyślij sygnał po wyjściu z sekcji krytycznej
            if self.dshot_enabled:
                try:
                    throttle = int(max(0, min(DSHOT_MAX_THROTTLE, round(self._current_dshot))))
                    self._send_dshot_frame(throttle)
                except Exception as e:
                    print(f"DShot error: {str(e)}")
                    if self._armed:
                        ui.notify("DShot ERROR! Disarming system", type='negative')
                        self.disarm_system()
            else:
                # PWM
                duty = int((self._current_pwm / self._pu) * self._pwm_range)
                duty = max(0, min(self._pwm_range, duty))
                success = False
                for _ in range(3):
                    try:
                        self.pi.set_PWM_dutycycle(self.gpio_pin, duty)
                        success = True
                        break
                    except Exception as e:
                        print(f"PWM error: {str(e)}")
                        time.sleep(0.02)
                if not success and self._armed:
                    ui.notify("PWM ERROR! Disarming system", type='negative')
                    self.disarm_system()

            # Tempo pętli (DShot szybszy)
            elapsed = time.time() - loop_start
            if self.dshot_enabled:
                time.sleep(max(0, 0.001 - elapsed))  # ~1 kHz
            else:
                time.sleep(max(0, 0.02 - elapsed))   # ~50 Hz

    # ---------- Sterowanie testem ----------
    def arm_system(self):
        with self._lock:
            self._pause_event.clear()
            self._running_event.clear()
            self._time_left_at_pause = 0.0
            self._armed = True
            if self.dshot_enabled:
                self._current_dshot = float(self.dshot_idle)
                self._target_dshot = float(self.dshot_idle)
            else:
                self._current_pwm = float(self.idle_pwm)
                self._target_pwm = float(self.idle_pwm)
            self._last_update_time = time.time()

        try:
            if not self.dshot_enabled:
                duty = int((self.idle_pwm / self._pu) * self._pwm_range)
                self.pi.set_PWM_dutycycle(self.gpio_pin, duty)
            else:
                # wyślij parę ramek IDLE, by ESC "złapał" sygnał
                for _ in range(3):
                    self._send_dshot_frame(int(self.dshot_idle))
        except Exception as e:
            ui.notify(f"Output error: {str(e)}", type='negative')
            self.disarm_system()
            return

        ui.notify("SYSTEM ARMED", type='warning')

    def disarm_system(self):
        with self._lock:
            self._pause_event.clear()
            self._running_event.clear()
            self._time_left_at_pause = 0.0
            self._armed = False

        try:
            if self.dshot_enabled:
                # kilka ramek 0 żeby wyhamować
                for _ in range(3):
                    self._send_dshot_frame(0)
            self.pi.set_PWM_dutycycle(self.gpio_pin, 0)
        except Exception as e:
            ui.notify(f"Output error: {str(e)}", type='negative')

        ui.notify("SYSTEM DISARMED", type='positive')

    def start_test(self):
        with self._lock:
            if not self._armed or self._running_event.is_set():
                return

            if self.dshot_enabled:
                if self.dshot_min >= self.dshot_max:
                    ui.notify("DShot: MIN musi być < MAX", type='negative')
                    return
            else:
                if self.min_pwm >= self.max_pwm:
                    ui.notify("PWM: MIN musi być < MAX", type='negative')
                    return

            now = time.time()
            self._running_event.set()
            self._pause_event.clear()
            self._end_time = now + self.duration_seconds
            self._last_update_time = now

            if self.dshot_enabled:
                self._target_dshot = random.uniform(self.dshot_min, self.dshot_max)
            else:
                self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)

        ui.notify("TEST STARTED", type='positive')

    def stop_test(self, notify: bool = True):
        with self._lock:
            if not self._running_event.is_set():
                return
            self._running_event.clear()
            self._pause_event.clear()
        if notify:
            ui.notify("TEST STOPPED", type='info')

    def toggle_pause(self):
        with self._lock:
            if not self._running_event.is_set():
                return
            if self._pause_event.is_set():
                self._pause_event.clear()
                self._end_time = time.time() + self._time_left_at_pause
                self._last_update_time = time.time()
                msg = "TEST RESUMED"
            else:
                self._pause_event.set()
                self._time_left_at_pause = max(0.0, self._end_time - time.time())
                msg = "TEST PAUSED"
        ui.notify(msg, type='info')

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
        if self.is_running:
            return f"RUNNING | Time left: {int(self.time_left)}s"
        if self.dshot_enabled:
            return f"ARMED (IDLE: {int(self.dshot_idle)} [DShot])"
        return f"ARMED (IDLE: {self.idle_pwm}µs)"

    @property
    def time_left(self) -> float:
        if not self.is_running:
            return float(self.duration_seconds)
        if self.is_paused:
            return self._time_left_at_pause
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
        return self.elapsed_time / self.duration_seconds

    @property
    def current_value(self) -> str:
        if self.dshot_enabled:
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

        # Aktualna wartość (PWM lub DShot)
        ui.label().classes('text-lg font-mono').bind_text_from(
            esc_test_stand, 'current_value'
        )

        # Czas i progres
        time_label = ui.label().classes('text-lg')
        progress_label = ui.label().classes('absolute-center text-black text-bold text-h6')

        with ui.linear_progress(show_value=False) \
                .bind_value_from(esc_test_stand, 'progress') \
                .style('height:35px;border-radius:8px;') \
                .props('instant-feedback'):
            pass

        def update_display():
            if esc_test_stand.is_running and not esc_test_stand.is_paused:
                if esc_test_stand.time_left <= 0:
                    esc_test_stand.stop_test()
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

        ui.button('STOP', on_click=esc_test_stand.stop_test, color='negative') \
            .props('icon=stop') \
            .bind_enabled_from(esc_test_stand, 'is_running')

        # Wybór protokołu (tylko gdy DISARMED)
        ui.button('PWM 50Hz', on_click=lambda: esc_test_stand.set_protocol('PWM50'), color='primary') \
            .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a) \
            .bind_enabled_from(esc_test_stand, 'protocol', backward=lambda p: p != 'PWM50')

        ui.button('PWM 490Hz', on_click=lambda: esc_test_stand.set_protocol('PWM490'), color='primary') \
            .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a) \
            .bind_enabled_from(esc_test_stand, 'protocol', backward=lambda p: p != 'PWM490')

        ui.button('DShot300', on_click=lambda: esc_test_stand.set_protocol('DSHOT300'), color='secondary') \
            .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a) \
            .bind_enabled_from(esc_test_stand, 'protocol', backward=lambda p: p != 'DSHOT300')

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

                    ui.number('TEST DURATION (s)',
                              min=1,
                              max=86400,
                              step=1) \
                        .bind_value(esc_test_stand, 'duration_seconds') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('MIN PWM (µs)',
                              min=MIN_PWM_LIMIT,
                              max=MAX_PWM_LIMIT,
                              step=10) \
                        .bind_value(esc_test_stand, 'min_pwm') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('MAX PWM (µs)',
                              min=MIN_PWM_LIMIT,
                              max=MAX_PWM_LIMIT,
                              step=10) \
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
                              min=DSHOT_MIN_THROTTLE,
                              max=DSHOT_MAX_THROTTLE,
                              step=1) \
                        .bind_value(esc_test_stand, 'dshot_idle') \
                        .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a)

                    ui.number('TEST DURATION (s)',
                              min=1,
                              max=86400,
                              step=1) \
                        .bind_value(esc_test_stand, 'duration_seconds') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('MIN THROTTLE',
                              min=DSHOT_MIN_THROTTLE,
                              max=DSHOT_MAX_THROTTLE,
                              step=10) \
                        .bind_value(esc_test_stand, 'dshot_min') \
                        .bind_enabled_from(esc_test_stand, 'is_running', backward=lambda r: not r)

                    ui.number('MAX THROTTLE',
                              min=DSHOT_MIN_THROTTLE,
                              max=DSHOT_MAX_THROTTLE,
                              step=10) \
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
