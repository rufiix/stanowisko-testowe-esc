import time
import random
import threading
import pigpio
from nicegui import ui, app

GPIO_PIN_ESC = 17
APP_PORT = 8081
PWM_FREQUENCY = 490
PWM_RANGE = 255

class ESCTestStand:
    def __init__(self, gpio_pin):
        self.gpio_pin = gpio_pin
        self.idle_pwm = 1050
        self.duration_seconds = 60
        self.min_pwm = 1100
        self.max_pwm = 2000
        self.ramp_up_per_s = 500
        self.ramp_down_per_s = 800

        self._pwm_frequency = PWM_FREQUENCY
        self._pwm_range = PWM_RANGE
        self._pu = 1_000_000 / self._pwm_frequency

        self._current_pwm = 0.0
        self._target_pwm = 0.0
        self._end_time = 0.0
        self._time_left_at_pause = 0.0
        self._last_update_time = time.time()

        # Reentrant lock, potrzebny przy nested calls
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._running_event = threading.Event()
        self._armed = False

        # Wyzerowanie stanów po uruchomieniu
        self._pause_event.clear()
        self._running_event.clear()
        self._time_left_at_pause = 0.0

        # Inicjalizacja pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.pi.stop()
            raise IOError("Błąd pigpio! Uruchom 'sudo pigpiod'")

        try:
            self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(self.gpio_pin, self._pwm_frequency)
            self.pi.set_PWM_range(self.gpio_pin, self._pwm_range)
        except Exception as err:
            self.pi.stop()
            raise IOError(f"Błąd inicjalizacji pigpio: {err}")

        # Start wątku PWM
        self._pwm_thread = threading.Thread(target=self._run_pwm_loop, daemon=True)
        self._pwm_thread.start()

    def __enter__(self):
        return self

    def __exit__(self, et, ev, tb):
        self.cleanup()

    def _run_pwm_loop(self):
        while not self._stop_event.is_set():
            start = time.time()
            with self._lock:
                now = start
                dt = now - self._last_update_time
                self._last_update_time = now

                if self._running_event.is_set() and not self._pause_event.is_set():
                    # Rampowanie między aktualnym a docelowym PWM
                    if abs(self._current_pwm - self._target_pwm) < 2:
                        self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)
                    if self._current_pwm < self._target_pwm:
                        self._current_pwm = min(self._current_pwm + self.ramp_up_per_s * dt, self._target_pwm)
                    else:
                        self._current_pwm = max(self._current_pwm - self.ramp_down_per_s * dt, self._target_pwm)
                    # Koniec testu?
                    if now >= self._end_time:
                        self._running_event.clear()
                        self._pause_event.clear()
                elif self._armed:
                    # Uzbrojony, ale nie w teście → idle
                    self._current_pwm = float(self.idle_pwm)
                else:
                    # Rozbrojony → zero
                    self._current_pwm = 0.0

                # Oblicz duty cycle
                duty = int((self._current_pwm / self._pu) * self._pwm_range) if self._pu > 0 else 0
                duty = max(0, min(self._pwm_range, duty))

            # Ustaw PWM na pinie, retry przy błędzie
            for _ in range(3):
                try:
                    self.pi.set_PWM_dutycycle(self.gpio_pin, duty)
                    break
                except Exception:
                    time.sleep(0.02)

            elapsed = time.time() - start
            time.sleep(max(0, 0.02 - elapsed))

    def cleanup(self):
        self._stop_event.set()
        self._pwm_thread.join()
        if self.pi.connected:
            try:
                self.pi.set_PWM_dutycycle(self.gpio_pin, 0)
            except Exception:
                pass
            self.pi.stop()

    def arm_system(self):
        with self._lock:
            # Wyczyść wszystkie flagi i uzbrój
            self._pause_event.clear()
            self._running_event.clear()
            self._time_left_at_pause = 0.0
            self._armed = True
            # Od razu ustaw idle_pwm
            duty = int((self.idle_pwm / self._pu) * self._pwm_range)
            try:
                self.pi.set_PWM_dutycycle(self.gpio_pin, duty)
            except Exception:
                pass
            self._last_update_time = time.time()
        ui.notify("UZBROJONY", type='warning')

    def disarm_system(self):
        with self._lock:
            # Wyczyść flagi i rozbrój
            self._pause_event.clear()
            self._running_event.clear()
            self._time_left_at_pause = 0.0
            self._armed = False
        # Ustaw zero dutycycle
        try:
            self.pi.set_PWM_dutycycle(self.gpio_pin, 0)
        except Exception:
            pass
        ui.notify("ROZBROJONY", type='positive')

    def start_test(self):
        with self._lock:
            if not self._armed or self._running_event.is_set():
                return
            if self.min_pwm >= self.max_pwm:
                ui.notify("Min>Max PWM", type='negative')
                return
            now = time.time()
            self._running_event.set()
            self._pause_event.clear()
            self._end_time = now + self.duration_seconds
            self._last_update_time = now
            self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)
        ui.notify("START", type='positive')

    def stop_test(self, notify=True):
        with self._lock:
            if not self._running_event.is_set():
                return
            self._running_event.clear()
            self._pause_event.clear()
        if notify:
            ui.notify("STOP", type='info')

    def toggle_pause(self):
        with self._lock:
            if not self._running_event.is_set():
                return
            if self._pause_event.is_set():
                # Wznów
                self._pause_event.clear()
                self._end_time = time.time() + self._time_left_at_pause
                self._last_update_time = time.time()
                msg = "WZNOWIONO"
            else:
                # Pauza
                self._pause_event.set()
                self._time_left_at_pause = max(0.0, self._end_time - time.time())
                msg = "PAUZA"
        ui.notify(msg, type='info')

    def set_pwm_frequency(self, freq):
        if self._armed:
            ui.notify("Zmień tylko gdy ROZBROJONY", type='negative')
            return
        if freq not in (50, 490):
            ui.notify("Tylko 50Hz lub 490Hz", type='negative')
            return
        with self._lock:
            if freq == self._pwm_frequency:
                return
            self._pwm_frequency = freq
            self._pu = 1_000_000 / freq
            try:
                self.pi.set_PWM_frequency(self.gpio_pin, freq)
            except Exception as err:
                ui.notify(f"Błąd częstotliwości: {err}", type='negative')
                return
        ui.notify(f"PWM: {freq}Hz", type='info')

    @property
    def pwm_frequency(self):
        return self._pwm_frequency

    @property
    def is_armed(self):
        return self._armed

    @property
    def is_running(self):
        return self._running_event.is_set()

    @property
    def is_paused(self):
        return self._pause_event.is_set()

    @property
    def can_start(self):
        return self._armed and not self._running_event.is_set()

    @property
    def status_text(self):
        if not self._armed:
            return "ROZBROJONY"
        if self.is_running and self.is_paused:
            return "PAUZA"
        if self.is_running:
            return "URUCHOMIONY"
        return f"UZBROJONY ({self.idle_pwm}µs)"

    @property
    def time_left(self):
        if not self.is_running:
            return float(self.duration_seconds)
        if self.is_paused:
            return float(self._time_left_at_pause)
        return max(0.0, self._end_time - time.time())

    @property
    def elapsed_time(self):
        if not self.is_running:
            return 0.0
        return self.duration_seconds - self.time_left

    @property
    def progress(self):
        if not self.is_running or self.duration_seconds <= 0:
            return 0.0
        return self.elapsed_time / self.duration_seconds

    @property
    def current_pwm(self):
        return self._current_pwm


# inicjalizacja i obsługa błędu pigpio
try:
    s = ESCTestStand(GPIO_PIN_ESC)
    init_error = None
    app.on_shutdown(s.cleanup)
except Exception as err:
    s = None
    init_error = err


def controls_section():
    ui.label('TEST ESC').classes('text-h3 text-bold q-my-md text-center w-full')
    with ui.card().classes('w-full max-w-2xl mx-auto'):
        ui.label('STATUS').classes('text-h5')
        ui.label().classes('text-xl').bind_text_from(s, 'status_text')
        ui.label().classes('text-lg font-mono').bind_text_from(
            s, 'current_pwm', lambda p: f"PWM: {p:.2f}µs"
        )

        # natychmiastowa inicjalizacja etykiet czasu i progresu
        tl = ui.label().classes('text-lg')
        tl.set_text("CZAS: 00:00")
        pl = ui.label().classes('absolute-center text-black text-bold text-h6')
        with ui.linear_progress(show_value=False) \
               .bind_value_from(s, 'progress') \
               .style('height:35px;border-radius:8px;') \
               .props('instant-feedback'):
            pass

        def update_labels():
            if s.is_running and not s.is_paused:
                remaining = s.time_left
                if remaining <= 0:
                    s.stop_test()
                    return
                tl.set_text(f"CZAS: {int(remaining//60):02d}:{int(remaining%60):02d}")
                elapsed = s.elapsed_time
                pl.set_text(f"{s.current_pwm:.0f}µs | {int(elapsed//60):02d}:{int(elapsed%60):02d}")

        ui.timer(0.1, update_labels)


def buttons_section():
    ui.separator().classes('q-my-md')
    with ui.row().classes('w-full justify-around items-center'):
        ui.button('UZBROJ', on_click=s.arm_system, color='warning') \
          .props('icon=lock_open') \
          .bind_enabled_from(s, 'is_armed', backward=lambda a: not a)

        ui.button('ROZBROJ', on_click=s.disarm_system, color='positive') \
          .props('icon=lock') \
          .bind_enabled_from(s, 'is_armed')

        ui.button('START', on_click=s.start_test, color='negative') \
          .props('icon=play_arrow') \
          .bind_enabled_from(s, 'can_start')

        pb = ui.button('PAUZA', on_click=s.toggle_pause, color='info') \
               .props('icon=pause_circle') \
               .bind_enabled_from(s, 'is_running')

        ui.button('STOP', on_click=s.stop_test, color='negative') \
          .props('icon=stop') \
          .bind_enabled_from(s, 'is_running')

        # częstotliwość PWM
        ui.button('50Hz', on_click=lambda: s.set_pwm_frequency(50), color='primary') \
          .bind_enabled_from(s, 'is_armed', backward=lambda a: not a) \
          .bind_enabled_from(s, 'pwm_frequency', backward=lambda f: f != 50)

        ui.button('490Hz', on_click=lambda: s.set_pwm_frequency(490), color='primary') \
          .bind_enabled_from(s, 'is_armed', backward=lambda a: not a) \
          .bind_enabled_from(s, 'pwm_frequency', backward=lambda f: f != 490)

        def update_pause_btn():
            if s.is_paused:
                pb.set_text('WZNÓW')
                pb.set_icon('play_arrow')
            else:
                pb.set_text('PAUZA')
                pb.set_icon('pause_circle')

        ui.timer(0.1, update_pause_btn)


def settings_section():
    with ui.card().classes('w-full max-w-2xl mx-auto q-mt-md'):
        ui.label('USTAWIENIA').classes('text-h5')
        with ui.grid(columns=2).classes('w-full gap-4'):
            ui.number('IDLE PWM (µs)', step=1) \
              .bind_value(s, 'idle_pwm') \
              .bind_enabled_from(s, 'is_armed', backward=lambda a: not a)
            ui.number('CZAS (s)', min=1, max=86400, step=1) \
              .bind_value(s, 'duration_seconds') \
              .bind_enabled_from(s, 'is_running', backward=lambda r: not r)
            ui.number('MIN PWM (µs)', min=900, max=2500, step=10) \
              .bind_value(s, 'min_pwm') \
              .bind_enabled_from(s, 'is_running', backward=lambda r: not r)
            ui.number('MAX PWM (µs)', min=900, max=2500, step=10) \
              .bind_value(s, 'max_pwm') \
              .bind_enabled_from(s, 'is_running', backward=lambda r: not r)
            ui.number('RAMPA +', min=1, max=1_000_000, step=100) \
              .bind_value(s, 'ramp_up_per_s') \
              .bind_enabled_from(s, 'is_running', backward=lambda r: not r)
            ui.number('RAMPA -', min=1, max=1_000_000, step=100) \
              .bind_value(s, 'ramp_down_per_s') \
              .bind_enabled_from(s, 'is_running', backward=lambda r: not r)


@ui.page('/')
def main_page():
    if init_error:
        ui.label("BŁĄD").classes("text-h4 text-negative text-center w-full")
        ui.html(f"<p>Błąd pigpio:</p><pre>{init_error}</pre>") \
          .classes("text-body1 text-center w-full")
    else:
        controls_section()
        buttons_section()
        settings_section()


ui.run(title='TEST ESC', port=APP_PORT, host='0.0.0.0', show=False, reload=False)
