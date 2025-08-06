import time
import random
import threading

import pigpio
from nicegui import ui, app

GPIO_PIN_ESC = 17
APP_PORT = 8081
PWM_FREQUENCY = 490     # częstotliwość w Hz
PWM_RANGE = 255         # zakres duty cycle od 0 do 255


class ESCTestStand:
    def __init__(self, gpio_pin):
        self.gpio_pin = gpio_pin

        # parametry testu (wskaźniki w µs)
        self.idle_pwm = 1050
        self.duration_seconds = 60
        self.min_pwm = 1100
        self.max_pwm = 2000
        self.ramp_up_per_s = 500
        self.ramp_down_per_s = 800

        # stan wewnętrzny
        self._current_pwm = 0.0
        self._target_pwm = 0.0
        self._end_time = 0.0
        self._time_left_at_pause = 0.0
        self._last_update_time = time.time()

        # synchronizacja i flagi
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._running_event = threading.Event()
        self._armed = False

        # połączenie z pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise IOError("Nie można połączyć się z demonem pigpio. Uruchom 'sudo pigpiod'.")
        self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)

        # konfiguracja PWM: 490Hz, 8-bit (0–255)
        self.pi.set_PWM_frequency(self.gpio_pin, PWM_FREQUENCY)
        self.pi.set_PWM_range(self.gpio_pin, PWM_RANGE)

        # start pętli PWM
        self._pwm_thread = threading.Thread(target=self._run_pwm_loop, daemon=True)
        self._pwm_thread.start()
        print("System uruchomiony w trybie ROZBROJONY (bezpieczny).")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()

    def _run_pwm_loop(self):
        period_us = 1_000_000 / PWM_FREQUENCY
        while not self._stop_event.is_set():
            with self._lock:
                now = time.time()
                dt = now - self._last_update_time
                self._last_update_time = now

                if self._running_event.is_set() and not self._pause_event.is_set():
                    # losowy cel i rampowanie
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
                elif self._armed:
                    # przy uzbrojeniu utrzymuj idle
                    self._current_pwm = float(self.idle_pwm)
                else:
                    self._current_pwm = 0.0

                # przelicz na 0–255
                fraction = self._current_pwm / period_us
                duty = max(0, min(PWM_RANGE, int(fraction * PWM_RANGE)))

            # retry przy ustawianiu PWM
            for attempt in range(3):
                try:
                    self.pi.set_PWM_dutycycle(self.gpio_pin, duty)
                    break
                except Exception as e:
                    time.sleep(0.02)
                    if attempt == 2:
                        print(f"Błąd przy ustawianiu dutycycle: {e}")

            time.sleep(0.02)

        print("Pętla PWM zakończona.")

    def cleanup(self):
        print("Rozpoczynanie procedury zamykania...")
        self._stop_event.set()
        self._pwm_thread.join()
        print("Zatrzymywanie i sprzątanie zasobów ESC...")
        if self.pi.connected:
            try:
                self.pi.set_PWM_dutycycle(self.gpio_pin, 0)
            except Exception:
                pass
            self.pi.stop()
        print("Zasoby ESC zwolnione.")

    def arm_system(self):
        with self._lock:
            if not self._armed:
                self._armed = True
                ui.notify("System UZBROJONY.", type='warning')

    def disarm_system(self):
        with self._lock:
            if self._running_event.is_set():
                self.stop_test()
            if self._armed:
                self._armed = False
                ui.notify("System ROZBROJONY.", type='positive')

    def start_test(self):
        with self._lock:
            if not self._armed or self._running_event.is_set():
                return
            if self.min_pwm >= self.max_pwm:
                ui.notify("Min PWM musi być mniejsze niż Max PWM.", type='negative')
                return
            now = time.time()
            self._running_event.set()
            self._pause_event.clear()
            self._end_time = now + self.duration_seconds
            self._last_update_time = now
            self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)
            ui.notify("Test ESC rozpoczęty!", type='positive')

    def stop_test(self):
        with self._lock:
            if not self._running_event.is_set():
                return
            self._running_event.clear()
            self._pause_event.clear()
            ui.notify("Test zatrzymany. Powrót do trybu IDLE.")

    def toggle_pause(self):
        with self._lock:
            if not self._running_event.is_set():
                return
            if self._pause_event.is_set():
                self._pause_event.clear()
                self._end_time = time.time() + self._time_left_at_pause
                self._last_update_time = time.time()
                ui.notify("Test WZNOWIONY.", type='info')
            else:
                self._pause_event.set()
                self._time_left_at_pause = max(0.0, self._end_time - time.time())
                ui.notify("Test ZAPAUZOWANY.", type='info')

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
            return "ROZBROJONY (BEZPIECZNY)"
        if self.is_running and self.is_paused:
            return "SPAUZOWANY (w trybie IDLE)"
        if self.is_running:
            return "Uruchomiony"
        return f"UZBROJONY / IDLE ({self.idle_pwm} µs)"

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


# inicjalizacja
try:
    stand = ESCTestStand(GPIO_PIN_ESC)
    pigpio_error = None
except Exception as e:
    stand = None
    pigpio_error = e


# budowa UI
def create_status_card():
    ui.label('Stanowisko Testowe ESC')\
      .classes('text-h3 text-bold q-my-md text-center w-full')
    with ui.card().classes('w-full max-w-2xl mx-auto'):
        ui.label('Status na żywo').classes('text-h5')
        ui.label().classes('text-xl').bind_text_from(stand, 'status_text')
        ui.label().classes('text-lg font-mono')\
          .bind_text_from(stand, '_current_pwm', lambda p: f"Aktualne PWM: {p:.2f} µs")

        time_left_label = ui.label().classes('text-lg')
        with ui.linear_progress(show_value=False)\
                .bind_value_from(stand, 'progress')\
                .style('height: 35px; border-radius: 8px;')\
                .props('instant-feedback'):
            progress_bar_label = ui.label()\
                                  .classes('absolute-center text-black text-bold text-h6')

        def update_ui():
            remaining = stand.time_left
            time_left_label.set_text(f"Czas do końca: {int(remaining//60):02d}:{int(remaining%60):02d}")
            elapsed = stand.elapsed_time
            progress_bar_label.set_text(
                f"{stand._current_pwm:.0f} µs | Czas: {int(elapsed//60):02d}:{int(elapsed%60):02d}"
            )
            if remaining <= 0 and stand.is_running and not stand.is_paused:
                stand.stop_test()

        ui.timer(0.1, update_ui)


def create_control_card():
    ui.separator().classes('q-my-md')
    with ui.row().classes('w-full justify-around items-center'):
        with ui.column():
            ui.button('UZBRÓJ', on_click=stand.arm_system, color='warning')\
              .props('icon=lock_open')\
              .bind_enabled_from(stand, 'is_armed', backward=lambda x: not x)
            ui.button('ROZBRÓJ', on_click=stand.disarm_system, color='positive')\
              .props('icon=lock')\
              .bind_enabled_from(stand, 'is_armed')
        with ui.column():
            ui.button('Start Test', on_click=stand.start_test, color='negative')\
              .props('icon=play_arrow')\
              .bind_enabled_from(stand, 'can_start')
            ui.button('Pauza/Wznów', on_click=stand.toggle_pause)\
              .bind_enabled_from(stand, 'is_running')
            ui.button('Stop Test', on_click=stand.stop_test)\
              .bind_enabled_from(stand, 'is_running')


def create_settings_card():
    with ui.card().classes('w-full max-w-2xl mx-auto q-mt-md'):
        ui.label('Ustawienia').classes('text-h5')
        with ui.grid(columns=2).classes('w-full gap-4'):
            ui.number(label='IDLE PWM (µs)', step=1)\
              .bind_value(stand, 'idle_pwm')\
              .bind_enabled_from(stand, 'is_armed', backward=lambda x: not x)
            ui.number(label='Czas trwania (s)', min=1, max=86400, step=1)\
              .bind_value(stand, 'duration_seconds')\
              .bind_enabled_from(stand, 'is_running', backward=lambda x: not x)
            ui.number(label='Min. PWM w teście (µs)', min=900, max=2500, step=10)\
              .bind_value(stand, 'min_pwm')\
              .bind_enabled_from(stand, 'is_running', backward=lambda x: not x)
            ui.number(label='Max. PWM w teście (µs)', min=900, max=2500, step=10)\
              .bind_value(stand, 'max_pwm')\
              .bind_enabled_from(stand, 'is_running', backward=lambda x: not x)
            ui.number(label='Rampa wzrostu (PWM/s)', min=1, max=1_000_000, step=100)\
              .bind_value(stand, 'ramp_up_per_s')\
              .bind_enabled_from(stand, 'is_running', backward=lambda x: not x)
            ui.number(label='Rampa spadku (PWM/s)', min=1, max=1_000_000, step=100)\
              .bind_value(stand, 'ramp_down_per_s')\
              .bind_enabled_from(stand, 'is_running', backward=lambda x: not x)


@ui.page('/')
def main_page():
    if pigpio_error:
        ui.label("BŁĄD KRYTYCZNY")\
          .classes("text-h4 text-negative text-center w-full")
        ui.html(
            "<p>Nie udało się połączyć z demonem <b>pigpio</b>:</p>"
            f"<pre>{pigpio_error}</pre>"
        ).classes("text-body1 text-center w-full")
        return

    create_status_card()
    create_control_card()
    create_settings_card()


if stand:
    app.on_shutdown(stand.cleanup)

ui.run(title='Stanowisko Testowe ESC', port=APP_PORT, host='0.0.0.0', show=False, reload=False)
