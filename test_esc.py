import time
import random
import threading
import pigpio
from nicegui import ui, app

# --- Konfiguracja Aplikacji ---
GPIO_PIN_ESC = 17
APP_PORT = 8081

class ESCTestStand:
    def __init__(self, gpio_pin):
        self.gpio_pin = gpio_pin

        # Flagi stanu
        self._is_armed = False
        self._is_running = False
        self._is_paused = False
        self._stop_thread = False

        # Parametry testu
        self.idle_pwm = 1050
        self.duration_seconds = 60
        self.min_pwm = 1100
        self.max_pwm = 2000
        self.ramp_up_per_s = 500
        self.ramp_down_per_s = 800

        # Zmienne wewnętrzne
        self._current_pwm = 0.0
        self._target_pwm = 0.0
        self._end_time = 0.0
        self._last_update_time = time.time()
        self._time_left_at_pause = 0.0

        # Lock do synchronizacji wątków
        self._lock = threading.Lock()

        # Inicjalizacja pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise IOError("Nie można połączyć się z demonem pigpio. Uruchom 'sudo pigpiod'.")
        self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)

        # Uruchomienie pętli PWM w tle
        self._pwm_thread = threading.Thread(target=self._run_pwm_loop, daemon=True)
        self._pwm_thread.start()
        print("System uruchomiony w trybie ROZBROJONY (bezpieczny).")

    def _run_pwm_loop(self):
        """Pętla sterująca sygnałem PWM."""
        while not self._stop_thread:
            with self._lock:
                now = time.time()

                if self._is_running and not self._is_paused:
                    dt = now - self._last_update_time
                    self._last_update_time = now

                    # Jeżeli osiągnięto target, wybierz nowy
                    if abs(self._current_pwm - self._target_pwm) < 2:
                        self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)

                    # Rampowanie w górę / w dół
                    if self._current_pwm < self._target_pwm:
                        self._current_pwm = min(self._current_pwm + self.ramp_up_per_s * dt,
                                                self._target_pwm)
                    else:
                        self._current_pwm = max(self._current_pwm - self.ramp_down_per_s * dt,
                                                self._target_pwm)

                elif self._is_armed:
                    # Tryb IDLE
                    self._current_pwm = float(self.idle_pwm)
                else:
                    # Rozbrojony
                    self._current_pwm = 0.0

                pwm_to_set = int(self._current_pwm)

            try:
                self.pi.set_servo_pulsewidth(self.gpio_pin, pwm_to_set)
            except Exception as e:
                print(f"Błąd przy ustawianiu PWM: {e}")

            time.sleep(0.02)

        print("Pętla PWM zakończona.")

    def cleanup(self):
        print("Rozpoczynanie procedury zamykania...")
        self._stop_thread = True
        self._pwm_thread.join()
        print("Zatrzymywanie i sprzątanie zasobów ESC...")
        if self.pi.connected:
            self.pi.set_servo_pulsewidth(self.gpio_pin, 0)
            self.pi.stop()
        print("Zasoby ESC zwolnione.")

    def arm_system(self):
        with self._lock:
            if not self._is_armed:
                self._is_armed = True
                ui.notify("System UZBROJONY.", type='warning')

    def disarm_system(self):
        with self._lock:
            if self._is_running:
                self.stop_test()
            if self._is_armed:
                self._is_armed = False
                ui.notify("System ROZBROJONY.", type='positive')

    def _reset_test_state(self):
        self._is_running = False
        self._is_paused = False
        self._end_time = 0.0
        self._time_left_at_pause = 0.0

    def start_test(self):
        with self._lock:
            if not self._is_armed or self._is_running:
                return
            self._reset_test_state()
            now = time.time()
            self._end_time = now + self.duration_seconds
            self._last_update_time = now
            self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)
            self._is_running = True
            ui.notify("Test ESC rozpoczęty!", type='positive')

    def stop_test(self):
        with self._lock:
            if not self._is_running:
                return
            self._reset_test_state()
            ui.notify("Test zatrzymany. Powrót do trybu IDLE.")

    def toggle_pause(self):
        with self._lock:
            if not self._is_running:
                return
            self._is_paused = not self._is_paused
            if self._is_paused:
                self._time_left_at_pause = max(0.0, self._end_time - time.time())
                ui.notify("Test ZAPAUZOWANY.", type='info')
            else:
                self._end_time = time.time() + self._time_left_at_pause
                self._last_update_time = time.time()
                ui.notify("Test WZNOWIONY.", type='info')

    # Właściwości do UI
    @property
    def is_armed(self):
        return self._is_armed

    @property
    def is_running(self):
        return self._is_running

    @property
    def is_paused(self):
        return self._is_paused

    @property
    def can_start(self):
        return self._is_armed and not self._is_running

    @property
    def status_text(self):
        if not self._is_armed:
            return "ROZBROJONY (BEZPIECZNY)"
        if self._is_running and self._is_paused:
            return "SPAUZOWANY (w trybie IDLE)"
        if self._is_running:
            return "Uruchomiony"
        return f"UZBROJONY / IDLE ({self.idle_pwm} µs)"

    @property
    def time_left(self):
        if not self._is_running:
            return float(self.duration_seconds)
        if self._is_paused:
            return float(self._time_left_at_pause)
        return max(0.0, self._end_time - time.time())

    @property
    def elapsed_time(self):
        if not self._is_running:
            return 0.0
        return self.duration_seconds - self.time_left

    @property
    def progress(self):
        if not self._is_running or self.duration_seconds <= 0:
            return 0.0
        return (self.elapsed_time) / self.duration_seconds


# Inicjalizacja instancji
try:
    stand = ESCTestStand(gpio_pin=GPIO_PIN_ESC)
    pigpio_error = None
except Exception as e:
    stand = None
    pigpio_error = e

# --- Definicja UI ---
@ui.page('/')
def main_page():
    if pigpio_error:
        ui.label("BŁĄD KRYTYCZNY").classes("text-h4 text-negative text-center w-full")
        ui.html(f"<p>Nie udało się połączyć z demonem <b>pigpio</b>:", 
                f"<pre>{pigpio_error}</pre></p>").classes("text-body1 text-center w-full")
        return

    ui.label('Stanowisko Testowe ESC').classes('text-h3 text-bold q-my-md text-center w-full')
    with ui.card().classes('w-full max-w-2xl mx-auto'):

        ui.label('Status na żywo').classes('text-h5')
        ui.label().classes('text-xl').bind_text_from(stand, 'status_text')
        ui.label().classes('text-lg font-mono').bind_text_from(
            stand, '_current_pwm', lambda p: f"Aktualne PWM: {p:.2f} µs"
        )

        time_left_label = ui.label().classes('text-lg')
        with ui.linear_progress(show_value=False) \
               .bind_value_from(stand, 'progress') \
               .style('height: 35px; border-radius: 8px;') \
               .props('instant-feedback'):
            progress_bar_label = ui.label().classes('absolute-center text-black text-bold text-h6')

        def update_ui():
            remaining = stand.time_left
            time_left_label.set_text(
                f"Czas do końca: {int(remaining // 60):02d}:{int(remaining % 60):02d}"
            )

            elapsed = stand.elapsed_time
            text = f"{stand._current_pwm:.0f} µs | Czas: {int(elapsed // 60):02d}:{int(elapsed % 60):02d}"
            progress_bar_label.set_text(text)

            if remaining <= 0 and stand.is_running and not stand.is_paused:
                stand.stop_test()

        ui.timer(0.1, update_ui)

        ui.separator().classes('q-my-md')
        with ui.row().classes('w-full justify-around items-center'):
            with ui.column():
                ui.button('UZBRÓJ', on_click=stand.arm_system, color='warning') \
                  .props('icon=lock_open') \
                  .bind_enabled_from(stand, 'is_armed', backward=lambda x: not x)

                ui.button('ROZBRÓJ', on_click=stand.disarm_system, color='positive') \
                  .props('icon=lock') \
                  .bind_enabled_from(stand, 'is_armed')

            with ui.column():
                ui.button('Start Test', on_click=stand.start_test, color='negative') \
                  .props('icon=play_arrow') \
                  .bind_enabled_from(stand, 'can_start')

                ui.button('Pauza/Wznów', on_click=stand.toggle_pause) \
                  .bind_enabled_from(stand, 'is_running')

                ui.button('Stop Test', on_click=stand.stop_test) \
                  .bind_enabled_from(stand, 'is_running')

    with ui.card().classes('w-full max-w-2xl mx-auto q-mt-md'):
        ui.label('Ustawienia').classes('text-h5')
        with ui.grid(columns=2).classes('w-full gap-4'):
            ui.number(label='IDLE PWM (µs)', step=1) \
              .bind_value(stand, 'idle_pwm') \
              .bind_enabled_from(stand, 'is_armed', backward=lambda x: not x)

            ui.number(label='Czas trwania (s)', min=1, max=86400, step=1) \
              .bind_value(stand, 'duration_seconds') \
              .bind_enabled_from(stand, 'is_running', backward=lambda x: not x)

            ui.number(label='Min. PWM w teście (µs)', min=900, max=2500, step=10) \
              .bind_value(stand, 'min_pwm') \
              .bind_enabled_from(stand, 'is_running', backward=lambda x: not x)

            ui.number(label='Max. PWM w teście (µs)', min=900, max=2500, step=10) \
              .bind_value(stand, 'max_pwm') \
              .bind_enabled_from(stand, 'is_running', backward=lambda x: not x)

            ui.number(label='Rampa wzrostu (PWM/s)', min=1, max=1_000_000, step=100) \
              .bind_value(stand, 'ramp_up_per_s') \
              .bind_enabled_from(stand, 'is_running', backward=lambda x: not x)

            ui.number(label='Rampa spadku (PWM/s)', min=1, max=1_000_000, step=100) \
              .bind_value(stand, 'ramp_down_per_s') \
              .bind_enabled_from(stand, 'is_running', backward=lambda x: not x)

if stand:
    app.on_shutdown(stand.cleanup)

ui.run(title='Stanowisko Testowe ESC', port=APP_PORT, host='0.0.0.0', show=False, reload=False)
