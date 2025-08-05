# --- OSTATECZNA, BEZPIECZNA WERSJA (ZAWSZE STARTUJE ROZBROJONY) ---
import time
import random
import threading
import pigpio
import sys
from nicegui import ui, app

# --- Konfiguracja Aplikacji ---
GPIO_PIN_ESC = 18
APP_PORT = 8081

class ESCTestStand:
    def __init__(self, gpio_pin):
        self.gpio_pin = gpio_pin
        # <<< ZMIANA: Stan początkowy jest na sztywno ustawiony jako ROZBROJONY >>>
        self._is_armed = False
        
        self.idle_pwm = 1050
        self.duration_minutes = 10
        self.min_pwm = 1100
        self.max_pwm = 2000
        self.ramp_up_per_s = 500
        self.ramp_down_per_s = 800
        
        self._is_running = False
        self._is_paused = False
        self._test_thread = None
        self._current_pwm = 0
        self._target_pwm = 0
        self._start_time = 0
        self._end_time = 0
        self._pause_time = 0
        
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise IOError("Nie można połączyć się z demonem pigpio. Uruchom 'sudo pigpiod'.")
        self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)
        # <<< ZMIANA: Zawsze startujemy z PWM ustawionym na 0 >>>
        self.pi.set_servo_pulsewidth(self.gpio_pin, 0)
        print("System uruchomiony w trybie ROZBROJONY (bezpieczny).")

    # <<< ZMIANA: Usunięto metody _load_state_and_clear() oraz _save_state() >>>

    def arm_system(self):
        if self._is_armed: return
        self._is_armed = True
        self._current_pwm = self.idle_pwm
        self.pi.set_servo_pulsewidth(self.gpio_pin, self.idle_pwm)
        ui.notify("System UZBROJONY. Silnik w trybie IDLE.", type='warning')

    def disarm_system(self):
        if not self._is_armed: return
        if self._is_running:
            self.stop_test()
            time.sleep(0.1)
        self._is_armed = False
        self._current_pwm = 0
        self.pi.set_servo_pulsewidth(self.gpio_pin, 0)
        ui.notify("System ROZBROJONY. Silnik wyłączony.", type='positive')

    def cleanup(self):
        print("Zatrzymywanie i sprzątanie zasobów ESC...")
        if hasattr(self, 'pi') and self.pi.connected:
            self.pi.set_servo_pulsewidth(self.gpio_pin, 0)
            self.pi.stop()
        print("Zasoby ESC zwolnione.")
    
    # ... reszta kodu (metody, UI) jest w pełni poprawna ...
    def update_idle_pwm(self):
        if self._is_armed and not self._is_running:
            self._current_pwm = self.idle_pwm
            self.pi.set_servo_pulsewidth(self.gpio_pin, self.idle_pwm)
    def stop_test(self):
        if not self._is_running: return
        self._is_running = False
        if self._test_thread is not None: self._test_thread.join(timeout=1.0)
        self.pi.set_servo_pulsewidth(self.gpio_pin, self.idle_pwm)
        self._current_pwm = self.idle_pwm
        ui.notify("Test zatrzymany. Powrót do trybu IDLE.")
    @property
    def is_armed(self): return self._is_armed
    @property
    def is_running(self): return self._is_running
    @property
    def status_text(self):
        if not self._is_armed: return "ROZBROJONY (BEZPIECZNY)"
        if not self._is_running: return f"UZBROJONY / IDLE ({self.idle_pwm} µs)"
        if self._is_paused: return "Spauzowany"
        return "Uruchomiony"
    @property
    def time_left(self):
        if not self._is_running: return self.duration_minutes * 60
        if self._is_paused: return self._end_time - self._pause_time
        return self._end_time - time.time()
    @property
    def elapsed_time(self):
        if not self._is_running or self._start_time == 0: return 0
        if self._is_paused: return self._pause_time - self._start_time
        return time.time() - self._start_time
    @property
    def progress(self):
        total_duration = self.duration_minutes * 60
        if total_duration == 0 or not self._is_running: return 0
        return (total_duration - max(0, self.time_left)) / total_duration
    def start_test(self):
        if self._is_running or not self._is_armed: return
        self._is_running = True
        self._is_paused = False
        self._test_thread = threading.Thread(target=self._run_test_loop, daemon=True)
        self._test_thread.start()
        ui.notify("Test ESC rozpoczęty!", type='positive')
    def toggle_pause(self):
        if not self._is_running: return
        self._is_paused = not self._is_paused
        if self._is_paused:
            self._pause_time = time.time()
            ui.notify("Test ESC spauzowany.", type='info')
        else:
            pause_duration = time.time() - self._pause_time
            self._end_time += pause_duration
            ui.notify("Test ESC wznowiony.", type='info')
    def _run_test_loop(self):
        self._start_time = time.time()
        self._end_time = self._start_time + self.duration_minutes * 60
        last_update_time = self._start_time
        self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)
        while self._is_running and time.time() < self._end_time:
            if self._is_paused:
                last_update_time = time.time()
                time.sleep(0.1)
                continue
            current_time = time.time()
            dt = current_time - last_update_time
            last_update_time = current_time
            if abs(self._current_pwm - self._target_pwm) < 2:
                self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)
            if self._current_pwm < self._target_pwm:
                self._current_pwm = min(self._current_pwm + self.ramp_up_per_s * dt, self._target_pwm)
            elif self._current_pwm > self._target_pwm:
                self._current_pwm = max(self._current_pwm - self.ramp_down_per_s * dt, self._target_pwm)
            self.pi.set_servo_pulsewidth(self.gpio_pin, int(self._current_pwm))
            time.sleep(0.02)
        self.stop_test()

try:
    stand = ESCTestStand(gpio_pin=GPIO_PIN_ESC)
    pigpio_error = None
except Exception as e:
    stand = None
    pigpio_error = e

@ui.page('/')
def main_page():
    if pigpio_error:
        ui.label("BŁĄD KRYTYCZNY").classes("text-h4 text-negative text-center w-full")
        ui.html(f"<p>Nie udało się połączyć z biblioteką <b>pigpio</b>: <pre>{pigpio_error}</pre></p>").classes("text-body1 text-center w-full")
        ui.label("Upewnij się, że demon 'sudo pigpiod' jest uruchomiony.").classes("text-center w-full")
        return

    ui.label('Stanowisko Testowe ESC').classes('text-h3 text-bold q-my-md text-center w-full')
    with ui.card().classes('w-full max-w-2xl mx-auto'):
        ui.label('Status na żywo').classes('text-h5')
        ui.label().classes('text-xl').bind_text_from(stand, 'status_text')
        ui.label().classes('text-lg font-mono').bind_text_from(stand, '_current_pwm', lambda p: f"Aktualne PWM: {p:.2f} µs")
        ui.label().classes('text-lg').bind_text_from(stand, 'time_left', lambda t: f"Czas do końca: {int(t//60):02d}:{int(t%60):02d}")
        with ui.linear_progress(show_value=False).bind_value_from(stand, 'progress').style('height: 35px; border-radius: 8px;').props('instant-feedback'):
            progress_bar_label = ui.label().classes('absolute-center text-black text-bold text-h6')
        def update_progress_bar_text():
            elapsed = stand.elapsed_time
            text_to_show = f'{stand._current_pwm:.0f} µs | Czas: {int(elapsed // 60):02d}:{int(elapsed % 60):02d}'
            progress_bar_label.set_text(text_to_show)
        ui.timer(0.1, update_progress_bar_text)
        ui.separator().classes('q-my-md')
        with ui.row().classes('w-full justify-around items-center'):
            with ui.column():
                 ui.button('UZBRÓJ', on_click=stand.arm_system, color='warning').props('icon=lock_open').bind_enabled_from(stand, 'is_armed', backward=lambda x: not x)
                 ui.button('ROZBRÓJ', on_click=stand.disarm_system, color='positive').props('icon=lock').bind_enabled_from(stand, 'is_armed')
            with ui.column():
                ui.button('Start Test', on_click=stand.start_test, color='negative').props('icon=play_arrow').bind_enabled(lambda: stand.is_armed and not stand.is_running)
                ui.button('Pauza/Wznów', on_click=stand.toggle_pause).bind_enabled_from(stand, 'is_running')
                ui.button('Stop Test', on_click=stand.stop_test).bind_enabled_from(stand, 'is_running')
            
    with ui.card().classes('w-full max-w-2xl mx-auto q-mt-md'):
        ui.label('Ustawienia').classes('text-h5')
        with ui.grid(columns=2).classes('w-full gap-4'):
            # Pole IDLE można edytować tylko w stanie rozbrojonym.
            ui.number(label='IDLE PWM (µs)', min=800, max=1200, step=1, value=1050) \
                .bind_value(stand, 'idle_pwm') \
                .bind_enabled_from(stand, 'is_armed', backward=lambda armed: not armed)
            
            # Pozostałe pola są zablokowane TYLKO, gdy test jest faktycznie uruchomiony.
            ui.number(label='Czas trwania (min)', min=1, max=1440, step=1, value=10) \
                .bind_value(stand, 'duration_minutes') \
                .bind_enabled_from(stand, 'is_running', backward=lambda running: not running)
            
            ui.number(label='Min. PWM w teście (µs)', min=900, max=2500, step=10, value=1100) \
                .bind_value(stand, 'min_pwm') \
                .bind_enabled_from(stand, 'is_running', backward=lambda running: not running)

            ui.number(label='Max. PWM w teście (µs)', min=900, max=2500, step=10, value=2000) \
                .bind_value(stand, 'max_pwm') \
                .bind_enabled_from(stand, 'is_running', backward=lambda running: not running)

            ui.number(label='Rampa wzrostu (PWM/s)', min=1, max=1000000, step=100, value=500) \
                .bind_value(stand, 'ramp_up_per_s') \
                .bind_enabled_from(stand, 'is_running', backward=lambda running: not running)
            
            ui.number(label='Rampa spadku (PWM/s)', min=1, max=1000000, step=100, value=800) \
                .bind_value(stand, 'ramp_down_per_s') \
                .bind_enabled_from(stand, 'is_running', backward=lambda running: not running)

if stand:
    app.on_shutdown(stand.cleanup)

ui.run(title='Stanowisko Testowe ESC', port=APP_PORT, host='0.0.0.0', show=False, reload=False)
