import time, random, threading, pigpio
from nicegui import ui, app

GPIO_PIN_ESC = 17
APP_PORT = 8081
PWM_FREQUENCY = 490
PWM_RANGE = 255

class ESCTestStand:
    def __init__(s, gpio_pin):
        s.gpio_pin = gpio_pin
        s.idle_pwm = 1050
        s.duration_seconds = 60
        s.min_pwm = 1100
        s.max_pwm = 2000
        s.ramp_up_per_s = 500
        s.ramp_down_per_s = 800

        # stan PWM
        s._pwm_frequency = PWM_FREQUENCY    # 50 lub 490 Hz
        s._pwm_range = PWM_RANGE
        s._pu = 1_000_000 / s._pwm_frequency  # mikrosekundy na cykl

        s._current_pwm = 0.0
        s._target_pwm = 0.0
        s._end_time = 0.0
        s._time_left_at_pause = 0.0
        s._last_update_time = time.time()
        s._lock = threading.Lock()
        s._stop_event = threading.Event()
        s._pause_event = threading.Event()
        s._running_event = threading.Event()
        s._armed = False

        pi = pigpio.pi()
        if not pi.connected:
            pi.stop()
            raise IOError("Nie można połączyć się z demonem pigpio. Uruchom 'sudo pigpiod'.")
        s.pi = pi

        s.pi.set_mode(s.gpio_pin, pigpio.OUTPUT)
        s.pi.set_PWM_frequency(s.gpio_pin, s._pwm_frequency)
        s.pi.set_PWM_range(s.gpio_pin, s._pwm_range)

        s._pwm_thread = threading.Thread(target=s._run_pwm_loop, daemon=True)
        s._pwm_thread.start()

    def __enter__(s):
        return s

    def __exit__(s, et, ev, etb):
        s.cleanup()

    def _run_pwm_loop(s):
        while not s._stop_event.is_set():
            start = time.time()
            with s._lock:
                now = start
                dt = now - s._last_update_time
                s._last_update_time = now

                if s._running_event.is_set() and not s._pause_event.is_set():
                    # rampa
                    if abs(s._current_pwm - s._target_pwm) < 2:
                        s._target_pwm = random.uniform(s.min_pwm, s.max_pwm)
                    if s._current_pwm < s._target_pwm:
                        s._current_pwm = min(s._current_pwm + s.ramp_up_per_s * dt, s._target_pwm)
                    else:
                        s._current_pwm = max(s._current_pwm - s.ramp_down_per_s * dt, s._target_pwm)
                    # autostop
                    if now >= s._end_time:
                        s._running_event.clear()
                        s._pause_event.clear()
                elif s._armed:
                    s._current_pwm = float(s.idle_pwm)
                else:
                    s._current_pwm = 0.0

                # mikrosekundy → dutycycle
                d = int((s._current_pwm / s._pu) * s._pwm_range) if s._pu > 0 else 0
                d = max(0, min(s._pwm_range, d))

            # wysyłka do pigpio
            for attempt in range(3):
                try:
                    s.pi.set_PWM_dutycycle(s.gpio_pin, d)
                    break
                except Exception as e:
                    time.sleep(0.02)
                    if attempt == 2:
                        print(f"Błąd dutycycle: {e}")
            # krótka pauza, by nie zatykać CPU
            elapsed = time.time() - start
            time.sleep(max(0, 0.02 - elapsed))

    def cleanup(s):
        s._stop_event.set()
        s._pwm_thread.join()
        if s.pi.connected:
            try:
                s.pi.set_PWM_dutycycle(s.gpio_pin, 0)
            except Exception:
                pass
            s.pi.stop()

    def arm_system(s):
        with s._lock:
            if not s._armed:
                s._armed = True
        ui.notify("UZBROJONY", type='warning')

    def disarm_system(s):
        with s._lock:
            if s._running_event.is_set():
                s.stop_test(0)
            if s._armed:
                s._armed = False
        ui.notify("ROZBROJONY", type='positive')

    def start_test(s):
        with s._lock:
            if not s._armed or s._running_event.is_set():
                return
            if s.min_pwm >= s.max_pwm:
                ui.notify("Min>Max PWM", type='negative')
                return
            now = time.time()
            s._running_event.set()
            s._pause_event.clear()
            s._end_time = now + s.duration_seconds
            s._last_update_time = now
            s._target_pwm = random.uniform(s.min_pwm, s.max_pwm)
        ui.notify("START", type='positive')

    def stop_test(s, notify_ui=1):
        with s._lock:
            if not s._running_event.is_set():
                return
            s._running_event.clear()
            s._pause_event.clear()
        if notify_ui:
            ui.notify("STOP", type='info')

    def toggle_pause(s):
        with s._lock:
            if not s._running_event.is_set():
                return
            if s._pause_event.is_set():
                s._pause_event.clear()
                s._end_time = time.time() + s._time_left_at_pause
                s._last_update_time = time.time()
                msg = "WZNOWIONO"
            else:
                s._pause_event.set()
                s._time_left_at_pause = max(0.0, s._end_time - time.time())
                msg = "PAUZA"
        ui.notify(msg, type='info')

    def set_pwm_frequency(s, freq: int):
        # tylko gdy ROZBROJONY!
        if s._armed:
            ui.notify("Zmień tryb tylko gdy system jest ROZBROJONY", type='negative')
            return
        if freq not in (50, 490):
            ui.notify("Dozwolone wartości: 50 Hz lub 490 Hz", type='negative')
            return
        with s._lock:
            if freq == s._pwm_frequency:
                return
            s._pwm_frequency = freq
            s._pu = 1_000_000 / freq
            try:
                s.pi.set_PWM_frequency(s.gpio_pin, freq)
            except Exception as e:
                ui.notify(f"Błąd ustawiania częstotliwości: {e}", type='negative')
                return
        ui.notify(f"Częstotliwość PWM ustawiona: {freq} Hz", type='info')

    @property
    def pwm_frequency(s):
        return s._pwm_frequency

    @property
    def is_armed(s):
        return s._armed

    @property
    def is_running(s):
        return s._running_event.is_set()

    @property
    def is_paused(s):
        return s._pause_event.is_set()

    @property
    def can_start(s):
        return s._armed and not s._running_event.is_set()

    @property
    def status_text(s):
        if not s._armed:
            return "ROZBROJONY"
        if s.is_running and s.is_paused:
            return "PAUZA"
        if s.is_running:
            return "URUCHOMIONY"
        return f"UZBROJONY ({s.idle_pwm}µs)"

    @property
    def time_left(s):
        if not s.is_running:
            return float(s.duration_seconds)
        if s.is_paused:
            return float(s._time_left_at_pause)
        return max(0.0, s._end_time - time.time())

    @property
    def elapsed_time(s):
        if not s.is_running:
            return 0.0
        return s.duration_seconds - s.time_left

    @property
    def progress(s):
        if not s.is_running or s.duration_seconds <= 0:
            return 0.0
        return s.elapsed_time / s.duration_seconds

    @property
    def current_pwm(s):
        return s._current_pwm


try:
    s = ESCTestStand(GPIO_PIN_ESC)
    app.on_shutdown(s.cleanup)
    e = None
except Exception as err:
    s = None
    e = err


def cs():
    ui.label('TEST ESC').classes('text-h3 text-bold q-my-md text-center w-full')
    with ui.card().classes('w-full max-w-2xl mx-auto'):
        ui.label('STATUS').classes('text-h5')
        ui.label().classes('text-xl').bind_text_from(s, 'status_text')
        ui.label().classes('text-lg font-mono')\
            .bind_text_from(s, 'current_pwm', lambda p: f"PWM: {p:.2f}µs")
        tl = ui.label().classes('text-lg')
        with ui.linear_progress(show_value=False)\
                .bind_value_from(s, 'progress')\
                .style('height:35px;border-radius:8px;')\
                .props('instant-feedback'):
            pl = ui.label().classes('absolute-center text-black text-bold text-h6')

        def update_labels():
            # tylko gdy running i nie paused
            if s.is_running and not s.is_paused:
                remaining = s.time_left
                if remaining <= 0:
                    s.stop_test()
                    return
                tl.set_text(f"CZAS: {int(remaining//60):02d}:{int(remaining%60):02d}")
                elapsed = s.elapsed_time
                pl.set_text(f"{s.current_pwm:.0f}µs|{int(elapsed//60):02d}:{int(elapsed%60):02d}")

        ui.timer(0.1, update_labels)


def cc():
    ui.separator().classes('q-my-md')
    with ui.row().classes('w-full justify-around items-center'):
        ui.button('UZBROJ', on_click=s.arm_system, color='warning')\
          .props('icon=lock_open')\
          .bind_enabled_from(s, 'is_armed', backward=lambda a: not a)
        ui.button('ROZBROJ', on_click=s.disarm_system, color='positive')\
          .props('icon=lock')\
          .bind_enabled_from(s, 'is_armed')
        ui.button('START', on_click=s.start_test, color='negative')\
          .props('icon=play_arrow')\
          .bind_enabled_from(s, 'can_start')
        pb = ui.button('PAUZA', on_click=s.toggle_pause, color='info')\
          .props('icon=pause_circle')\
          .bind_enabled_from(s, 'is_running')
        ui.button('STOP', on_click=s.stop_test, color='negative')\
          .props('icon=stop')\
          .bind_enabled_from(s, 'is_running')

        # wybór trybu PWM tylko gdy ROZBROJONY i inna częstotliwość
        with ui.row().classes('items-center q-ml-md'):
            ui.label().bind_text_from(s, 'pwm_frequency', 
                                      lambda f: f'PWM: {f} Hz')\
              .classes('text-caption')
            ui.button('50 Hz', on_click=lambda: s.set_pwm_frequency(50), color='primary')\
              .bind_enabled_from(s, 'is_armed', backward=lambda a: not a)\
              .bind_enabled_from(s, 'pwm_frequency', backward=lambda f: f != 50)
            ui.button('490 Hz', on_click=lambda: s.set_pwm_frequency(490), color='primary')\
              .bind_enabled_from(s, 'is_armed', backward=lambda a: not a)\
              .bind_enabled_from(s, 'pwm_frequency', backward=lambda f: f != 490)

        def update_pause_button():
            if s.is_paused:
                pb.set_text('WZNOW')
                pb.set_icon('play_arrow')
            else:
                pb.set_text('PAUZA')
                pb.set_icon('pause_circle')
        ui.timer(0.1, update_pause_button)


def cst():
    with ui.card().classes('w-full max-w-2xl mx-auto q-mt-md'):
        ui.label('USTAWIENIA').classes('text-h5')
        with ui.grid(columns=2).classes('w-full gap-4'):
            ui.number('IDLE PWM (µs)', step=1)\
              .bind_value(s, 'idle_pwm')\
              .bind_enabled_from(s, 'is_armed', backward=lambda a: not a)
            ui.number('CZAS (s)', min=1, max=86400, step=1)\
              .bind_value(s, 'duration_seconds')\
              .bind_enabled_from(s, 'is_running', backward=lambda r: not r)
            ui.number('MIN PWM (µs)', min=900, max=2500, step=10)\
              .bind_value(s, 'min_pwm')\
              .bind_enabled_from(s, 'is_running', backward=lambda r: not r)
            ui.number('MAX PWM (µs)', min=900, max=2500, step=10)\
              .bind_value(s, 'max_pwm')\
              .bind_enabled_from(s, 'is_running', backward=lambda r: not r)
            ui.number('RAMPA +', min=1, max=1_000_000, step=100)\
              .bind_value(s, 'ramp_up_per_s')\
              .bind_enabled_from(s, 'is_running', backward=lambda r: not r)
            ui.number('RAMPA -', min=1, max=1_000_000, step=100)\
              .bind_value(s, 'ramp_down_per_s')\
              .bind_enabled_from(s, 'is_running', backward=lambda r: not r)


@ui.page('/')
def mp():
    if e:
        ui.label("BŁĄD").classes("text-h4 text-negative text-center w-full")
        ui.html(f"<p>Błąd pigpio:</p><pre>{e}</pre>")\
          .classes("text-body1 text-center w-full")
        return
    cs()
    cc()
    cst()

ui.run(title='TEST ESC', port=APP_PORT, host='0.0.0.0', show=False, reload=False)
