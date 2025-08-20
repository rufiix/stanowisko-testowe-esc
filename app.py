import logging
from nicegui import ui, app

from esc_test_stand import (
    ESCTestStand,
    MIN_PWM_LIMIT,
    MAX_PWM_LIMIT,
    fmt_mmss,
)
from dshot import DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE

# -------------------- KONFIG --------------------
GPIO_PIN_ESC = 17
APP_PORT = 8081

# Wyciszenie logów NiceGUI
logging.getLogger('nicegui').setLevel(logging.WARNING)
logging.getLogger('nicegui.binding').setLevel(logging.WARNING)

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
            esc_test_stand, 'protocol', lambda p: f"Protocol: {p}"
        )

        ui.label().classes('text-lg font-mono').bind_text_from(
            esc_test_stand, 'current_value'
        )

        time_label = ui.label().classes('text-lg')

        # Pasek postępu z napisem wewnątrz (absolute-center działa tylko dla dzieci)
        prog = ui.linear_progress(show_value=False) \
            .style('height:35px;border-radius:8px;position:relative;') \
            .props('instant-feedback')
        prog.bind_value_from(esc_test_stand, 'progress')

        with prog:
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
        ui.button('ARM', on_click=lambda: esc_test_stand.arm_system(), color='warning') \
            .props('icon=lock_open') \
            .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a)

        ui.button('DISARM', on_click=lambda: esc_test_stand.disarm_system(), color='positive') \
            .props('icon=lock') \
            .bind_enabled_from(esc_test_stand, 'is_armed')

        ui.button('START', on_click=lambda: esc_test_stand.start_test(), color='negative') \
            .props('icon=play_arrow') \
            .bind_enabled_from(esc_test_stand, 'can_start')

        pause_button = ui.button('PAUSE', on_click=lambda: esc_test_stand.toggle_pause(), color='info') \
            .props('icon=pause') \
            .bind_enabled_from(esc_test_stand, 'is_running')

        ui.button('STOP', on_click=lambda: esc_test_stand.stop_test(), color='negative') \
            .props('icon=stop') \
            .bind_enabled_from(esc_test_stand, 'is_running')

        ui.button('EMERGENCY STOP', on_click=lambda: esc_test_stand.stop_test(hard=True), color='negative') \
            .props('icon=bolt') \
            .bind_enabled_from(esc_test_stand, 'can_emergency_stop')

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


@ui.page('/')
def main_page():
    if init_error:
        ui.label("INITIALIZATION ERROR").classes("text-h4 text-negative text-center w-full")
        ui.html(f"<p>pigpio error:</p><pre>{init_error}</pre>").classes("text-body1 text-center w-full")
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
    favicon='🚀',
)
