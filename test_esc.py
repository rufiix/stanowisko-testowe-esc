import time
import random
import threading
import pigpio
from nicegui import ui, app
from typing import Optional

GPIO_PIN_ESC = 17
APP_PORT = 8081
PWM_FREQUENCY = 490
PWM_RANGE = 255
DEFAULT_IDLE_PWM = 1050
DEFAULT_DURATION = 60
MIN_PWM_LIMIT = 900
MAX_PWM_LIMIT = 2500

class ESCTestStand:
    def __init__(self, gpio_pin: int):
        self.gpio_pin = gpio_pin
        self.idle_pwm = DEFAULT_IDLE_PWM
        self.duration_seconds = DEFAULT_DURATION
        self.min_pwm = 1100
        self.max_pwm = 2000
        self.ramp_up_per_s = 500
        self.ramp_down_per_s = 800

        self._pwm_frequency = PWM_FREQUENCY
        self._pwm_range = PWM_RANGE
        self._pu = 1_000_000 / self._pwm_frequency  # Period in microseconds

        self._current_pwm = 0.0
        self._target_pwm = 0.0
        self._end_time = 0.0
        self._time_left_at_pause = 0.0
        self._last_update_time = time.time()

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._running_event = threading.Event()
        self._armed = False

        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.pi.stop()
            raise IOError("pigpio daemon not running! Execute 'sudo pigpiod'")

        try:
            self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(self.gpio_pin, self._pwm_frequency)
            self.pi.set_PWM_range(self.gpio_pin, self._pwm_range)
        except Exception as err:
            self.pi.stop()
            raise IOError(f"pigpio initialization error: {err}")

        # Start PWM thread
        self._pwm_thread = threading.Thread(target=self._run_pwm_loop, daemon=True)
        self._pwm_thread.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()

    def _run_pwm_loop(self):
        while not self._stop_event.is_set():
            start_time = time.time()
            
            # Calculate time delta outside lock
            now = time.time()
            dt = now - self._last_update_time
            self._last_update_time = now

            with self._lock:
                if self._running_event.is_set() and not self._pause_event.is_set():
                    # Ramp between current and target PWM
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
                    
                    # Check test end condition
                    if now >= self._end_time:
                        self._running_event.clear()
                        self._pause_event.clear()
                elif self._armed:
                    # Armed but not running test → idle PWM
                    self._current_pwm = float(self.idle_pwm)
                else:
                    # Disarmed → zero PWM
                    self._current_pwm = 0.0

                # Calculate duty cycle
                duty = int((self._current_pwm / self._pu) * self._pwm_range)
                duty = max(0, min(self._pwm_range, duty))

            # Set PWM with retry logic
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

            # Maintain consistent loop timing
            elapsed = time.time() - start_time
            time.sleep(max(0, 0.02 - elapsed))

    def cleanup(self):
        """Clean up resources safely"""
        self._stop_event.set()
        self._pwm_thread.join(timeout=1.0)
        if self._pwm_thread.is_alive():
            print("Warning: PWM thread did not terminate properly")
        
        if self.pi.connected:
            try:
                self.pi.set_PWM_dutycycle(self.gpio_pin, 0)
                self.pi.stop()
            except Exception:
                pass

    def arm_system(self):
        """Arm the ESC system"""
        with self._lock:
            self._pause_event.clear()
            self._running_event.clear()
            self._time_left_at_pause = 0.0
            self._armed = True
            self._current_pwm = float(self.idle_pwm)
            self._target_pwm = float(self.idle_pwm)
            self._last_update_time = time.time()
        
        try:
            duty = int((self.idle_pwm / self._pu) * self._pwm_range)
            self.pi.set_PWM_dutycycle(self.gpio_pin, duty)
        except Exception as e:
            ui.notify(f"PWM error: {str(e)}", type='negative')
            self.disarm_system()
            return
        
        ui.notify("SYSTEM ARMED", type='warning')

    def disarm_system(self):
        """Disarm the ESC system"""
        with self._lock:
            self._pause_event.clear()
            self._running_event.clear()
            self._time_left_at_pause = 0.0
            self._armed = False
        
        try:
            self.pi.set_PWM_dutycycle(self.gpio_pin, 0)
        except Exception as e:
            ui.notify(f"PWM error: {str(e)}", type='negative')
        
        ui.notify("SYSTEM DISARMED", type='positive')

    def start_test(self):
        """Start the ESC test"""
        with self._lock:
            if not self._armed or self._running_event.is_set():
                return
            
            if self.min_pwm >= self.max_pwm:
                ui.notify("MIN PWM must be less than MAX PWM", type='negative')
                return
            
            now = time.time()
            self._running_event.set()
            self._pause_event.clear()
            self._end_time = now + self.duration_seconds
            self._last_update_time = now
            self._target_pwm = random.uniform(self.min_pwm, self.max_pwm)
        
        ui.notify("TEST STARTED", type='positive')

    def stop_test(self, notify: bool = True):
        """Stop the running test"""
        with self._lock:
            if not self._running_event.is_set():
                return
            
            self._running_event.clear()
            self._pause_event.clear()
        
        if notify:
            ui.notify("TEST STOPPED", type='info')

    def toggle_pause(self):
        """Toggle pause state of the test"""
        with self._lock:
            if not self._running_event.is_set():
                return
            
            if self._pause_event.is_set():
                # Resume
                self._pause_event.clear()
                self._end_time = time.time() + self._time_left_at_pause
                self._last_update_time = time.time()
                msg = "TEST RESUMED"
            else:
                # Pause
                self._pause_event.set()
                self._time_left_at_pause = max(0.0, self._end_time - time.time())
                msg = "TEST PAUSED"
        
        ui.notify(msg, type='info')

    def set_pwm_frequency(self, freq: int):
        """Set PWM frequency (50Hz or 490Hz)"""
        if self._armed or self.is_running:
            ui.notify("Can only change frequency when DISARMED and TEST STOPPED", 
                     type='negative')
            return
        
        if freq not in (50, 490):
            ui.notify("Only 50Hz or 490Hz supported", type='negative')
            return
        
        with self._lock:
            if freq == self._pwm_frequency:
                return
            
            self._pwm_frequency = freq
            self._pu = 1_000_000 / freq
            
            try:
                self.pi.set_PWM_frequency(self.gpio_pin, freq)
            except Exception as err:
                ui.notify(f"Frequency error: {err}", type='negative')
                return
        
        ui.notify(f"PWM frequency set to {freq}Hz", type='info')

    @property
    def pwm_frequency(self) -> int:
        """Get current PWM frequency"""
        return self._pwm_frequency

    @property
    def is_armed(self) -> bool:
        """Check if system is armed"""
        return self._armed

    @property
    def is_running(self) -> bool:
        """Check if test is running"""
        return self._running_event.is_set()

    @property
    def is_paused(self) -> bool:
        """Check if test is paused"""
        return self._pause_event.is_set()

    @property
    def can_start(self) -> bool:
        """Check if test can be started"""
        return self._armed and not self._running_event.is_set()

    @property
    def status_text(self) -> str:
        """Get current status text"""
        if not self._armed:
            return "DISARMED"
        if self.is_paused:
            return f"PAUSED | Time left: {int(self.time_left)}s"
        if self.is_running:
            return f"RUNNING | Time left: {int(self.time_left)}s"
        return f"ARMED (IDLE: {self.idle_pwm}µs)"

    @property
    def time_left(self) -> float:
        """Get remaining test time in seconds"""
        if not self.is_running:
            return float(self.duration_seconds)
        if self.is_paused:
            return self._time_left_at_pause
        return max(0.0, self._end_time - time.time())

    @property
    def elapsed_time(self) -> float:
        """Get elapsed test time in seconds"""
        if not self.is_running:
            return 0.0
        return self.duration_seconds - self.time_left

    @property
    def progress(self) -> float:
        """Get test progress (0.0 to 1.0)"""
        if not self.is_running or self.duration_seconds <= 0:
            return 0.0
        return self.elapsed_time / self.duration_seconds

    @property
    def current_pwm(self) -> float:
        """Get current PWM value in microseconds"""
        return self._current_pwm


# Initialize ESC test stand
try:
    esc_test_stand = ESCTestStand(GPIO_PIN_ESC)
    init_error = None
    app.on_shutdown(esc_test_stand.cleanup)
except Exception as err:
    esc_test_stand = None
    init_error = err


def create_controls_section():
    """Create the controls section of the UI"""
    ui.label('ESC TEST STAND').classes('text-h3 text-bold q-my-md text-center w-full')
    
    with ui.card().classes('w-full max-w-2xl mx-auto'):
        # Status display
        ui.label('STATUS').classes('text-h5')
        ui.label().classes('text-xl').bind_text_from(
            esc_test_stand, 'status_text'
        )
        
        # Current PWM display
        ui.label().classes('text-lg font-mono').bind_text_from(
            esc_test_stand, 'current_pwm', 
            lambda p: f"Current PWM: {p:.1f}µs"
        )

        # Time and progress display
        time_label = ui.label().classes('text-lg')
        progress_label = ui.label().classes('absolute-center text-black text-bold text-h6')
        
        with ui.linear_progress(show_value=False) \
               .bind_value_from(esc_test_stand, 'progress') \
               .style('height:35px;border-radius:8px;') \
               .props('instant-feedback'):
            pass

        def update_display():
            """Update the time and progress display"""
            if esc_test_stand.is_running and not esc_test_stand.is_paused:
                remaining = esc_test_stand.time_left
                if remaining <= 0:
                    esc_test_stand.stop_test()
            
            # Always update time display
            time_left = esc_test_stand.time_left
            time_label.set_text(
                f"Time remaining: {int(time_left//60):02d}:{int(time_left%60):02d}"
            )
            
            # Update progress display
            elapsed = esc_test_stand.elapsed_time if esc_test_stand.is_running else 0
            current_pwm = esc_test_stand.current_pwm
            progress_label.set_text(
                f"{current_pwm:.0f}µs | Elapsed: {int(elapsed//60):02d}:{int(elapsed%60):02d}"
            )

        ui.timer(0.1, update_display)


def create_buttons_section():
    """Create the buttons section of the UI"""
    ui.separator().classes('q-my-md')
    
    with ui.row().classes('w-full justify-around items-center'):
        # Arm/Disarm buttons
        ui.button('ARM', on_click=esc_test_stand.arm_system, color='warning') \
          .props('icon=lock_open') \
          .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a)

        ui.button('DISARM', on_click=esc_test_stand.disarm_system, color='positive') \
          .props('icon=lock') \
          .bind_enabled_from(esc_test_stand, 'is_armed')

        # Test control buttons
        ui.button('START', on_click=esc_test_stand.start_test, color='negative') \
          .props('icon=play_arrow') \
          .bind_enabled_from(esc_test_stand, 'can_start')

        pause_button = ui.button('PAUSE', on_click=esc_test_stand.toggle_pause, color='info') \
                         .props('icon=pause') \
                         .bind_enabled_from(esc_test_stand, 'is_running')

        ui.button('STOP', on_click=esc_test_stand.stop_test, color='negative') \
          .props('icon=stop') \
          .bind_enabled_from(esc_test_stand, 'is_running')

        # PWM frequency buttons
        ui.button('50Hz', on_click=lambda: esc_test_stand.set_pwm_frequency(50), color='primary') \
          .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a) \
          .bind_enabled_from(esc_test_stand, 'pwm_frequency', backward=lambda f: f != 50)

        ui.button('490Hz', on_click=lambda: esc_test_stand.set_pwm_frequency(490), color='primary') \
          .bind_enabled_from(esc_test_stand, 'is_armed', backward=lambda a: not a) \
          .bind_enabled_from(esc_test_stand, 'pwm_frequency', backward=lambda f: f != 490)

        def update_pause_button():
            """Update pause button text and icon based on state"""
            if esc_test_stand.is_paused:
                pause_button.set_text('RESUME')
                pause_button.set_icon('play_arrow')
            else:
                pause_button.set_text('PAUSE')
                pause_button.set_icon('pause')

        update_pause_button()  # Initial update
        ui.timer(0.1, update_pause_button)


def create_settings_section():
    """Create the settings section of the UI"""
    with ui.card().classes('w-full max-w-2xl mx-auto q-mt-md'):
        ui.label('SETTINGS').classes('text-h5')
        
        with ui.grid(columns=2).classes('w-full gap-4'):
            # ESC settings
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
            
            # PWM limits
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
            
            # Ramp rates
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


@ui.page('/')
def main_page():
    """Main application page"""
    if init_error:
        ui.label("INITIALIZATION ERROR").classes("text-h4 text-negative text-center w-full")
        ui.html(f"<p>pigpio error:</p><pre>{init_error}</pre>") \
          .classes("text-body1 text-center w-full")
        ui.label("Make sure pigpio daemon is running (sudo pigpiod)").classes("text-center w-full")
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
