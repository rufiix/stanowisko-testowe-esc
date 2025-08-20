import time
import threading
from collections import deque
from typing import Optional

import pigpio

from pigpio_worker import PigpioWorker

# DShot300 (cyfrowy)
# Bit time ≈ 3.33 µs, ale używamy 3 µs (integer µs) => wymagane pigpiod -s 1
DSHOT_BIT_TOTAL_US = 3
DSHOT1_HIGH_US = 2
DSHOT1_LOW_US = DSHOT_BIT_TOTAL_US - DSHOT1_HIGH_US  # 1 us
DSHOT0_HIGH_US = 1
DSHOT0_LOW_US = DSHOT_BIT_TOTAL_US - DSHOT0_HIGH_US  # 2 us
# ~1 kHz ramek: 16 bitów * 3us = 48us => gap ≈ 952us
DSHOT_FRAME_GAP_US = max(10, 1000 - 16 * DSHOT_BIT_TOTAL_US)
DSHOT_MIN_THROTTLE = 48   # <48 to komendy; >=48 to gaz
DSHOT_MAX_THROTTLE = 2047
DSHOT_DEFAULT_IDLE = DSHOT_MIN_THROTTLE  # bezpieczny idle (nie komendy)

# DShot: bezpieczne opóźnienie przed kasowaniem poprzedniej fali
DSHOT_WAVE_DELETE_DELAY_S = 0.02  # 20 ms


class DShotTransmitter:
    """Nadajnik DShot oparty o pigpio wave z bezpiecznym przełączaniem i kasowaniem (via PigpioWorker)."""

    MAX_PENDING_DELETES = 100  # Limit kolejki oczekujących usunięć

    def __init__(self, worker: PigpioWorker, gpio: int):
        self.worker = worker
        self.gpio = gpio
        self._current_wid: Optional[int] = None
        self._last_throttle: Optional[int] = None
        self._lock = threading.Lock()
        self._tx_lock = threading.Lock()  # serializacja wave*
        self._pending_delete = deque()  # (wid, safe_after_time)
        self._pending_ids = set()       # deduplikacja WID w kolejce
        self._req_counter = 0

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

    def _build_wave(self, throttle: int) -> int:
        if not self.worker.connected:
            raise ConnectionError("pigpio disconnected")

        packet = self._make_packet(throttle, telemetry=0)

        pulses = []
        on_mask = 1 << self.gpio
        off_mask = on_mask

        # MSB -> LSB
        for i in range(15, -1, -1):
            bit = (packet >> i) & 1
            if bit:
                pulses.append(pigpio.pulse(on_mask, 0, DSHOT1_HIGH_US))
                pulses.append(pigpio.pulse(0, off_mask, DSHOT1_LOW_US))
            else:
                pulses.append(pigpio.pulse(on_mask, 0, DSHOT0_HIGH_US))
                pulses.append(pigpio.pulse(0, off_mask, DSHOT0_LOW_US))
        # przerwa między ramkami
        pulses.append(pigpio.pulse(0, off_mask, DSHOT_FRAME_GAP_US))

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

        self.worker.call('wave_add_new', timeout=1.0)
        self.worker.call('wave_add_generic', pulses, timeout=1.0)
        wid = self.worker.call('wave_create', timeout=1.0)
        if wid < 0:
            # Zamiast wave_clear() — sprzątnij nieaktywne fale i spróbuj ponownie
            self._emergency_cleanup()
            self.worker.call('wave_add_new', timeout=1.0)
            self.worker.call('wave_add_generic', pulses, timeout=1.0)
            wid = self.worker.call('wave_create', timeout=1.0)
            if wid < 0:
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

                # Sprawdź aktualność w tym samym scope locka
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
                    while len(self._pending_delete) > self.MAX_PENDING_DELETES:
                        oldest_wid, _ = self._pending_delete.popleft()
                        self._pending_ids.discard(oldest_wid)
                        try:
                            self.worker.call('wave_delete', oldest_wid, timeout=0.1)
                        except Exception:
                            pass

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
