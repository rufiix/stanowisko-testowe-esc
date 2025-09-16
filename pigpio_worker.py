import time
import threading
import queue
from typing import Optional, Any
from concurrent.futures import Future, CancelledError, InvalidStateError

import pigpio


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
