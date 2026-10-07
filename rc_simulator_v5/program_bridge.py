"""Trusted Python program runner, separate process, JSON IPC, bounded watchdog.

Only five ultrasonic distances and simulation time are sent to the worker.
This is fault isolation, NOT an OS/filesystem/network security sandbox.
"""
from __future__ import annotations
import ast
from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import tokenize

ROOT = Path(__file__).resolve().parent
SENSOR_NAMES = ('Fr', 'FrLh', 'RrLh', 'FrRh', 'RrRh')

class ProgramError(RuntimeError):
    """The program cannot safely continue in the simulator."""

@dataclass(frozen=True)
class ProgramSpec:
    path: Path
    interface: str
    sha256: str

    @classmethod
    def inspect(cls, path, interface='auto'):
        path = Path(path).expanduser().resolve()
        if path.suffix.lower() != '.py' or not path.is_file():
            raise ValueError('存在する .py ファイルを選択してください。')
        raw = path.read_bytes()
        if len(raw) > 2_000_000:
            raise ValueError('プログラムは2MB以下にしてください。')
        with tokenize.open(path) as stream:
            tree = ast.parse(stream.read(), filename=str(path))
        if interface not in ('auto', 'function', 'legacy'):
            raise ValueError('実行方式は auto / function / legacy です。')
        if interface == 'auto':
            imports = []
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    imports.extend(a.name for a in node.names)
                elif isinstance(node, ast.ImportFrom):
                    imports.append(node.module or '')
            interface = 'legacy' if any(
                name.startswith(('RPi', 'Adafruit_PCA9685')) for name in imports
            ) else 'function'
        return cls(path, interface, hashlib.sha256(raw).hexdigest())


class ProgramProcess:
    def __init__(self, spec: ProgramSpec, *, timeout=1.0, startup_timeout=5.0,
                 hardware_path=None):
        self.spec = spec
        self.timeout = float(timeout)
        self.startup_timeout = float(startup_timeout)
        self.hardware_path = Path(hardware_path or ROOT/'hardware.json').resolve()
        self.proc = None
        self.events = queue.Queue(maxsize=32)
        self.messages = deque(maxlen=800)
        self._log_lock = threading.Lock()
        self.pending = False
        self.ready = False
        self.closed = False
        self.deadline = 0.0
        self.pending_time = None
        self.requests = 0
        self.last_payload_keys = ()

    def start(self):
        if self.closed:
            raise ProgramError('プログラムは停止済みです。再読込してください。')
        if self.proc is not None:
            return
        if hashlib.sha256(self.spec.path.read_bytes()).hexdigest() != self.spec.sha256:
            raise ProgramError('選択後に .py が変更されました。「再読込」で新しい内容を確認してください。')
        executable = Path(sys.executable)
        if executable.name.lower() == 'pythonw.exe':
            executable = executable.with_name('python.exe')
        env = os.environ.copy()
        env['PYTHONUNBUFFERED'] = '1'
        env['PYTHONIOENCODING'] = 'utf-8'
        env['PYTHONDONTWRITEBYTECODE'] = '1'
        args = [str(executable), '-u', str(ROOT/'program_worker.py'),
                str(self.spec.path), self.spec.interface, str(self.hardware_path), self.spec.sha256]
        self.proc = subprocess.Popen(
            args, shell=False, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, encoding='utf-8', errors='replace',
            bufsize=1, cwd=str(self.spec.path.parent), env=env,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0) if os.name == 'nt' else 0,
        )
        self.deadline = time.monotonic() + self.startup_timeout
        threading.Thread(target=self._read_protocol, daemon=True, name='rc-python-ipc').start()
        threading.Thread(target=self._read_logs, daemon=True, name='rc-python-log').start()

    def _put(self, event):
        try:
            self.events.put(event, timeout=.2)
        except queue.Full:
            # A trusted, compliant worker has at most one outstanding response.
            pass

    def _read_protocol(self):
        try:
            for line in iter(lambda: self.proc.stdout.readline(262145), ''):
                if len(line) > 262144:
                    self._put({'kind': 'error', 'error': 'IPC応答が大きすぎます。'})
                    return
                try:
                    obj = json.loads(line)
                    if not isinstance(obj, dict):
                        raise ValueError('expected JSON object')
                except (ValueError, TypeError) as exc:
                    self._put({'kind': 'error', 'error': f'IPC形式エラー: {exc}'})
                    return
                self._put(obj)
        except (OSError, ValueError):
            pass
        finally:
            self._put({'kind': 'eof'})

    def _read_logs(self):
        try:
            for line in iter(lambda: self.proc.stderr.readline(4096), ''):
                with self._log_lock:
                    self.messages.append(line.rstrip('\r\n')[:4096])
        except (OSError, ValueError):
            pass

    def log_text(self):
        with self._log_lock:
            return '\n'.join(self.messages)

    def _receive(self, block):
        remaining = max(0.0, self.deadline - time.monotonic())
        try:
            event = self.events.get(timeout=remaining if block else 0)
        except queue.Empty:
            if time.monotonic() >= self.deadline:
                self.close()
                raise ProgramError('Pythonが時間内に応答しないため停止しました（無限ループ・重い処理など）。')
            return None
        kind = event.get('kind')
        if kind in ('error', 'eof'):
            message = event.get('error') or 'Pythonプロセスが予期せず終了しました。'
            if event.get('traceback'):
                message += '\n' + event['traceback']
            self.close()
            raise ProgramError(message)
        return event

    def step(self, sensors: dict, sim_time: float, dt: float, *, block=True):
        """None means waiting. Never move physics until a matching reply exists."""
        self.start()
        if not self.ready:
            event = self._receive(block)
            if event is None:
                return None
            if event.get('kind') != 'ready':
                raise ProgramError('Pythonの起動応答が不正です。')
            self.ready = True
        if not self.pending:
            values = {}
            for name in SENSOR_NAMES:
                value = sensors.get(name)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                    raise ProgramError(f'センサー {name} が不正です。')
                values[name] = float(value)
            payload = {'op': 'step', 'sensors': values, 'time': float(sim_time), 'dt': float(dt)}
            self.last_payload_keys = tuple(payload)
            try:
                self.proc.stdin.write(json.dumps(payload, allow_nan=False) + '\n')
                self.proc.stdin.flush()
            except (BrokenPipeError, OSError, ValueError) as exc:
                self.close()
                raise ProgramError(f'Pythonと通信できません: {exc}') from exc
            self.pending = True
            self.pending_time = sim_time
            self.deadline = time.monotonic() + self.timeout
            self.requests += 1
        elif abs(float(sim_time) - self.pending_time) > 1e-8:
            raise ProgramError('実行中にシミュレーション時刻が変化しました。リセットしてください。')
        event = self._receive(block)
        if event is None:
            return None
        self.pending = False
        if event.get('kind') not in ('result', 'tick', 'finished'):
            raise ProgramError('Pythonから不正な制御応答を受信しました。')
        return event

    def check_watchdog(self):
        """Safe to call while paused; completed replies are not considered stalled."""
        if self.proc and not self.closed and (self.pending or not self.ready):
            if self.events.empty() and time.monotonic() > self.deadline:
                self.close()
                raise ProgramError('応答のないPythonプログラムを停止しました。')

    def close(self):
        if self.closed:
            return
        self.closed = True
        proc = self.proc
        if proc is None:
            return
        try:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=.25)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=.5)
        except (OSError, subprocess.TimeoutExpired):
            pass
        for stream in (proc.stdin, proc.stdout, proc.stderr):
            try:
                stream.close()
            except (OSError, ValueError):
                pass

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
