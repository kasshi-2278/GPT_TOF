"""Small explicit emulation of the APIs visible in sonic_drive.py photos.

Not a complete GPIO/ESC emulator. Called only in the worker process.
Clock advances in virtual time; the parent provides ultrasonic snapshots at
120 Hz. This module never imports a real GPIO or I2C driver.
"""
from __future__ import annotations
import builtins
import json
import math
import sys
import types
from pathlib import Path

BCM_TO_BOARD = {2:3, 3:5, 4:7, 14:8, 15:10, 17:11, 18:12, 27:13, 22:15,
                23:16, 24:18, 10:19, 9:21, 25:22, 11:23, 8:24, 7:26,
                0:27, 1:28, 5:29, 6:31, 12:32, 13:33, 19:35, 16:36,
                26:37, 20:38, 21:40}


def load_hardware(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    names = ('Fr','FrLh','RrLh','FrRh','RrRh')
    pins = data.get('sensor_pins_board', {})
    if set(pins) != set(names):
        raise ValueError('hardware.json: 5個のsensor_pins_boardを設定してください。')
    used = []
    for name in names:
        if len(pins[name]) != 2 or any(type(p) is not int or p not in BCM_TO_BOARD.values() for p in pins[name]):
            raise ValueError(f'hardware.json: {name}のBOARDピンが不正です。')
        used.extend(pins[name])
    if len(set(used)) != 10:
        raise ValueError('hardware.json: GPIOピンが重複しています。')
    for key in ('steering_channel','throttle_channel'):
        if type(data[key]) is not int or not 0 <= data[key] <= 15:
            raise ValueError(f'hardware.json: {key}は0〜15です。')
    if data['steering_channel'] == data['throttle_channel']:
        raise ValueError('hardware.json: 操舵と駆動のチャンネルが同一です。')
    for keys in [('steering_right_pwm','steering_center_pwm','steering_left_pwm'),
                 ('throttle_reverse_pwm','throttle_stopped_pwm','throttle_forward_pwm')]:
        a,b,c = [data[k] for k in keys]
        if not all(type(v) is int for v in (a,b,c)) or not 0 < a < b < c < 4096:
            raise ValueError(f'hardware.json: PWM値の順序/範囲が不正です: {keys}')
    for k in ('pwm_frequency_hz','sound_speed_cm_s','minimum_neutral_s'):
        if not isinstance(data[k], (int,float)) or not math.isfinite(data[k]) or data[k] <= 0:
            raise ValueError(f'hardware.json: {k}は正の有限数です。')
    if data['legacy_esc_mode'] not in ('direct','brake_neutral_reverse'):
        raise ValueError('legacy_esc_mode は direct / brake_neutral_reverse です。')
    return data


def decode_pwm(throttle, steering, config):
    def convert(value, low, center, high):
        if not isinstance(value,(int,float)) or not math.isfinite(value):
            raise ValueError('PWM値は有限数である必要があります。')
        if not low-1 <= value <= high+1:
            raise ValueError(f'PWM {value:.2f} が校正範囲 {low}〜{high} を外れました。hardware.jsonを確認してください。')
        denominator = high-center if value >= center else center-low
        return max(-100.0, min(100.0, 100*(value-center)/denominator))
    return (convert(throttle,config['throttle_reverse_pwm'],config['throttle_stopped_pwm'],config['throttle_forward_pwm']),
            convert(steering,config['steering_right_pwm'],config['steering_center_pwm'],config['steering_left_pwm']))


class ESCModel:
    """Generic brake -> neutral -> reverse handshake, not a specific real ESC."""
    def __init__(self, config):
        self.config=config
        self.braked=False
        self.reverse_latched=False
        self.neutral_since=None
        self.stage='NEUTRAL'

    def apply(self, requested, at_time):
        if self.config['legacy_esc_mode'] == 'direct':
            self.stage='DIRECT'
            return requested
        if requested > .01:
            self.braked=False; self.reverse_latched=False; self.neutral_since=None
            self.stage='FORWARD'
            return requested
        if requested >= -.01:
            if self.neutral_since is None:
                self.neutral_since=at_time
            self.stage='NEUTRAL'
            return 0.0
        if self.reverse_latched:
            self.stage='REVERSE'
            return requested
        if self.braked and self.neutral_since is not None and at_time-self.neutral_since >= self.config['minimum_neutral_s']-1e-7:
            self.reverse_latched=True; self.stage='REVERSE'
            return requested
        self.braked=True; self.neutral_since=None; self.stage='BRAKE'
        return 0.0


class VirtualClock:
    def __init__(self, bridge):
        self.bridge=bridge
        self.t=0.0
        self.start=0.0
        self.end=0.0
        self.sensors={}
        self.sequence=0

    def grant(self, msg):
        if msg.get('op') != 'step':
            raise ValueError('Expected simulation time grant')
        start=float(msg['time']); dt=float(msg['dt'])
        if not math.isfinite(start) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('Invalid simulation time grant')
        if self.sequence and abs(start-self.t) > 1e-6:
            raise ValueError('Simulation time discontinuity; restart required')
        self.start=start; self.t=start; self.end=start+dt
        self.sensors=dict(msg['sensors']); self.sequence+=1

    def advance(self, duration):
        if not isinstance(duration,(int,float)) or not math.isfinite(duration) or duration < 0:
            raise ValueError('sleep duration must be a non-negative finite number')
        remaining=float(duration)
        while remaining > 1e-12:
            room=max(0.0,self.end-self.t)
            part=min(room,remaining)
            self.t+=part; remaining-=part
            if self.t >= self.end-1e-12:
                self.t=self.end
                self.bridge.boundary()

    def now(self):
        self.advance(.00001)  # 10us resolution for busy-wait echo polling
        return self.t

    def sleep(self, duration):
        self.advance(max(.000001,duration) if duration == 0 else duration)


class VirtualGPIO(types.ModuleType):
    BOARD=10; BCM=11; OUT=0; IN=1; LOW=0; HIGH=1
    PUD_OFF=20; PUD_DOWN=21; PUD_UP=22
    VERSION='RC-SIM-5'
    def __init__(self, clock, config):
        super().__init__('RPi.GPIO')
        self.clock=clock; self.config=config
        self.mode=None; self.modes={}; self.levels={}; self.echo={}
        self.trigger_map={v[0]:(name,v[1]) for name,v in config['sensor_pins_board'].items()}
        self.echo_pins={v[1] for v in config['sensor_pins_board'].values()}

    def setmode(self, mode):
        if mode not in (self.BOARD,self.BCM):
            raise ValueError('GPIO.setmode: BOARDまたはBCMを使用してください。')
        self.mode=mode

    def getmode(self):
        return self.mode

    def setwarnings(self, _):
        pass

    def _pin(self, pin):
        if self.mode is None:
            raise RuntimeError('GPIO.setmode(GPIO.BOARD) を先に呼び出してください。')
        if type(pin) is not int:
            raise ValueError('GPIOピン番号は整数です。')
        if self.mode == self.BCM:
            if pin not in BCM_TO_BOARD:
                raise ValueError(f'不明なBCMピン: {pin}')
            pin=BCM_TO_BOARD[pin]
        if pin not in self.trigger_map and pin not in self.echo_pins:
            raise ValueError(f'未対応GPIOピン(BOARD): {pin}。hardware.jsonを確認してください。')
        return pin

    def setup(self, channel, direction, initial=None, pull_up_down=None):
        if isinstance(channel,(list,tuple)):
            for pin in channel:
                self.setup(pin,direction,initial,pull_up_down)
            return
        pin=self._pin(channel)
        if direction not in (self.IN,self.OUT):
            raise ValueError('GPIO.setupの方向が不正です。')
        if (pin in self.trigger_map) != (direction == self.OUT):
            raise ValueError(f'GPIO {pin} のTrigger/Echo方向がhardware.jsonと一致しません。')
        self.modes[pin]=direction
        self.levels[pin]=self.LOW if initial is None else int(bool(initial))

    def output(self, channel, value):
        if isinstance(channel,(list,tuple)):
            values=value if isinstance(value,(list,tuple)) else [value]*len(channel)
            if len(values)!=len(channel):
                raise ValueError('GPIO.outputのピン数と値の数が違います。')
            for p,v in zip(channel,values):
                self.output(p,v)
            return
        pin=self._pin(channel)
        if self.modes.get(pin) != self.OUT:
            raise RuntimeError(f'GPIO {pin} は出力に設定されていません。')
        self.clock.advance(.000001)
        old=self.levels.get(pin,0); new=int(bool(value)); self.levels[pin]=new
        if old and not new:
            name,echo_pin=self.trigger_map[pin]
            distance=self.clock.sensors[name]
            start=self.clock.t+.0002
            pulse=max(.000002,2*distance/self.config['sound_speed_cm_s'])
            self.echo[echo_pin]=(start,start+pulse)

    def input(self, channel):
        pin=self._pin(channel)
        if self.modes.get(pin) != self.IN:
            raise RuntimeError(f'GPIO {pin} は入力に設定されていません。')
        self.clock.advance(.000001)
        start,end=self.echo.get(pin,(float('inf'),float('inf')))
        return self.HIGH if start <= self.clock.t < end else self.LOW

    def cleanup(self, channel=None):
        if channel is None:
            self.modes.clear(); self.levels.clear(); self.echo.clear()
        else:
            channels=channel if isinstance(channel,(list,tuple)) else [channel]
            for ch in channels:
                pin=self._pin(ch)
                self.modes.pop(pin,None); self.levels.pop(pin,None); self.echo.pop(pin,None)
        # Preserve selected numbering, allowing the initial cleanup in the photo.


class LegacyBridge:
    def __init__(self, receive, send, config):
        self.receive=receive; self.send=send; self.config=config
        self.clock=VirtualClock(self)
        self.events=[]
        self.throttle=float(config['throttle_stopped_pwm'])
        self.steering=float(config['steering_center_pwm'])
        self.frequency=float(config['pwm_frequency_hz'])
        self.writes=0

    def boundary(self):
        self.send({'kind':'tick','events':self.events,'comment':'実車互換Python実行中'})
        self.events=[]
        self.clock.grant(self.receive())

    def add_pwm(self, channel, on, off):
        c=self.config
        if type(channel) is not int or channel not in (c['steering_channel'],c['throttle_channel']):
            raise ValueError(f'未対応のPCA9685チャンネル: {channel}。hardware.jsonを確認してください。')
        if type(on) is not int or type(off) is not int or not 0<=on<=4095 or not 0<=off<=4096:
            raise ValueError('set_pwmのon/offは12bit値（off=4096は停止）です。')
        raw=(off-on)%4096
        disabled=(off==4096 or raw==0)
        # Convert pulse duration into calibrated counts at the reference frequency.
        value=raw*c['pwm_frequency_hz']/self.frequency
        if channel==c['throttle_channel']:
            self.throttle=float(c['throttle_stopped_pwm']) if disabled else value
        else:
            self.steering=float(c['steering_center_pwm']) if disabled else value
        decode_pwm(self.throttle,self.steering,c)  # fail rather than silently clip bad calibration
        event={'offset_s':self.clock.t-self.clock.start,'throttle_pwm':self.throttle,
               'steering_pwm':self.steering,'channel':channel,'on':on,'off':off,
               'frequency_hz':self.frequency}
        self.events.append(event); self.writes+=1
        if len(self.events)>500:
            raise RuntimeError('1フレームのPWM書込が500回を超えました。ループにsleepを入れてください。')
        self.clock.advance(.000001)

    def install(self):
        gpio=VirtualGPIO(self.clock,self.config)
        rpi=types.ModuleType('RPi'); rpi.__path__=[]; rpi.GPIO=gpio
        sys.modules['RPi']=rpi; sys.modules['RPi.GPIO']=gpio
        module=types.ModuleType('Adafruit_PCA9685')
        bridge=self
        class PCA9685:
            def __init__(self,address=0x40,**kwargs):
                if address!=bridge.config['pca9685_address']:
                    raise ValueError('PCA9685アドレスがhardware.jsonと違います。')
            def set_pwm_freq(self,freq_hz):
                if not isinstance(freq_hz,(int,float)) or not math.isfinite(freq_hz) or not 24<=freq_hz<=1600:
                    raise ValueError('PWM周波数は24〜1600Hzの有限値です。')
                bridge.frequency=float(freq_hz)
            def set_pwm(self,channel,on,off):
                bridge.add_pwm(channel,on,off)
            def set_all_pwm(self,on,off):
                self.set_pwm(bridge.config['throttle_channel'],on,off)
                self.set_pwm(bridge.config['steering_channel'],on,off)
        module.PCA9685=PCA9685
        sys.modules['Adafruit_PCA9685']=module
        # Keep unrelated formatting helpers, replacing only supported clock APIs.
        import time as real_time
        virtual_time=types.ModuleType('time')
        virtual_time.__dict__.update(real_time.__dict__)
        virtual_time.time=self.clock.now
        virtual_time.monotonic=self.clock.now
        virtual_time.perf_counter=self.clock.now
        virtual_time.time_ns=lambda: int(self.clock.now()*1e9)
        virtual_time.monotonic_ns=virtual_time.time_ns
        virtual_time.perf_counter_ns=virtual_time.time_ns
        virtual_time.sleep=self.clock.sleep
        sys.modules['time']=virtual_time
        def start_input(prompt=''):
            print(f'[SIM input→Enter] {prompt}')
            self.clock.advance(.000001)
            return ''
        builtins.input=start_input
