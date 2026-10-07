"""Private worker entry point. Do not start this file directly.

User code executes here, never in the Tk GUI. Trusted local code only.
"""
from __future__ import annotations
import hashlib
import inspect
import io
import json
import math
from numbers import Real
from pathlib import Path
import sys
import tokenize
import traceback
import types
from virtual_hardware import LegacyBridge, load_hardware

# Save pipes before replacing user's print/stdin environment.
_WIRE_IN=sys.stdin
_WIRE_OUT=sys.stdout
_LOG=sys.stderr

def send(obj):
    _WIRE_OUT.write(json.dumps(obj,ensure_ascii=False,allow_nan=False)+'\n')
    _WIRE_OUT.flush()

def receive():
    line=_WIRE_IN.readline(16385)
    if not line:
        raise SystemExit(0)
    if len(line)>16384:
        raise ValueError('Input message too large')
    return json.loads(line)

class LogWriter(io.TextIOBase):
    """Avoid an unbounded line when a program prints without a newline."""
    def __init__(self):
        self.buffered=''
        self.total=0
        self.limited=False
    def writable(self): return True
    def isatty(self): return False
    @property
    def encoding(self): return 'utf-8'
    def write(self,text):
        text=str(text)
        original_len=len(text)
        if self.total>=2_000_000:
            if not self.limited:
                _LOG.write('[SIM] 出力が2MBを超えたため以降のprintを省略します。\n'); _LOG.flush()
                self.limited=True
            return original_len
        text=text[:max(0,2_000_000-self.total)]
        self.total+=len(text); self.buffered+=text
        while '\n' in self.buffered or len(self.buffered)>=2048:
            pos=self.buffered.find('\n')
            n=pos+1 if 0<=pos<2048 else min(2048,len(self.buffered))
            part,self.buffered=self.buffered[:n],self.buffered[n:]
            _LOG.write(part if part.endswith('\n') else part+'\n'); _LOG.flush()
        return original_len
    def flush(self):
        if self.buffered:
            _LOG.write(self.buffered+'\n'); _LOG.flush(); self.buffered=''


def load_source(path, module_name, expected_hash):
    if hashlib.sha256(path.read_bytes()).hexdigest()!=expected_hash:
        raise RuntimeError('実行直前に.pyが変更されました。再読込してください。')
    with tokenize.open(path) as stream:
        source=stream.read()
    module=types.ModuleType(module_name)
    module.__file__=str(path)
    module.__package__=None
    sys.modules[module_name]=module
    sys.path.insert(0,str(path.parent))
    sys.argv=[str(path)]
    exec(compile(source,str(path),'exec'),module.__dict__)
    return module


def command_result(value):
    if isinstance(value,dict):
        accel=value.get('Accel',value.get('accel'))
        handle=value.get('Handle',value.get('handle'))
        comment=value.get('comment','外部Python（センサーのみ）')
    elif isinstance(value,(tuple,list)) and len(value) in (2,3):
        accel,handle=value[:2]
        comment=value[2] if len(value)==3 else '外部Python（センサーのみ）'
    else:
        raise ValueError('戻り値は (Accel, Handle[, comment]) またはdictにしてください。')
    for name,number in (('Accel',accel),('Handle',handle)):
        if isinstance(number,bool) or not isinstance(number,Real) or not math.isfinite(float(number)) or not -100<=number<=100:
            raise ValueError(f'{name} は -100〜100 の有限数です: {number!r}')
    return {'kind':'result','accel':float(accel),'handle':float(handle),'comment':str(comment)[:300]}


def run_function(path, digest):
    module=load_source(path,'rc_user_program',digest)
    if hasattr(module,'Controller'):
        controller=module.Controller()
        fn=getattr(controller,'step',None) or getattr(controller,'decide',None)
    else:
        fn=getattr(module,'decide',None)
    if not callable(fn):
        raise ValueError('decide(sensors[, dt]) または Controller.step(sensors, dt) を定義してください。実車ループは実車互換方式を選びます。')
    signature=inspect.signature(fn)
    try:
        signature.bind({},.05)
        takes_dt=True
    except TypeError:
        try:
            signature.bind({})
            takes_dt=False
        except TypeError as exc:
            raise ValueError('センサーとdt以外の必須引数は渡せません。座標/方位は利用できません。') from exc
    send({'kind':'ready','interface':'function'})
    while True:
        request=receive()
        sensors=dict(request['sensors'])
        value=fn(sensors,float(request['dt'])) if takes_dt else fn(sensors)
        send(command_result(value))


def run_legacy(path,digest,hardware):
    bridge=LegacyBridge(receive,send,hardware)
    send({'kind':'ready','interface':'legacy'})
    bridge.clock.grant(receive())
    bridge.install()
    load_source(path,'__main__',digest)
    send({'kind':'finished','comment':'Pythonプログラムが終了しました。'})


def main():
    logger=LogWriter()
    sys.stdout=logger; sys.stderr=logger
    try:
        path=Path(sys.argv[1]).resolve(); interface=sys.argv[2]
        config=load_hardware(sys.argv[3]); digest=sys.argv[4]
        if interface=='function':
            run_function(path,digest)
        elif interface=='legacy':
            run_legacy(path,digest,config)
        else:
            raise ValueError('Unknown interface')
    except SystemExit as exc:
        try:
            if exc.code in (0,None):
                send({'kind':'finished','comment':'Pythonが正常終了しました。'})
            else:
                send({'kind':'error','error':f'Python終了コード: {exc.code}'})
        except (OSError,ValueError): pass
    except BaseException as exc:
        try:
            send({'kind':'error','error':f'{type(exc).__name__}: {exc}',
                  'traceback':traceback.format_exc()[-12000:]})
        except (OSError,ValueError): pass
    finally:
        logger.flush()

if __name__=='__main__':
    main()
