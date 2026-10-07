"""The same deterministic simulation core is used by GUI and validation.

2D kinematic bicycle approximation located at the vehicle center. No tire
slip, suspension or ultrasonic reflections. Legacy PWM mode has an explicit
generic brake/neutral/reverse handshake (not a calibrated real ESC model).
"""
from __future__ import annotations
from dataclasses import dataclass
import math, random
from collections import deque
from config import *
from geometry import *
from controller_smooth import SmoothController
from controller_photo import decide as decide_photo
from pathlib import Path
from program_bridge import ProgramProcess, ProgramSpec, ProgramError
from virtual_hardware import load_hardware, decode_pwm, ESCModel

@dataclass
class Car:
    x: float
    y: float
    heading: float=0.0
    speed: float=0.0
    steer: float=0.0
    accel: float=0.0
    handle: float=0.0

    def polygon(self, config):
        return rectangle(self.x,self.y,self.heading,config.length_cm,config.width_cm)

    def move(self, config, dt):
        target=config.forward_cm_s*self.accel/100 if self.accel>=0 else config.reverse_cm_s*self.accel/100
        limit=config.braking_cm_s2 if abs(target)<abs(self.speed) or target*self.speed<0 else config.acceleration_cm_s2
        change=(target-self.speed)*min(1.0,config.motor_response_s_inv*dt)
        self.speed += clamp(change,-limit*dt,limit*dt)
        wanted=config.max_steer_deg*self.handle/100
        self.steer += clamp(wanted-self.steer,-config.servo_deg_s*dt,config.servo_deg_s*dt)
        yaw=-math.degrees(self.speed/config.wheelbase_cm*math.tan(math.radians(self.steer)))
        mid_heading=self.heading+yaw*dt/2
        dx,dy=direction(mid_heading)
        self.x += dx*self.speed*dt; self.y += dy*self.speed*dt
        self.heading=(self.heading+yaw*dt+180)%360-180

    def pwm(self):
        throttle=int(THROTTLE_STOPPED_PWM + (THROTTLE_FORWARD_PWM-THROTTLE_STOPPED_PWM)*self.accel/100) if self.accel>=0 else int(THROTTLE_STOPPED_PWM+(THROTTLE_STOPPED_PWM-THROTTLE_REVERSE_PWM)*self.accel/100)
        steer=int(STEERING_CENTER_PWM+(STEERING_LEFT_PWM-STEERING_CENTER_PWM)*self.handle/100) if self.handle>=0 else int(STEERING_CENTER_PWM+(STEERING_CENTER_PWM-STEERING_RIGHT_PWM)*self.handle/100)
        return throttle,steer

class Simulation:
    def __init__(self, world, config=None, seed=7, lap_limit=1, spawn_name='demo',
                 mode='program', hardware_path=None):
        self.world=world; self.config=config or VehicleConfig(); self.seed=seed
        self.lap_limit=lap_limit; self.mode=mode; self.spawn_name=spawn_name
        self.hardware_path=Path(hardware_path or Path(__file__).with_name('hardware.json'))
        self.hardware=load_hardware(self.hardware_path)
        self.program_spec=None; self.external=None; self.last_program_log=''
        self.program_timeout=1.0
        self.reset()

    @property
    def laps(self):
        return self.controller.laps if self.mode=='guide' and self.controller is not None else None

    def close_program(self):
        if self.external is not None:
            self.external.close()
            self.last_program_log=self.external.log_text()
            self.external=None

    def program_log(self):
        return self.external.log_text() if self.external is not None else self.last_program_log

    def reset(self):
        self.close_program()
        self.rng=random.Random(self.seed)
        p=self.world.data.get('spawns',{}).get(self.spawn_name,self.world.data['start'])
        self.car=Car(p['x'],p['y'],p['heading_deg'])
        self.controller=None
        if self.mode=='guide':
            route=self.world.data.get('demo_route',{}).get('points',[])
            if len(route)<4:
                raise ValueError('ガイド経路のないマップです。外部Pythonまたは手動を選択してください。')
            nearest=min(range(len(route)),key=lambda i: math.dist(route[i],(p['x'],p['y'])))
            self.controller=SmoothController(route[nearest:]+route[:nearest],self.config,self.lap_limit)
        self.steps=0; self.time=0.0; self.distance=0.0
        self.collision_id=None; self.finished=False; self.program_error=None; self.program_stopped=False
        self.state='Pythonファイルを選んでください' if self.mode=='program' and self.program_spec is None else '待機'
        self.sensors={}; self.hits={}; self.logs=[]; self.trail=deque(maxlen=20000)
        self.pwm_events=[]; self.esc=ESCModel(self.hardware)
        self.requested_accel=0.0; self.requested_handle=0.0
        self.last_raw_pwm=(self.hardware['throttle_stopped_pwm'],self.hardware['steering_center_pwm'])
        self._sampled_step=-1
        self.read_sensors()
        if self.world.collision(self.car.polygon(self.config)):
            raise ValueError('開始位置が壁・支柱に重なっています。')

    def load_program(self, path, interface='auto'):
        spec=ProgramSpec.inspect(path,interface)
        # Validate before abandoning a previously selected file.
        hardware=load_hardware(self.hardware_path)
        self.close_program(); self.hardware=hardware; self.program_spec=spec
        self.mode='program'; self.reset()
        self.state=f'選択済み: {spec.path.name} / {spec.interface}'
        return spec

    def reload_program(self):
        if self.program_spec is None:
            raise ValueError('先にPythonファイルを選択してください。')
        return self.load_program(self.program_spec.path,self.program_spec.interface)

    def invalidate_program(self):
        """Map edits discard a pending sensor sample, with vehicle pose retained."""
        self.close_program(); self._sampled_step=-1
        self.esc=ESCModel(self.hardware)
        self.car.speed=0; self.car.accel=0; self.car.handle=0
        self.state='環境変更：次回開始時にPythonを最初から実行'

    def stop_program(self, reason='ユーザー操作でPythonを停止'):
        self.close_program(); self.program_stopped=True
        self.car.speed=0; self.car.accel=0; self.car.handle=0
        self.state=reason

    def fail_program(self, error):
        self.program_error=str(error)
        self.stop_program('Pythonエラー停止：'+str(error).splitlines()[0])

    def check_program_health(self):
        if self.external is not None:
            try:
                self.external.check_watchdog()
            except ProgramError as exc:
                self.fail_program(exc)

    def read_sensors(self):
        c=self.car
        self.sensor_observation_id=getattr(self,"sensor_observation_id",0)+1
        self.sensor_observation_pose=(c.x,c.y,c.heading,self.time)
        for name,(forward,left,angle) in self.config.sensor_specs().items():
            origin=local_to_world(c.x,c.y,c.heading,forward,left)
            d,hit=self.world.raycast(origin,c.heading+angle,self.config.sensor_range_cm)
            measured=clamp(d+self.rng.gauss(0,self.config.sensor_noise_std_cm),0,self.config.sensor_range_cm) if self.config.sensor_noise_std_cm else d
            self.sensors[name]=measured; self.hits[name]=(origin,hit,d)

    def _motion(self, dt):
        if dt<=1e-12 or self.collision_id:
            return
        c=self.car; previous=(c.x,c.y,c.heading,c.speed,c.steer)
        c.move(self.config,dt)
        hit=self.world.collision(c.polygon(self.config))
        ex=self.world.extents
        if not (ex[0]-30<=c.x<=ex[2]+30 and ex[1]-30<=c.y<=ex[3]+30):
            hit='map_boundary'
        if hit:
            c.x,c.y,c.heading,_,c.steer=previous
            c.speed=0; c.accel=0
            self.collision_id=hit; self.state=f'衝突停止：{hit}'
            self.close_program()
        self.distance+=math.hypot(c.x-previous[0],c.y-previous[1])

    def _legacy_motion(self, events, dt):
        if not isinstance(events,list) or len(events)>500:
            raise ProgramError('PWMイベントが不正です。')
        # Validate entire batch before moving any physics.
        validated=[]; previous=0.0
        for event in events:
            offset=event.get('offset_s')
            if not isinstance(offset,(int,float)) or not math.isfinite(offset) or offset<previous-1e-9 or offset>dt+1e-8:
                raise ProgramError('PWMイベントの時刻が不正です。')
            accel,handle=decode_pwm(event['throttle_pwm'],event['steering_pwm'],self.hardware)
            validated.append((max(previous,min(dt,offset)),accel,handle,event))
            previous=offset
        at=0.0
        for offset,accel,handle,event in validated:
            self._motion(offset-at); at=offset
            if self.collision_id: return
            self.requested_accel=accel; self.requested_handle=handle
            applied=self.esc.apply(accel,self.time+offset)
            self.car.accel=applied; self.car.handle=handle
            self.last_raw_pwm=(event['throttle_pwm'],event['steering_pwm'])
            if len(self.pwm_events)<200000:
                self.pwm_events.append(dict(time_s=round(self.time+offset,9),
                    **event,requested_accel=accel,applied_accel=applied,
                    Handle=handle,esc_stage=self.esc.stage))
        self._motion(dt-at)

    def step(self, manual=(0.0,0.0), *, block=True):
        """Return False only when an external Python response is still pending."""
        if self.collision_id or self.finished or self.program_error or self.program_stopped:
            return True
        dt=1/PHYSICS_HZ; c=self.car
        legacy=self.mode=='program' and self.program_spec is not None and self.program_spec.interface=='legacy'
        period=1 if legacy else PHYSICS_HZ//CONTROL_HZ
        events=[]
        try:
            if self.steps%period==0:
                if self._sampled_step!=self.steps:
                    self.read_sensors(); self._sampled_step=self.steps
                if self.mode=='program':
                    if self.program_spec is None:
                        self.state='Pythonファイル未選択'; return False
                    if self.external is None:
                        self.external=ProgramProcess(self.program_spec,timeout=self.program_timeout,hardware_path=self.hardware_path)
                    response=self.external.step(self.sensors,self.time,dt if legacy else 1/CONTROL_HZ,block=block)
                    if response is None:
                        return False
                    if response['kind']=='finished':
                        self.finished=True; c.speed=0; c.accel=0; c.handle=0
                        self.state=response.get('comment','Python終了'); self.close_program()
                        self.logs.append(self.record()); return True
                    self.state=response.get('comment','外部Python（センサーのみ）')
                    if legacy:
                        if response['kind']!='tick':
                            raise ProgramError('実車互換モードの応答が不正です。')
                        events=response['events']
                    else:
                        a,h=response.get('accel'),response.get('handle')
                        if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or abs(v)>100 for v in (a,h)):
                            raise ProgramError('Accel/Handleの値が不正です。')
                        c.accel=a; c.handle=h
                        self.requested_accel=a; self.requested_handle=h
                elif self.mode=='guide':
                    c.accel,c.handle,self.state=self.controller.decide(self.sensors,c.x,c.y,c.heading,c.speed,1/CONTROL_HZ)
                    self.finished=self.controller.finished
                elif self.mode=='photo':
                    c.accel,c.handle,self.state=decide_photo(self.sensors)
                elif self.mode=='manual':
                    if len(manual)!=2 or not all(math.isfinite(v) for v in manual):
                        raise ValueError('Manual commands must be finite (Accel, Handle)')
                    c.accel,c.handle=[clamp(v,-100,100) for v in manual]
                    self.state='手動運転'
                else:
                    raise ValueError(f'Unknown mode {self.mode}')
            if self.finished:
                c.accel=0; c.speed=0
                return True
            if legacy:
                self._legacy_motion(events,dt)
            else:
                self._motion(dt)
            self.steps+=1; self.time=self.steps/PHYSICS_HZ
            if self.steps%6==0:
                self.trail.append((c.x,c.y))
                if len(self.logs)<200000:
                    self.logs.append(self.record())
            return True
        except (ProgramError,OSError,ValueError,KeyError,TypeError) as exc:
            if self.mode=='program':
                self.fail_program(exc)
                self.logs.append(self.record())
                return True
            raise

    def record(self):
        c=self.car; throttle,steering=c.pwm()
        spec=self.program_spec
        return dict(time_s=round(self.time,6),x_cm=c.x,y_cm=c.y,heading_deg=c.heading,
                    speed_cm_s=c.speed,steer_deg=c.steer,Accel=c.accel,Handle=c.handle,
                    throttle_pwm=throttle,steering_pwm=steering,
                    raw_throttle_pwm=self.last_raw_pwm[0],raw_steering_pwm=self.last_raw_pwm[1],
                    requested_accel=self.requested_accel,esc_stage=self.esc.stage,
                    source=self.mode,program=spec.path.name if self.mode=='program' and spec else '',
                    interface=spec.interface if self.mode=='program' and spec else '',
                    program_sha256=spec.sha256 if self.mode=='program' and spec else '',
                    state=self.state,laps=self.laps if self.laps is not None else '',**self.sensors)

    def run_seconds(self,seconds):
        for _ in range(int(round(seconds*PHYSICS_HZ))):
            if self.finished or self.collision_id or self.program_error or self.program_stopped:
                break
            if not self.step(block=True):
                break
        return self

    def close(self):
        self.close_program()
