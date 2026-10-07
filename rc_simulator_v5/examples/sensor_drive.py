"""Loadable five-ultrasonic-sensor-only controller for RC Simulator V5.

Inputs: Fr, FrLh, RrLh, FrRh, RrRh [cm], dt [seconds].
Outputs: Accel, Handle [-100,100], text. Handle > 0 turns LEFT.
No world coordinates, compass heading, odometry, route or map are used.

A local line fitted to two right ultrasonic returns estimates right-wall
orientation and clearance. This ASSUMES both returns lie on the same wall;
corners, angled reflections and beam dropouts can violate that assumption.
The front/right sensors override the demand near obstacles. Speed and
steering commands change progressively. Emergency stop is latched, because
blind reversing is unsafe with no directly rear-facing sensor.

The default direction is counterclockwise, following the right exterior wall.
These parameters are for the 28x20cm V5 vehicle and its
45/90-degree HC-SR04 sensor mounts. Exact mounting positions are provisional.
This is an editable starting point, not a proof of safe driving everywhere.
"""
import math
from config import VehicleConfig

# Mounts in the VEHICLE frame (forward, left), not global map coordinates.
_mounts=VehicleConfig().sensor_specs()
FRONT_SENSOR_FORWARD_CM,SENSOR_LEFT_OFFSET_CM,_front_angle=_mounts['FrLh']
REAR_SENSOR_FORWARD_CM,_side_offset,_side_angle=_mounts['RrLh']
FRONT_LEFT_ANGLE_DEG=-_front_angle
REAR_LEFT_ANGLE_DEG=-_side_angle
RANGE_CM=250.0
TARGET_LEFT_CLEARANCE_CM=44.0

class Controller:
    def __init__(self):
        self.handle=0.0
        self.accel=0.0
        self.stopped=False

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower,min(upper,value))

    def step(self, sensors, dt):
        # Mirror the inputs into the left-wall policy, then mirror its output.
        # Keep internal steering state in that frame, including latched stops.
        mirrored=dict(sensors)
        for left,right in [('FrLh','FrRh'),('RrLh','RrRh')]:
            mirrored.pop(left,None); mirrored.pop(right,None)
            if right in sensors: mirrored[left]=sensors[right]
            if left in sensors: mirrored[right]=sensors[left]
        accel,handle,state=self._step_left(mirrored,dt)
        state=state.replace('左壁追従','右壁追従').replace('左旋回','反転旋回').replace('右旋回','左旋回').replace('反転旋回','右旋回')
        return accel,-handle,state

    def _step_left(self, sensors, dt):
        names=('Fr','FrLh','RrLh','FrRh','RrRh')
        if not isinstance(dt,(int,float)) or not math.isfinite(dt) or not 0<dt<=.2:
            self.stopped=True
            return 0.0,0.0,'安全停止：dtが不正'
        if any(name not in sensors or not isinstance(sensors[name],(int,float)) or
               not math.isfinite(sensors[name]) or sensors[name]<0 for name in names):
            self.stopped=True
            return 0.0,0.0,'安全停止：センサー値が不正'
        f,fl,rl,fr,rr=[sensors[name] for name in names]
        if f<22 or min(fl,fr)<18 or min(rl,rr)<10:
            self.stopped=True
        if self.stopped:
            self.accel=0.0
            return 0.0,self.handle,'安全停止：周囲を確認し、位置変更/リセットしてください'

        # Echo endpoints in the vehicle's own forward/left frame.
        fa=FRONT_SENSOR_FORWARD_CM+fl*math.cos(math.radians(FRONT_LEFT_ANGLE_DEG))
        la=SENSOR_LEFT_OFFSET_CM+fl*math.sin(math.radians(FRONT_LEFT_ANGLE_DEG))
        fb=REAR_SENSOR_FORWARD_CM+rl*math.cos(math.radians(REAR_LEFT_ANGLE_DEG))
        lb=SENSOR_LEFT_OFFSET_CM+rl*math.sin(math.radians(REAR_LEFT_ANGLE_DEG))
        if fl<RANGE_CM-2 and rl<RANGE_CM-2 and fa-fb>15:
            slope=(la-lb)/(fa-fb)
            local_wall_angle=math.degrees(math.atan(slope))
            clearance=lb-slope*fb
            demand=self.clamp(3.0*local_wall_angle+1.2*(clearance-TARGET_LEFT_CLEARANCE_CM),-90,90)
        else:
            demand=0.0
        # Right wall too close -> turn left; left wall too close -> turn right.
        demand+=95*max(0,(55-rr)/55)+95*max(0,(85-fr)/85)
        demand-=110*max(0,(28-rl)/28)+120*max(0,(35-fl)/35)
        state='左壁追従（超音波のみ）'
        if f<85:
            left_space=.75*fl+.25*rl
            right_space=.75*fr+.25*rr
            demand=95 if left_space>right_space else -95
            state='前方回避：左旋回' if demand>0 else '前方回避：右旋回'
        demand=self.clamp(demand,-100,100)
        # Rates are per SECOND, independent of render FPS or simulation multiplier.
        self.handle+=self.clamp(demand-self.handle,-130*dt,130*dt)
        target_accel=28-12*min(1,abs(self.handle)/90)
        self.accel+=self.clamp(target_accel-self.accel,-90*dt,25*dt)
        return self.accel,self.handle,state
