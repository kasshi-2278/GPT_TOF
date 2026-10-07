"""Known-route visual demo. Requires true simulated pose, NOT sensor-only autonomy.

The route is supplied by the selected map instead of being hard-coded to V3.
All smoothing limits are per second; GUI frame rate does not enter the policy.
"""
from __future__ import annotations
from bisect import bisect_right
import math
from geometry import clamp

class SmoothController:
    def __init__(self, points, vehicle, lap_limit=1):
        self.path=[tuple(p) for p in points]
        self.vehicle=vehicle
        self.lap_limit=lap_limit
        self.cumulative=[0.0]
        for i in range(len(self.path)):
            self.cumulative.append(self.cumulative[-1]+math.dist(self.path[i],self.path[(i+1)%len(self.path)]))
        self.total_length=self.cumulative[-1]
        self.reset()

    def reset(self):
        self.index=0
        self.handle=0.0
        self.accel=0.0
        self.laps=0
        self.target=self.path[0]
        self.error_cm=0.0
        self.finished=False

    def point_at(self, distance):
        distance %= self.total_length
        i=min(len(self.path)-1,bisect_right(self.cumulative,distance)-1)
        length=self.cumulative[i+1]-self.cumulative[i]
        ratio=(distance-self.cumulative[i])/length
        a=self.path[i]; b=self.path[(i+1)%len(self.path)]
        return (a[0]+ratio*(b[0]-a[0]),a[1]+ratio*(b[1]-a[1]))

    def decide(self,sensors,x,y,heading,speed,dt):
        if dt<=0 or not math.isfinite(dt): raise ValueError('dt must be positive and finite')
        n=len(self.path)
        idx=min(range(max(0,self.index-4),self.index+65),
                key=lambda k: math.dist(self.path[k%n],(x,y)))
        self.index=max(self.index,idx)
        self.laps=self.index//n
        self.error_cm=math.dist(self.path[self.index%n],(x,y))
        lookahead=32.0+0.55*abs(speed)
        progress=self.cumulative[self.index%n]+self.laps*self.total_length
        self.target=self.point_at(progress+lookahead)
        dx,dy=self.target[0]-x,self.target[1]-y
        bearing=math.degrees(math.atan2(dx,dy))
        alpha=math.radians((bearing-heading+180)%360-180)
        distance=max(1.0,math.hypot(dx,dy))
        steer=-math.degrees(math.atan2(2*self.vehicle.wheelbase_cm*math.sin(alpha),distance))
        steer=clamp(steer,-self.vehicle.max_steer_deg,self.vehicle.max_steer_deg)
        target_handle=100*steer/self.vehicle.max_steer_deg
        # Continuous speed profile, avoiding piecewise jumps from V3.
        target_speed=34.0/(1.0+0.035*abs(steer))
        state='ガイド走行（自己位置使用）'
        front=sensors.get('Fr')
        if not isinstance(front,(float,int)) or not math.isfinite(front) or front<0:
            target_speed=0.0; state='停止：前方センサー無効'
        elif front < max(14.0,8.0+speed*speed/(2*self.vehicle.braking_cm_s2)):
            target_speed=0.0; state='停止：前方に障害物'
        if self.error_cm>100:
            target_speed=0.0; state='停止：ガイド経路から逸脱'
        if self.lap_limit and self.laps>=self.lap_limit:
            target_speed=0.0; state='周回完了・減速中'
            if abs(speed)<0.08: self.finished=True; state='周回完了'
        target_accel=100*target_speed/self.vehicle.forward_cm_s
        self.handle += clamp(target_handle-self.handle,-160*dt,160*dt)
        limit=160 if target_accel<self.accel else 60
        self.accel += clamp(target_accel-self.accel,-limit*dt,limit*dt)
        return clamp(self.accel,-100,100),clamp(self.handle,-100,100),state

    def route_points(self):
        return self.path
