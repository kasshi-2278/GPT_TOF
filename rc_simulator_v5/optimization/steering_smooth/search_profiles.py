from optimization.evaluate_three_laps import ROOT
from optimization.evaluate_smooth import evaluate_smooth
import json
base=(ROOT/'optimization/steering_smooth/Previous_Best.py').read_text()
addon='''

_GapController=Controller
class Controller(_GapController):
    SMOOTH_RATE=RATE
    SMOOTH_JERK=JERK
    SMOOTH_FILTER=FILTER
    NEUTRAL_BAND=BAND
    def __init__(self):
        super().__init__()
        self.output_handle=0.0
        self.output_rate=0.0
        self.profile_target=0.0
    def step(self,sensors,dt):
        accel,desired,state=super().step(sensors,dt)
        if self.stopped:
            self.output_handle=desired;self.output_rate=0;self.profile_target=desired
            return accel,desired,state
        urgent=(sensors['Fr']<90 or min(sensors['FrLh'],sensors['FrRh'])<25 or
                min(sensors['RrLh'],sensors['RrRh'])<15)
        if urgent:
            self.output_rate=(desired-self.output_handle)/dt
            self.output_handle=desired;self.profile_target=desired
            return accel,desired,'接近回避優先：'+state
        goal=0.0 if abs(desired)<self.NEUTRAL_BAND else desired
        alpha=1.0 if self.SMOOTH_FILTER==0 else 1-math.exp(-dt/self.SMOOTH_FILTER)
        self.profile_target+=alpha*(goal-self.profile_target)
        error=self.profile_target-self.output_handle
        rate=clamp(error/dt,-self.SMOOTH_RATE,self.SMOOTH_RATE)
        self.output_rate+=clamp(rate-self.output_rate,-self.SMOOTH_JERK*dt,self.SMOOTH_JERK*dt)
        proposed=self.output_handle+self.output_rate*dt
        if error*(self.profile_target-proposed)<=0:proposed=self.profile_target
        proposed=clamp(proposed,-100,100)
        self.output_rate=(proposed-self.output_handle)/dt
        self.output_handle=proposed;self.handle=proposed
        return accel,proposed,'平滑操舵：'+state
'''
params=[(rate,jerk,tau,0) for rate in (100,120,140) for jerk in (800,1600,3200) for tau in (0,.04)]
params += [(120,1600,0,band) for band in (.5,1,2,3)]
rows=[]
for i,(rate,jerk,tau,band) in enumerate(params,1):
 s=base+addon.replace('SMOOTH_RATE=RATE',f'SMOOTH_RATE={rate}').replace('SMOOTH_JERK=JERK',f'SMOOTH_JERK={jerk}').replace('SMOOTH_FILTER=FILTER',f'SMOOTH_FILTER={tau}').replace('NEUTRAL_BAND=BAND',f'NEUTRAL_BAND={band}')
 p=ROOT/f'optimization/steering_smooth/candidates/smooth_{i:03}.py';p.write_text(s)
 r=evaluate_smooth(p,f'smooth_{i:03}',external=False)
 r.update(rate=rate,jerk=jerk,tau=tau,band=band)
 rows.append(r);print(i,r['completed_three_laps'],r['total_time_s'],r['collisions'],r['danger'],r['weave'],round(r['straight_fraction'],3),r['smooth_score'],flush=True)
 (ROOT/'optimization/steering_smooth/search.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
