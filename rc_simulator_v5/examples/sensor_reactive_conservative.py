"""Five-ultrasonic-sensor-only example. No map, pose, heading or route input.

Not a guarantee of collision-free driving. Assumes the UNCALIBRATED V5
30x18cm car and the sensor mounting in config.py. Stops instead of blindly
reversing because there is no directly rear-facing sensor in this setup.
Positive Handle = left, negative = right. All distances are centimeters.
Edit this file and click 再読込 in the simulator.
"""
import math

class Controller:
    def __init__(self):
        self.handle=0.0
        self.accel=0.0
        self.turn=0
        self.turn_elapsed=0.0
        self.stopped=False

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower,min(upper,value))

    def step(self, sensors, dt):
        keys=('Fr','FrLh','RrLh','FrRh','RrRh')
        if not (0 < dt <= .2) or any(k not in sensors or not isinstance(sensors[k],(int,float)) or
            not math.isfinite(sensors[k]) or sensors[k]<0 for k in keys):
            self.stopped=True
            return 0.0,0.0,'停止：センサー値が無効'
        f,fl,rl,fr,rr=[sensors[k] for k in keys]
        if f<22 or min(fl,fr)<18 or min(rl,rr)<10:
            self.stopped=True
        if self.stopped:
            self.accel=0.0
            return 0.0,self.handle,'安全停止：前方/側方が近すぎます。位置を確認してリセット'

        if self.turn==0 and f<105:
            left_space=.75*fl+.25*rl
            right_space=.75*fr+.25*rr
            self.turn=1 if left_space>right_space else -1
            self.turn_elapsed=0.0
        if self.turn:
            self.turn_elapsed+=dt
            if f>155 and self.turn_elapsed>.7:
                self.turn=0

        def pressure(distance, threshold):
            return max(0.0,(threshold-distance)/threshold)
        avoid=100*(pressure(fr,95)-pressure(fl,95)) + 80*(pressure(rr,55)-pressure(rl,55))
        if self.turn:
            target=self.turn*82
            # Close side walls can oppose, but not instantly reverse, a turn.
            target=self.clamp(target+avoid*.45,-100,100)
            comment='センサー回避：左旋回' if self.turn>0 else 'センサー回避：右旋回'
        else:
            target=self.clamp(avoid,-80,80)
            comment='センサー走行'
        self.handle+=self.clamp(target-self.handle,-130*dt,130*dt)
        target_accel=28.0-11.0*min(1,abs(self.handle)/82)
        if f<70:
            target_accel=min(target_accel,16.0)
        self.accel+=self.clamp(target_accel-self.accel,-90*dt,25*dt)
        return self.accel,self.handle,comment
