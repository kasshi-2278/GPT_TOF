"""Decision rules transcribed from the user's photos, not hardware execution.

Independent `if` statements and Python `and` / `or` precedence are retained.
The uploaded .py file was not available; this is a photographic transcription.
It is NOT tuned for the large course and does not model ESC reverse pulses.
"""
import math

def decide(sensors):
    names=('Fr','FrLh','RrLh','FrRh','RrRh')
    if any(n not in sensors or not isinstance(sensors[n],(int,float))
           or not math.isfinite(sensors[n]) or sensors[n]<0 for n in names):
        return 0,0,'停止：センサー無効'
    Fr,FrLh,RrLh,FrRh,RrRh=[sensors[n] for n in names]
    Accel,Handle,comment=40,0,'(1) 直進'
    if RrLh<30 or FrLh<80:
        Accel,Handle,comment=40,-50,'(2) 少し右折'
    if RrRh<30 or FrRh<80:
        Accel,Handle,comment=40,50,'(3) 少し左折'
    if Fr<100 and RrRh+FrRh>RrLh+FrLh:
        Accel,Handle,comment=30,-100,'(4) だいぶ右折'
    if Fr<100 and RrRh+FrRh<=RrLh+FrLh:
        Accel,Handle,comment=30,100,'(5) だいぶ左折'
    if Fr<35 or (FrRh<35 and min(FrRh,RrRh)<=min(FrLh,RrLh)):
        Accel,Handle,comment=-50,-40,'(6) 後退の右旋回'
    if Fr<35 or (FrLh<35 and min(FrRh,RrRh)>min(FrLh,RrLh)):
        Accel,Handle,comment=-50,40,'(7) 後退の左旋回'
    return Accel,Handle,comment
