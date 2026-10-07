"""Read-only lap observer for course_1030. Never supplied to a controller.

A lap visits the same ordered regions as the existing one-lap evaluator,
then crosses the spawn's horizontal timing line southbound within +/-60 cm.
All three laps use the same line. Vehicle/controller state is never reset.
"""
import math

class LapTimer:
    def __init__(self,sim,target_laps=3):
        if not isinstance(target_laps,int) or target_laps<1:
            raise ValueError('target_laps must be a positive integer')
        self.target_laps=target_laps
        self.start=(sim.car.x,sim.car.y)
        self.start_time=sim.time
        self.previous=(sim.car.x,sim.car.y,sim.car.heading,sim.time,sim.distance)
        self.car_identity=id(sim.car)
        self.gate=0
        self.lap_times=[]
        self.crossing_times=[]
        self.last_crossing_time=sim.time
        self.last_crossing_distance=sim.distance

    @property
    def completed(self):return len(self.lap_times)>=self.target_laps

    @property
    def total_time(self):return sum(self.lap_times) if self.completed else None

    def sample(self,sim):
        if id(sim.car)!=self.car_identity or sim.time<self.previous[3]:
            self.__init__(sim,self.target_laps)
            return False
        current=(sim.car.x,sim.car.y,sim.car.heading,sim.time,sim.distance)
        if self.completed or sim.time<=self.previous[3]:
            self.previous=current
            return False
        x,y,heading,t,distance=current
        gates=(x>850 and y<200, x<320 and 200<y<330,
               x>800 and 350<y<470, x>800 and y>500,
               x<180 and y>480)
        if self.gate<len(gates) and gates[self.gate]:self.gate+=1
        px,py,ph,pt,pd=self.previous
        self.previous=current
        if self.gate!=len(gates) or not py>self.start[1]>=y:
            return False
        ratio=(py-self.start[1])/(py-y)
        crossing_x=px+ratio*(x-px)
        crossing_heading=ph+ratio*((heading-ph+180)%360-180)
        crossing_distance=pd+ratio*(distance-pd)
        if (abs(crossing_x-self.start[0])>60 or
                abs((crossing_heading+180)%360-180)<120 or
                crossing_distance-self.last_crossing_distance<3000):
            return False
        crossing_time=pt+ratio*(t-pt)
        self.lap_times.append(crossing_time-self.last_crossing_time)
        self.crossing_times.append(crossing_time)
        self.last_crossing_time=crossing_time
        self.last_crossing_distance=crossing_distance
        self.gate=0
        return True
