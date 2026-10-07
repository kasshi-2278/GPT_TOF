"""Measured sonar endpoints, with simulator pose used ONLY by the observer."""
import csv
import math
from collections import deque
from geometry import local_to_world

class SonarMap:
    def __init__(self, capacity=30000):
        self.points=deque(maxlen=capacity)
        self.track=deque(maxlen=6000)
        self.identity=None
        self.last_sample=None

    def clear(self):
        self.points.clear(); self.track.clear()
        # Do not immediately reinsert the last observation after clearing.

    def sample(self, sim):
        identity=(id(sim),id(sim.car),id(sim.world))
        if identity!=self.identity:
            self.clear();self.identity=identity;self.last_sample=None
        stamp=sim.sensor_observation_id
        if stamp==self.last_sample:return
        self.last_sample=stamp
        x,y,heading,time=sim.sensor_observation_pose
        self.track.append((x,y))
        for name,(forward,left,angle) in sim.config.sensor_specs().items():
            distance=sim.sensors.get(name)
            # Saturated readings mean no return, not a wall at maximum range.
            if distance is None or not math.isfinite(distance) or not 0<distance<sim.config.sensor_range_cm:
                continue
            ox,oy=local_to_world(x,y,heading,forward,left)
            px,py=local_to_world(ox,oy,heading+angle,distance,0)
            self.points.append((time,name,px,py,distance,ox,oy))

    def save_csv(self,path):
        with open(path,'w',newline='',encoding='utf-8-sig') as stream:
            writer=csv.writer(stream)
            writer.writerow(('time_s','sensor','x_cm','y_cm','distance_cm','sensor_x_cm','sensor_y_cm'))
            writer.writerows(self.points)
