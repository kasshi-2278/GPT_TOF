"""Observer isolation, projection and continuous-lap timing regressions."""
import copy
from types import SimpleNamespace
import unittest
from config import VehicleConfig
from simulation import Simulation
from world import World
from vehicle_view import perspective_columns,sensor_segments
from lap_timer import LapTimer

class ObserverTests(unittest.TestCase):
    def test_view_does_not_sample_sensors_or_change_random_state(self):
        sim=Simulation(World.load(),VehicleConfig(sensor_noise_std_cm=2),mode='manual')
        try:
            before=(copy.deepcopy(sim.sensors),copy.deepcopy(sim.hits),sim.rng.getstate(),sim.time,sim.distance)
            perspective_columns(sim.world,sim.car,sim.config,count=32)
            sensor_segments(sim.config,sim.sensors)
            after=(sim.sensors,sim.hits,sim.rng.getstate(),sim.time,sim.distance)
            self.assertEqual(before,after)
        finally:sim.close()

    def test_sensor_local_coordinates_match_mounts(self):
        result={n:(a,b,d) for n,a,b,d in sensor_segments(VehicleConfig(),{'Fr':50,'RrLh':50})}
        self.assertEqual(result['Fr'],((12,0),(62,0),50))
        self.assertAlmostEqual(result['RrLh'][1][0],7)
        self.assertAlmostEqual(result['RrLh'][1][1],60)

    def test_projection_corrects_fisheye_of_flat_wall(self):
        data={'schema_version':1,'units':'cm','extents_cm':[-500,-100,500,200],
              'start':{'x':0,'y':0,'heading_deg':0},'walls':[{'id':'w','a':[-500,100],'b':[500,100],
              'thickness_cm':2,'color':'red'}],'posts':[],'zones':[]}
        world=World(data);car=SimpleNamespace(x=0,y=0,heading=0)
        columns=perspective_columns(world,car,VehicleConfig(),count=21)
        for depth,color,hit in columns:
            self.assertTrue(hit);self.assertEqual(color,'red');self.assertAlmostEqual(depth,87)

class LapTests(unittest.TestCase):
    def sim(self):
        return SimpleNamespace(car=SimpleNamespace(x=85,y=200,heading=-180),time=0,distance=0)

    def update(self,sim,timer,x,y,time,distance,heading=-180):
        sim.car.x=x;sim.car.y=y;sim.car.heading=heading
        sim.time=time;sim.distance=distance
        return timer.sample(sim)

    def lap(self,sim,timer,offset,distance_offset):
        samples=[(900,150,10,900),(250,280,20,1800),(900,400,30,2400),
                 (900,550,40,3000),(100,500,50,3400),(85,210,60,3800),(85,190,62,3820)]
        for x,y,t,d in samples:self.update(sim,timer,x,y,t+offset,d+distance_offset)

    def test_three_laps_without_car_reset_and_interpolated_crossings(self):
        sim=self.sim();timer=LapTimer(sim);car_id=id(sim.car)
        for i in range(3):self.lap(sim,timer,i*62,i*3820)
        self.assertEqual(id(sim.car),car_id)
        self.assertTrue(timer.completed)
        self.assertEqual(timer.lap_times,[61,62,62])
        self.assertEqual(timer.total_time,185)

    def test_shortcut_near_start_does_not_count(self):
        sim=self.sim();timer=LapTimer(sim)
        self.update(sim,timer,85,210,100,4000)
        self.update(sim,timer,85,190,102,4020)
        self.assertEqual(timer.lap_times,[])
        self.assertIsNone(timer.total_time)

    def test_north_heading_does_not_count_southbound_crossing(self):
        sim=self.sim();timer=LapTimer(sim)
        for x,y,t,d in [(900,150,10,900),(250,280,20,1800),(900,400,30,2400),
                        (900,550,40,3000),(100,500,50,3400),(85,210,60,3800)]:
            self.update(sim,timer,x,y,t,d,heading=0)
        self.update(sim,timer,85,190,62,3820,heading=0)
        self.assertEqual(timer.lap_times,[])

    def test_simulation_reset_discards_old_timing(self):
        sim=self.sim();timer=LapTimer(sim);self.lap(sim,timer,0,0)
        sim.car=SimpleNamespace(x=85,y=200,heading=-180);sim.time=0;sim.distance=0
        timer.sample(sim)
        self.assertEqual(timer.lap_times,[]);self.assertEqual(timer.gate,0)

if __name__=='__main__':unittest.main()
