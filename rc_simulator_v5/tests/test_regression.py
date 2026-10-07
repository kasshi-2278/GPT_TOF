from __future__ import annotations
import argparse, csv, dataclasses, json, math, platform, sys, time, unittest
from copy import deepcopy
from pathlib import Path
from config import VehicleConfig, PHYSICS_HZ
from geometry import *
from simulation import Simulation, Car
from world import World, DEFAULT_MAP
from controller_photo import decide as photo_decide

ROOT=Path(__file__).resolve().parent

def simple_world(walls=None,posts=None,zones=None):
    data=deepcopy(World.load().data)
    data['walls']=walls or []; data['posts']=posts or []; data['zones']=zones or []
    # This fixture drives north toward test walls, independent of demo direction.
    data['start']['heading_deg']=0
    data['spawns']['demo']['heading_deg']=0
    return World(data)

def wall(a,b,thickness=2):
    return dict(id='test_wall',a=a,b=b,thickness_cm=thickness,color='white',solid=True)

class GeometryTests(unittest.TestCase):
    def test_left_sensor_transform(self):
        self.assertEqual(local_to_world(0,0,0,0,6),(-6.0,0.0))
    def test_east_forward_transform(self):
        p=local_to_world(0,0,90,15,0); self.assertAlmostEqual(p[0],15); self.assertAlmostEqual(p[1],0)
    def test_ray_perpendicular(self):
        self.assertAlmostEqual(ray_segment((0,0),(0,1),(-5,100),(5,100),250),100)
    def test_ray_collinear(self):
        self.assertAlmostEqual(ray_segment((0,0),(0,1),(0,10),(0,20),250),10)
    def test_ray_circle(self):
        self.assertAlmostEqual(ray_circle((0,0),(0,1),(0,100),5,250),95)
    def test_polygon_containment(self):
        self.assertTrue(polygons_overlap(rectangle(0,0,0,40,40),rectangle(0,0,45,2,2)))
    def test_polygon_tangency(self):
        self.assertTrue(polygons_overlap(rectangle(0,0,0,10,10),rectangle(10,0,0,10,10)))
    def test_separated_polygons(self):
        self.assertFalse(polygons_overlap(rectangle(0,0,0,10,10),rectangle(11,0,0,10,10)))
    def test_wall_inside_car(self):
        w=simple_world([wall((-1,0),(1,0))]); self.assertEqual(w.collision(rectangle(0,0,0,30,18)),'test_wall')
    def test_post_inside_car(self):
        w=simple_world(posts=[dict(id='p',center=[0,0],radius_cm=2)])
        self.assertEqual(w.collision(rectangle(0,0,0,30,18)),'p')
    def test_ray_from_inside(self):
        w=simple_world([wall((-10,0),(10,0))]); self.assertEqual(w.raycast((0,0),0,250)[0],0)
    def test_metric_sensor_range(self):
        w=simple_world([wall((-200,100),(200,100))]); d,_=w.raycast((0,15),0,250)
        self.assertAlmostEqual(d,84.0)
    def test_maximum_range(self):
        w=simple_world(); self.assertEqual(w.raycast((30,40),45,250)[0],250)

class MapTests(unittest.TestCase):
    def setUp(self): self.world=World.load(); self.data=self.world.data
    def test_width_chain(self):
        self.assertEqual(sum(self.data['dimensions']['horizontal_chain_cm']),1030)
        self.assertEqual(self.world.extents,[0,0,1030,610])
    def test_vertical_intervals(self):
        ys=self.data['dimensions']['main_horizontal_y_cm']
        self.assertEqual([b-a for a,b in zip(ys,ys[1:])],[140]*4)
    def test_parking_dimensions(self):
        for name in ('P1','P2','P3'):
            poly=next(z['polygon'] for z in self.data['zones'] if z['id']==name)
            x,y,xx,yy=bounds(poly); self.assertEqual(xx-x,124); self.assertEqual(yy-y,50)
    def test_orange_removed_shortcut_open(self):
        self.assertFalse(any(z['id']=='orange' for z in self.data['zones']))
        self.assertIsNone(self.world.collision(rectangle(697,330,0,30,18)))
    def test_post_count(self): self.assertEqual(len(self.data['posts']),46)
    def test_start_markers_nonphysical(self):
        self.assertEqual([m['x'] for m in self.data['start_markers']],[250,430,610])
        self.assertFalse(any(w['id'].startswith('start') for w in self.data['walls']))
    def test_parking_mouth_open(self):
        d,_=self.world.raycast((244,80),180,250); self.assertAlmostEqual(d,79.05)
    def test_colored_floor_not_blocking(self):
        d,_=self.world.raycast((697,330),0,250); self.assertAlmostEqual(d,139.05)
    def test_colored_zone_can_be_fixed_obstacle(self):
        self.data['zones'].append(dict(id='test_solid_zone',polygon=[[650,300],[745,300],[745,360],[650,360]],solid=True))
        other=World(self.data); self.assertEqual(other.raycast((697,330),0,250)[0],0)
    def test_invalid_units_rejected(self):
        self.data['units']='px'
        with self.assertRaises(ValueError): World(self.data)
    def test_invalid_wall_rejected(self):
        self.data['walls'][0]['b']=self.data['walls'][0]['a']
        with self.assertRaises(ValueError): World(self.data)
    def test_save_reload(self):
        import tempfile
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'roundtrip.json'; self.world.save(p)
            self.assertEqual(World.load(p).data,self.world.data)

class SimulationTests(unittest.TestCase):
    def test_front_half_hcsr04_geometry(self):
        cfg=VehicleConfig();specs=cfg.sensor_specs()
        self.assertEqual(specs['Fr'],(12.0,0.0,0.0))
        self.assertEqual(cfg.length_cm/2-specs['Fr'][0],2.0)
        self.assertEqual({name:spec[2] for name,spec in specs.items()},
                         dict(Fr=0,FrLh=-45,RrLh=-90,FrRh=45,RrRh=90))
        for forward,left,angle in specs.values():
            self.assertGreater(forward,0)
            self.assertLessEqual(forward,cfg.length_cm/2)
            self.assertLessEqual(abs(left),cfg.width_cm/2)
        for name in ('FrLh','RrLh'):
            right=specs[name.replace('Lh','Rh')]
            self.assertEqual(right,(specs[name][0],-specs[name][1],-specs[name][2]))
        w=simple_world([wall((-100,-100),(-100,400))])
        origin=local_to_world(0,0,0,*specs['RrLh'][:2])
        self.assertAlmostEqual(w.raycast(origin,-90,250)[0],89)
    def test_counterclockwise_defaults(self):
        data=World.load().data
        self.assertEqual(data['spawns']['demo']['heading_deg'],-180)
        for name in ('start1','start2','start3'):
            self.assertEqual(data['spawns'][name]['heading_deg'],90)
        points=data['demo_route']['points']
        self.assertGreater(sum(a[0]*b[1]-b[0]*a[1] for a,b in edges(points)),0)

    def test_sensor_controller_mirrors_steering(self):
        from examples.sensor_drive import Controller
        right=Controller();left=Controller()
        sensors=dict(Fr=200,FrLh=150,RrLh=140,FrRh=70,RrRh=80)
        mirrored=dict(Fr=200,FrLh=70,RrLh=80,FrRh=150,RrRh=140)
        for _ in range(10):
            accel,handle,state=right.step(sensors,.05)
            expected=left._step_left(mirrored,.05)
            self.assertEqual((accel,handle),(expected[0],-expected[1]))
            self.assertIn('右壁追従',state)
    def test_positive_handle_turns_left(self):
        c=Car(0,0,accel=40,handle=50)
        for i in range(120): c.move(VehicleConfig(),1/120)
        self.assertLess(c.heading,0); self.assertLess(c.x,0); self.assertGreater(c.y,0)
    def test_pwm_endpoints(self):
        c=Car(0,0,accel=100,handle=100); self.assertEqual(c.pwm(),(500,540))
        c.accel=-100;c.handle=-100;self.assertEqual(c.pwm(),(220,330))
        c.accel=0;c.handle=0;self.assertEqual(c.pwm(),(375,435))
    def test_sensor_seed_repeatable(self):
        cfg=VehicleConfig(sensor_noise_std_cm=2)
        a=Simulation(World.load(),cfg,seed=12); b=Simulation(World.load(),cfg,seed=12)
        self.assertEqual(a.sensors,b.sensors)
    def test_grouping_does_not_change_physics(self):
        a=Simulation(World.load(),mode='guide'); b=Simulation(World.load(),mode='guide')
        for i in range(600): a.step()
        for i in range(100):
            for j in range(6): b.step()
        self.assertEqual(a.car,b.car); self.assertEqual(a.time,b.time)
    def test_photo_boolean_precedence(self):
        command=photo_decide(dict(Fr=20,FrLh=200,RrLh=200,FrRh=10,RrRh=10))
        self.assertEqual(command[:2],(-50,40)) # Rule 7 overrides 6 when Fr<35.
    def test_non_finite_sensor_stops(self):
        self.assertEqual(photo_decide(dict(Fr=float('nan'),FrLh=100,RrLh=100,FrRh=100,RrRh=100))[:2],(0,0))
    def test_collision_latches_and_restores_last_valid_pose(self):
        sim=Simulation(simple_world([wall((-100,245),(200,245))])); sim.mode='manual'
        for i in range(1200):
            sim.step((40,0))
            if sim.collision_id: break
        self.assertEqual(sim.collision_id,'test_wall')
        old=dataclasses.asdict(sim.car); sim.step((100,100))
        self.assertEqual(dataclasses.asdict(sim.car),old)
        self.assertIsNone(sim.world.collision(sim.car.polygon(sim.config)))
    def test_camera_transform_and_zoom(self):
        from rc_simulator_v5 import Camera
        cam=Camera(); cam.fit([0,0,1030,610],1100,690)
        p=cam.world(322,281); cam.zoom(1.8,322,281)
        self.assertLess(math.dist(p,cam.world(322,281)),1e-8)
        self.assertLess(math.dist((135,342),cam.world(*cam.screen(135,342))),1e-8)
    def test_config_validation(self):
        with self.assertRaises(ValueError): VehicleConfig(wheelbase_cm=40)
        with self.assertRaises(ValueError): VehicleConfig(front_axle_from_front_cm=12)

    def test_user_vehicle_dimensions(self):
        cfg=VehicleConfig()
        self.assertEqual((cfg.length_cm,cfg.width_cm),(28,20))
        self.assertEqual(cfg.length_cm/2-cfg.front_axle_cm,7)
        self.assertEqual(cfg.length_cm/2-cfg.rear_axle_cm,24)
        self.assertEqual(cfg.front_axle_cm-cfg.rear_axle_cm,17)
        self.assertEqual(cfg.tire_diameter_cm,5.5)
        x,y,xx,yy=bounds(Car(0,0).polygon(cfg))
        self.assertEqual((xx-x,yy-y),(20,28))


def suite():
    result=unittest.TestSuite()
    for cls in (GeometryTests,MapTests,SimulationTests):
        result.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(cls))
    return result
