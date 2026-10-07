import csv
import tempfile
import unittest
from types import SimpleNamespace
from config import VehicleConfig
from sonar_mapping import SonarMap

class MappingTests(unittest.TestCase):
    def sim(self):
        return SimpleNamespace(car=object(),world=object(),config=VehicleConfig(),
            sensor_observation_id=1,sensor_observation_pose=(100,200,0,2),
            sensors={'Fr':50,'RrLh':50,'FrRh':250,'FrLh':float('nan'),'RrRh':0})

    def test_endpoints_use_measured_range_and_sample_pose(self):
        sim=self.sim();model=SonarMap();model.sample(sim)
        points={p[1]:p for p in model.points}
        self.assertEqual(set(points),{'Fr','RrLh'})
        self.assertEqual(points['Fr'][2:4],(100,262))
        self.assertAlmostEqual(points['RrLh'][2],40)
        self.assertAlmostEqual(points['RrLh'][3],207)
        model.sample(sim);self.assertEqual(len(model.points),2)
        self.assertEqual(sim.sensors['Fr'],50)

    def test_reset_clear_and_capacity(self):
        sim=self.sim();model=SonarMap(3);model.sample(sim)
        sim.sensor_observation_id=2;model.sample(sim);self.assertEqual(len(model.points),3)
        model.clear();model.sample(sim);self.assertEqual(len(model.points),0)
        sim.car=object();model.sample(sim);self.assertEqual(len(model.points),2)

    def test_csv_retains_measurements(self):
        model=SonarMap();model.sample(self.sim())
        with tempfile.NamedTemporaryFile(suffix='.csv') as f:
            model.save_csv(f.name)
            with open(f.name,encoding='utf-8-sig') as stream:rows=list(csv.DictReader(stream))
        self.assertEqual(len(rows),2);self.assertEqual(rows[0]['distance_cm'],'50')
