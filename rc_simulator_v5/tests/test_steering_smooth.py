import unittest
from unittest.mock import patch
from examples.sonic_drive_smooth import Controller,GapController

READINGS=dict(Fr=250,FrLh=150,RrLh=70,FrRh=150,RrRh=70)

class SteeringSmoothTests(unittest.TestCase):
    def test_nonurgent_steering_step_is_rate_limited(self):
        c=Controller();previous=0
        with patch.object(GapController,'step',return_value=(30,100,'目標 | target_deg=60')):
            for _ in range(30):
                a,h,state=c.step(READINGS,.05)
                self.assertLessEqual(abs(h-previous),140*.05+1e-9)
                self.assertGreaterEqual(h,previous);self.assertLessEqual(h,100)
                self.assertEqual(c.handle,h);previous=h

    def test_rate_change_eases_at_shorter_control_interval(self):
        c=Controller()
        with patch.object(GapController,'step',return_value=(30,100,'目標')):
            _,first,_=c.step(READINGS,.01)
            _,second,_=c.step(READINGS,.01)
        self.assertAlmostEqual(first,.8)
        self.assertAlmostEqual(second-first,1.4)

    def test_urgent_avoidance_is_not_delayed_by_smoothing(self):
        c=Controller();near=dict(READINGS,FrLh=20)
        with patch.object(GapController,'step',return_value=(10,70,'接近回避')):
            a,h,state=c.step(near,.05)
        self.assertEqual(h,70);self.assertIn('接近回避優先',state)

    def test_sensor_fault_stops_without_filter_delay(self):
        a,h,state=Controller().step({'Fr':250},.05)
        self.assertEqual(a,0);self.assertIn('安全停止',state)
