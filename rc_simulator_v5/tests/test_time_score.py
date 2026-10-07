import unittest
from optimization.time_score import time_bonus,add_time_score

class TimeScoreTests(unittest.TestCase):
    def test_faster_completion_adds_more(self):
        self.assertEqual(time_bonus(True,400),200)
        self.assertEqual(time_bonus(True,350),250)

    def test_incomplete_collision_and_error_do_not_earn_bonus(self):
        self.assertEqual(time_bonus(False,None),0)
        self.assertEqual(time_bonus(True,100,collisions=1),0)
        self.assertEqual(time_bonus(True,100,program_error='failed'),0)

    def test_slow_completion_does_not_add_negative_bonus(self):
        self.assertEqual(time_bonus(True,650),0)

    def test_invalid_completed_time_is_rejected(self):
        for duration in (None,0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):time_bonus(True,duration)

    def test_previous_score_is_retained_and_new_total_adds_bonus(self):
        base=dict(completed_three_laps=True,total_time_s=400,smooth_score=722.22)
        scored=add_time_score(base)
        self.assertEqual(base['smooth_score'],scored['smooth_score'])
        self.assertEqual(scored['time_bonus'],200)
        self.assertEqual(scored['race_score'],922.22)
        self.assertNotIn('race_score',base)
