import unittest
from optimization.straight_score import straight_metrics,add_straight_score

class StraightScoreTests(unittest.TestCase):
    def rows(self,speed=20,steer=0,seconds=2):
        return [dict(time_s=i*.05,x_cm=speed*i*.05,y_cm=0,heading_deg=90,speed_cm_s=speed,steer_deg=steer) for i in range(round(seconds/.05)+1)]
    def test_long_moving_straight_gets_full_bonus(self):
        r=add_straight_score({'score':100,'completed_three_laps':True},self.rows())
        self.assertAlmostEqual(r['straight_distance_cm'],40)
        self.assertEqual(r['smooth_score'],400)

    def test_stopping_does_not_earn_bonus(self):
        r=add_straight_score({'score':100,'completed_three_laps':True},self.rows(speed=0))
        self.assertEqual(r['straight_distance_cm'],0);self.assertEqual(r['straight_bonus'],0)

    def test_curve_and_short_bursts_are_not_straight(self):
        for rows in (self.rows(steer=3),self.rows(seconds=.8)):
            self.assertEqual(straight_metrics(rows)['straight_distance_cm'],0)

    def test_incomplete_driving_does_not_receive_bonus(self):
        r=add_straight_score({'score':-10,'completed_three_laps':False},self.rows())
        self.assertEqual(r['smooth_score'],-10)

    def test_steering_oscillation_breaks_segment(self):
        rows=self.rows()
        for row in rows:
            if .8<=row['time_s']<=1.2:row['steer_deg']=4
        self.assertEqual(straight_metrics(rows)['straight_distance_cm'],0)

    def test_wrapped_heading_is_continuous(self):
        rows=self.rows()
        for i,row in enumerate(rows):row['heading_deg']=179.9 if i==0 else -179.9
        self.assertAlmostEqual(straight_metrics(rows)['straight_fraction'],1)
