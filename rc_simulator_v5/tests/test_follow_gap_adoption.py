import unittest
from examples.sonic_drive_follow_gap import Controller
from vehicle_view import gap_sector

class FollowGapTests(unittest.TestCase):
    def test_obstacle_in_front_and_right_selects_left(self):
        c=Controller();target=c.gap_target([20,250,250,25,250],0)
        self.assertGreater(target,0)
        self.assertLessEqual(c.selected_gap[0],target)
        self.assertGreaterEqual(c.selected_gap[1],target)

    def test_no_free_gap_does_not_invent_a_target(self):
        c=Controller();self.assertIsNone(c.gap_target([10]*5,0))
        self.assertIsNone(c.selected_gap)

    def test_invalid_or_emergency_ranges_stop_before_gap(self):
        for distances in ({'Fr':250},{'Fr':5,'FrLh':250,'RrLh':250,'FrRh':250,'RrRh':250}):
            a,h,state=Controller().step(distances,.05)
            self.assertEqual(a,0);self.assertIn('安全停止',state)
            self.assertIsNone(gap_sector(state))

    def test_gap_metadata_is_checked_and_optional(self):
        self.assertEqual(gap_sector('FTG | gap_min_deg=-20 | gap_max_deg=35'),(-20,35))
        for state in ('legacy','FTG | gap_min_deg=40 | gap_max_deg=20','FTG | gap_min_deg=-70 | gap_max_deg=35','安全停止 | gap_min_deg=-20 | gap_max_deg=35'):
            self.assertIsNone(gap_sector(state))
