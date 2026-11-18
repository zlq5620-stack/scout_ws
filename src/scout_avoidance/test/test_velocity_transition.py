"""Collision prediction must include measured-speed deceleration, not teleport."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial import cKDTree
from scout_avoidance.local_planner_core import Settings,trajectory,MapCollision,check_motion,velocity_window


class TransitionTests(unittest.TestCase):
    def test_braking_from_measured_speed(self):
        s=Settings();p,_=trajectory(0.,0.,s,initial_velocity=(.12,0.))
        self.assertGreaterEqual(p[-1,0],.12**2/(2*s.brake_decel)-1e-9)
        self.assertLess(p[-1,0],.05)
    def test_current_speed_can_make_slow_candidate_collide(self):
        s=Settings();ctx={'pose':(2.,2.,0.),'map':MapCollision(np.zeros((100,100)),.05,(0,0,0)),
                         'obstacle_tree':cKDTree([[.41,0.]])}
        # Assuming instant .02 m/s misses the remaining motion at .12 m/s.
        self.assertIsNotNone(check_motion(.02,0.,ctx,s)[0])
        ctx['velocity']=(.12,0.)
        self.assertEqual(check_motion(.02,0.,ctx,s)[1],'SCAN_COLLISION')
    def test_overspeed_is_still_rejected(self):
        s=Settings();ctx={'velocity':(.16,0.),'obstacle_tree':cKDTree([[3.,0.]])}
        self.assertEqual(check_motion(.02,0.,ctx,s)[1],'FEEDBACK_SPEED_LIMIT')
    def test_angular_transition_remains_in_trajectory(self):
        s=Settings();p,_=trajectory(0.,0.,s,initial_velocity=(0.,.4))
        self.assertGreater(p[-1,2],0.);self.assertLess(p[-1,2],.12)

    def test_feedback_near_cap_has_safe_deceleration_target(self):
        s=Settings();window=velocity_window(.145,.49,s)
        self.assertIsNotNone(window);np.testing.assert_allclose(window[0],[.12]);np.testing.assert_allclose(window[1],[.4])
        self.assertIsNone(velocity_window(.16,0.,s))


if __name__=='__main__':unittest.main()
