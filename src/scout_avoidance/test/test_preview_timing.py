"""Timing regression without constructing ROS nodes or sending commands."""
import sys,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
try:
    from scout_avoidance.dwa_preview import DwaPreview
    from scout_avoidance.candidate_guard_preview import CandidateGuard
    from scout_avoidance.preview_inputs import NotReady
    from scout_avoidance.local_planner_core import Settings,MapCollision
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import Header
    from rclpy.time import Time
except ImportError as error:raise unittest.SkipTest('Source ROS Humble: '+str(error))
import numpy as np
from scipy.spatial import cKDTree


def context():
    header=Header();header.stamp=Time(seconds=100.).to_msg()
    return {'pose':(2.,2.,0.),'velocity':(0.,0.),'map':MapCollision(np.zeros((100,100)),.05,(0,0,0)),
            'obstacle_tree':cKDTree([[2.,0.],[-2.,0.],[0.,2.]]),'local_path':np.array([[0.,0.],[1.,0.]]),'goal_heading':0.,
            'source_scan_header':header,'source_scan_arrival':10.,'odom_header':header,'reference_header':header,'stamp':header.stamp}


class TimingTests(unittest.TestCase):
    def test_70_ms_work_leaves_30_ms_until_next_cycle(self):
        ctx=context();inputs=SimpleNamespace(snapshot=lambda:ctx,settings=Settings(),age=lambda _:0.,timeouts={'scan':.45,'odom':.30})
        n=SimpleNamespace(inputs=inputs,timer=Mock(),publish=Mock(),compute_ms=0.,get_parameter=lambda _:SimpleNamespace(value=False))
        with patch('scout_avoidance.dwa_preview.time.monotonic',return_value=10.07) as clock:
            clock.side_effect=[10.,10.07,10.07,10.07]
            DwaPreview.tick(n)
        self.assertAlmostEqual(n.compute_ms,70.,places=5)
        self.assertTrue(29000000<=n.timer.timer_period_ns<=31000000)
        self.assertEqual(n.publish.call_args.args[0]['state'],'CANDIDATE_READY')
    def guard(self):
        ctx=context();m=TwistStamped();m.header.frame_id='base_footprint';m.header.stamp=ctx['stamp'];m.twist.linear.x=.02
        inputs=SimpleNamespace(snapshot=lambda:ctx,settings=Settings(),age=lambda _:0.,timeouts={'scan':.45,'odom':.30})
        return SimpleNamespace(candidate=m,arrival=time.monotonic(),timeout=.30,inputs=inputs,get_parameter=lambda _:SimpleNamespace(value=False))
    def test_guard_keeps_reference_timestamp(self):
        g=self.guard();out=CandidateGuard.check(g)
        self.assertEqual(out[2],'CHECKED_PREVIEW');self.assertEqual(g.checked_stamp,g.candidate.header.stamp)
    def test_guard_rejects_result_that_expires_during_check(self):
        g=self.guard();expired=[False]
        g.inputs.age=lambda _: .4 if expired[0] else 0.
        def slow(*args):expired[0]=True;return (np.zeros((2,3)),2),'CLEAR',1.
        with patch('scout_avoidance.candidate_guard_preview.check_motion',side_effect=slow):
            with self.assertRaises(NotReady) as error:CandidateGuard.check(g)
        self.assertEqual(error.exception.state,'EXPIRED_CHECK')

    def test_guard_rechecks_changed_feedback_instead_of_window_fault(self):
        g=self.guard();ctx=g.inputs.snapshot();ctx['velocity']=(.07,.2);g.inputs.snapshot=lambda:ctx
        self.assertEqual(CandidateGuard.check(g)[2],'CHECKED_PREVIEW')


if __name__=='__main__':unittest.main()
