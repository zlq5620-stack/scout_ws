"""Worker completion/version/fresh-pose tests with fake node and fake futures."""
import sys,unittest
from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import Future
from unittest.mock import Mock
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[2]/'scout_avoidance')]
try:
    from scout_planning.astar_node import GlobalPlanner
    from scout_planning.footprint_planner import FootprintPlanner
except ImportError as error:raise unittest.SkipTest('Source ROS Humble: '+str(error))
import numpy as np


class AsyncTests(unittest.TestCase):
    def make(self):
        return SimpleNamespace(pending=None,generation=1,goal=(2.,1.,0.),next_plan=0.,replan_period=1.,
          planner=FootprintPlanner(np.zeros((80,80)),.05,(0,0,0)),current_pose=lambda:(1.,1.,0.),
          search_executor=Mock(),max_expansions=10000,search_budget=.35,empty_path=Mock(),status=Mock(),publish_result=Mock())
    def pending(self,c,revision=1,path=None):
        f=Future();f.set_result((path if path is not None else [(1.,1.,0.),(2.,1.,0.)],'PATH_READY',4));c.pending=(f,revision,c.goal)
    def test_submit_returns_without_executing_search(self):
        c=self.make();f=Future();c.search_executor.submit.return_value=f
        GlobalPlanner.tick(c);self.assertIs(c.pending[0],f);c.publish_result.assert_not_called()
        GlobalPlanner.tick(c);self.assertEqual(c.search_executor.submit.call_count,1)
    def test_old_goal_result_is_discarded(self):
        c=self.make();self.pending(c,revision=0);GlobalPlanner.tick(c);c.publish_result.assert_not_called();self.assertIsNone(c.pending)
    def test_fresh_pose_required_after_search(self):
        c=self.make();self.pending(c)
        def stale():raise ValueError('STALE_POSE')
        c.current_pose=stale;GlobalPlanner.tick(c);c.empty_path.assert_called_once();self.assertEqual(c.status.call_args.args[0],'STALE_POSE')
    def test_moved_start_reconnected_and_checked(self):
        c=self.make();self.pending(c);c.current_pose=lambda:(1.03,1.,0.)
        GlobalPlanner.tick(c);path=c.publish_result.call_args.args[0]
        self.assertEqual(tuple(path[0]),(1.03,1.,0.));self.assertTrue(c.planner.clear(path))
    def test_current_blocked_rejects_old_route(self):
        c=self.make();self.pending(c);c.current_pose=lambda:(.1,.1,0.)
        GlobalPlanner.tick(c);self.assertIsNone(c.publish_result.call_args.args[0]);self.assertEqual(c.publish_result.call_args.args[1],'START_BLOCKED')
    def test_failing_worker_does_not_publish_route(self):
        c=self.make();f=Future();f.set_exception(RuntimeError('test failure'));c.pending=(f,1,c.goal)
        GlobalPlanner.tick(c);c.publish_result.assert_not_called();self.assertEqual(c.status.call_args.args[0],'PLANNING_ERROR')


if __name__=='__main__':unittest.main()
