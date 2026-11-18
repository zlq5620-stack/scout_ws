"""Controller callback tests with fake CAN transport; no nodes or sockets."""
import sys,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[2]/'scout_avoidance')]
try:
    from scout_control.autonomy_controller import AutonomyController
    from geometry_msgs.msg import TwistStamped
except ImportError as e:
    raise unittest.SkipTest('Source ROS Humble before running node tests: '+str(e))
from scout_control.autonomy_core import ControlLatch
from scout_avoidance.local_planner_core import Settings,MapCollision
from scout_avoidance.preview_inputs import NotReady
from scipy.spatial import cKDTree
import numpy as np


class NodeTests(unittest.TestCase):
    def make(self,hardware=True):
        c=SimpleNamespace(latch=ControlLatch(hardware),enabled_at=0.,checked_at=10.,checked=None,
                          compute_ms=0.,validation_timer=Mock())
        ctx={'pose':(2.,2.,0.),'velocity':(0.,0.),'map':MapCollision(np.zeros((100,100)),.05,(0,0,0)),
             'obstacle_tree':cKDTree([[2.,0.],[-2.,0.],[0.,2.]]),
             'local_path':np.array([[0.,0.],[1.,0.]]),'goal_heading':0.}
        c.inputs=SimpleNamespace(settings=Settings(),snapshot=lambda:ctx,age=lambda h:0.,
          scan=SimpleNamespace(header=object()),odom=SimpleNamespace(header=object()),arrival={'scan':10.},timeouts={'scan':.45,'odom':.30})
        c.can=SimpleNamespace(healthy=lambda t:True,mode=1,motion=(0.,0.),receive=Mock(),send=Mock(),conflict=False)
        c.checked_velocity=lambda now:AutonomyController.checked_velocity(c,now)
        c.transmit=lambda:AutonomyController.transmit(c)
        m=TwistStamped();m.header.frame_id='base_footprint';m.twist.linear.x=.02;m.twist.angular.z=.08;c.checked=m
        return c

    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_enable_waits_new_then_move_and_stop(self,_):
        c=self.make();response=SimpleNamespace();AutonomyController.enable(c,SimpleNamespace(data=True),response)
        self.assertTrue(response.success);self.assertIsNone(c.checked);self.assertTrue(c.latch.armed)
        AutonomyController.validate(c);self.assertTrue(c.latch.armed);self.assertEqual(c.latch.command,(0.,0.))
        m=TwistStamped();m.header.frame_id='base_footprint';m.twist.linear.x=.02;c.checked=m;c.checked_at=10.
        AutonomyController.validate(c);self.assertAlmostEqual(c.latch.command[0],.02);self.assertEqual(c.latch.command[1],0.);c.transmit();self.assertAlmostEqual(c.can.send.call_args.args[0],.02)
        c.checked.twist.linear.x=0.;AutonomyController.validate(c);self.assertTrue(c.latch.armed);c.transmit();c.can.send.assert_called_with(0.,0.)
        self.assertEqual(c.latch.state,'WAITING_FOR_SAFE_COMMAND')
        c.checked.twist.linear.x=.02;c.last_command_at=9.9;AutonomyController.validate(c)
        self.assertTrue(c.latch.armed);self.assertEqual(c.latch.state,'AUTONOMOUS')
    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_dry_run_never_writes(self,_):
        c=self.make(False);AutonomyController.validate(c);c.transmit();c.can.send.assert_not_called()
        response=SimpleNamespace();AutonomyController.enable(c,SimpleNamespace(data=True),response);self.assertFalse(response.success);c.transmit();c.can.send.assert_not_called()
    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_manual_takeover_and_candidate_staleness(self,_):
        c=self.make();c.latch.arm(10.);c.can.mode=3;c.transmit();self.assertFalse(c.latch.armed);c.can.send.assert_not_called()
        c=self.make();c.latch.arm(10.);c.checked_at=9.;AutonomyController.validate(c);self.assertTrue(c.latch.armed);c.transmit();c.can.send.assert_called_with(0.,0.)
        c.latch.output(11.1,1,True);self.assertFalse(c.latch.armed)
    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_goal_change_and_invalid_inputs_latch(self,_):
        c=self.make();c.latch.arm(10.);AutonomyController.on_goal(c,None);self.assertFalse(c.latch.armed)
        c=self.make();c.latch.arm(10.);c.checked.twist.linear.y=.01;AutonomyController.validate(c);self.assertFalse(c.latch.armed);self.assertEqual(c.latch.state,'INVALID_CHECKED')
        c=self.make();c.latch.arm(10.)
        def stale():raise NotReady('STALE_SCAN','stale')
        c.inputs.snapshot=stale;AutonomyController.validate(c);self.assertTrue(c.latch.armed);self.assertEqual(c.latch.state,'INPUT_RECOVERY_PAUSE');self.assertEqual(c.latch.command,(0.,0.))
    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_collision_blocks_execution(self,_):
        c=self.make();c.latch.arm(10.);ctx=c.inputs.snapshot();ctx['obstacle_tree']=cKDTree([[.1,.1]])
        AutonomyController.validate(c);self.assertFalse(c.latch.armed);self.assertEqual(c.latch.state,'COLLISION_STOP')

    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_delayed_feedback_does_not_reject_safe_command(self,_):
        c=self.make();c.latch.arm(10.);c.inputs.snapshot()['velocity']=(.07,.20)
        c.last_applied_command=(.02,.08);c.last_command_at=9.9
        AutonomyController.validate(c)
        self.assertTrue(c.latch.armed);self.assertEqual(c.latch.state,'AUTONOMOUS')
        self.assertAlmostEqual(c.latch.command[0],.02);self.assertAlmostEqual(c.latch.command[1],.08)
    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_higher_candidate_is_slew_limited_before_recheck(self,_):
        c=self.make();c.latch.arm(10.);c.last_applied_command=(.02,.08);c.last_command_at=9.9
        c.checked.twist.linear.x=.12;c.checked.twist.angular.z=.4
        AutonomyController.validate(c)
        self.assertTrue(c.latch.armed)
        self.assertAlmostEqual(c.latch.command[0],.04);self.assertAlmostEqual(c.latch.command[1],.16)

    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_goal_reached_requires_pose_heading_and_stop(self,_):
        c=self.make();c.latch.arm(10.);ctx=c.inputs.snapshot()
        ctx['local_path']=np.array([[0.,0.],[.05,0.]])
        ctx['velocity']=(.05,0.)
        AutonomyController.validate(c);self.assertEqual(c.latch.state,'GOAL_BRAKING');self.assertTrue(c.latch.armed)
        ctx['velocity']=(0.,0.);AutonomyController.validate(c)
        self.assertEqual(c.latch.state,'GOAL_REACHED');self.assertFalse(c.latch.armed)
        self.assertEqual(c.last_stop_details['state'],'GOAL_REACHED')
    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_near_position_wrong_heading_is_not_goal_reached(self,_):
        c=self.make();c.latch.arm(10.);ctx=c.inputs.snapshot();ctx['local_path']=np.array([[0.,0.],[.05,0.]])
        ctx['goal_heading']=1.;c.checked.twist.linear.x=c.checked.twist.angular.z=0.
        AutonomyController.validate(c);self.assertEqual(c.latch.state,'WAITING_FOR_SAFE_COMMAND');self.assertTrue(c.latch.armed)
    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_software_stop_during_pause_cannot_auto_resume(self,_):
        c=self.make();c.latch.arm(10.);c.checked.twist.linear.x=c.checked.twist.angular.z=0.
        AutonomyController.validate(c);self.assertTrue(c.latch.armed)
        AutonomyController.enable(c,SimpleNamespace(data=False),SimpleNamespace())
        c.checked.twist.linear.x=.02;AutonomyController.validate(c);self.assertFalse(c.latch.armed)
    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_hardware_fault_still_disarms_immediately(self,_):
        c=self.make();c.latch.arm(10.);c.can.healthy=lambda _:False
        AutonomyController.validate(c);self.assertFalse(c.latch.armed);self.assertEqual(c.latch.state,'CAN_NOT_READY')

    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_goal_stop_also_requires_latest_can_speed_near_zero(self,_):
        c=self.make();c.latch.arm(10.);ctx=c.inputs.snapshot();ctx['local_path']=np.array([[0.,0.],[.05,0.]])
        c.can.motion=(.05,0.)
        AutonomyController.validate(c);self.assertEqual(c.latch.state,'GOAL_BRAKING');self.assertTrue(c.latch.armed)
        c.can.motion=(0.,0.);AutonomyController.validate(c);self.assertEqual(c.latch.state,'GOAL_REACHED')

    @patch('scout_control.autonomy_controller.time.monotonic', return_value=10.)
    def test_live_can_overspeed_stops_and_is_recorded(self, _):
        c = self.make()
        c.latch.arm(10.)
        c.latch.accept(10., .10, .08, 10.2)
        c.can.motion = (.154, 0.)
        c.transmit()
        self.assertFalse(c.latch.armed)
        self.assertEqual(c.latch.state, 'FEEDBACK_OVERSPEED')
        self.assertEqual(c.last_stop_details['latest_can_velocity'], [.154, 0.])
        c.can.send.assert_called_once_with(0., 0.)
        c.can.motion = (0., 0.)
        c.transmit()
        self.assertEqual(c.latch.state, 'FEEDBACK_OVERSPEED')

    @patch('scout_control.autonomy_controller.time.monotonic', return_value=10.)
    def test_zero_candidate_does_not_report_ready_or_enable(self, _):
        c = self.make()
        c.checked.twist.linear.x = 0.
        c.checked.twist.angular.z = 0.
        AutonomyController.validate(c)
        self.assertEqual(c.shadow_state, 'NO_SAFE_COMMAND')
        response = AutonomyController.enable(c, SimpleNamespace(data=True), SimpleNamespace())
        self.assertFalse(response.success)
        self.assertIn('NO_SAFE_COMMAND', response.message)
        self.assertFalse(c.latch.armed)
        c.can.send.assert_not_called()


if __name__=='__main__':unittest.main()
