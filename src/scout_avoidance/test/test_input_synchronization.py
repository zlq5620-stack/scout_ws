"""Regression: odometry arrives 20 ms ahead of TF; use real in-memory TF buffer.

No ROS nodes, network sockets, CAN sockets, or running robot are needed.
"""
import sys,time,unittest
from pathlib import Path
from types import SimpleNamespace
from collections import deque
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
try:
    from scout_avoidance.preview_inputs import PreviewInputs,NotReady
    from scout_avoidance.local_planner_core import MapCollision
    from geometry_msgs.msg import TransformStamped,PoseStamped
    from nav_msgs.msg import Odometry,Path as RosPath
    from sensor_msgs.msg import LaserScan
    from rclpy.time import Time
    from tf2_ros import Buffer
except ImportError as error:
    raise unittest.SkipTest('Source ROS Humble before tests: '+str(error))
import numpy as np


def stamp(t):return Time(seconds=t).to_msg()

def transform(parent,child,t,x=0.):
    tr=TransformStamped();tr.header.frame_id=parent;tr.child_frame_id=child;tr.header.stamp=stamp(t)
    tr.transform.translation.x=x;tr.transform.rotation.w=1.;return tr


class InputSyncTests(unittest.TestCase):
    def make(self):
        s=object.__new__(PreviewInputs)
        s.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=100.)))
        s.timeouts={'scan':.45,'odom':.30,'path':3.,'tf':.5};s.tf_sync_lag=.10;s.min_scan=12;s.min_front=3
        s.map=MapCollision(np.zeros((100,100)),.1,(-5,-5,0));s.map_error=None
        s.tf=Buffer();s.tf.set_transform_static(transform('map','odom',0),'test')
        s.tf.set_transform_static(transform('base_footprint','livox_level',0,.13),'test')
        for t in [99.7,99.8,99.9,99.98]:s.tf.set_transform(transform('odom','base_footprint',t),'test')
        s.odom=Odometry();s.odom.header.frame_id='odom';s.odom.child_frame_id='base_link';s.odom.header.stamp=stamp(100.)
        s.path=RosPath();s.path.header.frame_id='map';s.path.header.stamp=stamp(99.9)
        p=PoseStamped();p.pose.position.x=2.;p.pose.orientation.w=1.;s.path.poses=[p]
        s.scan=LaserScan();s.scan.header.frame_id='livox_level';s.scan.header.stamp=stamp(99.9)
        s.scan.range_min=.15;s.scan.range_max=12.;s.scan.angle_min=-3.14;s.scan.angle_increment=.01;s.scan.ranges=[3.]*628
        now=time.monotonic();s.arrival={'scan':now,'odom':now,'path':now};s.scan_history=deque([(s.scan,now)],maxlen=4)
        return s
    def test_tf_20_ms_behind_odom(self):
        s=self.make();context=s.snapshot()
        self.assertAlmostEqual(context['tf_sync_lag_s'],.02,places=5)
        self.assertEqual(context['stamp'],stamp(99.98))
        self.assertEqual(context['source_scan_header'].stamp,stamp(99.9))
    def test_new_scan_before_tf_uses_previous_valid_scan(self):
        s=self.make();old=s.scan
        scan=LaserScan();scan.header=type(scan.header)();scan.header.frame_id='livox_level';scan.header.stamp=stamp(100.)
        s.scan=scan;s.scan_history.append((scan,time.monotonic()))
        self.assertEqual(s.snapshot()['source_scan_header'].stamp,old.header.stamp)
    def test_no_available_scan_waits_without_extrapolation(self):
        s=self.make();s.scan.header.stamp=stamp(100.)
        with self.assertRaises(NotReady) as e:s.snapshot()
        self.assertEqual(e.exception.state,'WAITING_FOR_SCAN_TF')
    def test_large_tf_lag_rejected(self):
        s=self.make();s.odom.header.stamp=stamp(100.1)
        with self.assertRaises(NotReady) as e:s.snapshot()
        self.assertEqual(e.exception.state,'TF_SYNC_LAG')
    def test_genuinely_stale_scan_and_odom_rejected(self):
        s=self.make();s.scan.header.stamp=stamp(99.5)
        with self.assertRaises(NotReady) as e:s.snapshot()
        self.assertEqual(e.exception.state,'STALE_SCAN')
        s=self.make();s.odom.header.stamp=stamp(99.5)
        with self.assertRaises(NotReady) as e:s.snapshot()
        self.assertEqual(e.exception.state,'STALE_ODOM')


if __name__=='__main__':unittest.main()
