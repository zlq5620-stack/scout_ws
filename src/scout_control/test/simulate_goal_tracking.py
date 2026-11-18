"""Offline end-to-end tracking fixture: no ROS nodes and no CAN sockets."""
import sys,math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial import cKDTree
package_root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(package_root),str(package_root.parent/'scout_avoidance')]
from scout_control.autonomy_core import ControlLatch
from scout_control.autonomy_controller import AutonomyController
from scout_avoidance.candidate_guard_preview import CandidateGuard
from scout_avoidance.local_planner_core import Settings,MapCollision,choose_motion,integrate
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from rclpy.time import Time
s=Settings();clock=[10.];pose=[5.,5.,0.];actual=[0.,0.];target=[0.,0.];ctxbox=[None]
route=np.concatenate((np.column_stack((np.linspace(5,6,31),np.full(31,5.))),np.column_stack((np.full(30,6.),np.linspace(5,6,31)[1:]))))
mapcheck=MapCollision(np.zeros((200,200)),.05,(0,0,0))
inputs=SimpleNamespace(settings=s,snapshot=lambda:ctxbox[0],age=lambda _:0.,timeouts={'scan':.45,'odom':.30},scan=SimpleNamespace(header=Header()),odom=SimpleNamespace(header=Header()),arrival={'scan':10.})
c=SimpleNamespace(latch=ControlLatch(True),enabled_at=10.,checked_at=10.,checked=None,compute_ms=0.,
 inputs=inputs,last_applied_command=(0.,0.),last_command_at=9.9,last_validation={},last_stop_details={},validation_timer=SimpleNamespace(reset=lambda:None))
def send(v,w):target[:]=[v,w]
c.can=SimpleNamespace(healthy=lambda _:True,mode=1,motion=(0.,0.),receive=lambda:None,send=send,conflict=False)
c.checked_velocity=lambda now:AutonomyController.checked_velocity(c,now)
c.transmit=lambda:AutonomyController.transmit(c)
c.latch.arm(10.);states={};injected=False
angles=np.linspace(-math.pi,math.pi,720);tree=cKDTree(np.column_stack((3*np.cos(angles),3*np.sin(angles))))
with patch('scout_control.autonomy_controller.time.monotonic',side_effect=lambda:clock[0]):
 for step in range(550):
  clock[0]=10.+step*.1;inputs.arrival['scan']=clock[0]
  h=Header();h.stamp=Time(seconds=100.+step*.1).to_msg()
  co,si=math.cos(pose[2]),math.sin(pose[2]);local=(route-np.array(pose[:2]))@np.array([[co,-si],[si,co]])
  measured=(.145,.49) if step==5 else tuple(actual)
  c.can.motion=measured
  ctx={'pose':tuple(pose),'velocity':measured,'map':mapcheck,'obstacle_tree':tree,'local_path':local,
       'goal_heading':math.atan2(math.sin(math.pi/2-pose[2]),math.cos(math.pi/2-pose[2])),
       'source_scan_header':h,'source_scan_arrival':clock[0],'odom_header':h,'reference_header':h,'stamp':h.stamp}
  ctxbox[0]=ctx;result=choose_motion(ctx,s)
  msg=TwistStamped();msg.header.frame_id='base_footprint';msg.header.stamp=h.stamp
  msg.twist.linear.x=result['speed'];msg.twist.angular.z=result['turn']
  if step==8:msg.twist.linear.x=msg.twist.angular.z=0.;injected=True
  guard=SimpleNamespace(candidate=msg,arrival=clock[0],timeout=.3,inputs=inputs,get_parameter=lambda _:SimpleNamespace(value=False))
  try:v,w,_,_=CandidateGuard.check(guard)
  except Exception:v=w=0.
  checked=TwistStamped();checked.header=msg.header;checked.twist.linear.x=v;checked.twist.angular.z=w
  c.checked=checked;c.checked_at=clock[0]
  AutonomyController.validate(c);c.transmit();states[c.latch.state]=states.get(c.latch.state,0)+1
  def approach(old,new,amount):return max(old-amount,min(old+amount,new))
  nv=approach(actual[0],target[0],.2*.1);nw=approach(actual[1],target[1],.8*.1)
  pose[:]=integrate(*pose,(actual[0]+nv)/2,(actual[1]+nw)/2,.1);actual[:]=[nv,nw]
  if not c.latch.armed:break
print({'state':c.latch.state,'steps':step+1,'pose':pose,'distance_to_goal':float(np.linalg.norm(np.array(pose[:2])-route[-1])),'states':states,'injected_one_zero':injected})
assert c.latch.state=='GOAL_REACHED',c.last_stop_details
assert states.get('WAITING_FOR_SAFE_COMMAND',0)>0
assert states.get('AUTONOMOUS',0)>0
assert target==[0.,0.]
print('Offline L-turn route completed with one enable, transient zero and boundary feedback; no motors controlled')
