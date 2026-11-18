import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from tf2_ros import Buffer,TransformListener,TransformException

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

class DWA(Node):
    def __init__(self):
        super().__init__('scout_dwa')
        self.declare_parameter('robot_radius',0.47) # 整机含雷达支架的外接半径
        self.declare_parameter('max_speed',0.12)
        self.declare_parameter('max_turn_rate',0.5)
        self.declare_parameter('linear_accel',0.2)
        self.declare_parameter('angular_accel',0.8)
        self.path=None
        self.scan=None
        self.odom=None
        self.scan_time=0.0
        self.odom_time=0.0
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.create_subscription(Path,'/global_path',self.on_path,QoSProfile(
           depth=1,reliability=ReliabilityPolicy.RELIABLE,
           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cmd=self.create_publisher(Twist,'/cmd_vel_nav',10)
        self.create_timer(0.1,self.tick)

    def on_scan(self,m):self.scan=m;self.scan_time=time.monotonic()
    def on_odom(self,m):self.odom=m;self.odom_time=time.monotonic()
    def on_path(self,m):self.path=m
    def stop(self):self.cmd.publish(Twist())

    def points_in_base(self):
        tr=self.tf_buffer.lookup_transform('base_link',self.scan.header.frame_id,Time())
        q=tr.transform.rotation
        a=yaw(q)
        dx,dy=tr.transform.translation.x,tr.transform.translation.y
        ranges=np.asarray(self.scan.ranges,dtype=np.float64)
        valid=np.isfinite(ranges)&(ranges>=self.scan.range_min)&(ranges<=self.scan.range_max)
        angles=self.scan.angle_min+np.arange(len(ranges))*self.scan.angle_increment+a
        r=ranges[valid]
        return np.column_stack((dx+r*np.cos(angles[valid]),dy+r*np.sin(angles[valid])))

    def select_target(self,base_x,base_y):
        pts=np.array([(p.pose.position.x,p.pose.position.y) for p in self.path.poses])
        dist=np.hypot(pts[:,0]-base_x,pts[:,1]-base_y)
        near=int(np.argmin(dist))
        for idx in range(near,len(pts)):
            if math.hypot(*(pts[idx]-[base_x,base_y]))>=0.6:
                return pts[idx]
        return pts[-1]

    def tick(self):
        now=time.monotonic()
        if not self.path or not self.path.poses or self.odom is None or self.scan is None:
            self.stop();return
        if now-self.scan_time>0.45 or now-self.odom_time>0.45:
            self.stop();return
        try:
            tr=self.tf_buffer.lookup_transform('map','base_link',Time())
            robot_x,robot_y=tr.transform.translation.x,tr.transform.translation.y
            robot_yaw=yaw(tr.transform.rotation)
            obstacles=self.points_in_base()
        except TransformException:
            self.stop();return
        if obstacles.shape[0]<12:
            # 当前帧有效点不足时停止，而不是把无点当成无障碍
            self.stop();return
        target=self.select_target(robot_x,robot_y)
        ex,ey=target[0]-robot_x,target[1]-robot_y
        tx=math.cos(robot_yaw)*ex+math.sin(robot_yaw)*ey
        ty=-math.sin(robot_yaw)*ex+math.cos(robot_yaw)*ey
        cv=float(self.odom.twist.twist.linear.x)
        cw=float(self.odom.twist.twist.angular.z)
        max_v=float(self.get_parameter('max_speed').value)
        max_w=float(self.get_parameter('max_turn_rate').value)
        v_range=np.linspace(max(0,cv-0.02),min(max_v,max(0,cv)+0.02),5)
        w_range=np.linspace(max(-max_w,cw-0.08),min(max_w,cw+0.08),9)
        radius=float(self.get_parameter('robot_radius').value)
        best=None
        cost_best=float('inf')
        for v in v_range:
            for w in w_range:
                x=y=theta=0.0
                safe=True
                clearance=100.0
                for _ in range(12):  # 1.8秒预测时域，0.15秒一步
                    theta+=w*0.15
                    x+=v*math.cos(theta)*0.15
                    y+=v*math.sin(theta)*0.15
                    ds=np.hypot(obstacles[:,0]-x, obstacles[:,1]-y)
                    clearance=min(clearance,float(np.min(ds)))
                    if clearance<radius:
                        safe=False;break
                if not safe:continue
                dist=math.hypot(x-tx,y-ty)
                heading=abs(math.atan2(math.sin(math.atan2(ty-y,tx-x)-theta),
                                       math.cos(math.atan2(ty-y,tx-x)-theta)))
                cost=dist+0.18*heading+0.35/max(0.10,clearance-radius+0.10)-0.45*v
                if cost<cost_best:
                    cost_best=cost;best=(float(v),float(w))
        if best is None:self.stop();return
        msg=Twist()
        msg.linear.x=best[0]
        msg.angular.z=best[1]
        self.cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    n=DWA()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
