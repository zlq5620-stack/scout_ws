import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class SafetyGate(Node):
    def __init__(self):
        super().__init__('scout_safety_gate')
        self.declare_parameter('max_linear',0.12)
        self.declare_parameter('max_angular',0.5)
        self.declare_parameter('stop_distance',0.65)
        self.armed=False  # 默认禁止自动行驶！必须人工发布 /navigation_enabled
        self.nav=None
        self.scan=None
        self.nav_t=0.0
        self.scan_t=0.0
        self.create_subscription(Twist,'/cmd_vel_nav',self.on_nav,10)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Bool,'/navigation_enabled',self.on_arm,10)
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.create_timer(0.1,self.tick)
        self.get_logger().warn('软件控制默认 DISARMED，实机必须有硬件急停')

    def on_nav(self,m):self.nav=m;self.nav_t=time.monotonic()
    def on_scan(self,m):self.scan=m;self.scan_t=time.monotonic()
    def on_arm(self,m):
        self.armed=bool(m.data)
        self.get_logger().warn('软件自动控制 '+('ARMED' if self.armed else 'DISARMED'))
    def tick(self):
        cmd=Twist()
        now=time.monotonic()
        if (not self.armed or self.nav is None or self.scan is None or
            now-self.nav_t>0.35 or now-self.scan_t>0.45):
            self.pub.publish(cmd);return
        # 软件级前向停车防护：不替代硬件急停，不覆盖地面盲区
        m=self.scan
        front=[]
        valid=0
        for i,r in enumerate(m.ranges):
            if not math.isfinite(r) or r<m.range_min or r>m.range_max:continue
            valid+=1
            a=m.angle_min+i*m.angle_increment
            if abs(math.atan2(math.sin(a),math.cos(a)))<0.70:front.append(r)
        if valid<12 or len(front)<3:
            # 正前方没有足够测量时不允许自动行驶（即使其他方向有点）
            self.pub.publish(cmd);return
        requested=max(0.0,min(float(self.nav.linear.x),float(self.get_parameter('max_linear').value)))
        near=min(front) if front else float('inf')
        if near<float(self.get_parameter('stop_distance').value) and requested>0:
            self.pub.publish(cmd);return
        cmd.linear.x=requested
        cmd.angular.z=max(-float(self.get_parameter('max_angular').value),
                          min(float(self.nav.angular.z),float(self.get_parameter('max_angular').value)))
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    n=SafetyGate()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
