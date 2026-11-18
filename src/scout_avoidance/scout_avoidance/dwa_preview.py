"""DWA candidate generation. Direct hardware execution is handled separately."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from .local_planner_core import choose_motion
from .preview_inputs import PreviewInputs, NotReady


class DwaPreview(Node):
    def __init__(self):
        super().__init__('dwa_preview')
        self.inputs = PreviewInputs(self)
        self.command = self.create_publisher(TwistStamped, '/local_planning/candidate_cmd', 1)
        self.path_pub = self.create_publisher(Path, '/local_planning/local_path', 10)
        self.markers = self.create_publisher(MarkerArray, '/local_planning/markers', 10)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/local_planning/status', retained)
        self.last_state = None
        self.timer = self.create_timer(self.inputs.settings.control_dt, self.tick)
        self.compute_ms = 0.
        self.get_logger().info('Local DWA candidate generator: no direct /cmd_vel publisher')

    def publish(self, result, context=None, detail=''):
        stamp = self.get_clock().now().to_msg()
        cmd = TwistStamped()
        cmd.header.frame_id = 'base_footprint'
        cmd.header.stamp = context['stamp'] if context else stamp
        cmd.twist.linear.x = result.get('speed', 0.)
        cmd.twist.angular.z = result.get('turn', 0.)
        self.command.publish(cmd)
        path = Path()
        path.header.frame_id = 'base_footprint'
        path.header.stamp = context['stamp'] if context else stamp
        if result.get('trajectory') is not None:
            for x, y, yaw in result['trajectory']:
                p = PoseStamped()
                p.header = path.header
                p.pose.position.x, p.pose.position.y = float(x), float(y)
                p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                path.poses.append(p)
        self.path_pub.publish(path)
        markers = MarkerArray()
        clear = Marker();clear.action = Marker.DELETEALL;markers.markers.append(clear)
        if context:
            obs = Marker();obs.header = path.header;obs.ns='local_obstacles';obs.id=1
            obs.type=Marker.POINTS;obs.action=Marker.ADD;obs.pose.orientation.w=1.
            obs.scale.x=obs.scale.y=.035;obs.color.r=1.;obs.color.g=.25;obs.color.a=1.
            obs.points=[Point(x=float(x),y=float(y),z=.04) for x,y in context['obstacles']]
            markers.markers.append(obs)
            footprint = Marker();footprint.header=path.header;footprint.ns='collision_rectangle';footprint.id=2
            footprint.type=Marker.LINE_STRIP;footprint.action=Marker.ADD;footprint.pose.orientation.w=1.
            footprint.scale.x=.015;footprint.color.g=1.;footprint.color.a=.9
            hx, hy = self.inputs.settings.half_extents
            footprint.points=[Point(x=x,y=y,z=.03) for x,y in
                              [(hx,hy),(hx,-hy),(-hx,-hy),(-hx,hy),(hx,hy)]]
            markers.markers.append(footprint)
        self.markers.publish(markers)
        state = result['state']
        info = {'state':state, 'preview_only':not self.get_parameter('execution_connected').value, 'message':detail,
                'candidate_linear_mps':cmd.twist.linear.x,'candidate_angular_rps':cmd.twist.angular.z,
                'planning_compute_ms':self.compute_ms,
                'tf_sync_lag_s':context.get('tf_sync_lag_s') if context else None,
                'scan_age_s':self.inputs.age(context['source_scan_header']) if context else None,
                'reference_age_s':self.inputs.age(context['reference_header']) if context else None,
                'collision_model':'oriented_rectangle',
                'collision_length_m':2*self.inputs.settings.half_extents[0],
                'collision_width_m':2*self.inputs.settings.half_extents[1],
                'minimum_scan_clearance_m':result.get('clearance',0.),
                'rejected':result.get('rejected',{})}
        status=String();status.data=json.dumps(info,ensure_ascii=False);self.status_pub.publish(status)
        if state != self.last_state:
            self.get_logger().info(state + ': ' + detail)
            self.last_state=state

    def tick(self):
        started = time.monotonic()
        try:
            context = self.inputs.snapshot()
            scan_header = context['source_scan_header']
            scan_arrival = context['source_scan_arrival']
            result = choose_motion(context, self.inputs.settings)
            self.compute_ms = (time.monotonic() - started) * 1000
            # Never label a result based on old observations as newly computed data.
            if (self.inputs.age(scan_header) > self.inputs.timeouts['scan'] or
                    time.monotonic() - scan_arrival > self.inputs.timeouts['scan'] or
                    self.inputs.age(context['odom_header']) > self.inputs.timeouts['odom'] or
                    self.inputs.age(context['reference_header']) > self.inputs.timeouts['odom']):
                raise NotReady('PLANNING_INPUT_EXPIRED', '计算期间输入已过期，本轮候选作废')
            self.publish(result, context, '已连接执行节点；是否驱动车辆请查看/autonomy/status' if self.get_parameter('execution_connected').value else '候选轨迹与速度仅供预览，车辆仍由遥控器驾驶')
        except NotReady as error:
            self.publish({'state':error.state,'speed':0.,'turn':0.,'trajectory':None},detail=error.message)
        except (ValueError, OverflowError) as error:
            self.publish({'state':'INVALID_INPUT','speed':0.,'turn':0.,'trajectory':None},detail=str(error))
        finally:
            # Let queued scan/odometry callbacks run before the next search.
            self.timer.timer_period_ns = int(max(.005, self.inputs.settings.control_dt - (time.monotonic()-started))*1e9)
            self.timer.reset()


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=DwaPreview();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
