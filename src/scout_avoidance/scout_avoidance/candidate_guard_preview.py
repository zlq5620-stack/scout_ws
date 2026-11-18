"""Recheck local candidates; publish checked preview only, never /cmd_vel."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from .preview_inputs import PreviewInputs, NotReady
from .local_planner_core import check_motion


class CandidateGuard(Node):
    def __init__(self):
        super().__init__('candidate_guard_preview')
        self.inputs = PreviewInputs(self)
        self.declare_parameter('candidate_timeout', .30)
        self.timeout = float(self.get_parameter('candidate_timeout').value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('Invalid candidate timeout')
        self.candidate = None
        self.arrival = 0.
        self.create_subscription(TwistStamped, '/local_planning/candidate_cmd', self.receive, 1)
        self.command = self.create_publisher(TwistStamped, '/local_planning/checked_cmd', 1)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/local_planning/guard_status', retained)
        self.create_timer(self.inputs.settings.control_dt, self.tick)
        self.last_state = None
        self.get_logger().info('Candidate guard: independent execution node may consume checked_cmd; see /autonomy/status' if self.get_parameter('execution_connected').value else 'Candidate guard preview only; hardware execution is disabled')

    def receive(self, msg):
        self.candidate = msg
        self.arrival = time.monotonic()

    def check(self):
        if self.candidate is None:
            raise NotReady('WAITING_FOR_CANDIDATE', '等待局部候选速度')
        age = self.inputs.age(self.candidate.header)
        if time.monotonic() - self.arrival > self.timeout or age > self.timeout or age < -.2:
            raise NotReady('STALE_CANDIDATE', '候选速度已过期')
        if self.candidate.header.frame_id != 'base_footprint':
            raise NotReady('INVALID_CANDIDATE', '候选速度坐标错误')
        t = self.candidate.twist
        values = (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)
        if not all(math.isfinite(v) for v in values) or any(abs(v) > 1e-8 for v in values[1:5]):
            raise NotReady('INVALID_CANDIDATE', '候选速度包含非有限值或不支持的分量')
        context = self.inputs.snapshot()
        self.checked_stamp=context['stamp']
        v, w = t.linear.x, t.angular.z
        # Zero recommendation is always allowed, but no claim of instantaneous braking.
        if abs(v) < 1e-8 and abs(w) < 1e-8:
            return 0., 0., 'ZERO_CANDIDATE', '零速度建议；执行状态见/autonomy/status'
        # The DWA window belonged to its source-time feedback. Independently
        # re-predict from CURRENT measured speed; do not treat a changed window
        # at this later subscriber as a hardware fault.
        checked, reason, clearance = check_motion(v, w, context, self.inputs.settings)
        if checked is None:
            raise NotReady('COLLISION_CHECK_REJECTED', reason)
        if (self.inputs.age(self.candidate.header)>self.timeout or
                time.monotonic()-self.arrival>self.timeout or
                self.inputs.age(context['source_scan_header'])>self.inputs.timeouts['scan'] or
                self.inputs.age(context['reference_header'])>self.inputs.timeouts['odom']):
            raise NotReady('EXPIRED_CHECK','复核完成时输入已过期')
        return v, w, 'CHECKED_PREVIEW', ('重新检查通过，执行节点决定是否发送CAN' if self.get_parameter('execution_connected').value else '重新检查通过，仅供预览，未接入车辆控制')

    def tick(self):
        try:
            v, w, state, message = self.check()
        except NotReady as error:
            v, w, state, message = 0., 0., error.state, error.message
        except (ValueError, OverflowError) as error:
            v, w, state, message = 0., 0., 'INVALID_INPUT', str(error)
        cmd = TwistStamped()
        cmd.header.frame_id = 'base_footprint'
        cmd.header.stamp = self.checked_stamp if state=='CHECKED_PREVIEW' else self.get_clock().now().to_msg()
        cmd.twist.linear.x, cmd.twist.angular.z = float(v), float(w)
        self.command.publish(cmd)
        status = String()
        status.data = json.dumps({'state':state,'preview_only':not self.get_parameter('execution_connected').value,'message':message,
                                  'checked_linear_mps':v,'checked_angular_rps':w},ensure_ascii=False)
        self.status.publish(status)
        if state != self.last_state:
            self.get_logger().info(state + ': ' + message)
            self.last_state = state


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=CandidateGuard();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
