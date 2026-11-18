"""Receive-only AGX_V2 CAN odometry for handheld-RC localization.

No CAN send calls and no ROS command subscriptions. Integrates measured body
linear/angular velocity, rather than treating ROS command speed as feedback.
"""
import math
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

CAN_FRAME = struct.Struct('=IB3x8s')
# Match standard data IDs and exclude extended/RTR frames. CAN_ERR_FLAG must
# NOT be set in a CAN_RAW_FILTER mask: Linux routes such filters to RX_ERR.
CAN_DATA_FILTER_MASK = socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_SFF_MASK


def decode_frame(raw):
    if len(raw) != CAN_FRAME.size:
        return None
    can_id, size, payload = CAN_FRAME.unpack(raw)
    # Only standard data frames. Reject EFF, RTR and CAN error frames.
    if can_id & 0xE0000000 or size != 8:
        return None
    if can_id == 0x221:
        linear, angular, lateral, steering = struct.unpack('>hhhh', payload)
        return 'motion', (linear / 1000., angular / 1000.)
    if can_id == 0x211:
        state, mode, voltage, error = struct.unpack('>BBHH', payload[:6])
        return 'system', (state, mode, voltage / 10., error)
    return None


def integrate_pose(pose, linear, angular, dt):
    """Exact constant-twist SE(2) integration; works for straight/curved motion."""
    x, y, yaw = pose
    angle = angular * dt
    if abs(angular) < 1e-8:
        x += linear * math.cos(yaw) * dt
        y += linear * math.sin(yaw) * dt
    else:
        x += linear / angular * (math.sin(yaw + angle) - math.sin(yaw))
        y += linear / angular * (math.cos(yaw) - math.cos(yaw + angle))
    return x, y, math.atan2(math.sin(yaw + angle), math.cos(yaw + angle))


class CanOdometry(Node):
    def __init__(self):
        super().__init__('can_odometry_readonly')
        defaults = {'interface': 'can0', 'odom_frame': 'odom', 'base_frame': 'base_link',
                    'base_height': .079, 'motion_timeout': .30, 'system_timeout': 1.0,
                    'max_integration_gap': .15}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Live CAN receiver requires wall time; for replay set start_odometry:=false and replay odometry/TF')
        self.interface = self.get_parameter('interface').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.base_height = float(self.get_parameter('base_height').value)
        self.motion_timeout = float(self.get_parameter('motion_timeout').value)
        self.system_timeout = float(self.get_parameter('system_timeout').value)
        self.max_gap = float(self.get_parameter('max_integration_gap').value)
        if not (0 < self.max_gap <= self.motion_timeout and self.system_timeout > 0):
            raise ValueError('Invalid CAN timeout parameters')
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # Kernel filtering: receive only system/motion feedback, never send.
        filters = b''.join(struct.pack('=II', i, CAN_DATA_FILTER_MASK) for i in (0x211, 0x221))
        self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        try:
            self.sock.bind((self.interface,))
        except OSError:
            self.sock.close()
            raise
        self.sock.setblocking(False)
        self.pose = (0., 0., 0.)
        self.last_motion = None
        self.last_system = None
        self.system = None
        self.ready = False
        self.gap_count = 0
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/localization/can_diagnostics', 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(.005, self.receive)
        self.create_timer(.5, self.diagnostics)
        self.get_logger().info('Receive-only AGX_V2 odometry on %s; no CAN commands or cmd_vel subscription' % self.interface)

    def receive(self):
        # Bound each drain cycle; normal feedback is about 50 Hz.
        for _ in range(200):
            try:
                raw = self.sock.recv(CAN_FRAME.size)
            except BlockingIOError:
                return
            except OSError as error:
                self.get_logger().error('CAN receive failed: %s' % error, throttle_duration_sec=2.0)
                self.ready = False
                return
            decoded = decode_frame(raw)
            if decoded is None:
                continue
            now = time.monotonic()
            kind, values = decoded
            if kind == 'system':
                self.system = values
                self.last_system = now
                continue
            linear, angular = values
            system_ok = (self.system is not None and self.last_system is not None and
                         now - self.last_system < self.system_timeout and
                         self.system[0] == 0 and self.system[3] == 0)
            previous = self.last_motion
            self.last_motion = now
            was_ready = self.ready
            self.ready = system_ok
            if not system_ok:
                continue
            if previous is not None and was_ready:
                dt = now - previous
                if 0 < dt <= self.max_gap:
                    self.pose = integrate_pose(self.pose, linear, angular, dt)
                else:
                    self.gap_count += 1
                    self.get_logger().warn('CAN motion gap; interval not integrated. Recheck localization.')
            self.publish_odometry(linear, angular)

    def publish_odometry(self, linear, angular):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y, yaw = self.pose
        msg.pose.pose.position.z = self.base_height
        msg.pose.pose.orientation.z = math.sin(yaw / 2.)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.)
        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular
        # Initial uncertainty settings, not sensor calibration results.
        msg.pose.covariance = [0.] * 36
        msg.twist.covariance = [0.] * 36
        for i, p, v in ((0, .05, .02), (7, .05, .02), (14, 1e3, 1e3),
                        (21, 1e3, 1e3), (28, 1e3, 1e3), (35, .10, .05)):
            msg.pose.covariance[i] = p
            msg.twist.covariance[i] = v
        self.odom_pub.publish(msg)
        tr = TransformStamped()
        tr.header = msg.header
        tr.child_frame_id = self.base_frame
        tr.transform.translation.x = msg.pose.pose.position.x
        tr.transform.translation.y = msg.pose.pose.position.y
        tr.transform.translation.z = self.base_height
        tr.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(tr)

    def diagnostics(self):
        now = time.monotonic()
        status = DiagnosticStatus()
        status.name = 'localization/can_receive_only'
        status.hardware_id = self.interface
        stale = (self.last_motion is None or now - self.last_motion > self.motion_timeout or
                 self.last_system is None or now - self.last_system > self.system_timeout)
        if stale:
            self.ready = False
            status.level = DiagnosticStatus.ERROR
            status.message = 'Missing/stale CAN feedback; no fresh odometry/TF'
        elif not self.ready:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Chassis fault; odometry suspended'
        elif self.system[1] not in (1, 3):
            status.level = DiagnosticStatus.WARN
            status.message = 'Feedback active; control mode is neither CAN (1) nor RC (3)'
        else:
            status.level = DiagnosticStatus.OK
            status.message = ('CAN mode; feedback active; receive only' if self.system[1] == 1
                              else 'RC mode; feedback active; receive only')
        values = {'vehicle_state': self.system[0] if self.system else 'unknown',
                  'control_mode': self.system[1] if self.system else 'unknown',
                  'battery_voltage': self.system[2] if self.system else 'unknown',
                  'error_code': hex(self.system[3]) if self.system else 'unknown',
                  'integration_gaps': self.gap_count}
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diag_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'sock'):
            self.sock.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CanOdometry()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
