"""Freshness, coordinate transforms and observation checks shared by preview nodes."""
from collections import deque
import math
import time
import numpy as np
from scipy.spatial import cKDTree
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from .local_planner_core import Settings, MapCollision, wrap


class NotReady(Exception):
    def __init__(self, state, message):
        self.state, self.message = state, message
        super().__init__(message)


def rotation(q):
    a = np.array([q.x, q.y, q.z, q.w], float)
    norm = np.linalg.norm(a)
    if not np.all(np.isfinite(a)) or norm < 1e-8:
        raise ValueError('Invalid quaternion')
    x, y, z, w = a / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def translation(tr):
    p = np.array([tr.translation.x, tr.translation.y, tr.translation.z], float)
    if not np.all(np.isfinite(p)):
        raise ValueError('Invalid transform translation')
    return p


def yaw(q):
    r = rotation(q)
    return math.atan2(r[1, 0], r[0, 0])


class PreviewInputs:
    def __init__(self, node):
        self.node = node
        defaults = vars(Settings()).copy()
        defaults.update({'execution_connected':False, 'robot_height': .60, 'scan_timeout': .45, 'odom_timeout': .30,
                         'path_timeout': 3., 'tf_timeout': .5, 'tf_sync_lag': .10,
                         'min_scan_points': 12, 'min_front_points': 3})
        for key, value in defaults.items():
            node.declare_parameter(key, value)
        self.settings = Settings(**{key: float(node.get_parameter(key).value) for key in vars(Settings())})
        self.robot_height = float(node.get_parameter('robot_height').value)
        if not 0 < self.robot_height <= .75:
            raise ValueError('Configured height exceeds local scan/map height band')
        self.timeouts = {key: float(node.get_parameter(key + '_timeout').value) for key in ('scan', 'odom', 'path', 'tf')}
        if not all(math.isfinite(x) and x > 0 for x in self.timeouts.values()):
            raise ValueError('Invalid input timeouts')
        self.tf_sync_lag = float(node.get_parameter('tf_sync_lag').value)
        if not 0 < self.tf_sync_lag <= .15:
            raise ValueError('tf_sync_lag must be in (0, .15] seconds')
        self.min_scan = int(node.get_parameter('min_scan_points').value)
        self.min_front = int(node.get_parameter('min_front_points').value)
        if self.min_scan < 1 or self.min_front < 1:
            raise ValueError('Observation counts must be positive')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, node)
        self.map = None
        self.map_error = None
        self.scan = self.path = self.odom = None
        self.arrival = {}
        self.scan_history = deque(maxlen=4)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latest_scan = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subs = [node.create_subscription(OccupancyGrid, '/map', self.on_map, retained),
                     node.create_subscription(Path, '/global_path', lambda m: self.receive('path', m), retained),
                     node.create_subscription(Odometry, '/odom', lambda m: self.receive('odom', m), 1),
                     node.create_subscription(LaserScan, '/local_scan', lambda m: self.receive('scan', m), latest_scan)]

    def receive(self, kind, msg):
        setattr(self, kind, msg)
        self.arrival[kind] = time.monotonic()
        if kind == 'scan':
            self.scan_history.append((msg, self.arrival[kind]))

    def on_map(self, msg):
        self.map = None
        try:
            if msg.header.frame_id not in ('map', '/map') or msg.info.width * msg.info.height != len(msg.data):
                raise ValueError('Invalid map frame or size')
            q = msg.info.origin.orientation
            if abs(q.x) > 1e-5 or abs(q.y) > 1e-5:
                raise ValueError('Map must be planar')
            self.map = MapCollision(np.asarray(msg.data).reshape(msg.info.height, msg.info.width), msg.info.resolution,
                                    (msg.info.origin.position.x, msg.info.origin.position.y, yaw(q)))
            self.map_error = None
        except (ValueError, OverflowError) as error:
            self.map_error = str(error)

    def age(self, header):
        stamp = header.stamp.sec * 1000000000 + header.stamp.nanosec
        if stamp <= 0:
            raise NotReady('INVALID_TIMESTAMP', '输入时间戳无效')
        return (self.node.get_clock().now().nanoseconds - stamp) / 1e9

    def snapshot(self):
        if self.map is None:
            raise NotReady('WAITING_FOR_MAP', self.map_error or '等待保存的二维地图')
        for key in ('odom', 'path'):
            msg = getattr(self, key)
            if msg is None:
                raise NotReady('WAITING_FOR_' + key.upper(), '等待 ' + key + ' 输入')
            age = self.age(msg.header)
            if time.monotonic() - self.arrival[key] > self.timeouts[key] or age > self.timeouts[key] or age < -.2:
                raise NotReady('STALE_' + key.upper(), key + ' 过期或时间异常')
        if not self.path.poses:
            raise NotReady('NO_GLOBAL_PATH', '全局路径为空，先选择可达目标')
        if self.path.header.frame_id not in ('map', '/map') or len(self.path.poses) > 50000:
            raise NotReady('INVALID_PATH', '全局路径坐标或长度不符合要求')
        if self.odom.header.frame_id != 'odom' or self.odom.child_frame_id not in ('base_link', 'base_footprint'):
            raise NotReady('INVALID_ODOM', '里程计坐标不符合当前配置')
        velocity = (self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z)
        if not all(math.isfinite(v) for v in velocity):
            raise NotReady('INVALID_ODOM', '里程计速度无效')
        path = np.array([[p.pose.position.x, p.pose.position.y] for p in self.path.poses], float)
        if not np.all(np.isfinite(path)):
            raise NotReady('INVALID_PATH', '路径存在非有限坐标')
        if self.scan is None:
            raise NotReady('WAITING_FOR_SCAN', '等待 scan 输入')
        try:
            # Odometry and its TF travel on separate DDS subscriptions. Asking
            # TF at the newest odom stamp races the TF message by one cycle.
            latest = self.tf.lookup_transform('map', 'base_footprint', Time(), timeout=Duration(seconds=0.))
            latest_time = Time.from_msg(latest.header.stamp)
            odom_time = Time.from_msg(self.odom.header.stamp)
            target_ns = min(latest_time.nanoseconds, odom_time.nanoseconds)
            target_time = Time(nanoseconds=target_ns)
            lag = (odom_time.nanoseconds - target_ns) / 1e9
            if lag > self.tf_sync_lag:
                raise NotReady('TF_SYNC_LAG', 'TF落后里程计过多，不能继续使用旧位姿')
            pose_tr = (latest if target_ns == latest_time.nanoseconds else
                       self.tf.lookup_transform('map', 'base_footprint', target_time, timeout=Duration(seconds=0.)))
            if not -.2 <= self.age(pose_tr.header) <= min(self.timeouts['tf'], self.timeouts['odom']):
                raise NotReady('STALE_TF', '定位参考时刻已过期')
        except TransformException as error:
            raise NotReady('WAITING_FOR_TF', '坐标转换不可用：' + str(error)) from error
        # Select the newest received scan whose transform time is available;
        # do not wait in a callback for a future TF that the same executor must receive.
        history = getattr(self, 'scan_history', [(self.scan, self.arrival['scan'])])
        available = [(m, arrived) for m, arrived in history
                     if Time.from_msg(m.header.stamp).nanoseconds <= target_ns]
        if not available:
            raise NotReady('WAITING_FOR_SCAN_TF', '扫描时间晚于可用TF，等待下一轮消息')
        scan, scan_arrival = max(available, key=lambda item: Time.from_msg(item[0].header.stamp).nanoseconds)
        age = self.age(scan.header)
        if time.monotonic()-scan_arrival > self.timeouts['scan'] or not -.2 <= age <= self.timeouts['scan']:
            raise NotReady('STALE_SCAN', '可用扫描已过期')
        if not (0 < len(scan.ranges) <= 5000 and math.isfinite(scan.angle_increment) and scan.angle_increment > 0 and
                math.isfinite(scan.angle_min) and 0 <= scan.range_min < scan.range_max):
            raise NotReady('INVALID_SCAN', '扫描格式无效')
        r = np.asarray(scan.ranges, float)
        valid = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        if valid.sum() < self.min_scan:
            raise NotReady('INSUFFICIENT_SCAN', '有效回波不足，不能把没有点当成无障碍')
        angles = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        points = np.column_stack([r[valid] * np.cos(angles[valid]), r[valid] * np.sin(angles[valid]), np.zeros(valid.sum())])
        try:
            # Motion-compensate the scan frame to the odometry reference time.
            scan_tr = self.tf.lookup_transform_full('base_footprint', target_time,
                scan.header.frame_id, Time.from_msg(scan.header.stamp), 'odom', timeout=Duration(seconds=0.))
            points = points @ rotation(scan_tr.transform.rotation).T + translation(scan_tr.transform)
            pos = translation(pose_tr.transform)
            heading = yaw(pose_tr.transform.rotation)
            goal_heading = wrap(yaw(self.path.poses[-1].pose.orientation) - heading)
        except (TransformException, ValueError) as error:
            raise NotReady('WAITING_FOR_TF', '坐标转换不可用：' + str(error)) from error
        points = points[:, :2]
        forward = np.abs(np.arctan2(points[:, 1], points[:, 0])) <= .7
        if forward.sum() < self.min_front:
            raise NotReady('INSUFFICIENT_FRONT_SCAN', '正前方有效观测不足')
        delta = path - pos[:2]
        c, s = math.cos(heading), math.sin(heading)
        local_path = delta @ np.array([[c, -s], [s, c]])
        return {'pose': (float(pos[0]), float(pos[1]), heading), 'velocity': velocity,
                'map': self.map, 'obstacles': points, 'obstacle_tree': cKDTree(points),
                'local_path': local_path, 'goal_heading': goal_heading, 'stamp': target_time.to_msg(), 'source_scan_header':scan.header,
                'source_scan_arrival':scan_arrival, 'odom_header':self.odom.header,
                'tf_sync_lag_s':lag, 'reference_header':pose_tr.header}
