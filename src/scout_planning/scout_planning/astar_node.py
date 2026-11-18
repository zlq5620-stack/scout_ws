"""Global path preview only: no velocity publisher or control action client."""
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from scout_planning.footprint_planner import FootprintPlanner


def quaternion_yaw(q):
    values = (q.x, q.y, q.z, q.w)
    norm = math.sqrt(sum(v * v for v in values))
    if not all(math.isfinite(v) for v in values) or norm < 1e-8:
        raise ValueError('Quaternion must be finite and nonzero')
    x, y, z, w = (v / norm for v in values)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('scout_astar')
        for key, value in {'clearance_radius': .47, 'replan_period': 1.,
                           'pose_timeout': .5, 'max_expansions': 15000,
                           'robot_length':.61, 'robot_width':.56, 'safety_margin':.04,
                           'search_time_budget':.35}.items():
            self.declare_parameter(key, value)
        self.radius = float(self.get_parameter('clearance_radius').value)
        self.pose_timeout = float(self.get_parameter('pose_timeout').value)
        period = float(self.get_parameter('replan_period').value)
        self.max_expansions = int(self.get_parameter('max_expansions').value)
        if not (all(math.isfinite(v) for v in (self.radius, self.pose_timeout, period)) and
                0 < self.radius <= 2 and self.pose_timeout > 0 and period > 0 and self.max_expansions > 0):
            raise ValueError('Invalid planner parameters')
        self.length=float(self.get_parameter("robot_length").value)
        self.width=float(self.get_parameter("robot_width").value)
        self.margin=float(self.get_parameter("safety_margin").value)
        self.search_budget=float(self.get_parameter("search_time_budget").value)
        if not math.isfinite(self.search_budget) or not 0 < self.search_budget <= 2.:
            raise ValueError("Search budget must be in (0,2] seconds")
        self.search_executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix="global_search")
        self.pending=None
        self.generation=0
        self.next_plan=0.
        self.replan_period=period
        self.planner = None
        self.map_msg = None
        self.goal = None
        self.last_status = None
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Path, '/global_path', retained)
        self.inflated_pub = self.create_publisher(OccupancyGrid, '/planning/inflated_map', retained)
        self.status_pub = self.create_publisher(String, '/planning/status', retained)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, retained)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_service(Trigger, '/planning/clear_goal', self.clear_goal)
        self.create_timer(.05, self.tick)
        self.empty_path()
        self.status('WAITING_FOR_MAP', '等待地图；此节点只显示路径，不控制车辆')

    def status(self, state, message, **extra):
        info = {'state': state, 'message': message, 'collision_model':'oriented_rectangle', 'collision_length_m':self.length+2*self.margin,
                'collision_width_m':self.width+2*self.margin, **extra}
        msg = String()
        msg.data = json.dumps(info, ensure_ascii=False)
        self.status_pub.publish(msg)
        key = state, message
        if key != self.last_status:
            self.get_logger().info(state + ': ' + message)
            self.last_status = key

    def empty_path(self):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def on_map(self, msg):
        self.generation+=1
        self.next_plan=0.
        self.empty_path()
        self.planner = None
        self.map_msg = None
        try:
            if msg.header.frame_id not in ('map', '/map'):
                raise ValueError('Map must be in map frame')
            if not (msg.info.width > 0 and msg.info.height > 0 and
                    len(msg.data) == msg.info.width * msg.info.height):
                raise ValueError('Invalid map dimensions/payload')
            q = msg.info.origin.orientation
            yaw = quaternion_yaw(q)
            if abs(q.x) > 1e-5 or abs(q.y) > 1e-5:
                raise ValueError('Map origin must be planar (yaw only)')
            if not math.isfinite(msg.info.origin.position.z) or abs(msg.info.origin.position.z) > .001:
                raise ValueError('Map ground origin must be at z=0')
            values = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
            self.planner = FootprintPlanner(values, msg.info.resolution,
                (msg.info.origin.position.x, msg.info.origin.position.y, yaw), self.length,self.width,self.margin)
            self.map_msg = msg
        except (ValueError, OverflowError) as error:
            self.status('INVALID_MAP', str(error))
            return
        inflated = OccupancyGrid()
        inflated.header = msg.header
        inflated.info = msg.info
        # Raw map only: a single inflated image cannot express all headings.
        # Keep the compatibility topic, explicitly without circular inflation.
        inflated.data = list(msg.data)
        self.inflated_pub.publish(inflated)
        self.status('WAITING_FOR_GOAL', '地图已加载，在 RViz 用 2D Goal Pose 选择目标')
        if self.goal is not None:
            self.tick()

    def on_goal(self, msg):
        self.generation+=1
        self.next_plan=0.
        # Rejecting a new goal must never leave the previous path visible.
        self.goal = None
        self.empty_path()
        try:
            if msg.header.frame_id not in ('map', '/map'):
                raise ValueError('目标必须位于 map 坐标系')
            x, y = msg.pose.position.x, msg.pose.position.y
            if not all(math.isfinite(v) for v in (x, y, msg.pose.position.z)):
                raise ValueError('目标坐标必须是有限数值')
            self.goal = x, y, quaternion_yaw(msg.pose.orientation)
        except ValueError as error:
            self.status('INVALID_GOAL', str(error))
            return
        self.tick()

    def clear_goal(self, request, response):
        self.generation+=1
        self.next_plan=0.
        self.goal = None
        self.empty_path()
        self.status('GOAL_CLEARED', '目标与路径已清除')
        response.success = True
        response.message = 'Goal and preview path cleared; no motion command sent.'
        return response

    def current_pose(self):
        tr=self.tf_buffer.lookup_transform('map','base_footprint',Time(),timeout=Duration(seconds=0.))
        now=self.get_clock().now()
        stamp=Time.from_msg(tr.header.stamp,clock_type=now.clock_type)
        age=(now-stamp).nanoseconds/1e9
        if not -self.pose_timeout <= age <= self.pose_timeout:
            raise ValueError('STALE_POSE')
        x,y=tr.transform.translation.x,tr.transform.translation.y
        if not all(math.isfinite(v) for v in (x,y)):raise ValueError('INVALID_POSE')
        return (x,y,quaternion_yaw(tr.transform.rotation))

    def tick(self):
        # Only the pure search runs in a worker. ROS callbacks and publishers
        # stay on the executor, so TF continues to arrive during a search.
        if self.pending is not None:
            future,revision,goal=self.pending
            if not future.done():return
            self.pending=None
            if revision!=self.generation or self.goal!=goal:return
            try:
                cells,state,expanded=future.result()
                start=self.current_pose()
            except (TransformException,ValueError) as error:
                self.empty_path();self.status('STALE_POSE','规划结束时缺少新鲜定位，路线已清空');return
            except Exception as error:
                self.empty_path();self.status('PLANNING_ERROR',str(error));return
            # The vehicle may move while the search is running. Reconnect the
            # current footprint to a checked prefix; never publish a stale start.
            if cells is not None:
                if not self.planner.clear([start]):
                    cells=None;state='START_BLOCKED'
                elif math.hypot(start[0]-cells[0][0],start[1]-cells[0][1])>1e-6 or abs(math.atan2(math.sin(start[2]-cells[0][2]),math.cos(start[2]-cells[0][2])))>1e-6:
                    joined=None
                    for index,pose in enumerate(cells[:24]):
                        if math.hypot(pose[0]-start[0],pose[1]-start[1])>.25:continue
                        connection=self.planner.connector(start,pose)
                        if connection is not None:
                            joined=[start]+connection[0]+cells[index+1:];break
                    if joined is None:
                        self.empty_path();self.status('REPLAN_FROM_CURRENT','车辆在搜索期间移动，等待从当前位置重规划');self.next_plan=0.;return
                    cells=joined
            self.publish_result(cells,state,expanded,start)
            return
        if self.goal is None:return
        if self.planner is None:
            self.empty_path();self.status('WAITING_FOR_MAP','尚无有效地图');return
        if time.monotonic()<self.next_plan:return
        try:start=self.current_pose()
        except (TransformException,ValueError):
            self.empty_path();self.status('STALE_POSE','缺少新鲜定位，请检查CAN反馈与AMCL');return
        goal=self.goal
        future=self.search_executor.submit(self.planner.search,start,goal,self.max_expansions,self.search_budget)
        self.pending=(future,self.generation,goal)
        self.next_plan=time.monotonic()+self.replan_period

    def publish_result(self,cells,state,expanded,start):
        x,y,start_heading=start
        if cells is None:
            self.empty_path()
            reasons = {
                'START_BLOCKED': '当前朝向的车体矩形与障碍、未知或地图边界相交，请检查定位和净空',
                'GOAL_BLOCKED': '目标朝向的矩形与障碍或未知区域相交，请调整目标位置或朝向',
                'NO_PATH': '当前已知空闲区域没有连通路线',
                'EXPANSION_LIMIT': '搜索达到配置上限，未生成路径',
                'SEARCH_TIME_LIMIT':'朝向搜索达到时间上限；未找到路线不代表物理上无法通行',
            }
            self.status(state, reasons[state], expanded=expanded)
            return
        # Preserve in-place rotations and their headings; do not derive yaw
        # from coincident XY coordinates or decimate checked corner motions.
        unique=cells
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for wx,wy,heading in unique:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = wx, wy
            pose.pose.orientation.z = math.sin(heading / 2.)
            pose.pose.orientation.w = math.cos(heading / 2.)
            path.poses.append(pose)
        self.publisher.publish(path)
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(unique, unique[1:]))
        self.status('PATH_READY', '路线已生成；底盘执行由独立控制节点决定',
                    path_points=len(unique), path_length_m=round(length, 3), expanded=expanded,
                    start=[round(x, 3), round(y, 3)], goal=list(self.goal[:2]),
                    start_yaw_rad=round(start_heading,3), goal_yaw_rad=round(self.goal[2],3))

    def destroy_node(self):
        if hasattr(self,'search_executor'):
            self.search_executor.shutdown(wait=False,cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GlobalPlanner()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
