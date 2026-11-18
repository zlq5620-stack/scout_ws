"""Opt-in hardware execution of independently rechecked local candidates.

Does not publish odometry/TF, clear faults or accept cmd_vel.
Mode configuration is a separate explicit service, allowed only in standby.
Nonzero AGX_V2 0x111 requires explicit software enable.
"""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import TwistStamped,PoseStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool,Trigger
from scout_avoidance.preview_inputs import PreviewInputs,NotReady
from scout_avoidance.local_planner_core import check_motion
from .autonomy_core import ControlLatch,rate_limit_command,goal_progress
from .can_transport import CanTransport


class AutonomyController(Node):
    def __init__(self):
        super().__init__('scout_autonomy_controller')
        self.inputs=PreviewInputs(self)
        for name,value in {'hardware_enabled':False,'interface':'can0','command_timeout':.30,'run_limit':60.,'pause_timeout':1.}.items():
            self.declare_parameter(name,value)
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Autonomy commissioning requires wall time')
        self.latch=ControlLatch(self.get_parameter('hardware_enabled').value,
            float(self.get_parameter('command_timeout').value),float(self.get_parameter('run_limit').value),
            float(self.get_parameter('pause_timeout').value))
        # This commissioning backend has hard upper bounds as well as planner limits.
        if self.inputs.settings.max_speed>.12 or self.inputs.settings.max_turn_rate>.4:
            raise ValueError('Commissioning limits: v<=0.12 m/s, |w|<=0.4 rad/s')
        self.can=CanTransport(self.get_parameter('interface').value)
        self.checked=None;self.checked_at=0.;self.enabled_at=0.
        self.mode_request_until=None
        self.last_applied_command=(0.,0.)
        self.last_command_at=time.monotonic()-.1
        self.last_validation={};self.last_stop_details={}
        self.last_state=None;self.compute_ms=0.
        self.create_subscription(TwistStamped,'/local_planning/checked_cmd',self.receive,1)
        self.create_subscription(PoseStamped,'/goal_pose',self.on_goal,1)
        self.create_service(SetBool,'/autonomy/enable',self.enable)
        self.create_service(Trigger,'/autonomy/select_can_mode',self.select_can_mode)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub=self.create_publisher(String,'/autonomy/status',qos)
        self.create_timer(.005,self.can.receive)
        self.validation_timer=self.create_timer(.1,self.validate)
        self.create_timer(.02,self.transmit)
        self.create_timer(.2,self.status)
        self.get_logger().info('默认DISARMED；hardware_enabled=%s；CAN模式需遥控器许可及必要的显式接口配置'%self.latch.hardware_enabled)

    def receive(self,msg):self.checked=msg;self.checked_at=time.monotonic()

    def on_goal(self,msg):
        if self.latch.armed:self.latch.disarm('GOAL_CHANGED','目标已改变；停稳并重新软件使能')

    def checked_velocity(self,now):
        if self.checked is None:raise NotReady('WAITING_FOR_CHECKED','等待局部复核速度')
        if now-self.checked_at>.25 or not -.1<=self.inputs.age(self.checked.header)<=.25:
            raise NotReady('STALE_CHECKED','局部复核速度过期')
        if self.checked.header.frame_id!='base_footprint':raise NotReady('INVALID_CHECKED','候选坐标错误')
        t=self.checked.twist
        if not all(math.isfinite(v) for v in (t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)) or any(abs(v)>1e-8 for v in (t.linear.y,t.linear.z,t.angular.x,t.angular.y)):
            raise NotReady('INVALID_CHECKED','速度存在非法分量')
        v,w=t.linear.x,t.angular.z
        if not 0<=v<=self.inputs.settings.max_speed+1e-9 or abs(w)>self.inputs.settings.max_turn_rate+1e-9:
            raise NotReady('VELOCITY_LIMIT','候选超出限速')
        return min(v,.12),max(-.4,min(w,.4))

    def select_can_mode(self,request,response):
        """Configure CAN only, never arm. Mode 3 remains a manual override."""
        try:
            if not self.latch.hardware_enabled:
                raise NotReady('HARDWARE_DISABLED','检查模式不允许发送模式配置')
            if self.latch.armed:
                raise NotReady('DISARM_FIRST','请先撤销软件使能并确认车辆停稳')
            self.can.receive();now=time.monotonic()
            if not self.can.healthy(now):
                raise NotReady('CAN_NOT_READY','底盘反馈不新鲜、故障或存在其他CAN运动发送者')
            if abs(self.can.motion[0])>.01 or abs(self.can.motion[1])>.03:
                raise NotReady('NOT_STATIONARY','配置接口前必须停稳')
            if self.can.mode==1:
                self.mode_request_until=None
                self.latch.disarm('CAN_MODE_READY','底盘已报告CAN模式；软件仍未使能')
                response.success=True;response.message='底盘已在CAN模式1；未使能'
                return response
            if self.can.mode!=0:
                raise NotReady('SELECT_STANDBY','请切到反馈为0的档位；不会覆盖遥控模式3')
            self.latch.disarm('CAN_MODE_REQUESTED','准备写入零速度并请求CAN模式；软件未使能')
            self.latch.has_control=True
            self.can.request_can_mode()
            self.mode_request_until=time.monotonic()+1.
            response.success=True
            response.message='已发送零速度→CAN接口配置→零速度；请查看control_mode是否确认为1，尚未使能'
        except (NotReady,OSError,ValueError) as error:
            state=getattr(error,'state','CAN_MODE_REQUEST_FAILED');message=getattr(error,'message',str(error))
            # Calling this service while armed must not modify mode or run state.
            if not self.latch.armed:self.latch.disarm(state,message)
            response.success=False;response.message=state+': '+message
        return response

    def update_mode_request(self):
        now=time.monotonic()
        if self.mode_request_until is None:
            # READY is a current condition, not a permanent success receipt.
            if self.latch.state=='CAN_MODE_READY':
                if not self.can.healthy(now):
                    self.latch.disarm('CAN_NOT_READY','CAN模式确认后反馈过期或底盘异常')
                elif self.can.mode!=1:
                    state='MANUAL_OVERRIDE' if self.can.mode==3 else 'CAN_MODE_LOST'
                    self.latch.disarm(state,'底盘已离开CAN模式1；软件仍未使能，请核对遥控器档位')
            return
        if not self.can.healthy(now):
            self.mode_request_until=None
            self.latch.disarm('CAN_NOT_READY','配置后反馈不新鲜或底盘异常')
        elif self.can.mode==1:
            self.mode_request_until=None
            self.latch.disarm('CAN_MODE_READY','底盘反馈已确认CAN模式1；软件仍未使能')
        elif self.can.mode==3:
            self.mode_request_until=None
            self.latch.disarm('MANUAL_OVERRIDE','遥控模式优先，CAN模式请求未继续')
        elif now>self.mode_request_until:
            self.mode_request_until=None
            self.latch.disarm('CAN_MODE_NOT_CONFIRMED','请求后底盘仍未确认CAN模式1，请检查遥控器与固件')

    def enable(self,request,response):
        if not request.data:
            self.latch.disarm();self.transmit();response.success=True;response.message='DISARMED：零速度请求；请确认实际车辆停止'
            return response
        try:
            now=time.monotonic()
            if not self.latch.hardware_enabled:raise NotReady('HARDWARE_DISABLED','需用hardware_enabled:=true启动，当前仅检查')
            self.can.receive()
            now=time.monotonic()
            if not self.can.healthy(now):raise NotReady('CAN_NOT_READY','CAN反馈过期、故障或存在其他发送程序')
            if self.can.mode!=1:raise NotReady('SELECT_CAN_MODE','需底盘确认模式1；若为0，请在未使能时调用/autonomy/select_can_mode')
            if abs(self.can.motion[0])>.01 or abs(self.can.motion[1])>.03:raise NotReady('NOT_STATIONARY','使能前请停稳')
            context=self.inputs.snapshot()
            progress=goal_progress(context['local_path'][-1],context['goal_heading'],context['velocity'],
                self.inputs.settings.goal_tolerance,self.inputs.settings.yaw_tolerance)
            if progress['near'] and progress['stopped']:
                raise NotReady('ALREADY_AT_GOAL','当前已在目标误差范围内，请选择新目标')
            if self.checked_velocity(now) == (0., 0.):
                raise NotReady('NO_SAFE_COMMAND', '当前复核候选为零，等待局部规划给出可执行动作')
            self.mode_request_until=None
            self.latch.arm(now);self.enabled_at=now
            self.last_applied_command=(0.,0.)
            self.last_command_at=now-self.inputs.settings.control_dt
            # Discard the pre-enable command. Await a new guard output.
            self.checked=None;self.checked_at=0.
            response.success=True;response.message='ARMED_WAITING：新候选复核通过后车辆可能立即运动'
        except (NotReady,ValueError) as error:
            state=getattr(error,'state','INVALID_INPUT');message=getattr(error,'message',str(error))
            self.latch.disarm(state,message);response.success=False;response.message=state+': '+message
        return response

    def validate(self):
        started=time.monotonic()
        try:
            if not self.can.healthy(started):
                raise NotReady('CAN_NOT_READY','CAN反馈过期、故障或存在其他运动发送者')
            context=self.inputs.snapshot()
            scan_header=context.get('source_scan_header',self.inputs.scan.header);scan_arrival=context.get('source_scan_arrival',self.inputs.arrival['scan'])
            odom_header=context.get('odom_header',self.inputs.odom.header)
            progress=goal_progress(context['local_path'][-1],context['goal_heading'],context['velocity'],
                self.inputs.settings.goal_tolerance,self.inputs.settings.yaw_tolerance)
            progress['can_stopped']=(self.can.motion is not None and abs(self.can.motion[0])<=.01 and abs(self.can.motion[1])<=.03)
            if self.latch.armed and progress['near']:
                self.last_validation={'goal':progress,'measured':list(context['velocity'])}
                if progress['stopped'] and progress['can_stopped']:
                    self.last_stop_details={'state':'GOAL_REACHED','reason':'目标位置和朝向在容差内，实测速度已接近零',**self.last_validation}
                    self.latch.disarm('GOAL_REACHED','已到达目标位置和朝向，停车并结束本次任务')
                else:
                    self.latch.pause(started,'GOAL_BRAKING','已进入目标误差范围，请求零速度并等待停稳',started+self.latch.command_timeout)
                return
            v,w=self.checked_velocity(started)
            # Brief wait after explicit enable, so old candidates cannot start motion.
            if self.latch.armed and self.checked_at<self.enabled_at: return
            requested=(v,w)
            previous=getattr(self,'last_applied_command',(0.,0.))
            elapsed=max(0.,started-getattr(self,'last_command_at',started-.1))
            self.last_validation={'requested':list(requested),'measured':list(context['velocity']),
                                  'previous_applied':list(previous),'slew_elapsed_s':elapsed,'goal':progress}
            if requested!=(0.,0.):
                v,w=rate_limit_command(requested,previous,elapsed,
                    self.inputs.settings.linear_accel,self.inputs.settings.angular_accel,self.inputs.settings.control_dt)
                self.last_validation['rate_limited']=[v,w]
                # Recheck the actual rate-limited output, including the modeled
                # transition from measured speed and its remaining braking tail.
                checked,reason,_=check_motion(v,w,context,self.inputs.settings)
                if checked is None:raise NotReady('COLLISION_STOP',reason)
            now=time.monotonic()
            remaining=min(self.inputs.timeouts['scan']-self.inputs.age(scan_header),
                          self.inputs.timeouts['scan']-(now-scan_arrival),
                          self.inputs.timeouts['odom']-self.inputs.age(odom_header),
                          self.inputs.timeouts['odom']-self.inputs.age(context.get('reference_header',odom_header)),
                          .25-(now-self.checked_at),.25-self.inputs.age(self.checked.header))
            if remaining<=0:raise NotReady('EXPIRED_INPUT','独立执行检查期间输入过期')
            if self.latch.armed:
                if requested==(0.,0.):
                    self.last_validation['pause_cause']='UPSTREAM_ZERO_BEFORE_GOAL'
                    self.latch.pause(now,'WAITING_FOR_SAFE_COMMAND','目标尚未到达；请求零速度等待新候选',now+remaining)
                    if not self.latch.armed:
                        self.last_stop_details={'state':self.latch.state,'reason':self.latch.reason,**self.last_validation}
                else:self.latch.accept(now,v,w,now+remaining)
            else:
                # Report shadow readiness without authorizing any physical output.
                self.shadow_state = ('NO_SAFE_COMMAND' if requested == (0., 0.)
                                     else 'SHADOW_READY')
        except (NotReady,ValueError,OverflowError) as error:
            state=getattr(error,'state','INVALID_INPUT');message=getattr(error,'message',str(error))
            # A short initial post-enable wait is bounded by command_timeout.
            if self.latch.armed and self.checked is None and time.monotonic()-self.enabled_at<.25:return
            transient={'WAITING_FOR_TF','TF_SYNC_LAG','WAITING_FOR_SCAN_TF','STALE_TF',
                       'STALE_SCAN','STALE_ODOM','STALE_PATH','NO_GLOBAL_PATH','STALE_CHECKED',
                       'WAITING_FOR_CHECKED','EXPIRED_INPUT','INSUFFICIENT_SCAN','INSUFFICIENT_FRONT_SCAN'}
            if self.latch.armed:
                if state in transient:
                    self.last_validation={**getattr(self,'last_validation',{}),'pause_cause':state,'pause_message':message}
                    self.latch.pause(time.monotonic(),'INPUT_RECOVERY_PAUSE',state+': '+message,
                        time.monotonic()+self.latch.command_timeout)
                    if not self.latch.armed:
                        self.last_stop_details={'state':self.latch.state,'reason':self.latch.reason,**getattr(self,'last_validation',{})}
                else:
                    self.last_stop_details={'state':state,'reason':message,**getattr(self,'last_validation',{})}
                    self.latch.disarm(state,message)
            self.shadow_state=state
        finally:
            self.compute_ms=(time.monotonic()-started)*1000
            self.validation_timer.timer_period_ns=int(max(.005,.1-(time.monotonic()-started))*1e9)
            self.validation_timer.reset()

    def transmit(self):
        now = time.monotonic()
        healthy = self.can.healthy(now)
        was_armed = self.latch.armed
        motion = self.can.motion
        if was_armed and healthy and self.can.mode == 1 and motion is not None:
            linear, angular = motion
            if abs(linear) > .15 or abs(angular) > .5:
                self.latch.disarm(
                    'FEEDBACK_OVERSPEED',
                    f'CAN实测速度超限（线速度 {linear:.3f} m/s，角速度 {angular:.3f} rad/s）')
        send,command=self.latch.output(now,self.can.mode,healthy)
        if was_armed and not self.latch.armed:
            self.last_stop_details={'state':self.latch.state,'reason':self.latch.reason,
                **getattr(self,'last_validation',{}),
                'latest_can_velocity':list(self.can.motion) if self.can.motion is not None else None}
        # Never compete with another CAN writer. Manual mode receives no writes.
        if send and not self.can.conflict:
            try:
                self.can.send(*command)
                if command!=getattr(self,'last_applied_command',(0.,0.)):
                    self.last_applied_command=command;self.last_command_at=now
            except (OSError,ValueError) as error:self.latch.disarm('CAN_SEND_FAILED',str(error))

    def status(self):
        self.update_mode_request()
        data={'state':self.latch.state,'reason':self.latch.reason,'armed':self.latch.armed,
              'mode_request_pending':self.mode_request_until is not None,
              'can_mode_ready':self.can.mode==1 and self.can.healthy(time.monotonic()),
              'hardware_enabled':self.latch.hardware_enabled,'shadow_state':getattr(self,'shadow_state','WAITING'),
              'control_mode':self.can.mode,'can_healthy':self.can.healthy(time.monotonic()),
              'other_can_writer':self.can.conflict,'can_error':self.can.io_error,
              'can_frames_sent':self.can.sent,'command_linear_mps':self.latch.command[0],
              'command_angular_rps':self.latch.command[1],'execution_check_ms':self.compute_ms,
              'last_validation':getattr(self,'last_validation',{}),
              'last_stop_details':getattr(self,'last_stop_details',{}),
              'pause_timeout_s':self.latch.pause_timeout,'pause_started':self.latch.pause_started}
        m=String();m.data=json.dumps(data,ensure_ascii=False);self.status_pub.publish(m)
        if self.last_state!=self.latch.state:
            self.get_logger().info(self.latch.state+': '+self.latch.reason);self.last_state=self.latch.state

    def destroy_node(self):
        if hasattr(self,'can'):
            self.latch.disarm('SHUTDOWN','节点关闭')
            try:self.transmit()
            finally:self.can.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=None
    try:node=AutonomyController();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
