"""Small fail-closed control latch and AGX_V2 encoder; no I/O or ROS imports."""
import math
import struct

CAN_FRAME = struct.Struct('=IB3x8s')


def encode_motion(v, w):
    """Match vendored ugv_sdk EncodeCanFrameV2: signed big endian, scale 1000.

    Lateral velocity/steering are zero for this differential SCOUT Mini.
    This function encodes only 0x111, never mode changes or fault resets.
    """
    if not all(math.isfinite(x) for x in (v, w)) or not 0 <= v <= .12 or abs(w) > .4:
        raise ValueError('Command outside configured commissioning envelope')
    return CAN_FRAME.pack(0x111, 8, struct.pack('>hhhh', int(v*1000), int(w*1000), 0, 0))


def encode_can_mode():
    """Official AGX_V2 ControlModeConfig CAN request, all reserved bytes zero."""
    return CAN_FRAME.pack(0x421, 8, b'\x01' + bytes(7))


def rate_limit_command(requested, previous, elapsed, linear_accel=.2, angular_accel=.8, control_dt=.1):
    """Bound output changes against the last applied command, not noisy feedback.

    Feedback is still used by collision prediction and overspeed protection.
    Emergency zero is handled by the caller, without this normal slew limiter.
    """
    values=(*requested,*previous,elapsed,linear_accel,angular_accel,control_dt)
    if not all(math.isfinite(v) for v in values) or elapsed<0 or min(linear_accel,angular_accel,control_dt)<=0:
        raise ValueError('Invalid slew-limit inputs')
    encode_motion(*requested);encode_motion(*previous)
    dt=min(elapsed,control_dt) # a delayed callback must not permit a big jump
    def approach(target,old,limit):return max(old-limit,min(old+limit,target))
    return (approach(requested[0],previous[0],linear_accel*dt),
            approach(requested[1],previous[1],angular_accel*dt))


def goal_progress(local_goal, heading_error, measured, distance_tolerance=.15, yaw_tolerance=.12):
    values=(*local_goal,heading_error,*measured,distance_tolerance,yaw_tolerance)
    if len(local_goal)!=2 or len(measured)!=2 or not all(math.isfinite(v) for v in values):
        raise ValueError('Invalid goal feedback')
    distance=math.hypot(*local_goal)
    angle=abs(math.atan2(math.sin(heading_error),math.cos(heading_error)))
    return {'distance_m':distance,'yaw_error_rad':angle,
            'near':distance<=distance_tolerance and angle<=yaw_tolerance,
            'stopped':abs(measured[0])<=.01 and abs(measured[1])<=.03}


class ControlLatch:
    def __init__(self, hardware_enabled=False, command_timeout=.30, run_limit=60., pause_timeout=1.):
        if not .05 <= command_timeout <= .35 or not 1 <= run_limit <= 300:
            raise ValueError('Invalid control watchdog settings')
        if not math.isfinite(pause_timeout) or not .1<=pause_timeout<=2:
            raise ValueError('pause_timeout must be in [.1,2] seconds')
        self.pause_timeout=pause_timeout;self.pause_started=None
        self.hardware_enabled=bool(hardware_enabled)
        self.command_timeout=command_timeout
        self.run_limit=run_limit
        self.armed=False
        self.has_control=False
        self.state='DISARMED'
        self.reason='软件使能关闭'
        self.enabled_at=0.
        self.valid_until=0.
        self.command=(0.,0.)

    def disarm(self, state='DISARMED', reason='软件使能关闭'):
        self.armed=False;self.valid_until=0.;self.command=(0.,0.);self.pause_started=None
        self.state=state;self.reason=reason

    def arm(self, now):
        if not self.hardware_enabled:
            self.disarm('HARDWARE_DISABLED','启动参数hardware_enabled=false，未接通CAN发送')
            return False
        self.armed=True;self.has_control=True;self.enabled_at=now;self.pause_started=None
        # No movement until a newly received command passes independent checks.
        self.valid_until=now+self.command_timeout;self.command=(0.,0.)
        self.state='ARMED_WAITING';self.reason='等待使能后新收到的复核候选'
        return True

    def accept(self, now, v, w, valid_until):
        encode_motion(v,w) # fail on NaN, reverse, overflow instead of silently clamp
        if not self.armed:return
        if valid_until <= now:
            self.disarm('EXPIRED_INPUT','输入在计算期间过期');return
        self.pause_started=None
        self.command=(float(v),float(w))
        self.valid_until=min(now+self.command_timeout,valid_until)
        self.state='AUTONOMOUS';self.reason='独立碰撞复核通过，限速执行'

    def pause(self, now, state, reason, valid_until):
        """Keep the task armed briefly, but request ONLY zero during recovery."""
        if not self.armed:return
        if self.pause_started is None:self.pause_started=now
        deadline=self.pause_started+self.pause_timeout
        if now>=deadline:
            self.disarm('PAUSE_TIMEOUT','短暂等待超过上限；已撤销使能，需要人工检查后重新使能')
            return
        self.command=(0.,0.)
        self.valid_until=min(now+self.command_timeout,valid_until,deadline)
        self.state=state;self.reason=reason

    def output(self, now, mode, feedback_ok):
        if self.armed:
            if not feedback_ok:self.disarm('FEEDBACK_INVALID','底盘故障或CAN反馈过期')
            elif mode!=1:self.disarm('MANUAL_OVERRIDE','底盘已离开CAN模式，撤销软件使能')
            elif now-self.enabled_at>self.run_limit:self.disarm('RUN_LIMIT','本次自动运行时间达到上限')
            elif self.pause_started is not None and now>=self.pause_started+self.pause_timeout:self.disarm('PAUSE_TIMEOUT','短暂等待超过上限，撤销使能')
            elif now>self.valid_until:self.disarm('COMMAND_TIMEOUT','执行候选已过期')
        # Before explicit arm OR CAN configuration, no CAN writes. Manual mode never
        # transmit motion commands. After disarming in CAN mode keep zero alive.
        transmit=self.hardware_enabled and self.has_control and mode!=3
        return transmit,self.command if self.armed else (0.,0.)
