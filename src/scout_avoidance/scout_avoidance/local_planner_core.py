"""Pure, forward-only dynamic-window preview with map/scan collision checks."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Settings:
    robot_length: float = .61
    robot_width: float = .56
    safety_margin: float = .04
    max_speed: float = .12
    max_turn_rate: float = .4
    linear_accel: float = .2
    angular_accel: float = .8
    brake_decel: float = .15
    control_dt: float = .1
    horizon: float = 2.
    sim_dt: float = .05
    lookahead: float = .5
    goal_tolerance: float = .15
    yaw_tolerance: float = .12

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('Local planning parameters must be positive and finite')
        if self.max_speed > .3 or self.max_turn_rate > 1. or not .02 <= self.sim_dt <= .1 or self.horizon > 5:
            raise ValueError('Preview limits exceeded')

    @property
    def radius(self):
        return math.hypot(*self.half_extents)

    @property
    def half_extents(self):
        return (self.robot_length / 2 + self.safety_margin,
                self.robot_width / 2 + self.safety_margin)


class MapCollision:
    """Circle versus occupied/unknown cell squares, including rotated map origins."""
    def __init__(self, values, resolution, origin):
        self.values = np.asarray(values)
        self.resolution = float(resolution)
        self.origin = tuple(origin)
        if self.values.ndim != 2 or not 0 < self.values.size <= 2000000:
            raise ValueError('Invalid local collision map shape')
        if not math.isfinite(self.resolution) or self.resolution <= 0 or not all(math.isfinite(x) for x in self.origin):
            raise ValueError('Invalid map geometry')
        if not np.all(np.isfinite(self.values)) or np.any((self.values < -1) | (self.values > 100)):
            raise ValueError('Invalid occupancy data')
        self.height, self.width = self.values.shape
        ij = np.argwhere((self.values < 0) | (self.values >= 60))
        self.centers = (ij[:, ::-1] + .5) * resolution
        self.tree = cKDTree(self.centers) if len(ij) else None

    def clear(self, world_points, radius):
        points = np.asarray(world_points, float).reshape(-1, 2)
        if not np.all(np.isfinite(points)) or not math.isfinite(radius) or radius <= 0:
            return False
        ox, oy, a = self.origin
        delta = points - [ox, oy]
        c, s = math.cos(a), math.sin(a)
        local = delta @ np.array([[c, -s], [s, c]])
        if np.any(local[:, 0] <= radius) or np.any(local[:, 0] >= self.width * self.resolution - radius):
            return False
        if np.any(local[:, 1] <= radius) or np.any(local[:, 1] >= self.height * self.resolution - radius):
            return False
        if self.tree is None:
            return True
        r = radius + self.resolution / math.sqrt(2)
        distances = self.tree.query(local, k=1)[0]
        for p in local[distances <= r]:
            ids = self.tree.query_ball_point(p, r)
            offsets = np.maximum(np.abs(self.centers[ids] - p) - self.resolution / 2, 0.)
            if np.any(np.linalg.norm(offsets, axis=1) <= radius):
                return False
        return True

    def clear_rectangle(self, world_poses, half_length, half_width):
        """Separating-axis test: oriented footprint vs full occupied cell squares."""
        poses = np.asarray(world_poses, float).reshape(-1, 3)
        if not np.all(np.isfinite(poses)) or not all(math.isfinite(v) and v > 0 for v in (half_length, half_width)):
            return False
        ox, oy, a = self.origin
        c, s = math.cos(a), math.sin(a)
        centers = (poses[:, :2] - [ox, oy]) @ np.array([[c, -s], [s, c]])
        angles = poses[:, 2] - a
        radius = math.hypot(half_length, half_width)
        cell_half = self.resolution / 2
        c, sn = np.cos(angles), np.sin(angles)
        ac, ass = np.abs(c), np.abs(sn)
        ex = half_length * ac + half_width * ass
        ey = half_length * ass + half_width * ac
        if (np.any(centers[:, 0] <= ex) or np.any(centers[:, 0] >= self.width * self.resolution - ex)
                or np.any(centers[:, 1] <= ey) or np.any(centers[:, 1] >= self.height * self.resolution - ey)):
            return False
        if self.tree is None:
            return True
        # One batched spatial query, then vectorized SAT for all nearby cell pairs.
        neighbors = self.tree.query_ball_point(centers, radius + self.resolution / math.sqrt(2))
        lengths = np.fromiter((len(ids) for ids in neighbors), dtype=int, count=len(centers))
        if not lengths.sum():
            return True
        rows = np.repeat(np.arange(len(centers)), lengths)
        ids = np.concatenate([ids for ids in neighbors if len(ids)]).astype(int)
        d = self.centers[ids] - centers[rows]
        overlap = ((np.abs(d[:, 0]) <= ex[rows] + cell_half)
                   & (np.abs(d[:, 1]) <= ey[rows] + cell_half)
                   & (np.abs(d[:, 0] * c[rows] + d[:, 1] * sn[rows]) <= half_length + cell_half * (ac[rows] + ass[rows]))
                   & (np.abs(-d[:, 0] * sn[rows] + d[:, 1] * c[rows]) <= half_width + cell_half * (ac[rows] + ass[rows])))
        return not np.any(overlap)


def scan_rectangle_clearance(traj, tree, half_length, half_width):
    """Minimum signed point distance, using batched array operations."""
    poses = np.asarray(traj)
    radius = math.hypot(half_length, half_width)
    # A circumscribed circle is only a fast proof of non-collision. When points
    # are nearer, always use the rectangle; the circle never rejects a motion.
    center_distances = tree.query(poses[:, :2], k=1)[0]
    lower_bound = float(np.min(center_distances) - radius)
    if lower_bound > 0:
        return lower_bound
    # Pick one common conservative search set for the whole trajectory. Every
    # pose's nearest obstacle is at most distance-to-nearest-at-start + travel.
    travel = np.linalg.norm(poses[:, :2] - poses[0, :2], axis=1).max()
    nearest = float(tree.query(poses[0, :2], k=1)[0])
    ids = tree.query_ball_point(poses[0, :2], nearest + 2 * travel + radius + 1e-9)
    obstacles = tree.data[ids]
    c, sn = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    clearance = math.inf
    # Bound temporary arrays even for unusually dense scan input.
    for offset in range(0, len(obstacles), 1024):
        delta = obstacles[None, offset:offset + 1024, :] - poses[:, None, :2]
        qx = np.abs(delta[:, :, 0] * c + delta[:, :, 1] * sn) - half_length
        qy = np.abs(-delta[:, :, 0] * sn + delta[:, :, 1] * c) - half_width
        signed = np.hypot(np.maximum(qx, 0.), np.maximum(qy, 0.)) + np.minimum(np.maximum(qx, qy), 0.)
        clearance = min(clearance, float(np.min(signed)))
    return clearance


def integrate(x, y, yaw, speed, turn, dt):
    a = turn * dt
    if abs(turn) < 1e-8:
        return x + speed * math.cos(yaw) * dt, y + speed * math.sin(yaw) * dt, yaw
    return (x + speed / turn * (math.sin(yaw + a) - math.sin(yaw)),
            y + speed / turn * (math.cos(yaw) - math.cos(yaw + a)), wrap(yaw + a))


def trajectory(speed, turn, settings, initial_velocity=None):
    """Measured-speed transition to the candidate, then a modeled braking tail."""
    p = (0., 0., 0.)
    points = [p]
    count = int(math.ceil(settings.horizon / settings.sim_dt))
    dt = settings.horizon / count
    v,w=(speed,turn) if initial_velocity is None else initial_velocity
    def approach(old,target,amount):return max(old-amount,min(old+amount,target))
    for _ in range(count):
        accel=settings.brake_decel if abs(speed)<abs(v) else settings.linear_accel
        nv=approach(v,speed,accel*dt)
        nw=approach(w,turn,settings.angular_accel*dt)
        p = integrate(*p, (v+nv)/2, (w+nw)/2, dt)
        points.append(p);v,w=nv,nw
    active_count = len(points)
    # Check remaining motion under the configured braking model, not measured braking.
    for _ in range(400):
        if abs(v) < 1e-8 and abs(w) < 1e-8:
            break
        nv = approach(v,0.,settings.brake_decel * settings.sim_dt)
        nw = math.copysign(max(0., abs(w) - settings.angular_accel * settings.sim_dt), w)
        p = integrate(*p, (v + nv) / 2, (w + nw) / 2, settings.sim_dt)
        points.append(p)
        v, w = nv, nw
    else:
        raise ValueError('Braking simulation limit exceeded')
    return np.asarray(points), active_count


def world_xy(local, pose):
    x, y, a = pose
    c, s = math.cos(a), math.sin(a)
    return np.asarray(local) @ np.array([[c, s], [-s, c]]) + [x, y]


def check_motion(speed, turn, context, settings):
    if not all(math.isfinite(v) for v in (speed, turn)) or not 0 <= speed <= settings.max_speed + 1e-9 or abs(turn) > settings.max_turn_rate + 1e-9:
        return None, 'VELOCITY_LIMIT', 0.
    if context['obstacle_tree'].n < 1:
        return None, 'INSUFFICIENT_SCAN', 0.
    initial=context.get('velocity',(speed,turn))
    if (not all(math.isfinite(x) for x in initial) or abs(initial[0])>settings.max_speed+.03+1e-9
            or abs(initial[1])>settings.max_turn_rate+.1+1e-9):
        return None,'FEEDBACK_SPEED_LIMIT',0.
    traj, active = trajectory(speed, turn, settings, initial)
    # Bound translation AND corner rotation between adjacent trajectory samples.
    padding = ((max(settings.max_speed,abs(initial[0])) + settings.radius * max(settings.max_turn_rate,abs(initial[1])))
               * settings.sim_dt / 2 + .002)
    half_length, half_width = (v + padding for v in settings.half_extents)
    clearance = scan_rectangle_clearance(traj, context['obstacle_tree'], half_length, half_width)
    if clearance <= 0:
        return None, 'SCAN_COLLISION', clearance
    xy = world_xy(traj[:, :2], context['pose'])
    poses = np.column_stack((xy, traj[:, 2] + context['pose'][2]))
    if not context['map'].clear_rectangle(poses, half_length, half_width):
        return None, 'MAP_COLLISION_OR_UNKNOWN', clearance
    return (traj, active), 'CLEAR', clearance


def velocity_window(current_v, current_w, settings):
    if (not all(math.isfinite(x) for x in (current_v,current_w)) or
            abs(current_v)>settings.max_speed+.03+1e-9 or
            abs(current_w)>settings.max_turn_rate+.1+1e-9):
        return None
    # Near the boundary, the measured speed may briefly exceed the target cap.
    # Sample the capped deceleration target instead of producing an empty window;
    # trajectory() still models the full transition from the measured speed.
    lo_v = min(settings.max_speed,max(0., current_v - settings.linear_accel * settings.control_dt))
    hi_v = min(settings.max_speed,max(0., current_v + settings.linear_accel * settings.control_dt))
    lo_w = min(settings.max_turn_rate,max(-settings.max_turn_rate,current_w - settings.angular_accel * settings.control_dt))
    hi_w = min(settings.max_turn_rate,max(-settings.max_turn_rate,current_w + settings.angular_accel * settings.control_dt))
    v = np.linspace(lo_v, hi_v, 5)
    w = np.linspace(lo_w, hi_w, 9)
    if lo_v <= 0 <= hi_v:
        v = np.unique(np.append(v, 0.))
    if lo_w <= 0 <= hi_w:
        w = np.unique(np.append(w, 0.))
    return np.unique(v), np.unique(w)


def choose_motion(context, settings):
    path = context['local_path']
    goal_distance = float(np.linalg.norm(path[-1]))
    goal_heading = context['goal_heading']
    if goal_distance < settings.goal_tolerance and abs(wrap(goal_heading)) < settings.yaw_tolerance:
        state = 'GOAL_REACHED' if abs(context['velocity'][0]) < .03 and abs(context['velocity'][1]) < .05 else 'GOAL_NEAR_STOP'
        return {'state': state, 'speed': 0., 'turn': 0., 'trajectory': None, 'clearance': 0.}
    window = velocity_window(*context['velocity'], settings)
    if window is None:
        return {'state': 'FEEDBACK_OUTSIDE_WINDOW', 'speed': 0., 'turn': 0., 'trajectory': None, 'clearance': 0.}
    nearest = int(np.argmin(np.linalg.norm(path, axis=1)))
    target = path[-1]
    accumulated = 0.
    for i in range(nearest + 1, len(path)):
        accumulated += np.linalg.norm(path[i] - path[i - 1])
        if accumulated >= settings.lookahead:
            target = path[i]
            break
    align = goal_distance < settings.goal_tolerance
    path_tree = cKDTree(path)
    best = None
    rejected = {}
    for v in window[0]:
        if align and v > 1e-8:
            continue
        for w in window[1]:
            checked, reason, clearance = check_motion(float(v), float(w), context, settings)
            if checked is None:
                rejected[reason] = rejected.get(reason, 0) + 1
                continue
            traj, active = checked
            end = traj[active - 1]
            heading = abs(wrap(goal_heading - end[2])) if align else abs(wrap(math.atan2(target[1] - end[1], target[0] - end[0]) - end[2]))
            distance = float(np.linalg.norm(end[:2] - target))
            deviation = float(np.mean(path_tree.query(traj[:active, :2], k=1)[0]))
            cost = distance + .3 * heading + .3 * deviation + .02 / max(.03, clearance) - .2 * v / settings.max_speed
            if best is None or cost < best['cost']:
                best = {'state': 'ALIGN_GOAL' if align else 'CANDIDATE_READY',
                        'speed': float(v), 'turn': float(w), 'trajectory': traj,
                        'active_count': active, 'clearance': clearance, 'cost': float(cost)}
    return best or {'state': 'NO_CLEAR_CANDIDATE', 'speed': 0., 'turn': 0., 'trajectory': None,
                    'clearance': 0., 'rejected': rejected}
