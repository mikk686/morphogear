#!/usr/bin/env python3
"""Simple ROS2/MAVROS controller. Commands from Unity; FCU remains authoritative."""
import copy
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State, ExtendedState, StatusText, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


MASK_VELOCITY_YAW_RATE = (
    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
    PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
    PositionTarget.IGNORE_YAW)
MASK_VELOCITY_YAW_ANGLE = (
    PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
    PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
    PositionTarget.IGNORE_YAW_RATE)


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def xyz(v):
    return (v.x, v.y, v.z)


def finite(v):
    return all(math.isfinite(x) for x in v)


def inside(p, lo, hi, margin=0):
    return p is not None and all(a + margin <= x <= b - margin for x, a, b in zip(p, lo, hi))


class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')
        defaults = dict(frame_id='map', pose_topic='/mavros/vision_pose/pose',
                        box_min=[-2.0, -2.0, -0.2], box_max=[2.0, 2.0, 2.5],
                        bounds_configured=False, boundary_margin=0.2,
                        max_speed=0.5, max_yaw_rate=0.5, command_timeout=0.5,
                        pose_timeout=0.5, state_timeout=3.0, prediction_seconds=0.5,
                        service_timeout=5.0, action_timeout=30.0, publish_hz=20.0,
                        takeoff_altitude=1.0, home_z=0.0, position_tolerance=0.15)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.lo, self.hi = tuple(self.cfg['box_min']), tuple(self.cfg['box_max'])
        self.margin = self.cfg['boundary_margin']
        self.configured = self.cfg['bounds_configured']
        self.validate_box(self.lo, self.hi, self.margin)
        for k, v in self.cfg.items():
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                if not math.isfinite(v) or (k not in ('home_z', 'boundary_margin') and v <= 0):
                    raise ValueError('Invalid parameter: ' + k)
        if not self.cfg['frame_id'] or not self.cfg['pose_topic']:
            raise ValueError('Frame and pose topic must not be empty')
        self.state, self.ext = State(), ExtendedState()
        self.state_time = self.pose_time = self.ext_time = -math.inf
        self.position = self.goal = self.pose_command = None
        self.velocity = (0., 0., 0., 0.)
        self.velocity_time = -math.inf
        self.is_body_frame, self.use_yaw_rate = False, True
        self.yaw = None
        self.control = 'idle'
        self.operation = None  # one service request + optional telemetry confirmation
        self.event_seq = 0
        self.last_event = dict(sequence=0, command='', result='ready', detail='')
        self.fcu_text = ''
        self.stop_reason = ''
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.status_pub = self.create_publisher(String, '/state_info', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 1)
        self.vel_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 1)
        self.subs = []
        for typ, topic, callback, qos in [
            (State, '/mavros/state', self.on_state, qos_profile_sensor_data),
            (ExtendedState, '/mavros/extended_state', self.on_extended, qos_profile_sensor_data),
            (StatusText, '/mavros/statustext/recv', self.on_text, qos_profile_sensor_data),
            (PoseStamped, self.cfg['pose_topic'], self.on_pose, qos_profile_sensor_data),
            (Bool, 'arm_unity', self.on_arm, 1),
            (String, 'mode_unity', self.on_mode, 1),
            (Bool, 'takeoff_unity', self.on_takeoff, 1),
            (Bool, 'land_unity', self.on_land, 1),
            (String, 'bounds_unity', self.on_bounds, 1),
            (PoseStamped, 'setpose_unity', self.on_position, 1),
            (String, 'motion_unity', self.on_motion, 1),
            (TwistStamped, 'setvel_unity', self.on_velocity, 1)]:
            self.subs.append(self.create_subscription(typ, topic, callback, qos))
        self.create_timer(1 / self.cfg['publish_hz'], self.tick)
        self.create_timer(0.5, self.publish_status)

    @staticmethod
    def validate_box(lo, hi, margin):
        if (len(lo) != 3 or len(hi) != 3 or not finite((*lo, *hi, margin))
                or margin < 0 or any(a + 2 * margin >= b for a, b in zip(lo, hi))):
            raise ValueError('Need finite min < max on each ENU axis, with room for margin')

    def fresh(self, when, timeout):
        return time.monotonic() - when <= timeout

    def linked(self):
        return self.state.connected and self.fresh(self.state_time, self.cfg['state_timeout'])

    def grounded(self):
        return self.fresh(self.ext_time, self.cfg['state_timeout']) and self.ext.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def on_state(self, msg):
        self.state, self.state_time = msg, time.monotonic()

    def on_extended(self, msg):
        self.ext, self.ext_time = msg, time.monotonic()

    def on_text(self, msg):
        self.fcu_text = msg.text

    def on_pose(self, msg):
        # Quaternion describes base_link in configured ENU map.
        self.yaw = None
        q = msg.pose.orientation
        values = (q.x, q.y, q.z, q.w)
        if finite(values) and math.hypot(*values) > 1e-6:
            x, y, z, w = (v / math.hypot(*values) for v in values)
            self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y*y + z*z))
        # Receipt freshness, frame and finite coordinates; no monotonic stamp lockout.
        if msg.header.frame_id == self.cfg['frame_id'] and finite(xyz(msg.pose.position)):
            self.position, self.pose_time = xyz(msg.pose.position), time.monotonic()
        else:
            self.pose_time = -math.inf

    def motion_reason(self):
        if not self.linked():
            return 'FCU disconnected or state stale'
        if not self.state.armed or self.state.mode != 'GUIDED':
            return 'Movement needs armed GUIDED'
        if not self.configured:
            return 'Apply bounding box first (arming is still allowed)'
        if not self.fresh(self.pose_time, self.cfg['pose_timeout']):
            return 'Pose missing, stale, or wrong frame'
        if not inside(self.position, self.lo, self.hi):
            return 'Outside bounding box'
        return ''

    def event(self, command, result, detail=''):
        self.event_seq += 1
        self.last_event = dict(sequence=self.event_seq, command=command, result=result, detail=detail)
        self.publish_status()

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(dict(
            connected=self.state.connected, state_fresh=self.linked(), armed=self.state.armed,
            mode=self.state.mode, control=self.control,
            yaw_enu=self.yaw, yaw_valid=self.yaw is not None,
            is_body_frame=self.is_body_frame, use_yaw_rate=self.use_yaw_rate,
            operation=self.operation['name'] if self.operation else '',
            position=self.position, goal=self.goal, safety=self.motion_reason(),
            stop_reason=self.stop_reason, last_event=self.last_event, fcu_text=self.fcu_text,
            frame_id=self.cfg['frame_id'], box_min=self.lo, box_max=self.hi,
            boundary_margin=self.margin, bounds_configured=self.configured,
            takeoff_altitude=self.cfg['takeoff_altitude'], max_speed=self.cfg['max_speed'],
            max_yaw_rate=self.cfg['max_yaw_rate'], grounded=self.grounded()), allow_nan=False)))

    def stop(self, reason='', brake=True):
        self.control = 'brake' if brake else 'idle'
        self.goal = self.pose_command = None
        self.velocity = (0., 0., 0., 0.)
        self.stop_reason = reason

    def request(self, name, client, req, field, confirmed, priority=False):
        if self.operation and not priority:
            self.event(name, 'rejected', 'Wait for current operation: ' + self.operation['name'])
            return
        if not client.service_is_ready():
            self.event(name, 'failed', 'MAVROS service unavailable')
            return
        if self.operation:
            self.finish('superseded', 'New command; already delivered FCU request cannot be undone')
        self.stop(brake=False)
        try:
            future = client.call_async(req)
        except Exception as exc:
            self.event(name, 'failed', str(exc))
            return
        op = dict(name=name, client=client, future=future, field=field, confirmed=confirmed,
                  started=time.monotonic(), accepted=False)
        self.operation = op
        self.event(name, 'pending')

    def finish(self, result, detail=''):
        op, self.operation = self.operation, None
        if op:
            if not op['future'].done():
                op['client'].remove_pending_request(op['future'])
                op['future'].cancel()
            self.event(op['name'], result, detail)

    def on_arm(self, msg):
        # Forward normal arm/disarm requests.
        # No ground-state, pose, bounds, or mode prerequisites here.
        # FCU arming/disarming rules remain authoritative.
        desired = bool(msg.data)

        req = CommandBool.Request()
        req.value = desired

        self.request(
            'arm' if desired else 'disarm',
            self.arm_client,
            req,
            'success',
            lambda: self.linked() and self.state.armed == desired,
        )

    def change_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.request(mode, self.mode_client, req, 'mode_sent',
                     lambda: self.linked() and self.state.mode == mode, priority=(mode == 'LAND'))

    def on_mode(self, msg):
        mode = msg.data.strip().upper()
        if mode not in ('STABILIZE', 'POSHOLD', 'GUIDED'):
            self.event('mode', 'rejected', 'Use STABILIZE, POSHOLD, or GUIDED')
            return
        self.change_mode(mode)

    def on_land(self, msg):
        if msg.data:
            self.change_mode('LAND')  # no pose/fence/ground checks

    def on_takeoff(self, msg):
        if not msg.data:
            return

        # Still require connected, armed GUIDED, valid pose and configured bounds.
        # No ExtendedState / ON_GROUND requirement.
        reason = self.motion_reason()
        if reason:
            self.event('takeoff', 'rejected', reason)
            return

        target_z = self.cfg['home_z'] + self.cfg['takeoff_altitude']
        target = (*self.position[:2], target_z)

        if target_z <= self.position[2] + self.cfg['position_tolerance']:
            self.event(
                'takeoff',
                'rejected',
                'Takeoff target is not above current altitude; '
                'check home_z and takeoff_altitude',
            )
            return

        if not inside(target, self.lo, self.hi, self.margin):
            self.event(
                'takeoff',
                'rejected',
                'Takeoff target is outside bounding box with margin',
            )
            return

        req = CommandTOL.Request()
        req.altitude = self.cfg['takeoff_altitude']

        self.request(
            'takeoff',
            self.takeoff_client,
            req,
            'success',
            lambda: (
                self.position is not None
                and self.fresh(self.pose_time, self.cfg['pose_timeout'])
                and self.position[2]
                >= target_z - self.cfg['position_tolerance']
            ),
        )
        
    def on_bounds(self, msg):
        # Runtime changes are atomic and disarmed-only; never auto-apply Inspector defaults.
        if not self.linked() or self.state.armed or self.operation:
            self.event('bounds', 'rejected', 'Apply bounds while connected, disarmed and idle')
            return
        try:
            data = json.loads(msg.data)
            lo, hi = tuple(map(float, data['min'])), tuple(map(float, data['max']))
            margin = float(data.get('margin', self.margin))
            self.validate_box(lo, hi, margin)
            self.lo, self.hi, self.margin, self.configured = lo, hi, margin, True
            self.stop(brake=False)
            self.event('bounds', 'applied', 'Runtime ENU bounds applied; not persisted across restart')
        except (ValueError, TypeError, KeyError) as exc:
            self.event('bounds', 'rejected', str(exc))

    def allow_motion(self, name):
        reason = 'Operation pending' if self.operation else self.motion_reason()
        if reason:
            self.event(name, 'rejected', reason)
        return not reason

    def on_position(self, msg):
        if not self.allow_motion('position'):
            return
        q = msg.pose.orientation
        values = (q.x, q.y, q.z, q.w)
        target = xyz(msg.pose.position)
        if (msg.header.frame_id != self.cfg['frame_id'] or not finite(target + values)
                or not inside(target, self.lo, self.hi, self.margin) or math.hypot(*values) < 1e-6):
            self.event('position', 'rejected', 'Invalid frame, quaternion or target bounds')
            return
        self.pose_command = copy.deepcopy(msg)
        q = self.pose_command.pose.orientation
        q.x, q.y, q.z, q.w = (v / math.hypot(*values) for v in values)
        self.goal, self.control, self.stop_reason = target, 'position', ''
        self.event('position', 'accepted', 'Holding target until replaced')

    def on_velocity(self, msg):
        if not self.allow_motion('velocity'):
            return
        v, w = xyz(msg.twist.linear), xyz(msg.twist.angular)
        if msg.header.frame_id != self.cfg['frame_id'] or not finite(v + w) or w[0] != 0 or w[1] != 0:
            self.event('velocity', 'rejected', 'Need ENU linear velocity and yaw rate only')
            return
        self.accept_velocity((*v, w[2]), False, True)

    def on_motion(self, msg):
        """Atomic command: frame_id (ENU map), values [vx,vy,vz,r], two booleans.

        Body means heading-relative forward/left/up (level FLU), not tilted axes.
        r is ENU global yaw in radians or CCW yaw rate in radians/second.
        """
        try:
            data = json.loads(msg.data)
            body, rate = data['is_body_frame'], data['use_yaw_rate']
            values = data['values']
            if (type(body) is not bool or type(rate) is not bool
                    or not isinstance(values, list) or len(values) != 4
                    or any(type(x) not in (int, float) for x in values)
                    or not finite(values) or data['frame_id'] != self.cfg['frame_id']):
                raise ValueError('Invalid frame, flags or four finite numeric values')
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self.event('velocity', 'rejected', str(exc))
            return
        if not self.allow_motion('velocity'):
            return
        if body and self.yaw is None:
            self.event('velocity', 'rejected', 'Body velocity requires valid pose quaternion')
            return
        self.accept_velocity(values, body, rate)

    def accept_velocity(self, values, is_body_frame, use_yaw_rate):
        v, r = values[:3], values[3]
        scale = min(1., self.cfg['max_speed'] / max(math.hypot(*v), 1e-9))
        limit = self.cfg['max_yaw_rate']
        self.velocity = (*(x * scale for x in v),
                         max(-limit, min(limit, r)) if use_yaw_rate else wrap_angle(r))
        first = (self.control != 'velocity' or self.is_body_frame != is_body_frame
                 or self.use_yaw_rate != use_yaw_rate)
        self.is_body_frame, self.use_yaw_rate = is_body_frame, use_yaw_rate
        self.control, self.goal, self.stop_reason = 'velocity', None, ''
        self.velocity_time = time.monotonic()
        if first:
            self.event('velocity', 'accepted')

    def velocity_enu(self, values, is_body_frame):
        if not is_body_frame:
            return tuple(values[:3])
        if self.yaw is None:
            raise ValueError('Body velocity requires valid pose quaternion')
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c*values[0] - s*values[1], s*values[0] + c*values[1], values[2])

    def send_motion_target(self, values=(0., 0., 0., 0.), is_body_frame=False, use_yaw_rate=True):
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Only rate control uses a native body frame. For body + GLOBAL yaw,
        # rotate velocity every tick and use local ENU to avoid body-yaw ambiguity.
        native_body = is_body_frame and use_yaw_rate
        msg.coordinate_frame = (PositionTarget.FRAME_BODY_NED if native_body
                                else PositionTarget.FRAME_LOCAL_NED)
        msg.header.frame_id = 'base_link' if native_body else self.cfg['frame_id']
        msg.type_mask = MASK_VELOCITY_YAW_RATE if use_yaw_rate else MASK_VELOCITY_YAW_ANGLE
        velocity = values[:3] if native_body else self.velocity_enu(values, is_body_frame)
        msg.velocity.x, msg.velocity.y, msg.velocity.z = map(float, velocity)
        if use_yaw_rate:
            msg.yaw_rate = float(values[3])
        else:
            msg.yaw = wrap_angle(values[3])
        self.vel_pub.publish(msg)

    def send_velocity(self, values=(0., 0., 0., 0.)):
        # Legacy helper and brake: zero velocity + zero rate, never yaw=0 angle.
        self.send_motion_target(values, False, True)

    def advance_request(self):
        op = self.operation
        if not op:
            return
        elapsed = time.monotonic() - op['started']
        if not op['accepted'] and op['future'].done():
            try:
                response = op['future'].result()
                if response is None or not getattr(response, op['field']):
                    self.stop('FCU rejected request')
                    self.finish('rejected', self.fcu_text or 'Check MAVROS/FCU messages')
                    return
                op['accepted'] = True
                self.event(op['name'], 'accepted', 'Awaiting telemetry, not completion')
            except Exception as exc:
                self.stop(str(exc))
                self.finish('failed', str(exc))
                return
        if op['accepted'] and op['confirmed']():
            if op['name'] == 'takeoff':
                self.stop('Takeoff altitude reached')
            self.finish('confirmed', 'LAND confirmation means mode LAND, not touchdown' if op['name'] == 'LAND' else '')
        elif elapsed > (self.cfg['action_timeout'] if op['accepted'] else self.cfg['service_timeout']):
            self.stop('Request timeout')
            self.finish('timeout', 'FCU outcome unknown; verify telemetry before retrying')

    def tick(self):
        # Takeoff is the only lifecycle action subject to the motion fence/pose guard.
        if self.operation and self.operation['name'] == 'takeoff':
            reason = self.motion_reason()
            if reason:
                self.stop(reason)
                self.finish('stopped', reason + '; FCU request may already have executed')
        self.advance_request()
        if self.operation:
            return
        if self.control in ('position', 'velocity'):
            reason = self.motion_reason()
            if not reason and self.control == 'velocity':
                if not self.fresh(self.velocity_time, self.cfg['command_timeout']):
                    reason = 'Velocity command timeout'
                elif self.is_body_frame and self.yaw is None:
                    reason = 'Body velocity requires valid pose quaternion'
                else:
                    world_velocity = self.velocity_enu(self.velocity, self.is_body_frame)
                    predicted = tuple(p + v * self.cfg['prediction_seconds'] for p, v in zip(self.position, world_velocity))
                    outward = any((p <= a + self.margin and v < 0) or (p >= b - self.margin and v > 0)
                                  for p, v, a, b in zip(self.position, world_velocity, self.lo, self.hi))
                    if outward or not inside(predicted, self.lo, self.hi):
                        reason = 'Predicted bounding-box violation'
            if reason:
                self.stop(reason)
                self.event('motion', 'stopped', reason)
        # Never emit GUIDED setpoints in STABILIZE, POSHOLD, LAND or while disarmed.
        if self.state.mode != 'GUIDED' or not self.state.armed:
            self.stop(brake=False)
            return
        if self.control == 'position':
            self.pose_command.header.stamp = self.get_clock().now().to_msg()
            self.pose_pub.publish(self.pose_command)
        elif self.control == 'velocity':
            self.send_motion_target(self.velocity, self.is_body_frame, self.use_yaw_rate)
        elif self.control == 'brake':
            self.send_velocity()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FlightController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
