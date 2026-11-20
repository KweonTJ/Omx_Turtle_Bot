"""Run real callbacks against ROS message types without DDS or robot commands."""
import json
import math
from functools import partial
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from omx_leader_approach.approach_node import ApproachNode
from omx_leader_approach.alignment_core import AlignmentGate


class Collector:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def node():
    n = SimpleNamespace(cfg={'odom_frame': 'odom', 'distance_m': .382, 'speed_mps': .055,
                            'sensor_timeout_s': .3, 'overview_timeout_s': 10.,
                            'alignment_timeout_s': 16., 'drive_timeout_s': 18.},
                        state='IDLE', request=None, used_requests=set(), gate=AlignmentGate(),
                        imu_stamp=-math.inf, odom_stamp=-math.inf, overview_stamp=-math.inf,
                        imu_angle=0., omega=0., odom_angle=0., speed=0., ready_ticks=0,
                        target_imu_yaw=None, heading_error=None, state_since=100., now=100.,
                        command=Collector(), status=Collector(), select=Collector())
    n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(round(n.now*1e9))))
    n.get_logger = lambda: SimpleNamespace(warning=lambda text: None)
    for name in ('seconds', 'on_request', 'on_imu', 'on_odom', 'on_objects', 'tick'):
        setattr(n, name, partial(getattr(ApproachNode, name), n))
    return n


def request(node, **changes):
    payload = dict(command='START', request_id='test-1', object_id=2,
                   target_yaw_odom=0., odom_frame='odom')
    payload.update(changes)
    node.on_request(String(data=json.dumps(payload)))


def sensors(node, *, odom_yaw=0., imu_yaw=0., x=0., y=0., gyro=0., speed=0.):
    imu, odom = Imu(), Odometry()
    for message in (imu, odom):
        stamp = int(round(node.now*1e9))
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
    imu.header.frame_id = 'imu_link'
    imu.orientation.z, imu.orientation.w = math.sin(imu_yaw/2), math.cos(imu_yaw/2)
    imu.angular_velocity.z = gyro
    odom.header.frame_id = 'odom'
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(odom_yaw/2), math.cos(odom_yaw/2)
    odom.pose.pose.position.x, odom.pose.pose.position.y = float(x), float(y)
    odom.twist.twist.linear.x = speed
    node.on_imu(imu)
    node.on_odom(odom)


def overview(node):
    node.on_objects(String(data=json.dumps(dict(fresh=True, overview_ready=True,
                           stamp_ns=int(node.now*1e9), objects=[
                           dict(marker_id=i, pose_valid=True, usable_2d=True) for i in (1, 2, 3, 4)]))))


@pytest.mark.parametrize('changes', [dict(target_yaw_odom=None), dict(target_yaw_odom=True),
                                   dict(target_yaw_odom=float('nan')), dict(odom_frame='map'),
                                   dict(object_id=True), dict(object_id=5)])
def test_invalid_host_reference_never_starts(node, changes):
    request(node, **changes)
    node.tick()
    assert node.state == 'IDLE'
    assert not node.select.messages
    assert node.command.messages[-1].linear.x == node.command.messages[-1].angular.z == 0.


@pytest.mark.parametrize('raw', ['[]', 'null', '{', '{"objects":[null]}',
                               '{"objects":null}', '{"objects":[{"marker_id":1}]}'])
def test_invalid_perception_fails_closed(node, raw):
    node.on_objects(String(data=raw))
    assert node.overview_stamp == -math.inf


def align(node):
    request(node)
    node.now += .05
    sensors(node, odom_yaw=math.radians(12), imu_yaw=math.radians(42))
    overview(node)
    node.tick()
    assert node.state == 'ALIGN_PARALLEL'
    assert node.target_imu_yaw == pytest.approx(math.radians(30))
    node.tick()
    assert node.command.messages[-1].angular.z < 0
    assert node.command.messages[-1].linear.x == 0
    for _ in range(10):
        node.now += .05
        sensors(node, imu_yaw=math.radians(30))
        node.tick()
    assert node.state == 'DRIVE_STRAIGHT'


def test_full_handoff_align_drive_and_stop_without_depth(node):
    align(node)
    node.tick()
    assert node.command.messages[-1].linear.x == .055
    for _ in range(10):
        node.now += .05
        sensors(node, imu_yaw=math.radians(30), x=.382)
        node.tick()
    assert node.state == 'READY_FOR_ARM'
    assert node.command.messages[-1].linear.x == node.command.messages[-1].angular.z == 0
    status = json.loads(node.status.messages[-1].data)
    assert status['ready_for_arm'] and status['depth_used'] is False
    assert json.loads(node.select.messages[-1].data)['object_id'] == 2


def test_stale_sensor_stops_and_revokes_arm_ready(node):
    align(node)
    node.now += .4
    node.tick()
    assert node.command.messages[-1].linear.x == node.command.messages[-1].angular.z == 0
    assert not json.loads(node.status.messages[-1].data)['ready_for_arm']


def test_lateral_deviation_aborts(node):
    align(node)
    node.now += .05
    sensors(node, imu_yaw=math.radians(30), y=.03)
    node.tick()
    assert node.state == 'FAULT'
    assert node.command.messages[-1].linear.x == node.command.messages[-1].angular.z == 0


def test_stop_is_immediate_and_replayed_request_is_ignored(node):
    align(node)
    node.on_request(String(data='{"command":"STOP"}'))
    assert node.state == 'IDLE'
    assert node.command.messages[-1].linear.x == node.command.messages[-1].angular.z == 0
    request(node)
    assert node.state == 'IDLE'


def test_duplicate_odom_cannot_confirm_stopped_ten_times(node):
    align(node)
    node.now += .05
    sensors(node, imu_yaw=math.radians(30), x=.382)
    for _ in range(20):
        node.tick()
    assert node.state == 'DRIVE_STRAIGHT' and node.ready_ticks == 1
