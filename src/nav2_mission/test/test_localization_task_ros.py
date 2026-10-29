"""Real localization -> manager -> coverage bridge -> command entry, no hardware.

Only sensors, downstream Nav2 actions and the captured CAN sink are substituted.
"""
import json
import os
import signal
import subprocess
import tempfile
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_mission_interfaces.srv import StartTask, CancelTask, GetTaskStatus
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Path
from opennav_coverage_msgs.action import ComputeCoveragePath
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from vehicle_can_bridge.msg import Frame

from nav2_mission.coverage_navpath_bridge_node import CoverageNavPathBridge
from nav2_mission.localization_provider_node import LocalizationProvider, map_digest
from nav2_mission.task_control_node import TaskControlNode
from test_localization_provider_ros import Sensors


def wait_for(predicate, timeout=7):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    raise AssertionError('timed out at public task/action/command boundary')


@pytest.fixture
def complete_chain(tmp_path, request):
    source, task_type = request.param
    rclpy.init()
    amcl = Node('amcl')
    for key, value in [('base_frame_id', 'base_link'), ('odom_frame_id', 'odom'),
                       ('global_frame_id', 'map'), ('transform_tolerance', 1.0)]:
        amcl.declare_parameter(key, value)
    sensors = Sensors(amcl)
    sensors.source = source
    sensors.established()  # one durable pose precedes provider/manager startup
    flags = dict(raw=True, scan=True, edge=True, local=True, base_tf=True, linear=0.0)
    amcl.create_timer(0.02, lambda: sensors.pump(**flags))
    provider = LocalizationProvider(parameter_overrides=[
        Parameter('source', value=source), Parameter('source_timeout_sec', value=0.35),
        Parameter('lifecycle_timeout_sec', value=0.5),
        Parameter('rtk_position_timeout_sec', value=0.35),
        Parameter('rtk_quality_timeout_sec', value=0.35),
        Parameter('rtk_heading_timeout_sec', value=0.35),
        Parameter('map_id', value=map_digest(sensors.grid)),
        Parameter('localization_profile', value='complete-chain-' + source),
        Parameter('base_frame_id', value='base_link'),
        Parameter('scan_frame_id', value='base_link')])
    fake = Node('coverage_server')
    from nav2_mission.coverage_task import PLANNER_DEFAULTS
    for name, value in dict(PLANNER_DEFAULTS, task_parameter_owner='',
                            task_parameter_release='').items():
        fake.declare_parameter(name, Parameter.Type.INTEGER_ARRAY if isinstance(value, list)
                               else value)
        if isinstance(value, list):
            fake.set_parameters([Parameter(name, Parameter.Type.INTEGER_ARRAY, value)])
    group = ReentrantCallbackGroup()
    boundary = dict(requests=[], goals=[], cancels=[], checks=[], frames=[], safety=[],
                    hold='navigate' if task_type == 'inspection' else 'follow',
                    accept_hold='', release_accept=threading.Event(),
                    release_terminal=threading.Event(), initialposes=[])
    command = fake.create_publisher(Twist, '/cmd_vel', 10)
    manual = fake.create_publisher(Bool, '/vehicle/remote_manual_override', 10)
    emergency = fake.create_publisher(Bool, '/task_control/emergency_stop', 10)
    fake.create_subscription(String, '/task_control/state',
                             lambda msg: boundary['safety'].append(json.loads(msg.data)), 100)
    from geometry_msgs.msg import PoseWithCovarianceStamped
    fake.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                             boundary['initialposes'].append, 10)

    def frame(msg):
        if msg.id == 6 and msg.dlc == 8 and msg.data[0] == 1:
            boundary['frames'].append((time.monotonic(), int.from_bytes(
                bytes(msg.data[1:5]), 'big', signed=True)))

    fake.create_subscription(Frame, '/can_tx', frame, 100)

    def lifecycle(req, res):
        res.current_state.id, res.current_state.label = 3, 'active'
        return res

    for name in ('coverage_server', 'controller_server'):
        fake.create_service(GetState, '/' + name + '/get_state', lifecycle, callback_group=group)

    def check(req, res):
        boundary['checks'].append(req.path)
        res.is_valid = True
        return res

    fake.create_service(IsPathValid, '/is_path_valid', check, callback_group=group)
    path = Path()
    path.header.frame_id = 'map'
    for x, y in ((4., 4.), (6., 4.), (6., 6.), (1.25, 0.)):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    boundary['path'] = path

    def server(kind, action, name):
        def accept(req):
            boundary['requests'].append((kind, req))
            if boundary['accept_hold'] == kind:
                boundary['release_accept'].wait(10)
            return GoalResponse.ACCEPT

        def execute(handle):
            boundary['goals'].append((kind, handle.request))
            while boundary['hold'] == kind and not handle.is_cancel_requested:
                if kind != 'compute':
                    velocity = Twist()
                    velocity.linear.x = 0.2
                    command.publish(velocity)
                time.sleep(0.02)
            if handle.is_cancel_requested:
                boundary['cancels'].append(kind)
                boundary['release_terminal'].wait(10)
                handle.canceled()
            else:
                handle.succeed()
            result = action.Result()
            if kind == 'compute':
                result.nav_path = path
            return result

        return ActionServer(fake, action, name, execute_callback=execute,
                            goal_callback=accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                            callback_group=group)

    nav_stage = 'navigate' if task_type == 'inspection' else 'approach'
    servers = [server('compute', ComputeCoveragePath, '/compute_coverage_path'),
               server(nav_stage, NavigateToPose, '/navigate_to_pose'),
               server('follow', FollowPath, '/follow_path')]
    bridge = CoverageNavPathBridge()
    manager = TaskControlNode(parameter_overrides=[
        Parameter('report_dir', value=str(tmp_path / 'reports')),
        Parameter('map_id', value=map_digest(sensors.grid)),
        Parameter('localization_profile', value='complete-chain-' + source),
        Parameter('require_localization_schema', value=True)])
    executable = os.environ['COMMAND_ENTRY_EXECUTABLE']
    log = tempfile.TemporaryFile(mode='w+')
    args = [
        executable, '--ros-args', '-p', 'task_control_required:=true',
        '-p', 'feedback_timeout_sec:=0.35', '-p', 'heartbeat_timeout_sec:=0.5',
        '-p', 'command_timeout_sec:=0.5', '-p', 'stopped_hold_sec:=0.15',
        '-p', 'stopping_timeout_sec:=5.0', '-p', 'safety_publish_period_sec:=0.02']
    process = subprocess.Popen(args, stdout=log, stderr=log, env=os.environ.copy())
    executor = MultiThreadedExecutor(num_threads=14)
    for node in (amcl, fake, provider, bridge, manager):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def call(service, name, req):
        client = fake.create_client(service, name, callback_group=group)
        assert client.wait_for_service(timeout_sec=4), name
        future = client.call_async(req)
        wait_for(future.done)
        result = future.result()
        fake.destroy_client(client)
        return result

    def status():
        result = call(GetTaskStatus, '/task_control/status', GetTaskStatus.Request())
        return json.loads(result.status_json)

    def start():
        config = (
            {'frame_id': 'map', 'point': {'id': 'gate', 'type': 'security',
             'x': 2., 'y': 0., 'yaw': 0., 'stay_time': 0.0}} if task_type == 'inspection' else
            {'frame_id': 'map', 'polygons': [[[0, 0], [8, 0], [8, 8], [0, 8]]]})
        return call(StartTask, '/task_control/start', StartTask.Request(
            task_type=task_type, request_json=json.dumps(config)))

    wait_for(lambda: sensors.states and sensors.states[-1]['valid'])
    wait_for(lambda: boundary['safety'] and boundary['safety'][-1]['stopped'] and
             boundary['safety'][-1]['ready'])
    yield dict(start=start, status=status, call=call, boundary=boundary, sensors=sensors,
               flags=flags, manual=manual, emergency=emergency, process=process, manager=manager)
    boundary['hold'] = ''
    boundary['release_accept'].set()
    boundary['release_terminal'].set()
    flags.update(raw=True, scan=True, edge=True, local=True, base_tf=True, linear=0.0)
    flags.pop('feedback_stamp', None)
    sensors.rtk_status = None
    manager.shutdown_tasks()
    time.sleep(0.2)
    executor.shutdown(timeout_sec=3)
    thread.join(timeout=3)
    for item in servers:
        item.destroy()
    for node in (manager, bridge, provider, fake, amcl):
        node.destroy_node()
    process.send_signal(signal.SIGINT)
    process.wait(timeout=4)
    log.close()
    rclpy.shutdown()


MATRIX = [(source, task) for source in ('amcl', 'rtk_ekf')
          for task in ('inspection', 'coverage')]


@pytest.mark.parametrize('complete_chain', MATRIX, indirect=True)
def test_source_loss_drains_actions_stops_commands_and_requires_explicit_restart(complete_chain):
    io = complete_chain
    start, status, boundary, flags = io['start'], io['status'], io['boundary'], io['flags']
    reply = start()
    assert reply.accepted, reply.message
    flags['linear'] = 0.2
    wait_for(lambda: boundary['frames'] and boundary['frames'][-1][1] > 0)
    if reply.accepted and io['sensors'].source == 'amcl':
        flags['scan'] = False
    else:
        flags['raw'] = False
    stopping = wait_for(lambda: (value if (value := status())['phase'] == 'stopping' else None))
    assert stopping['active'] and stopping['error']
    wait_for(lambda: boundary['cancels'])
    wait_for(lambda: boundary['frames'][-1][1] == 0)
    assert not start().accepted
    # Cancel acceptance cannot release ownership, even after source recovery.
    flags.update(raw=True, scan=True, linear=0.0)
    flags['feedback_stamp'] = io['sensors'].node.get_clock().now().to_msg()
    time.sleep(0.4)
    boundary['release_terminal'].set()
    time.sleep(0.3)
    assert status()['active'] and not start().accepted, 'Old zero feedback cannot confirm cleanup'
    flags.pop('feedback_stamp')
    ended = wait_for(lambda: (value if not (value := status())['active'] else None))
    assert ended['task_id'] == reply.task_id and ended['result'] == 'failed'
    assert ended['cleanup_confirmed'] and not boundary['initialposes']
    count = len(boundary['goals'])
    time.sleep(0.15)
    assert len(boundary['goals']) == count
    reset = io['call'](Trigger, '/task_control/reset_stop', Trigger.Request())
    assert reset.success, reset.message
    second = start()
    assert second.accepted and second.task_id != reply.task_id, second.message
    io['call'](CancelTask, '/task_control/cancel', CancelTask.Request(task_id=second.task_id))
    wait_for(lambda: not status()['active'])
    if ended['task_type'] == 'coverage':
        assert len(boundary['checks']) >= 2
        follow = next(req for kind, req in boundary['goals'] if kind == 'follow')
        assert follow.path == boundary['path']
