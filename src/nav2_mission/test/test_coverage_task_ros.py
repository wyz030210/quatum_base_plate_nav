"""Public task API with the real manager and coverage bridge, fake downstream only."""
import json
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_mission_interfaces.srv import StartTask, CancelTask, GetTaskStatus
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Path
from opennav_coverage_msgs.action import ComputeCoveragePath
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from nav2_mission.coverage_navpath_bridge_node import CoverageNavPathBridge
from nav2_mission.task_control_node import TaskControlNode


def wait_until(predicate, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.02)
    raise AssertionError('timed out waiting for public boundary')


@pytest.fixture
def coverage_system(tmp_path, request):
    overrides = getattr(request, 'param', {})
    args = ['--ros-args']
    for name, value in overrides.items():
        args += ['-p', name + ':=' + json.dumps(value)]
    rclpy.init(args=args)
    fake = Node('coverage_server')
    from nav2_mission.coverage_task import PLANNER_DEFAULTS
    for name, value in dict(PLANNER_DEFAULTS, task_parameter_owner='',
                            task_parameter_release='').items():
        value = 1.234 if name == 'min_turning_radius' else value
        fake.declare_parameter(name, Parameter.Type.INTEGER_ARRAY if isinstance(value, list)
                               else value)
        if isinstance(value, list):
            fake.set_parameters([Parameter(name, Parameter.Type.INTEGER_ARRAY, value)])
    group = ReentrantCallbackGroup()
    boundary = {'requests': [], 'goals': [], 'cancels': [], 'checks': [],
                'hold': '', 'accept_hold': '', 'release_accept': threading.Event(),
                'release_terminal': threading.Event(), 'reject': '', 'fail': '',
                'invalid_check': 0, 'stop_confirmed': True, 'hold_parameters': False,
                'parameters_started': threading.Event(), 'parameters_release': threading.Event()}
    stop = {'stop_requested': True}
    boundary['stop_state'] = stop
    def parameters(values):
        if boundary['hold_parameters'] and any(
                value.name == 'task_parameter_owner' and value.value for value in values):
            boundary['parameters_started'].set()
            boundary['parameters_release'].wait(10)
        return SetParametersResult(successful=True)

    fake.add_on_set_parameters_callback(parameters)
    loc_pub = fake.create_publisher(String, '/task_control/localization', 10)
    state_pub = fake.create_publisher(String, '/task_control/state', 10)
    fake.create_subscription(Bool, '/task_control/stop',
                             lambda msg: stop.update(stop_requested=msg.data), 10, callback_group=group)

    def states():
        loc_pub.publish(String(data=json.dumps({
            'valid': True, 'frame_id': 'map', 'source': 'fake_rtk',
            'source_stamp_sec': fake.get_clock().now().nanoseconds / 1e9,
            'x': 0.0, 'y': 0.0, 'yaw': 0.0})))
        state_pub.publish(String(data=json.dumps(dict(
            ready=True, stopped=stop['stop_requested'] and boundary['stop_confirmed'],
            manual_override=False, emergency_stop=False, fault='',
            feedback_age_sec=0.0, **stop))))

    fake.create_timer(0.02, states, callback_group=group)
    tf = StaticTransformBroadcaster(fake)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.transform.rotation.w = 1.0
    tf.sendTransform(transform)

    def lifecycle(req, res):
        res.current_state.id = 3
        res.current_state.label = 'active'
        return res

    for name in ('coverage_server', 'controller_server'):
        fake.create_service(GetState, '/' + name + '/get_state', lifecycle,
                            callback_group=group)

    def check(req, res):
        boundary['checks'].append(req.path)
        res.is_valid = len(boundary['checks']) != boundary['invalid_check']
        res.invalid_pose_indices = [] if res.is_valid else [2]
        return res

    fake.create_service(IsPathValid, '/is_path_valid', check, callback_group=group)
    path = Path()
    path.header.frame_id = 'map'
    # Revisited start is closer to the robot than the actual first pose.
    for x, y in ((4., 4.), (6., 4.), (6., 6.), (0.1, 0.)):
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
            return GoalResponse.REJECT if boundary['reject'] == kind else GoalResponse.ACCEPT

        def execute(handle):
            boundary['goals'].append((kind, handle.request))
            if kind == 'follow':
                handle.publish_feedback(FollowPath.Feedback(distance_to_goal=2.5, speed=0.2))
            while boundary['hold'] == kind and not handle.is_cancel_requested:
                time.sleep(0.01)
            if handle.is_cancel_requested:
                boundary['cancels'].append(kind)
                boundary['release_terminal'].wait(10)
                handle.canceled()
            elif boundary['fail'] == kind:
                handle.abort()
            else:
                handle.succeed()
            result = action.Result()
            if kind == 'compute':
                result.nav_path = path
            return result

        return ActionServer(fake, action, name, execute_callback=execute,
                            goal_callback=accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                            callback_group=group)

    servers = [server('compute', ComputeCoveragePath, '/compute_coverage_path'),
               server('approach', NavigateToPose, '/navigate_to_pose'),
               server('follow', FollowPath, '/follow_path')]
    bridge = CoverageNavPathBridge()
    boundary['report_dir'] = tmp_path / 'reports'
    manager = TaskControlNode(parameter_overrides=[
        Parameter('report_dir', value=str(boundary['report_dir']))])
    executor = MultiThreadedExecutor(num_threads=12)
    for node in (fake, bridge, manager):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def call(service, name, request):
        client = fake.create_client(service, name, callback_group=group)
        assert client.wait_for_service(timeout_sec=4), name
        future = client.call_async(request)
        wait_until(future.done)
        result = future.result()
        fake.destroy_client(client)
        return result

    def status():
        return json.loads(call(GetTaskStatus, '/task_control/status',
                               GetTaskStatus.Request()).status_json)

    def start():
        return call(StartTask, '/task_control/start', StartTask.Request(
            task_type='coverage', request_json=json.dumps({
                'frame_id': 'map', 'polygons': [[[0, 0], [8, 0], [8, 8], [0, 8]]],
                'planner': {'operation_width': 0.7, 'default_swath_angle': 0.3}})))

    time.sleep(0.3)
    yield start, status, call, boundary
    boundary['hold'] = ''
    boundary['release_accept'].set()
    boundary['release_terminal'].set()
    boundary['parameters_release'].set()
    manager.shutdown_tasks()
    time.sleep(0.2)
    executor.shutdown(timeout_sec=3)
    thread.join(timeout=3)
    for server_ in servers:
        server_.destroy()
    for node in (manager, bridge, fake):
        node.destroy_node()
    rclpy.shutdown()


def test_rectangle_executes_full_path_and_waits_for_fresh_stop(coverage_system):
    start, status, call, boundary = coverage_system
    boundary.update(hold='follow')
    reply = start()
    assert reply.accepted, reply.message
    boundary['stop_confirmed'] = False
    wait_until(lambda: any(kind == 'follow' for kind, _ in boundary['goals']))
    running = wait_until(lambda: (value if (value := status())['phase'] == 'following'
                                  else None))
    assert running['result'] is None
    assert len(boundary['checks']) == 2
    approach = next(req for kind, req in boundary['goals'] if kind == 'approach')
    follow = next(req for kind, req in boundary['goals'] if kind == 'follow')
    assert (approach.pose.pose.position.x, approach.pose.pose.position.y) == (4., 4.)
    assert approach.pose.header.frame_id == 'map' and approach.behavior_tree == ''
    assert follow.path == boundary['path']
    boundary['hold'] = ''
    wait_until(lambda: status()['phase'] == 'stopping')
    assert status()['result'] is None and not start().accepted
    boundary['stop_confirmed'] = True
    ended = wait_until(lambda: (value if not (value := status())['active'] else None))
    assert ended['result'] == 'succeeded' and ended['cleanup_confirmed'], ended
    report = json.loads(open(ended['report_path']).read())
    assert report['task_id'] == reply.task_id and not report['simulated']
    assert {'planning', 'approaching', 'following', 'stopping'} <= set(report['details']['phases'])
    assert report['config']['planner']['operation_width'] == 0.7
    assert report['details']['effective_planner']['min_turning_radius'] == 1.234
    assert report['details']['effective_planner']['operation_width'] == 0.7
    assert report['details']['planned_path']['poses'][0]['x'] == 4.0
    assert report['details']['feedback']['distance_remaining'] == 2.5


@pytest.mark.parametrize('stage', ['compute', 'approach', 'follow'])
@pytest.mark.parametrize('pending', [True, False])
def test_cancel_drains_pending_or_accepted_children_before_next_task(coverage_system, stage, pending):
    start, status, call, boundary = coverage_system
    boundary['hold'] = stage
    if pending:
        boundary['accept_hold'] = stage
    reply = start()
    assert reply.accepted, reply.message
    wait_until(lambda: any(kind == stage for kind, _ in boundary['requests']))
    canceled = call(CancelTask, '/task_control/cancel',
                    CancelTask.Request(task_id=reply.task_id))
    assert canceled.accepted
    time.sleep(0.15)
    blocked = status()
    assert blocked['active'] and blocked['phase'] == 'stopping'
    assert blocked['result'] is None and not start().accepted
    boundary['release_accept'].set()
    wait_until(lambda: stage in boundary['cancels'])
    time.sleep(0.1)
    assert status()['active'], 'cancel acknowledgement must not release ownership'
    assert not start().accepted
    boundary['release_terminal'].set()
    ended = wait_until(lambda: (value if not (value := status())['active'] else None))
    assert ended['result'] == 'canceled' and ended['cleanup_confirmed']
    stages = [kind for kind, _ in boundary['goals']]
    assert stages == {'compute': ['compute'], 'approach': ['compute', 'approach'],
                      'follow': ['compute', 'approach', 'follow']}[stage]
    boundary.update(hold='', accept_hold='')
    second = start()
    assert second.accepted and second.task_id != reply.task_id
    final = wait_until(lambda: (value if not (value := status())['active'] else None))
    assert final['task_id'] == second.task_id and final['result'] == 'succeeded'


@pytest.mark.parametrize('field', ['coordinate', 'planner'])
def test_nonrepresentable_finite_numbers_are_rejected_without_losing_service(coverage_system, field):
    start, status, call, boundary = coverage_system
    config = {'frame_id': 'map', 'polygons': [[[0, 0], [8, 0], [8, 8], [0, 8]]]}
    if field == 'coordinate':
        config['polygons'][0][1][0] = 10 ** 400
    else:
        config['planner'] = {'default_swath_angle': 10 ** 400}
    reply = call(StartTask, '/task_control/start', StartTask.Request(
        task_type='coverage', request_json=json.dumps(config)))
    assert not reply.accepted and 'finite' in reply.message
    assert not status()['active'] and not boundary['requests']


@pytest.mark.parametrize('invalid', ['nonfinite', 'frame', 'quaternion'])
def test_illegal_planner_path_is_rejected_before_any_navigation(coverage_system, invalid):
    start, status, call, boundary = coverage_system
    if invalid == 'nonfinite':
        boundary['path'].poses[0].pose.position.x = float('nan')
    elif invalid == 'frame':
        boundary['path'].header.frame_id = 'different_map'
    else:
        boundary['path'].poses[0].pose.orientation.w = 0.0
    reply = start()
    assert reply.accepted
    ended = wait_until(lambda: (value if not (value := status())['active'] else None))
    assert ended['result'] == 'failed' and '911' in ended['error'], ended
    assert [kind for kind, _ in boundary['goals']] == ['compute']


@pytest.mark.parametrize('coverage_system', [{'navigate_to_start_timeout_sec': 0.15}], indirect=True)
def test_approach_timeout_asserts_common_stop_before_delayed_child_terminal(coverage_system):
    start, status, call, boundary = coverage_system
    boundary['hold'] = 'approach'
    assert start().accepted
    wait_until(lambda: 'approach' in boundary['cancels'])
    time.sleep(0.15)
    blocked = status()
    assert boundary['stop_state']['stop_requested'], blocked
    assert blocked['active'] and blocked['phase'] == 'stopping'
    assert '910' in blocked['error'] and not start().accepted
    assert [kind for kind, _ in boundary['goals']] == ['compute', 'approach']
    boundary['release_terminal'].set()
    ended = wait_until(lambda: (value if not (value := status())['active'] else None))
    assert ended['result'] == 'failed' and ended['cleanup_confirmed']


@pytest.mark.parametrize('coverage_system', [{'dependency_timeout_sec': 0.15}], indirect=True)
def test_canceled_pending_parameter_acquire_is_drained_and_released_before_next_task(coverage_system):
    start, status, call, boundary = coverage_system
    boundary['hold_parameters'] = True
    reply = start()
    assert reply.accepted
    wait_until(boundary['parameters_started'].is_set)
    assert call(CancelTask, '/task_control/cancel',
                CancelTask.Request(task_id=reply.task_id)).accepted
    # Cross the parameter response timeout while the server may still commit it.
    time.sleep(0.35)
    assert status()['active'] and not start().accepted
    assert boundary['stop_state']['stop_requested'] and not boundary['requests']
    boundary['parameters_release'].set()
    ended = wait_until(lambda: (value if not (value := status())['active'] else None))
    assert ended['result'] == 'canceled' and not boundary['requests']
    owner = call(GetParameters, '/coverage_server/get_parameters',
                 GetParameters.Request(names=['task_parameter_owner']))
    assert owner.values[0].string_value == ''
    boundary['hold_parameters'] = False
    assert start().accepted
    final = wait_until(lambda: (value if not (value := status())['active'] else None))
    assert final['result'] == 'succeeded', final


@pytest.mark.parametrize('failure,code,expected', [
    ('compute', 908, ['compute']),
    ('check1', 911, ['compute']),
    ('approach', 910, ['compute', 'approach']),
    ('check2', 911, ['compute', 'approach']),
    ('follow', 903, ['compute', 'approach', 'follow']),
    ('empty', 902, ['compute']),
])
def test_execution_failure_reports_original_code_without_skipping_stages(
        coverage_system, failure, code, expected):
    start, status, call, boundary = coverage_system
    if failure.startswith('check'):
        boundary['invalid_check'] = int(failure[-1])
    elif failure == 'empty':
        boundary['path'].poses = []
    else:
        boundary['fail'] = failure
    reply = start()
    assert reply.accepted
    ended = wait_until(lambda: (value if not (value := status())['active'] else None))
    assert ended['result'] == 'failed' and str(code) in ended['error'], ended
    assert ended['cleanup_confirmed'] and boundary['stop_state']['stop_requested']
    assert [kind for kind, _ in boundary['goals']] == expected
    report = json.loads(open(ended['report_path']).read())
    assert report['task_id'] == reply.task_id and report['error'] == ended['error']


def test_coverage_report_write_error_is_visible_after_successful_motion(coverage_system):
    start, status, call, boundary = coverage_system
    boundary['report_dir'].write_text('blocks report directory creation')
    assert start().accepted
    ended = wait_until(lambda: (value if not (value := status())['active'] else None))
    assert ended['result'] == 'failed' and ended['execution_result'] == 'succeeded'
    assert ended['report_error'] and not ended['report_path'] and ended['cleanup_confirmed']


def test_bridge_parameters_cannot_disable_path_checks_during_frozen_task(coverage_system):
    start, status, call, boundary = coverage_system
    boundary['hold'] = 'compute'
    assert start().accepted
    wait_until(lambda: any(kind == 'compute' for kind, _ in boundary['goals']))
    result = call(SetParametersAtomically, '/coverage_navpath_bridge/set_parameters_atomically',
                  SetParametersAtomically.Request(parameters=[
                      Parameter('validate_path_costmap', value=False).to_parameter_msg()]))
    assert not result.result.successful and 'frozen' in result.result.reason
    boundary['hold'] = ''
    ended = wait_until(lambda: (value if not (value := status())['active'] else None))
    assert ended['result'] == 'succeeded' and len(boundary['checks']) == 2
