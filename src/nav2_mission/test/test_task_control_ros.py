"""Real ROS task services with only external navigation and state providers faked."""
import json
import threading
import time

import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from nav2_mission_interfaces.srv import StartTask, CancelTask, GetTaskStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String

from nav2_mission.task_control_node import TaskControlNode


@pytest.fixture
def ros_system(tmp_path, request):
    rclpy.init()
    fake = Node('fake_task_boundaries')
    goals = []
    commands = []
    boundary = {'accept_delay': 0.0, 'execute_time': 0.08, 'requests': [], 'canceled': []}
    stop_state = {'stop_requested': True, 'stopped': True}
    loc_pub = fake.create_publisher(String, '/task_control/localization', 10)
    stop_pub = fake.create_publisher(String, '/task_control/state', 10)

    def command(msg):
        commands.append(msg.data)
        stop_state.update(stop_requested=msg.data, stopped=msg.data)

    fake.create_subscription(Bool, '/task_control/stop', command, 10)

    def states():
        stamp = fake.get_clock().now().nanoseconds / 1e9
        loc_pub.publish(String(data=json.dumps({
            'valid': True, 'frame_id': 'map', 'source_stamp_sec': stamp,
            'source': 'fake_rtk', 'x': 0.0, 'y': 0.0, 'yaw': 0.0})))
        stop_pub.publish(String(data=json.dumps(dict(
            ready=True, manual_override=False, emergency_stop=False, fault='',
            feedback_age_sec=0.0, **stop_state))))

    fake.create_timer(0.02, states)

    def execute(handle):
        goals.append(handle.request)
        deadline = time.monotonic() + boundary['execute_time']
        while time.monotonic() < deadline and not handle.is_cancel_requested:
            time.sleep(0.01)
        if handle.is_cancel_requested:
            boundary['canceled'].append(handle.request)
            handle.canceled()
        elif boundary.get('outcomes') and not boundary['outcomes'].pop(0):
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    def accept(request):
        boundary['requests'].append(request)
        time.sleep(boundary['accept_delay'])
        return GoalResponse.ACCEPT

    server = ActionServer(fake, NavigateToPose, '/test_task/navigate_to_pose',
                          execute_callback=execute,
                          goal_callback=accept, callback_group=ReentrantCallbackGroup(),
                          cancel_callback=lambda _: CancelResponse.ACCEPT)
    process_mode = getattr(request, 'param', None) == 'process'
    node = None
    process = None
    if process_mode:
        import subprocess
        import sys
        process = subprocess.Popen([
            sys.executable, '-m', 'nav2_mission.task_control_node', '--ros-args',
            '-p', 'navigate_action_name:=/test_task/navigate_to_pose',
            '-p', f'report_dir:={tmp_path}',
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    else:
        node = TaskControlNode(parameter_overrides=[
            Parameter('navigate_action_name', value='/test_task/navigate_to_pose'),
            Parameter('report_dir', value=str(tmp_path)),
        ])
    boundary['process'] = process
    boundary['report_dir'] = tmp_path
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fake)
    if node is not None:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def call(service, name, request):
        client = fake.create_client(service, name)
        assert client.wait_for_service(timeout_sec=3)
        future = client.call_async(request)
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert future.done(), name
        result = future.result()
        fake.destroy_client(client)
        return result

    time.sleep(0.2)
    yield call, goals, commands, boundary
    if node is not None:
        node.shutdown_tasks()
    if process is not None and process.poll() is None:
        process.terminate()
        process.wait(timeout=7)
    executor.shutdown(timeout_sec=2)
    thread.join(timeout=2)
    server.destroy()
    if node is not None:
        node.destroy_node()
    fake.destroy_node()
    rclpy.shutdown()


def test_ros_start_cancel_status_are_request_scoped_and_nonblocking(ros_system):
    call, goals, commands, boundary = ros_system
    req = StartTask.Request()
    req.task_type = 'inspection'
    req.request_json = json.dumps({'frame_id': 'map', 'point': {
        'id': 'gate', 'type': 'security', 'x': 1.0, 'y': 2.0,
        'yaw': 0.2, 'stay_time': 0.3, 'allow_skip': False}})
    started = call(StartTask, '/task_control/start', req)
    assert started.accepted
    observed = set()
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        status = json.loads(call(GetTaskStatus, '/task_control/status',
                                 GetTaskStatus.Request()).status_json)
        observed.add(status['phase'])
        if status.get('result'):
            break
        time.sleep(0.02)
    assert status['result'] == 'succeeded', status
    assert 'inspecting' in observed
    assert len(goals) == 1 and goals[0].behavior_tree == ''
    assert False in commands and commands[-1] is True
    assert status['task_id'] == started.task_id and status['cleanup_confirmed']
    report = json.loads(open(status['report_path']).read())
    assert report['simulated']
    canceled = call(CancelTask, '/task_control/cancel', CancelTask.Request())
    assert canceled.accepted


def test_cli_calls_real_ros_status_service(ros_system):
    import subprocess
    import sys
    result = subprocess.run([sys.executable, '-m', 'nav2_mission.task_cli', 'status'],
                            text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['phase'] == 'idle'


def test_ros_cancel_before_goal_acceptance_revokes_late_goal_and_isolates_next_task(ros_system):
    call, goals, commands, boundary = ros_system
    boundary['accept_delay'] = 0.4
    boundary['execute_time'] = 2.0
    req = StartTask.Request(task_type='inspection', request_json=json.dumps({
        'frame_id': 'map', 'point': {'id': 'late', 'type': 'security',
        'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'stay_time': 0.0}}))
    first = call(StartTask, '/task_control/start', req)
    assert first.accepted
    deadline = time.monotonic() + 3
    while not boundary['requests'] and time.monotonic() < deadline:
        time.sleep(0.01)
    assert boundary['requests']
    cancel = call(CancelTask, '/task_control/cancel', CancelTask.Request(task_id=first.task_id))
    assert cancel.accepted
    pending = json.loads(call(GetTaskStatus, '/task_control/status',
                              GetTaskStatus.Request()).status_json)
    assert pending['active'] and pending['phase'] == 'stopping'
    assert not call(StartTask, '/task_control/start', req).accepted
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        ended = json.loads(call(GetTaskStatus, '/task_control/status',
                                GetTaskStatus.Request()).status_json)
        if not ended['active']:
            break
        time.sleep(0.02)
    assert ended['result'] == 'canceled', ended
    assert len(boundary['canceled']) == 1
    assert 'inspecting' not in ended['phases']
    boundary['accept_delay'] = 0.0
    boundary['execute_time'] = 0.08
    second = call(StartTask, '/task_control/start', req)
    assert second.accepted and second.task_id != first.task_id
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        status = json.loads(call(GetTaskStatus, '/task_control/status',
                                 GetTaskStatus.Request()).status_json)
        if not status['active']:
            break
        time.sleep(0.02)
    assert status['task_id'] == second.task_id and status['result'] == 'succeeded'
    assert len(goals) == 2


@pytest.mark.parametrize('ros_system', ['process'], indirect=True)
@pytest.mark.parametrize('signal_name', ['SIGINT', 'SIGTERM'])
def test_process_signal_cancels_owned_motion_stops_and_writes_report(ros_system, signal_name):
    import signal
    from pathlib import Path
    call, goals, commands, boundary = ros_system
    boundary['execute_time'] = 3.0
    req = StartTask.Request(task_type='inspection', request_json=json.dumps({
        'frame_id': 'map', 'point': {'id': 'interrupted', 'type': 'security',
        'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'stay_time': 2.0}}))
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        reply = call(StartTask, '/task_control/start', req)
        if reply.accepted:
            break
        time.sleep(0.05)
    assert reply.accepted
    deadline = time.monotonic() + 3
    while not goals and time.monotonic() < deadline:
        time.sleep(0.01)
    assert goals
    process = boundary['process']
    process.send_signal(getattr(signal, signal_name))
    stdout, stderr = process.communicate(timeout=7)
    assert process.returncode == 0, stdout + stderr
    report_path = Path(boundary['report_dir']) / (reply.task_id + '.json')
    assert report_path.exists(), stdout + stderr
    report = json.loads(report_path.read_text())
    assert report['result'] == 'failed' and report['error'] == 'supervisor shutdown'
    assert report['cleanup_confirmed']
    assert len(boundary['canceled']) == 1 and commands[-1] is True
