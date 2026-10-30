"""Operator pose goals share the same public lifecycle as inspection/coverage."""
import json
import time

from geometry_msgs.msg import PoseStamped
from nav2_mission_interfaces.srv import CancelTask, GetTaskStatus, StartTask

pytest_plugins = ['test_task_control_ros']


def status(call):
    return json.loads(call(GetTaskStatus, '/task_control/status',
                           GetTaskStatus.Request()).status_json)


def wait_for(call, predicate):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        observed = status(call)
        if predicate(observed):
            return observed
        time.sleep(0.02)
    raise AssertionError(observed)


def test_ordinary_navigation_is_managed_and_has_no_simulated_inspection(ros_system):
    call, goals, commands, boundary = ros_system
    boundary['execute_time'] = 0.4
    request = StartTask.Request(task_type='navigation', request_json=json.dumps({
        'frame_id': 'map', 'point': {'x': 3.0, 'y': 2.0, 'yaw': 0.5}}))
    started = call(StartTask, '/task_control/start', request)
    assert started.accepted, started.message
    assert not call(StartTask, '/task_control/start', request).accepted
    ended = wait_for(call, lambda value: value.get('result'))
    assert ended['result'] == 'succeeded' and ended['cleanup_confirmed']
    assert not ended['simulated'] and 'inspecting' not in ended['phases']
    assert goals[0].pose.pose.position.x == 3.0
    assert json.loads(open(ended['report_path']).read())['task_type'] == 'navigation'


def test_rviz_pose_goal_uses_manager_and_refuses_active_replacement(ros_system):
    import math
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    call, goals, commands, boundary = ros_system
    boundary['execute_time'] = 2.0
    operator = Node('managed_pose_operator')
    publisher = operator.create_publisher(PoseStamped, '/task_control/goal_pose', 10)
    replies = []
    operator.create_subscription(String, '/task_control/goal_response',
                                 lambda msg: replies.append(json.loads(msg.data)), 10)
    try:
        deadline = time.monotonic() + 3
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 4.0
        goal.pose.orientation.z = math.sin(0.2)
        goal.pose.orientation.w = math.cos(0.2)
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            publisher.publish(goal)
            count = len(replies)
            while len(replies) == count and time.monotonic() < deadline:
                rclpy.spin_once(operator, timeout_sec=0.05)
            if replies and replies[-1]['accepted']:
                break
            time.sleep(0.1)
        assert replies and replies[-1]['accepted'], replies
        replies.clear()
        started = wait_for(call, lambda value: value.get('phase') == 'navigating')
        goal.pose.position.x = 99.0
        publisher.publish(goal)
        deadline = time.monotonic() + 3
        while len(replies) < 1 and time.monotonic() < deadline:
            rclpy.spin_once(operator, timeout_sec=0.05)
        assert len(replies) == 1 and not replies[-1]['accepted'], replies
        call(CancelTask, '/task_control/cancel', CancelTask.Request())
        ended = wait_for(call, lambda value: not value['active'])
        assert ended['task_id'] == started['task_id'] and ended['result'] == 'canceled'
        assert len(goals) == 1 and goals[0].pose.pose.position.x == 4.0
    finally:
        operator.destroy_node()


def test_raw_navigation_client_blocks_admission_and_aborts_managed_motion(ros_system):
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    call, goals, commands, boundary = ros_system
    boundary['execute_time'] = 3.0
    req = StartTask.Request(task_type='navigation', request_json=json.dumps({
        'frame_id': 'map', 'point': {'x': 1.0, 'y': 0.0, 'yaw': 0.0}}))
    outsider = Node('raw_navigation_outsider')
    client = ActionClient(outsider, NavigateToPose, '/test_task/navigate_to_pose')
    try:
        time.sleep(0.2)
        rejected = call(StartTask, '/task_control/start', req)
        assert not rejected.accepted and 'external' in rejected.message, rejected.message
        client.destroy()
        client = None
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            started = call(StartTask, '/task_control/start', req)
            if started.accepted:
                break
            time.sleep(0.1)
        assert started.accepted, started.message
        wait_for(call, lambda value: value.get('phase') == 'navigating')
        client = ActionClient(outsider, NavigateToPose, '/test_task/navigate_to_pose')
        ended = wait_for(call, lambda value: not value['active'])
        assert ended['result'] == 'failed' and 'external' in ended['error']
        assert ended['cleanup_confirmed'] and commands[-1] is True
    finally:
        if client is not None:
            client.destroy()
        outsider.destroy_node()


def test_navigation_rejects_unrepresentable_coordinates_before_authorization(ros_system):
    call, goals, commands, boundary = ros_system
    for value in (10 ** 400, float('nan'), float('inf')):
        request = StartTask.Request(task_type='navigation', request_json=json.dumps({
            'frame_id': 'map', 'point': {'x': value, 'y': 0.0, 'yaw': 0.0}}))
        rejected = call(StartTask, '/task_control/start', request)
        assert not rejected.accepted and 'finite' in rejected.message
    assert not goals and False not in commands
    assert status(call)['phase'] == 'idle'


def test_active_raw_goal_blocks_new_task_even_after_client_exits(ros_system):
    import threading
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    call, goals, commands, boundary = ros_system
    boundary['execute_time'] = 2.0
    outsider = Node('raw_goal_before_manager_request')
    client = ActionClient(outsider, NavigateToPose, '/test_task/navigate_to_pose')
    executor = SingleThreadedExecutor()
    executor.add_node(outsider)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        assert client.wait_for_server(timeout_sec=3)
        future = client.send_goal_async(NavigateToPose.Goal())
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert future.done() and future.result().accepted
        time.sleep(0.25)
        client.destroy()
        client = None
        request = StartTask.Request(task_type='navigation', request_json=json.dumps({
            'frame_id': 'map', 'point': {'x': 1.0, 'y': 0.0, 'yaw': 0.0}}))
        rejected = call(StartTask, '/task_control/start', request)
        assert not rejected.accepted and 'external navigation conflict' in rejected.message
        assert False not in commands
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            started = call(StartTask, '/task_control/start', request)
            if started.accepted:
                break
            time.sleep(0.1)
        assert started.accepted, started.message
        assert call(CancelTask, '/task_control/cancel',
                    CancelTask.Request(task_id=started.task_id)).accepted
        wait_for(call, lambda value: not value['active'])
    finally:
        if client is not None:
            client.destroy()
        executor.shutdown(timeout_sec=3)
        worker.join(timeout=3)
        outsider.destroy_node()
