"""Real Humble navigator/vehicle BT, fake planner/controller and no hardware."""
import json
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import BackUp, ComputePathThroughPoses, ComputePathToPose, FollowPath, Wait
from nav2_msgs.srv import ClearEntireCostmap
from nav2_mission_interfaces.srv import CancelTask, GetTaskStatus, StartTask
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.graph import get_action_client_names_and_types_by_node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from nav2_mission.task_control_node import TaskControlNode


def wait(predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(0.03)
    raise AssertionError('timed out waiting for real navigator boundary')


def test_humble_navigator_owns_its_follow_path_and_drains_it(tmp_path):
    rclpy.init()
    fake = Node('bt_external_boundaries')
    group = ReentrantCallbackGroup()
    observed = {'follow': 0, 'stop': True}
    servers = []

    def action(cls, name):
        def execute(handle):
            result = cls.Result()
            if cls in (ComputePathToPose, ComputePathThroughPoses):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.orientation.w = 1.0
                result.path.header.frame_id = 'map'
                result.path.poses = [pose, handle.request.goal] if cls == ComputePathToPose else [pose]
            if cls == FollowPath:
                observed['follow'] += 1
                while not handle.is_cancel_requested:
                    time.sleep(0.02)
                handle.canceled()
            else:
                handle.succeed()
            return result
        servers.append(ActionServer(fake, cls, name, execute, callback_group=group,
                                    cancel_callback=lambda _: CancelResponse.ACCEPT))

    for cls, name in ((ComputePathToPose, '/compute_path_to_pose'),
                      (ComputePathThroughPoses, '/compute_path_through_poses'),
                      (FollowPath, '/follow_path'), (Wait, '/wait'), (BackUp, '/backup')):
        action(cls, name)
    for name in ('/global_costmap/clear_entirely_global_costmap',
                 '/local_costmap/clear_entirely_local_costmap'):
        fake.create_service(ClearEntireCostmap, name, lambda request, response: response,
                            callback_group=group)
    tf = StaticTransformBroadcaster(fake)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'base_link'
    transform.transform.rotation.w = 1.0
    tf.sendTransform(transform)
    loc = fake.create_publisher(String, '/task_control/localization', 10)
    safety = fake.create_publisher(String, '/task_control/state', 10)
    fake.create_subscription(Bool, '/task_control/stop',
                             lambda msg: observed.update(stop=msg.data), 10)

    def states():
        stamp = fake.get_clock().now().nanoseconds / 1e9
        loc.publish(String(data=json.dumps({'valid': True, 'frame_id': 'map', 'source': 'fake',
                                           'source_stamp_sec': stamp, 'x': 0, 'y': 0, 'yaw': 0})))
        safety.publish(String(data=json.dumps({'ready': True, 'stopped': observed['stop'],
            'stop_requested': observed['stop'], 'manual_override': False, 'emergency_stop': False,
            'fault': '', 'feedback_age_sec': 0.0})))
    fake.create_timer(0.02, states, callback_group=group)
    manager = TaskControlNode(parameter_overrides=[Parameter('report_dir', value=str(tmp_path))])
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(fake)
    executor.add_node(manager)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    tree_dir = Path(__file__).resolve().parents[2] / 'wheeltec_nav2' / 'behavior_trees'
    process = subprocess.Popen(['/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator', '--ros-args',
        '-p', f'default_nav_to_pose_bt_xml:={tree_dir / "navigate_to_pose_ackermann.xml"}',
        '-p', f'default_nav_through_poses_bt_xml:={tree_dir / "navigate_through_poses_ackermann.xml"}',
        '-p', 'wait_for_service_timeout:=1000'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def call(kind, name, request):
        client = fake.create_client(kind, name)
        assert client.wait_for_service(timeout_sec=10), name
        future = client.call_async(request)
        wait(future.done)
        result = future.result()
        fake.destroy_client(client)
        return result
    try:
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            assert call(ChangeState, '/bt_navigator/change_state', request).success
        def graph():
            return {name: dict(get_action_client_names_and_types_by_node(fake, name, ns))
                    for name, ns in fake.get_node_names_and_namespaces()}
        clients = wait(lambda: (g if '/follow_path' in g.get(
            'bt_navigator_navigate_to_pose_rclcpp_node', {}) else None) if (g := graph()) else None)
        assert '/navigate_to_pose' in clients['bt_navigator']
        assert '/follow_path' in clients['bt_navigator_navigate_through_poses_rclcpp_node']
        request = StartTask.Request(task_type='navigation', request_json=json.dumps({
            'frame_id': 'map', 'point': {'x': 1.0, 'y': 0.0, 'yaw': 0.0}}))
        started = call(StartTask, '/task_control/start', request)
        assert started.accepted, started.message
        wait(lambda: observed['follow'])
        def child_observed():
            status = json.loads(call(GetTaskStatus, '/task_control/status', GetTaskStatus.Request()).status_json)
            assert status['active'] and not status.get('error'), status
            return status if status.get('motion_ownership', {}).get('inferred_navigator_follow_ids') else None
        status = wait(child_observed)
        assert not status['motion_ownership']['unknown_active_goals']
        assert call(CancelTask, '/task_control/cancel', CancelTask.Request(task_id=started.task_id)).accepted
        def ended():
            status = json.loads(call(GetTaskStatus, '/task_control/status', GetTaskStatus.Request()).status_json)
            return status if not status['active'] else None
        status = wait(ended)
        assert status['result'] == 'canceled' and status['cleanup_confirmed'], status
        assert observed['stop']
    finally:
        process.terminate()
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
        manager.shutdown_tasks()
        executor.shutdown(timeout_sec=3)
        worker.join(timeout=3)
        for server in servers:
            server.destroy()
        manager.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()
