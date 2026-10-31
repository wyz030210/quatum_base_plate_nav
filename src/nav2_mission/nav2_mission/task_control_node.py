"""ROS start/cancel/status services for the unified task manager."""
import json
import math
import os
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_mission_interfaces.srv import CancelTask, GetTaskStatus, StartTask
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .coverage_editor_ros import CoverageEditorRos
from .coverage_task import CoverageTask
from .coverage_task_ros import RosCoverage
from .task_control import TaskManager
from .motion_ownership import MotionOwnership
from .task_ros_adapters import RosLocalization, RosNavigation, RosStop, decode_json


class TaskControlNode(Node):
    def __init__(self, **kwargs):
        super().__init__('task_control_node', **kwargs)
        defaults = {
            'navigate_action_name': '/navigate_to_pose',
            'follow_action_name': '/follow_path',
            'navigator_node': '/bt_navigator_navigate_to_pose_rclcpp_node',
            'through_navigator_node': '/bt_navigator_navigate_through_poses_rclcpp_node',
            'navigator_server_node': '/bt_navigator',
            'through_action_name': '/navigate_through_poses',
            'coverage_bridge_node': '/coverage_navpath_bridge',
            'coverage_action_name': '/execute_coverage_navpath',
            'coverage_prepare_service': '/coverage/prepare',
            'coverage_state_topic': '/coverage/execution_state',
            'localization_topic': '/task_control/localization',
            'map_id': '', 'localization_profile': '', 'require_localization_schema': False,
            'report_dir': os.path.expanduser('~/.ros/task_reports'),
            'state_timeout_sec': 1.0, 'cancel_timeout_sec': 5.0, 'task_timeout_sec': 600.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def param(name):
            return self.get_parameter(name).value

        self.ownership = MotionOwnership(
            self, param('navigate_action_name'), param('follow_action_name'),
            param('coverage_action_name'), navigator=param('navigator_node'),
            bridge=param('coverage_bridge_node'),
            through_navigator=param('through_navigator_node'),
            navigator_server=param('navigator_server_node'), through=param('through_action_name'))
        self.stop_adapter = RosStop(self)
        self.navigation_adapter = RosNavigation(
            self, param('navigate_action_name'), self.ownership)
        self.manager = TaskManager(
            self.navigation_adapter,
            RosLocalization(self, param('localization_topic')), self.stop_adapter,
            param('report_dir'), state_timeout=param('state_timeout_sec'),
            cancel_timeout=param('cancel_timeout_sec'), task_timeout=param('task_timeout_sec'),
            map_id=param('map_id'), localization_profile=param('localization_profile'),
            require_localization_schema=param('require_localization_schema'),
            motion_guard=self.ownership)
        self.coverage_adapter = RosCoverage(
            self, param('coverage_action_name'), param('coverage_prepare_service'),
            param('coverage_state_topic'), ownership=self.ownership)
        self.manager.handlers['coverage'] = CoverageTask(self.coverage_adapter)
        self.coverage_editor = CoverageEditorRos(self, self.manager)
        self.task_services = [
            self.create_service(StartTask, '/task_control/start', self.start_task),
            self.create_service(CancelTask, '/task_control/cancel', self.cancel_task),
            self.create_service(GetTaskStatus, '/task_control/status', self.get_status),
        ]
        self.status_publisher = self.create_publisher(
            String, '/task_control/status_snapshot', 10)
        self.pose_reply = self.create_publisher(String, '/task_control/goal_response', 10)
        self.pose_subscription = self.create_subscription(
            PoseStamped, '/task_control/goal_pose', self.pose_goal, 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'Task control ready; state freshness={self.manager.state_timeout}s, '
            f'cancel timeout={self.manager.cancel_timeout}s, '
            f'task timeout={self.manager.task_timeout}s; '
            'safety/health adapters must provide fresh state')

    def start_task(self, request, response):
        try:
            reply = self.manager.start(request.task_type, decode_json(request.request_json))
        except ValueError as exc:
            reply = {'accepted': False, 'task_id': '', 'message': str(exc)}
        response.accepted = reply['accepted']
        response.task_id = reply['task_id']
        response.message = reply['message']
        return response

    def pose_goal(self, message):
        try:
            pose = message.pose
            quaternion = pose.orientation
            values = (pose.position.x, pose.position.y, pose.position.z,
                      quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            if (not all(math.isfinite(value) for value in values) or
                    abs(sum(value * value for value in values[3:]) - 1.0) > 0.01 or
                    abs(quaternion.x) > 1e-6 or abs(quaternion.y) > 1e-6):
                raise ValueError('goal requires a finite planar pose and unit quaternion')
            request = {'frame_id': message.header.frame_id, 'point': {
                'x': pose.position.x, 'y': pose.position.y,
                'yaw': 2.0 * math.atan2(quaternion.z, quaternion.w)}}
            reply = self.manager.start('navigation', request)
        except ValueError as exc:
            reply = {'accepted': False, 'task_id': '', 'message': str(exc)}
        self.pose_reply.publish(String(data=json.dumps(reply, allow_nan=False)))

    def cancel_task(self, request, response):
        reply = self.manager.cancel(request.task_id)
        response.accepted = reply['accepted']
        response.task_id = reply['task_id']
        response.message = reply['message']
        return response

    def get_status(self, request, response):
        response.status_json = json.dumps(self.manager.status(), allow_nan=False)
        return response

    def tick(self):
        self.stop_adapter.beat()
        self.manager.tick()
        self.coverage_editor.publish()
        if not self.manager.active or self.manager.status()['phase'] == 'stopping':
            self.stop_adapter.request_stop('')
        self.status_publisher.publish(String(
            data=json.dumps(self.manager.status(), allow_nan=False)))

    def destroy_node(self):
        # rclpy Node.destroy_node does not destroy action-client waitables.
        self.navigation_adapter.client.destroy()
        self.coverage_adapter.client.destroy()
        return super().destroy_node()

    def shutdown_tasks(self):
        self.manager.shutdown()
        self.stop_adapter.request_stop('')


def main(args=None):
    # Keep ROS alive until our owned actions and stop feedback have been serviced.
    # rclpy's default signal handler shuts the context down before this cleanup.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    shutdown_requested = threading.Event()

    def request_shutdown(signum, frame):
        shutdown_requested.set()

    previous_handlers = {
        signum: signal.signal(signum, request_shutdown)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    node = TaskControlNode()
    try:
        while rclpy.ok() and not shutdown_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_tasks()
        deadline = time.monotonic() + node.manager.cancel_timeout
        while rclpy.ok() and node.manager.active and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
