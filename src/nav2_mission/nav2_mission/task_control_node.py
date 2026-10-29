"""ROS start/cancel/status services for the unified task manager."""
import json
import os
import signal
import threading
import time

import rclpy
from nav2_mission_interfaces.srv import CancelTask, GetTaskStatus, StartTask
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .task_control import TaskManager
from .task_ros_adapters import RosLocalization, RosNavigation, RosStop, decode_json


class TaskControlNode(Node):
    def __init__(self, **kwargs):
        super().__init__('task_control_node', **kwargs)
        defaults = {
            'navigate_action_name': '/navigate_to_pose',
            'localization_topic': '/task_control/localization',
            'map_id': '', 'localization_profile': '', 'require_localization_schema': False,
            'report_dir': os.path.expanduser('~/.ros/task_reports'),
            'state_timeout_sec': 1.0, 'cancel_timeout_sec': 5.0, 'task_timeout_sec': 600.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def param(name):
            return self.get_parameter(name).value

        self.stop_adapter = RosStop(self)
        self.manager = TaskManager(
            RosNavigation(self, param('navigate_action_name')),
            RosLocalization(self, param('localization_topic')), self.stop_adapter,
            param('report_dir'), state_timeout=param('state_timeout_sec'),
            cancel_timeout=param('cancel_timeout_sec'), task_timeout=param('task_timeout_sec'),
            map_id=param('map_id'), localization_profile=param('localization_profile'),
            require_localization_schema=param('require_localization_schema'))
        self.task_services = [
            self.create_service(StartTask, '/task_control/start', self.start_task),
            self.create_service(CancelTask, '/task_control/cancel', self.cancel_task),
            self.create_service(GetTaskStatus, '/task_control/status', self.get_status),
        ]
        self.status_publisher = self.create_publisher(
            String, '/task_control/status_snapshot', 10)
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
        if not self.manager.active or self.manager.status()['phase'] == 'stopping':
            self.stop_adapter.request_stop('')
        self.status_publisher.publish(String(
            data=json.dumps(self.manager.status(), allow_nan=False)))

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
