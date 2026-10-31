"""Edit-only region service for standalone planning, with no motion clients."""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from .coverage_editor_ros import CoverageEditorRos


class EditOnlyManager:
    active = False

    def __init__(self):
        self.handlers = {}

    def status(self):
        return {}

    def start(self, task_type, request):
        return {'accepted': False, 'task_id': '',
                'message': 'standalone preview cannot execute tasks'}


class CoverageEditorNode(Node):
    def __init__(self, **kwargs):
        super().__init__('coverage_editor', **kwargs)
        self.editor = CoverageEditorRos(self, EditOnlyManager())
        self.timer = self.create_timer(0.2, self.editor.publish)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageEditorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
