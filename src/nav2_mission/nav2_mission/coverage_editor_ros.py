"""RViz and service adapter, sharing TaskControlNode's default callback group."""
import json

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from nav2_mission_interfaces.srv import RegionCommand
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .coverage_session import CoverageSession
from .coverage_reference import ObservedCoverage
from .task_ros_adapters import decode_json


class CoverageEditorRos:
    def __init__(self, node, manager):
        self.node = node
        self.manager = manager
        self.session = CoverageSession(manager)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = node.create_publisher(MarkerArray, '/coverage/region_markers', qos)
        self.state = node.create_publisher(String, '/coverage/editor_state', qos)
        self.plan = node.create_publisher(Path, '/coverage/planned_path', qos)
        self.trace = node.create_publisher(MarkerArray, '/coverage/executed_trace', qos)
        self.progress = node.create_publisher(String, '/coverage/reference_statistics', qos)
        handler = manager.handlers.get('coverage')
        self.observer = None
        if handler is not None:
            self.observer = ObservedCoverage(handler, max_age=manager.state_timeout)
            manager.handlers['coverage'] = self.observer
            self.localization = node.create_subscription(
                String, node.get_parameter('localization_topic').value, self.observe, 10)
        self.service = node.create_service(RegionCommand, '/task_control/region', self.command)
        self.points = node.create_subscription(PointStamped, '/clicked_point', self.point, 10)
        self.publish()

    def observe(self, message):
        try:
            data = decode_json(message.data)
        except ValueError:
            data = {}
        self.observer.observe(data, self.node.get_clock().now().nanoseconds / 1e9)

    def command(self, request, response):
        try:
            reply = self.session.command(request.operation, decode_json(request.request_json or '{}'))
        except ValueError as exc:
            reply = self.session.reply(False, str(exc))
        response.accepted = reply['accepted']
        response.task_id = reply['task_id']
        response.message = reply['message']
        response.state_json = json.dumps(reply['state'], allow_nan=False)
        self.publish()
        return response

    def point(self, message):
        reply = self.session.command('point', {
            'x': message.point.x, 'y': message.point.y,
            'frame_id': message.header.frame_id})
        if not reply['accepted']:
            self.node.get_logger().warn(reply['message'])
        self.publish()

    def publish(self):
        state = self.session.status()
        self.state.publish(String(data=json.dumps(state, allow_nan=False)))
        frame = state['frame_id']
        rings = state['polygons']
        closed = state['mode'] != 'OUTER_DRAFT'
        status = self.manager.status()
        config = status.get('config', {})
        if status.get('active') and status.get('task_type') == 'coverage':
            frame, rings, closed = config['frame_id'], config['polygons'], True
        array = MarkerArray()
        array.markers.append(Marker(action=Marker.DELETEALL))
        for index, ring in enumerate([*rings, state['draft_hole']]):
            if not ring:
                continue
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns, marker.id = 'coverage_region', index
            marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.04
            marker.color.a = 1.0
            marker.color.g = 0.9 if index == 0 else 0.3
            marker.color.r = 0.2 if index == 0 else 1.0
            points = [*ring, ring[0]] if index < len(rings) and closed else ring
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=0.04) for p in points]
            array.markers.append(marker)
        self.markers.publish(array)
        details = status.get('details', {}) if status.get('task_type') == 'coverage' else {}
        planned = details.get('planned_path') or {}
        path = Path()
        path.header.frame_id = planned.get('frame_id', frame)
        path.header.stamp = self.node.get_clock().now().to_msg()
        for item in planned.get('poses', []):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(item['x']), float(item['y'])
            pose.pose.position.z = float(item['z'])
            orientation = item['orientation']
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = orientation
            path.poses.append(pose)
        self.plan.publish(path)
        traced = details.get('executed_trace', {})
        traces = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for index, segment in enumerate(traced.get('segments', [])):
            marker = Marker()
            marker.header.frame_id = traced['frame_id']
            marker.header.stamp = path.header.stamp
            marker.ns, marker.id = 'coverage_trace', index
            marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.025
            marker.color.b, marker.color.g, marker.color.a = 1.0, 0.7, 1.0
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=0.06) for p in segment]
            traces.markers.append(marker)
        self.trace.publish(traces)
        self.progress.publish(String(data=json.dumps(details.get('reference_statistics', {}),
                                                    allow_nan=False)))
