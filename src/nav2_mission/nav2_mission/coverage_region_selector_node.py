#!/usr/bin/env python3
"""RViz polygon selection for coverage execution or planning-only preview."""

from __future__ import annotations

import math
import threading
from typing import List, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Point32, PointStamped, Polygon, PolygonStamped
from nav_msgs.msg import Path
from opennav_coverage_msgs.action import ComputeCoveragePath, NavigateCompleteCoverage
from opennav_coverage_msgs.msg import Coordinate, Coordinates
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class CoverageRegionSelector(Node):
    """Collect polygon points from RViz and manually execute coverage on demand."""

    def __init__(self, **kwargs) -> None:
        super().__init__("coverage_region_selector", **kwargs)

        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("marker_topic", "/coverage_ui/markers")
        self.declare_parameter("polygon_topic", "/coverage_ui/polygon")
        self.declare_parameter("default_frame_id", "map")
        self.declare_parameter("close_distance", 0.35)
        self.declare_parameter("min_points", 3)
        self.declare_parameter("execute_action_name", "/execute_coverage_navpath")
        self.declare_parameter("server_wait_timeout_sec", 5.0)
        self.declare_parameter("plan_only", False)
        self.declare_parameter("auto_plan_on_close", False)
        self.declare_parameter("compute_action_name", "/compute_coverage_path")
        self.declare_parameter("preview_path_topic", "/coverage_ui/preview_path")

        self._clicked_point_topic = str(self.get_parameter("clicked_point_topic").value)
        self._marker_topic = str(self.get_parameter("marker_topic").value)
        self._polygon_topic = str(self.get_parameter("polygon_topic").value)
        self._default_frame_id = str(self.get_parameter("default_frame_id").value)
        self._close_distance = float(self.get_parameter("close_distance").value)
        self._min_points = int(self.get_parameter("min_points").value)
        self._execute_action_name = str(self.get_parameter("execute_action_name").value)
        self._server_wait_timeout_sec = float(
            self.get_parameter("server_wait_timeout_sec").value
        )
        self._plan_only = bool(self.get_parameter("plan_only").value)
        self._auto_plan_on_close = self._plan_only and bool(
            self.get_parameter("auto_plan_on_close").value
        )
        if self._plan_only:
            self._execute_action_name = str(self.get_parameter("compute_action_name").value)

        self._points: List[Tuple[float, float]] = []
        self._frame_id = self._default_frame_id
        self._closed = False
        self._goal_active = False
        self._state_lock = threading.Lock()

        preview_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ui_qos = preview_qos if self._plan_only else 10
        self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic, ui_qos)
        self._polygon_pub = self.create_publisher(PolygonStamped, self._polygon_topic, ui_qos)
        self._preview_path_pub = None
        if self._plan_only:
            self._preview_path_pub = self.create_publisher(
                Path, str(self.get_parameter("preview_path_topic").value), preview_qos
            )
            self._preview_timer = self.create_timer(1.0, self._refresh_preview)
        self._clicked_sub = self.create_subscription(
            PointStamped, self._clicked_point_topic, self._on_clicked_point, 10
        )
        self._execute_client = ActionClient(
            self, ComputeCoveragePath if self._plan_only else NavigateCompleteCoverage,
            self._execute_action_name
        )

        self._clear_srv = self.create_service(Trigger, "clear", self._on_clear)
        self._undo_srv = self.create_service(Trigger, "undo_last_point", self._on_undo)
        self._close_only_srv = self.create_service(
            Trigger, "close_polygon", self._on_close_polygon
        )
        self._execute_srv = self.create_service(
            Trigger, "execute_polygon", self._on_execute_polygon
        )
        self._close_execute_srv = self.create_service(
            Trigger, "close_and_execute", self._on_close_and_execute
        )

        self.get_logger().info(
            "coverage_region_selector ready: "
            f"clicked_point_topic={self._clicked_point_topic}, "
            f"default_frame={self._default_frame_id}, "
            f"action={self._execute_action_name}, plan_only={self._plan_only}"
        )

    def _refresh_preview(self) -> None:
        with self._state_lock:
            self._publish_visualization_locked()

    def _clear_preview_locked(self) -> None:
        if self._preview_path_pub is not None:
            path = Path()
            path.header.frame_id = self._frame_id or self._default_frame_id
            path.header.stamp = self.get_clock().now().to_msg()
            self._preview_path_pub.publish(path)

    def _on_clicked_point(self, msg: PointStamped) -> None:
        with self._state_lock:
            if self._goal_active:
                self.get_logger().warn("Coverage is running; point input ignored.")
                return
            frame_id = msg.header.frame_id.strip() if msg.header.frame_id else ""
            if not frame_id:
                frame_id = self._default_frame_id

            x = float(msg.point.x)
            y = float(msg.point.y)
            if not math.isfinite(x) or not math.isfinite(y):
                self.get_logger().warn("Ignoring a non-finite polygon point.")
                return

            if not self._points:
                self._frame_id = frame_id
                self._points.append((x, y))
                self._closed = False
                self.get_logger().info(
                    f"Started polygon in frame={self._frame_id}, points=1"
                )
                self._publish_visualization_locked()
                return

            if frame_id != self._frame_id:
                self.get_logger().warn(
                    f"Frame changed from {self._frame_id} to {frame_id}, starting a new polygon."
                )
                self._reset_locked(frame_id)
                self._points.append((x, y))
                self._publish_visualization_locked()
                return

            if self._closed:
                self.get_logger().info("Previous polygon completed. Starting a new one.")
                self._reset_locked(frame_id)
                self._points.append((x, y))
                self._publish_visualization_locked()
                return

            if (
                len(self._points) >= self._min_points
                and self._distance(self._points[0], (x, y)) <= self._close_distance
            ):
                self._close_polygon_locked()
                if self._auto_plan_on_close:
                    ok, message = self._send_goal_locked()
                    if not ok:
                        self.get_logger().error(message)
                return

            self._points.append((x, y))
            self.get_logger().info(
                f"Added point {len(self._points)} at ({x:.3f}, {y:.3f})"
            )
            self._publish_visualization_locked()

    def _on_clear(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            if self._goal_active:
                response.success = False
                response.message = "Coverage is running; clear is blocked."
                return response
            self._reset_locked(self._default_frame_id)
            self._publish_visualization_locked()
            response.success = True
            response.message = "Polygon cleared."
            return response

    def _on_undo(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self._state_lock:
            if self._goal_active:
                response.success = False
                response.message = "Coverage is running; undo is blocked."
                return response
            if not self._points:
                response.success = False
                response.message = "No point to undo."
                return response
            if self._closed:
                self._closed = False
            else:
                self._points.pop()
            self._clear_preview_locked()
            self._publish_visualization_locked()
            response.success = True
            response.message = f"Points remaining: {len(self._points)}"
            return response

    def _on_close_polygon(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._state_lock:
            if self._goal_active:
                response.success = False
                response.message = "Coverage is running; close is blocked."
                return response
            if self._close_polygon_locked():
                response.success = True
                response.message = f"Polygon closed with {len(self._points)} points."
                if self._auto_plan_on_close:
                    response.success, response.message = self._send_goal_locked()
            else:
                response.success = False
                response.message = (
                    f"Need at least {self._min_points} points, current={len(self._points)}."
                )
            return response

    def _on_execute_polygon(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._state_lock:
            ok, msg = self._send_goal_locked()
            response.success = ok
            response.message = msg
            return response

    def _on_close_and_execute(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._state_lock:
            if self._goal_active:
                response.success = False
                response.message = "Coverage is running; new request rejected."
                return response
            if not self._closed:
                if not self._close_polygon_locked():
                    response.success = False
                    response.message = (
                        f"Need at least {self._min_points} points, current={len(self._points)}."
                    )
                    return response
            ok, msg = self._send_goal_locked()
            response.success = ok
            response.message = msg
            return response

    def _publish_visualization_locked(self) -> None:
        stamp = self.get_clock().now().to_msg()
        frame_id = self._frame_id or self._default_frame_id

        vertices = Marker()
        vertices.header.frame_id = frame_id
        vertices.header.stamp = stamp
        vertices.ns = "coverage_ui"
        vertices.id = 1
        vertices.type = Marker.SPHERE_LIST
        vertices.action = Marker.ADD
        vertices.pose.orientation.w = 1.0
        vertices.scale.x = 0.12
        vertices.scale.y = 0.12
        vertices.scale.z = 0.12
        vertices.color.r = 0.15
        vertices.color.g = 0.75
        vertices.color.b = 0.95
        vertices.color.a = 1.0

        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp
        line.ns = "coverage_ui"
        line.id = 2
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.05
        line.color.r = 1.0
        line.color.g = 0.82
        line.color.b = 0.08
        line.color.a = 1.0

        for x, y in self._points:
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.02
            vertices.points.append(point)
            line.points.append(point)

        if self._closed and len(line.points) >= 1:
            line.points.append(line.points[0])

        status = Marker()
        status.header.frame_id = frame_id
        status.header.stamp = stamp
        status.ns = "coverage_ui"
        status.id = 3
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.orientation.w = 1.0
        status.scale.z = 0.22
        status.color.r = 1.0
        status.color.g = 1.0
        status.color.b = 1.0
        status.color.a = 1.0
        if self._plan_only:
            status.color.r = 0.15
            status.color.g = 0.15
            status.color.b = 0.15
        if self._points:
            status.pose.position.x = float(self._points[-1][0])
            status.pose.position.y = float(self._points[-1][1])
        status.pose.position.z = 0.35
        state = "RUNNING" if self._goal_active else ("CLOSED" if self._closed else "DRAWING")
        status.text = (
            f"{state} | pts={len(self._points)} | frame={frame_id} | "
            "close by clicking near first point"
        )

        marker_array = MarkerArray()
        marker_array.markers = [vertices, line] if self._plan_only else [vertices, line, status]
        if self._plan_only:
            # Publish Point needs selectable geometry when no map or sensor is running.
            ground = Marker()
            ground.header = line.header
            ground.ns = "coverage_ui"
            ground.id = 4
            ground.type = Marker.CUBE
            ground.action = Marker.ADD
            ground.pose.orientation.w = 1.0
            ground.pose.position.z = -0.02
            ground.scale.x = 200.0
            ground.scale.y = 200.0
            ground.scale.z = 0.02
            ground.color.r = 0.92
            ground.color.g = 0.94
            ground.color.b = 0.95
            ground.color.a = 1.0
            marker_array.markers.append(ground)
        self._marker_pub.publish(marker_array)

        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = frame_id
        polygon_msg.header.stamp = stamp
        polygon_msg.polygon = self._build_polygon_msg()
        self._polygon_pub.publish(polygon_msg)

    def _build_polygon_msg(self) -> Polygon:
        polygon = Polygon()
        for x, y in self._points:
            point = Point32()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.0
            polygon.points.append(point)
        return polygon

    def _reset_locked(self, frame_id: str) -> None:
        self._points.clear()
        self._frame_id = frame_id
        self._closed = False
        self._clear_preview_locked()

    def _close_polygon_locked(self) -> bool:
        if len(self._points) < self._min_points:
            return False
        if self._closed:
            return True
        self._closed = True
        self.get_logger().info(f"Polygon closed with {len(self._points)} points.")
        self._publish_visualization_locked()
        return True

    def _send_goal_locked(self) -> Tuple[bool, str]:
        if self._goal_active:
            return False, "Coverage is running; new request rejected."
        if not self._closed:
            return False, "Polygon is not closed."
        if len(self._points) < self._min_points:
            return False, f"Need at least {self._min_points} points, current={len(self._points)}."
        if not self._execute_client.wait_for_server(timeout_sec=self._server_wait_timeout_sec):
            return False, f"Action server unavailable: {self._execute_action_name}"

        if self._plan_only:
            goal = ComputeCoveragePath.Goal()
            goal.generate_headland = True
            goal.generate_route = True
            goal.generate_path = True
            goal.use_gml_file = False
            polygon = Coordinates()
            for x, y in self._points + [self._points[0]]:
                point = Coordinate()
                point.axis1 = float(x)
                point.axis2 = float(y)
                polygon.coordinates.append(point)
        else:
            goal = NavigateCompleteCoverage.Goal()
            goal.field_filepath = ""
            goal.behavior_tree = ""
            polygon = self._build_polygon_msg()
        goal.frame_id = self._frame_id or self._default_frame_id
        goal.polygons.append(polygon)
        self._clear_preview_locked()

        send_future = self._execute_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

        self._goal_active = True
        self._publish_visualization_locked()
        self.get_logger().info(
            f"Sent coverage goal to {self._execute_action_name}: "
            f"points={len(self._points)}, frame={goal.frame_id}"
        )
        return True, "Goal sent."

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pylint: disable=broad-except
            with self._state_lock:
                self._goal_active = False
                self._publish_visualization_locked()
            self.get_logger().error(f"Failed to send coverage goal: {exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            with self._state_lock:
                self._goal_active = False
                self._publish_visualization_locked()
            self.get_logger().error("Coverage goal rejected by server.")
            return

        self.get_logger().info("Coverage goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:  # pylint: disable=broad-except
            with self._state_lock:
                self._goal_active = False
                self._publish_visualization_locked()
            self.get_logger().error(f"Coverage result retrieval failed: {exc}")
            return

        with self._state_lock:
            self._goal_active = False
            if (
                self._plan_only and wrapped is not None
                and wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and wrapped.result.error_code == 0
            ):
                self._preview_path_pub.publish(wrapped.result.nav_path)
            self._publish_visualization_locked()

        if wrapped is None:
            self.get_logger().error("Coverage result is empty.")
            return

        status = int(wrapped.status)
        error_code = int(wrapped.result.error_code)
        if status == GoalStatus.STATUS_SUCCEEDED and error_code == 0:
            if self._plan_only:
                self.get_logger().info(
                    f"Coverage preview ready: {len(wrapped.result.nav_path.poses)} path poses."
                )
            else:
                self.get_logger().info("Coverage finished successfully.")
        else:
            self.get_logger().warn(
                f"Coverage finished with status={status}, error_code={error_code}"
            )

    @staticmethod
    def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def destroy_node(self) -> bool:
        self._execute_client.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageRegionSelector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
