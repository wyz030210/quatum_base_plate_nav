#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

STATUS_TEXT = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}

@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw_deg: float
    action: str = "none"  # Default action is "none"

class PatrolMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("patrol_mission_node")

        self.declare_parameter("mission_file", "")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        #self.declare_parameter("photo_service_name", "/ascamera_hp60c/capture")  改
        self.declare_parameter("detection_service_name", "/detect_facility")

        self.mission_file = self.get_parameter("mission_file").value
        self.navigate_action_name = self.get_parameter("navigate_action_name").value
        #self.photo_service_name = self.get_parameter("photo_service_name").value    改
        self.detection_service_name = self.get_parameter("detection_service_name").value

        if not self.mission_file:
            raise RuntimeError("Parameter 'mission_file' is empty")

        self._last_feedback_log_time = 0.0
        self._load_mission(Path(self.mission_file))

        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action_name)
        
        # Create photo trigger service client
        #from std_srvs.srv import Trigger
        #self.photo_client = self.create_client(Trigger, self.photo_service_name)
        #self.get_logger().info(f"Photo service client created: {self.photo_service_name}") 改

        # Create detection trigger service client
        from std_srvs.srv import Trigger
        self.detection_client = self.create_client(Trigger, self.detection_service_name)
        self.get_logger().info(f"Detection service client created: {self.detection_service_name}")

    def _status_text(self, status_code: int) -> str:
        return STATUS_TEXT.get(status_code, f"CODE_{status_code}")
    
    def _load_mission(self, mission_path: Path) -> None:
        if not mission_path.exists():
            raise FileNotFoundError(f"Mission file not found: {mission_path}")

        with mission_path.open("r", encoding="utf-8") as f:
            root = yaml.safe_load(f) or {}

        patrol = root.get("patrol", {})
        self.frame_id = str(patrol.get("frame_id", "map"))

        loop_cfg = patrol.get("loop", {})
        self.loop_enabled = bool(loop_cfg.get("enabled", False))
        self.max_loops = int(loop_cfg.get("max_loops", 1))
        if self.max_loops < 0:
            raise ValueError("max_loops must be >= 0, use 0 for infinite")

        timing_cfg = patrol.get("timing", {})
        self.goal_timeout_sec = max(1.0, float(timing_cfg.get("goal_timeout_sec", 120.0)))
        self.waypoint_pause_sec = max(0.0, float(timing_cfg.get("waypoint_pause_sec", 1.0)))

        failure_cfg = patrol.get("failure", {})
        self.retry_per_waypoint = max(0, int(failure_cfg.get("retry_per_waypoint", 1)))
        self.failure_policy = str(failure_cfg.get("policy", "skip_waypoint"))
        self.max_total_failures = max(0, int(failure_cfg.get("max_total_failures", 5)))

        if self.failure_policy not in ("skip_waypoint", "abort_mission"):
            raise ValueError("failure.policy must be 'skip_waypoint' or 'abort_mission'")
        
        points = patrol.get("waypoints", [])
        if not points:
            raise ValueError("No waypoints found in mission file")

        self.waypoints: List[Waypoint] = []
        for i, p in enumerate(points):
            name = str(p.get("name", f"wp_{i}"))
            x = float(p["x"])
            y = float(p["y"])
            yaw_deg = float(p.get("yaw_deg", 0.0))
            action = str(p.get("action", "none"))
            self.waypoints.append(Waypoint(name=name, x=x, y=y, yaw_deg=yaw_deg, action=action))

        self.get_logger().info(f"Loaded mission: {mission_path}")
        self.get_logger().info(f"waypoints={len(self.waypoints)}, loop_enabled={self.loop_enabled}, max_loops={self.max_loops}")

    def _to_pose_stamped(self, wp: Waypoint) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = wp.x
        msg.pose.position.y = wp.y
        msg.pose.position.z = 0.0

        yaw_rad = math.radians(wp.yaw_deg)
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        return msg

    # def _trigger_photo(self, waypoint_name: str) -> bool:
    #     """Trigger camera to take a photo at the waypoint"""
    #     if not self.photo_client.service_is_ready():
    #         self.get_logger().warn(f"Photo service not ready at {waypoint_name}, skipping")
    #         return False
        
    #     self.get_logger().info(f"📸 Triggering photo at {waypoint_name}")
        
    #     from std_srvs.srv import Trigger
    #     request = Trigger.Request()
    #     future = self.photo_client.call_async(request)
        
    #     # Wait for response with timeout
    #     start_time = self.get_clock().now()
    #     while rclpy.ok():
    #         rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
            
    #         if future.done():
    #             response = future.result()
    #             if response.success:
    #                 self.get_logger().info(f"✅ Photo captured successfully: {response.message}")
    #                 return True
    #             else:
    #                 self.get_logger().error(f"❌ Photo capture failed: {response.message}")
    #                 return False
            
    #         # Check timeout
    #         elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
    #         if elapsed > self.action_timeout_sec:
    #             self.get_logger().error(f"Photo service timeout at {waypoint_name}")
    #             return False
        
    #     return False  改

    def _trigger_detection(self, waypoint_name: str) -> bool:
        """Trigger facility image detection algorithm at the waypoint"""
        if not self.detection_client.service_is_ready():
            self.get_logger().warn(f"Detection service not ready at {waypoint_name}, skipping")
            return False
        
        self.get_logger().info(f"🔍 Triggering detection at {waypoint_name}")
        
        from std_srvs.srv import Trigger
        request = Trigger.Request()
        future = self.detection_client.call_async(request)
        
        # Wait for response with timeout
        start_time = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f"✅ Detection completed: {response.message}")
                    return True
                else:
                    self.get_logger().error(f"❌ Detection failed: {response.message}")
                    return False
            
            # Check timeout
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.action_timeout_sec:
                self.get_logger().error(f"Detection service timeout at {waypoint_name}")
                return False
        
        return False
    
    def _feedback_cb(self, feedback_msg) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self._last_feedback_log_time < 2.0:
            return
        self._last_feedback_log_time = now_sec

        fb = feedback_msg.feedback
        if hasattr(fb, "distance_remaining"):
            self.get_logger().info(f"distance_remaining={fb.distance_remaining:.2f} m")
        else:
            self.get_logger().info("navigation feedback received")

    def wait_for_nav2(self, timeout_sec: float = 60.0) -> bool:
        self.get_logger().info(f"Waiting action server: {self.navigate_action_name}")
        ok = self.nav_client.wait_for_server(timeout_sec=timeout_sec)
        if not ok:
            self.get_logger().error(f"Action server not available: {self.navigate_action_name}")
            return False
        self.get_logger().info("Nav2 action server is ready")
        return True
    
    def _navigate_once(self, wp: Waypoint, attempt: int) -> Tuple[bool, str]:
        goal = NavigateToPose.Goal()
        goal.pose = self._to_pose_stamped(wp)

        self.get_logger().info(f"Send goal [{wp.name}] attempt={attempt}, x={wp.x:.2f}, y={wp.y:.2f}, yaw={wp.yaw_deg:.1f}")

        send_future = self.nav_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if goal_handle is None:
            return False, "SEND_FAILED"
        if not goal_handle.accepted:
            return False, "GOAL_REJECTED"

        result_future = goal_handle.get_result_async()
        start_time = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.2)

            if result_future.done():
                result_wrap = result_future.result()
                status = int(result_wrap.status)
                status_text = self._status_text(status)
                return (status == GoalStatus.STATUS_SUCCEEDED), status_text

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > self.goal_timeout_sec:
                self.get_logger().warn(f"Goal timeout ({self.goal_timeout_sec}s), canceling [{wp.name}]")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                return False, "TIMEOUT"

        return False, "RCLPY_SHUTDOWN"
    
    def run(self) -> None:
        if not self.wait_for_nav2():
            return

        loop_idx = 0
        total_failures = 0

        while rclpy.ok():
            loop_idx += 1

            if not self.loop_enabled and loop_idx > 1:
                break
            if self.loop_enabled and self.max_loops > 0 and loop_idx > self.max_loops:
                break

            self.get_logger().info(f"===== Patrol Loop {loop_idx} Start =====")

            for idx, wp in enumerate(self.waypoints, start=1):
                success = False
                final_reason = "UNKNOWN"

                for attempt in range(1, self.retry_per_waypoint + 2):
                    success, reason = self._navigate_once(wp, attempt)
                    final_reason = reason
                    if success:
                        self.get_logger().info(f"[{idx}/{len(self.waypoints)}] {wp.name} reached")
                        break
                    self.get_logger().warn(f"[{idx}/{len(self.waypoints)}] {wp.name} failed: {reason}")

                if success:
                    if self.waypoint_pause_sec > 0.0:
                        time.sleep(self.waypoint_pause_sec)

                    # Execute action at waypoint (e.g., take photo)
                    # if wp.action == "take_photo":
                    #     self._trigger_photo(wp.name)  改

                    # Execute action at waypoint (e.g., facility detection)
                    if wp.action == "detect_facility":
                        self._trigger_detection(wp.name)
                    
                    # Pause at waypoint
                    if self.waypoint_pause_sec > 0.0:
                        self.get_logger().info(f"⏱️ Pausing for {self.waypoint_pause_sec}s at {wp.name}")
                        time.sleep(self.waypoint_pause_sec)

                    continue

                # Handle failure
                total_failures += 1
                self.get_logger().error(f"{wp.name} failed after retries, reason={final_reason}, total_failures={total_failures}")

                if self.failure_policy == "abort_mission":
                    self.get_logger().error("Mission aborted by failure policy")
                    return

                if self.failure_policy == "skip_waypoint":
                    if total_failures > self.max_total_failures:
                        self.get_logger().error("Mission stopped: max_total_failures exceeded")
                        return
                    self.get_logger().warn("Skip current waypoint and continue")
                    continue

            self.get_logger().info(f"===== Patrol Loop {loop_idx} Done =====")

        self.get_logger().info("Mission finished")

def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PatrolMissionNode()
        node.run()
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Mission node exception: {e}")
        else:
            print(f"[patrol_mission_node] exception: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

        

        
        
