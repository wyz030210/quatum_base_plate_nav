#!/usr/bin/env python3
"""Bridge coverage planning to FollowPath using only nav_path output."""

from __future__ import annotations

import math
import json
import threading
import time
from pathlib import Path
from typing import Any, Iterable

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from copy import deepcopy
from osgeo import ogr
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Path as NavPathMsg
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.srv import IsPathValid
from opennav_coverage_msgs.action import ComputeCoveragePath, NavigateCompleteCoverage
from opennav_coverage_msgs.msg import Coordinate, Coordinates
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from nav2_mission_interfaces.srv import PrepareCoverage
from .coverage_task import prepare_coverage, finite
from .coverage_bridge_state import BridgeRun, DeferredTerminal, child_ids, path_json
from .task_ros_adapters import decode_json
from tf2_ros import Buffer, TransformListener


def _yaw_from_quaternion(q) -> float:
    """从四元数取 yaw（绕 z）。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_difference(a: float, b: float) -> float:
    """a - b，归一化到 (-pi, pi]。"""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class CoverageNavPathBridge(Node):
    ERR_ACTION_SERVER_UNAVAILABLE = 901
    ERR_EMPTY_NAV_PATH = 902
    ERR_FOLLOW_PATH_FAILED = 903
    ERR_TIMEOUT = 904
    ERR_INVALID_TASK = 905
    ERR_DEPENDENCY_NOT_ACTIVE = 906
    ERR_CANCELED = 907
    ERR_COMPUTE_FAILED = 908
    ERR_SAFETY_ABORT = 909          # 被遥控抢占或软件急停中止
    ERR_APPROACH_FAILED = 910       # 导航到覆盖起点失败
    ERR_UNSAFE_NAV_PATH = 911

    WAIT_DONE = "done"
    WAIT_TIMEOUT = "timeout"
    WAIT_CANCELED = "canceled"
    WAIT_ABORTED = "aborted"        # 安全中止（非客户端取消）
    WAIT_SHUTDOWN = "shutdown"

    def __init__(self) -> None:
        super().__init__("coverage_navpath_bridge")

        default_task_file = (
            Path(get_package_share_directory("nav2_mission"))
            / "config"
            / "coverage_navpath_task.yaml"
        )

        self.declare_parameter("execute_action_name", "/execute_coverage_navpath")
        self.declare_parameter("compute_action_name", "/compute_coverage_path")
        self.declare_parameter("follow_path_action_name", "/follow_path")
        self.declare_parameter("task_file", str(default_task_file))
        self.declare_parameter("default_frame_id", "map")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "")
        self.declare_parameter("server_wait_timeout_sec", 10.0)
        # follow_timeout_sec 是【覆盖任务的总执行超时】（见本文件 :318-323，
        # 等待 FollowPath 返回结果；超时会主动 cancel 该 goal）。
        # 全场覆盖需 22-33 分钟，300.0 会在中途取消任务，故取 3600.0。
        # 注意：launch/coverage_navpath_bridge.launch.py 的同名参数会覆盖此处默认值，
        # 两处必须保持一致。
        self.declare_parameter("compute_timeout_sec", 120.0)
        self.declare_parameter("follow_timeout_sec", 3600.0)
        self.declare_parameter("dependency_timeout_sec", 5.0)
        self.declare_parameter("enable_lifecycle_checks", True)
        self.declare_parameter("trim_path_to_robot", False)
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("validate_path_costmap", True)
        self.declare_parameter("path_valid_service", "/is_path_valid")
        # 阿克曼车掉头需要地头空间（同 coverage_params.yaml 的 default_headland_width）
        self.declare_parameter("generate_headland", True)
        self.declare_parameter("generate_route", True)
        self.declare_parameter("generate_path", True)
        self.declare_parameter(
            "required_lifecycle_nodes",
            ["coverage_server", "controller_server"],
        )

        # ── 安全相关（方案 §4.4）────────────────────────────────
        # 遥控器手动接管信号。由 can_bridge_node 从 CAN ID 0x07 的 data[1] 发布，
        # 约 1 Hz；语义：true = 遥控器已手动接管，ROS 指令此刻被固件忽略。
        self.declare_parameter("manual_override_topic", "/vehicle/remote_manual_override")
        # 软件急停入口：向该话题发 std_msgs/Bool 且 data=true 即触发。
        self.declare_parameter("emergency_stop_topic", "/coverage/emergency_stop")
        # 急停/抢占时下发零速的话题（底盘 vehicle_can_bridge 订阅的就是它）
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # 零速重复发送次数（底盘命令侧没有看门狗，多发几帧抗丢帧）
        self.declare_parameter("estop_zero_velocity_repeats", 5)

        # ── 导航到覆盖起点（approach 阶段）──────────────────────
        # 为什么需要：BOUSTROPHEDON 路由下 F2C 生成的路径【不从车所在位置开始】，
        # 起点在所选多边形的边缘。而 MPPI 的 local costmap 只有 6x6 m（±3 m），
        # 起点若在窗口外，controller_server 会报
        #   "Resulting plan has 0 poses in it"
        # 并 abort（FollowPath status=6）。仿真侧的 coverage_task_node 也是靠
        # 这一步用 NavigateToPose 先把车开到起点解决的。
        self.declare_parameter("navigate_to_start", True)
        self.declare_parameter("navigate_to_pose_action_name", "/navigate_to_pose")
        self.declare_parameter("navigate_to_start_timeout_sec", 120.0)
        # 已经够近就直接开始，不必再导航（位置 / 航向容差）
        self.declare_parameter("start_position_tolerance", 0.5)
        self.declare_parameter("start_yaw_tolerance", 0.5)

        self._execute_action_name = str(self.get_parameter("execute_action_name").value)
        self._compute_action_name = str(self.get_parameter("compute_action_name").value)
        self._follow_path_action_name = str(
            self.get_parameter("follow_path_action_name").value
        )
        self._task_file = str(self.get_parameter("task_file").value)
        self._default_frame_id = str(self.get_parameter("default_frame_id").value)
        self._controller_id = str(self.get_parameter("controller_id").value)
        self._goal_checker_id = str(self.get_parameter("goal_checker_id").value)
        self._server_wait_timeout_sec = float(
            self.get_parameter("server_wait_timeout_sec").value
        )
        self._compute_timeout_sec = float(self.get_parameter("compute_timeout_sec").value)
        self._follow_timeout_sec = float(self.get_parameter("follow_timeout_sec").value)
        self._dependency_timeout_sec = float(
            self.get_parameter("dependency_timeout_sec").value
        )
        self._enable_lifecycle_checks = bool(
            self.get_parameter("enable_lifecycle_checks").value
        )
        self._trim_path_to_robot = bool(self.get_parameter("trim_path_to_robot").value)
        self._base_frame_id = str(self.get_parameter("base_frame_id").value)
        self._generate_headland = bool(self.get_parameter("generate_headland").value)
        self._generate_route = bool(self.get_parameter("generate_route").value)
        self._generate_path = bool(self.get_parameter("generate_path").value)
        self._required_lifecycle_nodes = [
            str(name).strip()
            for name in self.get_parameter("required_lifecycle_nodes").value
            if str(name).strip()
        ]
        self._manual_override_topic = str(
            self.get_parameter("manual_override_topic").value
        )
        self._emergency_stop_topic = str(
            self.get_parameter("emergency_stop_topic").value
        )
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._estop_zero_velocity_repeats = max(
            1, int(self.get_parameter("estop_zero_velocity_repeats").value)
        )
        self._navigate_to_start = bool(self.get_parameter("navigate_to_start").value)
        self._navigate_to_pose_action_name = str(
            self.get_parameter("navigate_to_pose_action_name").value
        )
        self._navigate_to_start_timeout_sec = float(
            self.get_parameter("navigate_to_start_timeout_sec").value
        )
        self._start_position_tolerance = float(
            self.get_parameter("start_position_tolerance").value
        )
        self._start_yaw_tolerance = float(
            self.get_parameter("start_yaw_tolerance").value
        )

        self._callback_group = ReentrantCallbackGroup()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._coverage_client = ActionClient(
            self,
            ComputeCoveragePath,
            self._compute_action_name,
            callback_group=self._callback_group,
        )
        self._follow_path_client = ActionClient(
            self,
            FollowPath,
            self._follow_path_action_name,
            callback_group=self._callback_group,
        )
        self._navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            self._navigate_to_pose_action_name,
            callback_group=self._callback_group,
        )
        self._path_valid_client = self.create_client(
            IsPathValid, str(self.get_parameter("path_valid_service").value),
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            NavigateCompleteCoverage,
            self._execute_action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self._active_goal_lock = threading.Lock()
        self._active_goal = False

        self._prepared = None
        self._run = None
        self._state_pub = self.create_publisher(String, '/coverage/execution_state', 10)
        self._prepare_service = self.create_service(
            PrepareCoverage, '/coverage/prepare', self._prepare_coverage,
            callback_group=self._callback_group)
        self.declare_parameter('planner_node_name', '/coverage_server')
        planner = self.get_parameter('planner_node_name').value.rstrip('/')
        self._planner_set = self.create_client(
            SetParametersAtomically, planner + '/set_parameters_atomically',
            callback_group=self._callback_group)
        self._planner_get = self.create_client(
            GetParameters, planner + '/get_parameters', callback_group=self._callback_group)
        self.add_on_set_parameters_callback(self._guard_parameters)

        # ── 安全层（方案 §4.4）──────────────────────────────────
        # 背景：本工作区【没有】任何 ROS 侧急停/看门狗 —— cmd_vel_to_can_node
        # 全文零个定时器，停发指令不会让车停下（连上一帧都不重传）。唯一的兜底是
        # STM32 的 500 ms 超时→退回 RC。因此中止覆盖时必须【显式】发零速。
        self._safety_abort_lock = threading.Lock()
        self._safety_abort = False          # 置位后 _wait_future 会中止任务
        self._safety_abort_reason = ""
        self._manual_override_active = False

        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(
            Bool,
            self._manual_override_topic,
            self._manual_override_callback,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            self._emergency_stop_topic,
            self._emergency_stop_callback,
            10,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "coverage_navpath_bridge ready: "
            f"execute={self._execute_action_name}, "
            f"compute={self._compute_action_name}, "
            f"follow={self._follow_path_action_name}"
        )
        self.get_logger().info(
            "safety: manual_override=%s, emergency_stop=%s, zero_vel_on=%s"
            % (self._manual_override_topic, self._emergency_stop_topic, self._cmd_vel_topic)
        )
        self.get_logger().info(
            "approach: %s (action=%s, timeout=%.0fs, tol=%.2fm/%.2frad)"
            % (
                "enabled" if self._navigate_to_start else "disabled",
                self._navigate_to_pose_action_name,
                self._navigate_to_start_timeout_sec,
                self._start_position_tolerance,
                self._start_yaw_tolerance,
            )
        )

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        self._coverage_client.destroy()
        self._follow_path_client.destroy()
        self._navigate_to_pose_client.destroy()
        return super().destroy_node()

    def _guard_parameters(self, parameters):
        with self._active_goal_lock:
            busy = self._active_goal or self._prepared is not None
        return SetParametersResult(
            successful=not busy,
            reason='coverage settings frozen while reserved or active' if busy else '')

    def _prepare_coverage(self, request, response):
        with self._active_goal_lock:
            if request.discard:
                if self._active_goal:
                    response.message = 'outer goal ownership pending'
                elif self._prepared is None or self._prepared['goal_id'] == request.goal_id:
                    self._prepared = None
                    response.accepted = True
                return response
            if self._active_goal or self._prepared is not None:
                response.message = 'coverage bridge reserved or active'
                return response
            try:
                ids = child_ids(request.goal_id)
                config = prepare_coverage(decode_json(request.config_json))
                if not request.task_id:
                    raise ValueError('task id is required')
                self._prepared = dict(
                    goal_id=request.goal_id, task_id=request.task_id, config=config)
                response.accepted = True
                response.state_json = json.dumps(dict(
                    goal_id=request.goal_id, task_id=request.task_id, phase='preparing',
                    owned_goal_ids=ids, config=config), allow_nan=False)
            except (ValueError, TypeError, KeyError) as exc:
                response.message = str(exc)
        return response

    def _send_owned(self, client, goal, outer, stage):
        if getattr(self, '_run', None) is None:
            return client.send_goal_async(goal)
        if outer.is_cancel_requested or self._safety_abort_active():
            raise RuntimeError('coverage stopped before child dispatch')
        return self._run.send(client, goal, stage)

    def _owned_result(self, handle):
        if getattr(self, '_run', None) is not None:
            return self._run.result_for(handle)
        return handle.get_result_async()

    def _emit(self, phase, **values):
        if getattr(self, '_run', None) is not None:
            self._run.emit(phase, **values)

    def _goal_callback(self, _goal_request: NavigateCompleteCoverage.Goal) -> GoalResponse:
        with self._active_goal_lock:
            if self._active_goal:
                self.get_logger().warn("Rejecting goal: another coverage task is running.")
                return GoalResponse.REJECT
            if self._manual_override_active:
                # 遥控器已被人工接管时不允许启动自主覆盖 —— 否则一拨回自动挡
                # STM32 重新接受 ROS 指令，车会在人手持遥控器的状态下自己跑起来。
                self.get_logger().warn(
                    "Rejecting goal: remote manual override is active "
                    f"({self._manual_override_topic} is true). Release the RC takeover first."
                )
                return GoalResponse.REJECT
            self._active_goal = True
        # 新任务开始，清掉上一次的安全中止标志
        with self._safety_abort_lock:
            self._safety_abort = False
            self._safety_abort_reason = ""
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Received cancel request for coverage task.")
        return CancelResponse.ACCEPT

    # ── 安全层（方案 §4.4）──────────────────────────────────────

    def _manual_override_callback(self, msg: Bool) -> None:
        """遥控器手动接管 → 立即中止覆盖任务。

        该信号由 can_bridge_node 从 CAN ID 0x07 的 data[1] 解析后发布（约 1 Hz）。
        语义：true = 遥控器已手动接管，ROS 指令此刻被固件忽略（硬件层面已生效）。

        ⚠️ 为什么必须订阅它：硬件层面遥控器始终优先，所以不订阅也【不会撞车】；
        但覆盖任务会仍然停留在"执行中"，而一旦操作员把遥控器拨回自动挡，
        STM32 会重新接受 ROS 指令（见 docs/DEPLOYMENT.md:273），
        覆盖任务就会从原地继续跑 —— 表现为"我以为它停了，它又动了"。
        """
        override = bool(msg.data)
        with self._active_goal_lock:
            was_active = self._manual_override_active
            self._manual_override_active = override
        if override and not was_active:
            self._request_safety_abort(
                f"remote manual override ({self._manual_override_topic}=true)"
            )

    def _emergency_stop_callback(self, msg: Bool) -> None:
        """软件急停入口。向 emergency_stop_topic 发 Bool(true) 即触发。

        全工作区原本没有任何 ROS 侧急停话题/服务/看门狗，唯一的软件停车是
        scripts/keyboard_control.py 的 Ctrl-C —— 而覆盖任务运行时那个脚本并不在跑。
        """
        if bool(msg.data):
            self._request_safety_abort(
                f"software emergency stop ({self._emergency_stop_topic})"
            )

    def _request_safety_abort(self, reason: str) -> None:
        """置位安全中止标志并立即下发零速。

        置位后 _wait_future() 会返回 WAIT_ABORTED，进而取消内层 FollowPath、
        把外层 goal 置为 abort —— 覆盖任务随即终止。
        """
        with self._safety_abort_lock:
            if self._safety_abort:
                return          # 已触发过，避免重复刷日志
            self._safety_abort = True
            self._safety_abort_reason = reason
        with self._active_goal_lock:
            active = self._active_goal

        self.get_logger().error(
            f"SAFETY ABORT: {reason}. "
            + ("Aborting the running coverage task and " if active else "")
            + "commanding zero velocity."
        )
        # 先发零速再管任务状态：底盘命令侧没有看门狗，不发这一帧车就不会停。
        self._publish_zero_velocity()

    def _publish_zero_velocity(self) -> None:
        """向 cmd_vel 下发若干帧零速。

        为什么不能只发一帧：底盘侧没有任何"最后指令保持"逻辑，也没有看门狗，
        这个节点的零速是唯一的停车指令来源；而 nav2 的 velocity_smoother
        同时也在向 /cmd_vel 发布，多帧可抗竞争与丢帧。
        """
        msg = Twist()       # 全零：linear.x/y/z = 0, angular.x/y/z = 0
        for _ in range(self._estop_zero_velocity_repeats):
            self._cmd_vel_pub.publish(msg)
        self.get_logger().warn(
            f"Published {self._estop_zero_velocity_repeats} zero-velocity Twist "
            f"frame(s) on {self._cmd_vel_topic}."
        )

    def _execute_callback(self, goal_handle):
        goal_id = bytes(goal_handle.goal_id.uuid).hex()
        with self._active_goal_lock:
            prepared = self._prepared
            if prepared is not None and prepared['goal_id'] != goal_id:
                goal_handle.abort()
                self._active_goal = False
                return NavigateCompleteCoverage.Result(error_code=self.ERR_INVALID_TASK)
        run = BridgeRun(self, goal_handle, prepared['config'] if prepared else None,
                        prepared['task_id'] if prepared else '')
        self._run = run
        proxy = DeferredTerminal(goal_handle)
        result = NavigateCompleteCoverage.Result(error_code=self.ERR_INVALID_TASK)
        try:
            run.acquire()
            result = self._execute_path(proxy)
        except Exception as exc:
            self.get_logger().error(f'Coverage execution failed: {exc}')
            run.state['error'] = str(exc)
            if goal_handle.is_cancel_requested:
                result.error_code = self.ERR_CANCELED
                proxy.canceled()
            elif self._safety_abort_active():
                result.error_code = self.ERR_SAFETY_ABORT
                proxy.abort()
            else:
                proxy.abort()
        finally:
            # Stop the common command gate before waiting on any delayed inner
            # cancellation/result or parameter release. Ownership still stays here.
            failed = proxy.terminal != 'succeed'
            run.emit('stopping', error_code=int(result.error_code),
                     execution_failed=failed)
            run.drain()
            run.release()
            self._publish_zero_velocity()
            run.emit('stopping', error_code=int(result.error_code))
            terminal = proxy.terminal
            if terminal == 'canceled' and not goal_handle.is_cancel_requested:
                terminal = 'abort'
            getattr(goal_handle, terminal)()
            with self._active_goal_lock:
                self._active_goal = False
                if self._prepared is prepared:
                    self._prepared = None
            self._run = None
        return result

    def _execute_path(self, goal_handle) -> NavigateCompleteCoverage.Result:
        result = NavigateCompleteCoverage.Result()
        result.error_code = NavigateCompleteCoverage.Result.NONE

        coverage_goal_handle = None
        follow_goal_handle = None

        try:
            if self._enable_lifecycle_checks and not self._check_required_nodes_active():
                result.error_code = self.ERR_DEPENDENCY_NOT_ACTIVE
                goal_handle.abort()
                return result

            if not self._wait_for_action_server(self._coverage_client):
                if self._abort_result_if_safety(goal_handle, result):
                    return result
                self.get_logger().error(
                    f"Action server unavailable: {self._compute_action_name}"
                )
                result.error_code = self.ERR_ACTION_SERVER_UNAVAILABLE
                goal_handle.abort()
                return result

            if not self._wait_for_action_server(self._follow_path_client):
                if self._abort_result_if_safety(goal_handle, result):
                    return result
                self.get_logger().error(
                    f"Action server unavailable: {self._follow_path_action_name}"
                )
                result.error_code = self.ERR_ACTION_SERVER_UNAVAILABLE
                goal_handle.abort()
                return result

            if goal_handle.is_cancel_requested:
                result.error_code = self.ERR_CANCELED
                goal_handle.canceled()
                return result

            try:
                frame_id, polygons = (
                    (self._run.config['frame_id'], self._run.config['polygons'])
                    if self._run.config else self._resolve_input(goal_handle.request))
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(f"Invalid coverage task: {exc}")
                result.error_code = self.ERR_INVALID_TASK
                goal_handle.abort()
                return result

            compute_goal = self._build_compute_goal(frame_id, polygons)
            self.get_logger().info(
                f"Sending ComputeCoveragePath goal: polygons={len(polygons)}, frame_id={frame_id}"
            )

            self._emit('planning')
            send_compute_future = self._send_owned(
                self._coverage_client, compute_goal, goal_handle, 'compute')
            wait_status = self._wait_future(
                send_compute_future,
                self._server_wait_timeout_sec,
                goal_handle,
            )
            if wait_status != self.WAIT_DONE:
                return self._finish_wait_failure(
                    wait_status,
                    goal_handle,
                    result,
                    "Timeout waiting for ComputeCoveragePath goal response.",
                )

            coverage_goal_handle = send_compute_future.result()
            if coverage_goal_handle is None or not coverage_goal_handle.accepted:
                self.get_logger().error("ComputeCoveragePath goal was rejected.")
                result.error_code = self.ERR_COMPUTE_FAILED
                goal_handle.abort()
                return result

            compute_result_future = self._owned_result(coverage_goal_handle)
            wait_status = self._wait_future(
                compute_result_future,
                self._compute_timeout_sec,
                goal_handle,
                cancel_fn=lambda: self._cancel_goal(coverage_goal_handle),
            )
            if wait_status != self.WAIT_DONE:
                return self._finish_wait_failure(
                    wait_status,
                    goal_handle,
                    result,
                    "Timeout waiting for ComputeCoveragePath result.",
                )

            compute_result_wrap = compute_result_future.result()
            if compute_result_wrap is None:
                self.get_logger().error("ComputeCoveragePath returned no result.")
                result.error_code = self.ERR_COMPUTE_FAILED
                goal_handle.abort()
                return result

            compute_status = int(compute_result_wrap.status)
            compute_result = compute_result_wrap.result
            if compute_status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f"ComputeCoveragePath failed with status={compute_status}"
                )
                result.error_code = self._coverage_error_code(compute_result)
                self._log_compute_failure_hint(result.error_code)
                goal_handle.abort()
                return result

            if int(compute_result.error_code) != ComputeCoveragePath.Result.NONE:
                self.get_logger().error(
                    f"ComputeCoveragePath error_code={int(compute_result.error_code)}"
                )
                result.error_code = int(compute_result.error_code)
                self._log_compute_failure_hint(result.error_code)
                goal_handle.abort()
                return result

            if len(compute_result.nav_path.poses) == 0:
                self.get_logger().error("ComputeCoveragePath produced empty nav_path.")
                result.error_code = self.ERR_EMPTY_NAV_PATH
                goal_handle.abort()
                return result

            invalid = self._path_error(compute_result.nav_path, frame_id)
            if invalid:
                result.error_code = self.ERR_UNSAFE_NAV_PATH
                self._emit('validating', error=invalid, error_code=int(result.error_code),
                           execution_failed=True)
                goal_handle.abort()
                return result
            self._emit('validating', planned_path=path_json(compute_result.nav_path))

            # 先把车导航到覆盖路径的起点（否则 MPPI 因路径落在 local costmap
            # 窗口外而报 "Resulting plan has 0 poses" 并 abort）
            if not self._validate_nav_path(compute_result.nav_path, goal_handle, result):
                return result
            if not self._approach_start(compute_result.nav_path, goal_handle, result):
                return result
            if not self._validate_nav_path(compute_result.nav_path, goal_handle, result):
                return result

            if self._abort_result_if_safety(goal_handle, result):
                return result
            if goal_handle.is_cancel_requested:
                result.error_code = self.ERR_CANCELED
                goal_handle.canceled()
                return result
            follow_goal = FollowPath.Goal()
            follow_path = compute_result.nav_path
            if self._trim_path_to_robot:
                follow_path = self._trim_path_for_follow(follow_path)

            if len(follow_path.poses) == 0:
                self.get_logger().error("FollowPath path is empty after trimming.")
                result.error_code = self.ERR_EMPTY_NAV_PATH
                goal_handle.abort()
                return result

            follow_goal.path = follow_path
            follow_goal.controller_id = self._controller_id
            follow_goal.goal_checker_id = self._goal_checker_id

            self.get_logger().info(
                f"Sending FollowPath goal with poses={len(follow_goal.path.poses)}"
            )

            self._emit('following')
            send_follow_future = self._send_owned(
                self._follow_path_client, follow_goal, goal_handle, 'follow')
            wait_status = self._wait_future(
                send_follow_future,
                self._server_wait_timeout_sec,
                goal_handle,
            )
            if wait_status != self.WAIT_DONE:
                return self._finish_wait_failure(
                    wait_status,
                    goal_handle,
                    result,
                    "Timeout waiting for FollowPath goal response.",
                )

            follow_goal_handle = send_follow_future.result()
            if follow_goal_handle is None or not follow_goal_handle.accepted:
                self.get_logger().error("FollowPath goal was rejected.")
                result.error_code = self.ERR_FOLLOW_PATH_FAILED
                goal_handle.abort()
                return result

            follow_result_future = self._owned_result(follow_goal_handle)
            wait_status = self._wait_future(
                follow_result_future,
                self._follow_timeout_sec,
                goal_handle,
                cancel_fn=lambda: self._cancel_goal(follow_goal_handle),
            )
            if wait_status != self.WAIT_DONE:
                return self._finish_wait_failure(
                    wait_status,
                    goal_handle,
                    result,
                    "Timeout waiting for FollowPath result.",
                )

            follow_result_wrap = follow_result_future.result()
            if follow_result_wrap is None:
                self.get_logger().error("FollowPath returned no result.")
                result.error_code = self.ERR_FOLLOW_PATH_FAILED
                goal_handle.abort()
                return result

            follow_status = int(follow_result_wrap.status)
            if follow_status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Coverage nav_path execution succeeded.")
                result.error_code = NavigateCompleteCoverage.Result.NONE
                goal_handle.succeed()
                return result

            if follow_status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn("FollowPath was canceled.")
                result.error_code = self.ERR_CANCELED
                goal_handle.canceled()
                return result

            self.get_logger().error(f"FollowPath failed with status={follow_status}")
            result.error_code = self.ERR_FOLLOW_PATH_FAILED
            goal_handle.abort()
            return result
        finally:
            pass  # Outer execution retains ownership and drains children before returning.

    def _finish_wait_failure(
        self,
        wait_status: str,
        goal_handle,
        result: NavigateCompleteCoverage.Result,
        timeout_msg: str,
    ) -> NavigateCompleteCoverage.Result:
        if wait_status == self.WAIT_ABORTED:
            self._abort_result_if_safety(goal_handle, result)
            return result
        if wait_status == self.WAIT_CANCELED:
            result.error_code = self.ERR_CANCELED
            goal_handle.canceled()
            return result
        if wait_status == self.WAIT_TIMEOUT:
            self.get_logger().error(timeout_msg)
            result.error_code = self.ERR_TIMEOUT
            goal_handle.abort()
            return result

        result.error_code = self.ERR_TIMEOUT
        goal_handle.abort()
        return result

    def _log_compute_failure_hint(self, error_code: int) -> None:
        """给 coverage_server 的失败补一条可操作的诊断。

        最常见的 801 (INTERNAL_F2C_ERROR) 是"所选多边形相对地头宽度太小"：
        coverage_server 会从多边形各边向内缩 default_headland_width，若多边形在
        某个方向上比 2×headland 还窄，内缩后几何为空，F2C 报
            "Geometry does not contain point 0"
        这个错误码本身看不出原因，所以在这里把话说明白。
        """
        if error_code == 801:
            self.get_logger().error(
                "→ 这是 F2C 几何错误，最常见原因是【所选多边形相对地头宽度太小】。\n"
                "   coverage_server 会从多边形各边向内缩 default_headland_width"
                "（见 coverage_params.yaml，本车为 1.30 m）。\n"
                "   若多边形某个方向的宽度 < 2 × 1.30 = 2.60 m，几何会被缩成空，\n"
                "   F2C 原始报错为 'Geometry does not contain point 0'。\n"
                "   要求：多边形短边 >= 2×headland + operation_width"
                "（本车约 3.2 m），\n"
                "   【第一次试跑建议用 >= 8×8 m 的区域】。请重新点选一个更大的区域。"
            )
        elif error_code == 803:
            self.get_logger().error(
                "→ 坐标无效（INVALID_COORDS / INVALID_REQUEST）。\n"
                "   检查：多边形是否自交、顶点是否 >= 3 个、是否都在同一坐标系下。"
            )

    def _coverage_error_code(self, compute_result) -> int:
        if compute_result is None:
            return self.ERR_COMPUTE_FAILED
        if int(compute_result.error_code) != 0:
            return int(compute_result.error_code)
        return self.ERR_COMPUTE_FAILED

    def _check_required_nodes_active(self) -> bool:
        for node_name in self._required_lifecycle_nodes:
            state = self._query_lifecycle_state(node_name)
            if state != "active":
                self.get_logger().error(
                    f"Required node '{node_name}' is not active (state={state})."
                )
                return False
        return True

    def _query_lifecycle_state(self, node_name: str) -> str | None:
        service_name = f"/{node_name.strip('/')}/get_state"
        client = self.create_client(
            GetState, service_name, callback_group=self._callback_group
        )
        if not client.wait_for_service(timeout_sec=self._dependency_timeout_sec):
            self.get_logger().error(f"Lifecycle service unavailable: {service_name}")
            return None

        future = client.call_async(GetState.Request())
        wait_status = self._wait_future(future, self._dependency_timeout_sec)
        if wait_status != self.WAIT_DONE:
            self.get_logger().error(
                f"Lifecycle state query timeout: {service_name} ({wait_status})"
            )
            return None

        response = future.result()
        if response is None:
            return None
        return response.current_state.label

    def _resolve_input(
        self, goal: NavigateCompleteCoverage.Goal
    ) -> tuple[str, list[list[tuple[float, float]]]]:
        if goal.polygons:
            polygons = []
            for index, polygon in enumerate(goal.polygons):
                points = [(float(point.x), float(point.y)) for point in polygon.points]
                polygons.append(
                    self._normalize_polygon(points, f"goal.polygon[{index}]")
                )
            frame_id = goal.frame_id.strip() if goal.frame_id else self._default_frame_id
            return frame_id, polygons

        task_path = Path(goal.field_filepath).expanduser() if goal.field_filepath else Path(
            self._task_file
        ).expanduser()
        frame_id_from_file, polygons = self._load_polygons_from_yaml(task_path)
        frame_id = (
            goal.frame_id.strip()
            if goal.frame_id
            else (frame_id_from_file or self._default_frame_id)
        )
        return frame_id, polygons

    def _load_polygons_from_yaml(
        self, task_path: Path
    ) -> tuple[str | None, list[list[tuple[float, float]]]]:
        if not task_path.exists():
            raise FileNotFoundError(f"Task file not found: {task_path}")

        with task_path.open("r", encoding="utf-8") as file:
            root = yaml.safe_load(file) or {}

        task = root.get("coverage_task", root)
        frame_id_raw = task.get("frame_id")
        frame_id = str(frame_id_raw).strip() if frame_id_raw is not None else None

        raw_polygons = task.get("polygons")
        if not isinstance(raw_polygons, list) or len(raw_polygons) == 0:
            raise ValueError("Task YAML must provide non-empty 'polygons'.")

        polygons = []
        for poly_idx, raw_polygon in enumerate(raw_polygons):
            raw_points = raw_polygon.get("points") if isinstance(raw_polygon, dict) else raw_polygon
            if not isinstance(raw_points, list):
                raise ValueError(
                    f"Polygon {poly_idx} is invalid: expected list of points."
                )

            points = [
                self._parse_point(point, poly_idx, point_idx)
                for point_idx, point in enumerate(raw_points)
            ]
            polygons.append(self._normalize_polygon(points, f"yaml.polygon[{poly_idx}]"))

        return frame_id, polygons

    def _parse_point(
        self, point: Any, poly_idx: int, point_idx: int
    ) -> tuple[float, float]:
        if isinstance(point, dict):
            if "x" in point and "y" in point:
                return float(point["x"]), float(point["y"])
            if "axis1" in point and "axis2" in point:
                return float(point["axis1"]), float(point["axis2"])
        elif isinstance(point, Iterable) and not isinstance(point, (str, bytes)):
            values = list(point)
            if len(values) >= 2:
                return float(values[0]), float(values[1])

        raise ValueError(
            f"Invalid point at polygon {poly_idx}, index {point_idx}: {point}"
        )

    def _normalize_polygon(
        self, points: list[tuple[float, float]], source_name: str
    ) -> list[tuple[float, float]]:
        cleaned = []
        for point in points:
            if not all(math.isfinite(value) for value in point):
                raise ValueError(f"{source_name} contains non-finite coordinates.")
            if not cleaned or cleaned[-1] != point:
                cleaned.append(point)

        if len(set(cleaned)) < 3:
            raise ValueError(f"{source_name} needs at least 3 distinct points.")

        if cleaned[0] != cleaned[-1]:
            cleaned.append(cleaned[0])
        ring = ogr.Geometry(ogr.wkbLinearRing)
        for x, y in cleaned:
            ring.AddPoint_2D(x, y)
        polygon = ogr.Geometry(ogr.wkbPolygon)
        polygon.AddGeometry(ring)
        if not polygon.IsValid() or polygon.GetArea() <= 1e-8:
            raise ValueError(f"{source_name} is self-intersecting or has zero area.")
        return cleaned

    def _build_compute_goal(
        self, frame_id: str, polygons: list[list[tuple[float, float]]]
    ) -> ComputeCoveragePath.Goal:
        goal = ComputeCoveragePath.Goal()
        goal.generate_headland = self._generate_headland
        goal.generate_route = self._generate_route
        goal.generate_path = self._generate_path
        goal.use_gml_file = False
        goal.frame_id = frame_id

        for polygon in polygons:
            coords = Coordinates()
            for x, y in polygon:
                point = Coordinate()
                point.axis1 = float(x)
                point.axis2 = float(y)
                coords.coordinates.append(point)
            goal.polygons.append(coords)
        return goal

    @staticmethod
    def _path_error(path, frame_id):
        if path.header.frame_id != frame_id:
            return 'planned path frame does not match frozen coverage frame'
        for pose in path.poses:
            if pose.header.frame_id and pose.header.frame_id != frame_id:
                return 'planned pose frame does not match frozen coverage frame'
            p, q = pose.pose.position, pose.pose.orientation
            if not all(finite(value) for value in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                return 'planned path contains nonfinite coordinates'
            norm = math.hypot(q.x, q.y, q.z, q.w)
            if not math.isclose(norm, 1.0, abs_tol=0.001):
                return 'planned path contains invalid quaternion'
        return ''

    def _validate_nav_path(self, nav_path: NavPathMsg, goal_handle, result) -> bool:
        if not self.get_parameter("validate_path_costmap").value:
            return True
        request = IsPathValid.Request()
        request.path = nav_path
        deadline = time.monotonic() + self._dependency_timeout_sec
        while not self._path_valid_client.service_is_ready():
            if self._abort_result_if_safety(goal_handle, result):
                return False
            if goal_handle.is_cancel_requested:
                result.error_code = self.ERR_CANCELED
                goal_handle.canceled()
                return False
            if time.monotonic() >= deadline:
                self.get_logger().error(
                    "Path validation service is unavailable; refusing execution.")
                result.error_code = self.ERR_UNSAFE_NAV_PATH
                goal_handle.abort()
                return False
            time.sleep(0.05)
        future = self._path_valid_client.call_async(request)
        status = self._wait_future(future, self._dependency_timeout_sec, goal_handle)
        if status == self.WAIT_ABORTED:
            self._abort_result_if_safety(goal_handle, result)
            return False
        if status == self.WAIT_CANCELED:
            result.error_code = self.ERR_CANCELED
            goal_handle.canceled()
            return False
        response = future.result() if status == self.WAIT_DONE else None
        if response is None or not response.is_valid:
            indices = list(response.invalid_pose_indices[:10]) if response is not None else []
            self.get_logger().error(
                f"Coverage path failed current costmap validation: invalid poses={indices}")
            result.error_code = self.ERR_UNSAFE_NAV_PATH
            goal_handle.abort()
            return False
        return True

    def _trim_path_for_follow(self, nav_path: NavPathMsg) -> NavPathMsg:
        # Spatial proximity is not coverage progress: turns revisit earlier locations.
        self.get_logger().warn(
            "trim_path_to_robot is deprecated; preserving the complete coverage path."
        )
        return nav_path

    def _robot_pose_in(self, frame_id: str):
        """取 base_frame 在指定坐标系下的 (x, y, yaw)；失败返回 None。"""
        try:
            transform = self._tf_buffer.lookup_transform(
                frame_id, self._base_frame_id, rclpy.time.Time()
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(
                f"TF lookup failed ({self._base_frame_id}->{frame_id}): {exc}"
            )
            return None
        t = transform.transform.translation
        return float(t.x), float(t.y), _yaw_from_quaternion(transform.transform.rotation)

    def _approach_start(self, nav_path: NavPathMsg, goal_handle, result) -> bool:
        """把车导航到覆盖路径的起点。

        返回 True 表示可以继续 FollowPath；False 表示已收尾（调用方直接 return result）。

        背景：BOUSTROPHEDON 路由下 F2C 生成的路径不从车所在位置开始，起点在所选
        多边形的边缘。MPPI 的 local costmap 只有 6x6 m（±3 m），起点若在窗口外，
        controller_server 会报 "Resulting plan has 0 poses in it" 并 abort。
        仿真侧 coverage_task_node 的 _prepare_approach 就是解决这个的。
        """
        if not self._navigate_to_start or len(nav_path.poses) == 0:
            return True

        target = deepcopy(nav_path.poses[0])
        frame_id = (
            nav_path.header.frame_id
            or target.header.frame_id
            or self._default_frame_id
        )
        target.header.frame_id = frame_id
        target.header.stamp = self.get_clock().now().to_msg()

        robot = self._robot_pose_in(frame_id)
        if robot is None:
            self.get_logger().error("Approach failed: robot pose unavailable via TF.")
            result.error_code = self.ERR_APPROACH_FAILED
            goal_handle.abort()
            return False

        rx, ry, ryaw = robot
        tx = float(target.pose.position.x)
        ty = float(target.pose.position.y)
        tyaw = _yaw_from_quaternion(target.pose.orientation)
        distance = math.hypot(tx - rx, ty - ry)
        yaw_error = abs(_angle_difference(tyaw, ryaw))

        if (
            distance <= self._start_position_tolerance
            and yaw_error <= self._start_yaw_tolerance
        ):
            self.get_logger().info(
                f"Already near coverage start (d={distance:.2f} m, "
                f"dyaw={yaw_error:.2f} rad); skipping approach."
            )
            return True

        self.get_logger().info(
            f"Navigating to coverage start ({tx:.2f}, {ty:.2f}) in {frame_id}: "
            f"distance={distance:.2f} m, dyaw={yaw_error:.2f} rad"
        )

        if not self._wait_for_action_server(self._navigate_to_pose_client):
            if self._abort_result_if_safety(goal_handle, result):
                return False
            self.get_logger().error(
                "Approach failed: action server unavailable: "
                f"{self._navigate_to_pose_action_name}"
            )
            result.error_code = self.ERR_APPROACH_FAILED
            goal_handle.abort()
            return False

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = target

        self._emit('approaching')
        send_future = self._send_owned(
            self._navigate_to_pose_client, nav_goal, goal_handle, 'approach')
        wait_status = self._wait_future(
            send_future, self._server_wait_timeout_sec, goal_handle
        )
        if wait_status == self.WAIT_ABORTED:
            self._abort_result_if_safety(goal_handle, result)
            return False
        if wait_status == self.WAIT_CANCELED:
            result.error_code = self.ERR_CANCELED
            goal_handle.canceled()
            return False
        if wait_status != self.WAIT_DONE:
            self.get_logger().error("Timed out waiting for NavigateToPose goal response.")
            result.error_code = self.ERR_APPROACH_FAILED
            goal_handle.abort()
            return False

        nav_handle = send_future.result()
        if nav_handle is None or not nav_handle.accepted:
            self.get_logger().error("NavigateToPose goal was rejected.")
            result.error_code = self.ERR_APPROACH_FAILED
            goal_handle.abort()
            return False

        nav_result_future = self._owned_result(nav_handle)
        wait_status = self._wait_future(
            nav_result_future,
            self._navigate_to_start_timeout_sec,
            goal_handle,
            cancel_fn=lambda: self._cancel_goal(nav_handle),
        )
        if wait_status == self.WAIT_ABORTED:
            self._abort_result_if_safety(goal_handle, result)
            return False
        if wait_status == self.WAIT_CANCELED:
            result.error_code = self.ERR_CANCELED
            goal_handle.canceled()
            return False
        if wait_status != self.WAIT_DONE:
            self._cancel_goal(nav_handle)
            self.get_logger().error(
                f"Approach timed out after {self._navigate_to_start_timeout_sec:.0f} s."
            )
            result.error_code = self.ERR_APPROACH_FAILED
            goal_handle.abort()
            return False

        wrapped = nav_result_future.result()
        status = int(wrapped.status) if wrapped is not None else -1
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Approach failed: NavigateToPose status={status}. "
                "若起点被障碍包住或不可达，需要人工把车挪到多边形附近再重试。"
            )
            result.error_code = self.ERR_APPROACH_FAILED
            goal_handle.abort()
            return False

        self.get_logger().info("Reached coverage start; proceeding to FollowPath.")
        return True

    def _safety_abort_active(self) -> bool:
        with self._safety_abort_lock:
            return self._safety_abort

    def _abort_result_if_safety(self, goal_handle, result) -> bool:
        """安全中止已置位时收尾并返回 True（调用方应直接 return result）。

        与 _wait_future 的 WAIT_ABORTED 分支共用同一套语义，
        避免"等 server"和"等结果"两条路径出现行为分叉。
        """
        if not self._safety_abort_active():
            return False
        with self._safety_abort_lock:
            reason = self._safety_abort_reason
        self.get_logger().error(f"Coverage task aborted by the safety layer: {reason}")
        result.error_code = self.ERR_SAFETY_ABORT
        goal_handle.abort()
        return True

    def _wait_for_action_server(self, client) -> bool:
        """等 action server 上线，同时响应安全中止。

        ⚠️ 不要用 client.wait_for_server(timeout_sec=...)：那是一次**阻塞**调用，
        期间无法感知 _safety_abort。若遥控抢占恰好发生在等待阶段，零速会立即发出
        （安全动作仍生效），但任务要等整个超时结束才收尾、error_code 也会变成
        901(ACTION_SERVER_UNAVAILABLE) 而不是 909 —— 实测踩到过。
        """
        deadline = time.monotonic() + self._server_wait_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._safety_abort_active():
                return False
            if client.server_is_ready():
                return True
            time.sleep(0.05)
        return False

    def _wait_future(
        self,
        future,
        timeout_sec: float,
        goal_handle=None,
        cancel_fn=None,
    ) -> str:
        start_time = time.monotonic()
        while rclpy.ok():
            # 安全中止优先于一切（遥控抢占 / 软件急停）。
            # 置位后取消内层 goal 并返回 WAIT_ABORTED，由 _finish_wait_failure 收尾。
            with self._safety_abort_lock:
                safety_abort = self._safety_abort
            if safety_abort:
                if cancel_fn is not None:
                    cancel_fn()
                return self.WAIT_ABORTED

            if goal_handle is not None and goal_handle.is_cancel_requested:
                if cancel_fn is not None:
                    cancel_fn()
                return self.WAIT_CANCELED

            if future.done():
                return self.WAIT_DONE

            if timeout_sec > 0.0 and (time.monotonic() - start_time) >= timeout_sec:
                if cancel_fn is not None:
                    cancel_fn()
                return self.WAIT_TIMEOUT

            time.sleep(0.05)

        return self.WAIT_SHUTDOWN

    def _cancel_goal(self, goal_handle) -> None:
        if goal_handle is None:
            return
        try:
            goal_handle.cancel_goal_async()
        except Exception:  # pylint: disable=broad-except
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageNavPathBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
