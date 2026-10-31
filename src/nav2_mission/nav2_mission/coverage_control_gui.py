"""Persistent coverage controls: edit, preview, and managed task operations."""
import copy
import json
import math
import queue
import signal
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid, Path
from nav2_mission_interfaces.srv import CancelTask, RegionCommand
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


OPERATION_LABELS = {
    'close_outer': '闭合外边界', 'begin_hole': '开始孔洞', 'close_hole': '闭合孔洞',
    'undo': '撤销', 'clear': '清空', 'status': '查询状态', 'configure': '应用参数',
    'point': '添加顶点', 'start': '启动覆盖任务',
}


def region_signature(state):
    return json.dumps({key: state.get(key) for key in
                       ('frame_id', 'mode', 'polygons', 'draft_hole', 'planner')},
                      sort_keys=True)


def map_center_summary(path, occupancy):
    """An explicit center-sample diagnostic, never a footprint collision claim."""
    if occupancy is None or occupancy.header.frame_id != path.header.frame_id:
        return '未收到同坐标系地图，未检查地图栅格。'
    info = occupancy.info
    if (not math.isfinite(info.resolution) or info.resolution <= 0 or
            len(occupancy.data) != info.width * info.height):
        return '地图数据无效，未检查地图栅格。'
    q = info.origin.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    occupied = unknown = outside = 0
    for item in path.poses:
        dx = item.pose.position.x - info.origin.position.x
        dy = item.pose.position.y - info.origin.position.y
        x = math.floor((c*dx + s*dy) / info.resolution)
        y = math.floor((-s*dx + c*dy) / info.resolution)
        if x < 0 or y < 0 or x >= info.width or y >= info.height:
            outside += 1
        else:
            value = occupancy.data[y * info.width + x]
            occupied += value >= 65
            unknown += value < 0
    if occupied or unknown or outside:
        return f'中心点检查：占据 {occupied}、未知 {unknown}、地图外 {outside} 个采样。'
    return '中心采样未落入占据/未知栅格；未检查车体轮廓或动态障碍。'


class CoverageControls(Node):
    """All widget actions share one worker; ROS callbacks never touch Tk."""

    def __init__(self, **kwargs):
        super().__init__('coverage_controls', **kwargs)
        self.declare_parameter('enable_execution', False)
        self.declare_parameter('preview_timeout_sec', 120.0)
        self.execution_enabled = self.get_parameter('enable_execution').value
        if type(self.execution_enabled) is not bool:
            raise ValueError('enable_execution must be a boolean')
        self.preview_timeout = float(self.get_parameter('preview_timeout_sec').value)
        if not math.isfinite(self.preview_timeout) or self.preview_timeout <= 0:
            raise ValueError('preview_timeout_sec must be finite and positive')
        # A dedicated compute-only server keeps previews off the task planner.
        self.compute = ActionClient(
            self, ComputeCoveragePath, '/coverage_preview/compute_coverage_path')
        planner = '/coverage_preview/coverage_server/'
        self.set_params = self.create_client(
            SetParametersAtomically, planner + 'set_parameters_atomically')
        self.get_params = self.create_client(GetParameters, planner + 'get_parameters')
        self.region = self.create_client(RegionCommand, '/task_control/region')
        self.cancel_task = (self.create_client(CancelTask, '/task_control/cancel')
                            if self.execution_enabled else None)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path = self.create_publisher(Path, '/coverage_ui/preview_path', durable)
        self.events = queue.Queue()
        self.state = None
        self.occupancy = None
        self._signature = None
        self._closing = threading.Event()
        self._worker = None
        self._worker_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self.initialized = False
        self.create_subscription(String, '/coverage/editor_state', self.on_state, durable)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
        if self.execution_enabled:
            self.create_subscription(String, '/task_control/status_snapshot', self.on_task, 10)
        self.create_timer(0.5, self.bootstrap, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def bootstrap(self):
        if (not self.initialized and not self._closing.is_set() and
                not self._worker_lock.locked()):
            self.submit('initialize')

    def initialize(self):
        state = json.loads(self.edit('status').state_json)
        if not state.get('planner_initialized'):
            names = ['operation_width', 'default_swath_angle']
            reply = self.call(self.get_params, GetParameters.Request(names=names))
            settings = dict(zip(names, map(parameter_value_to_python, reply.values)))
            if set(settings) != set(names) or any(v is None for v in settings.values()):
                raise RuntimeError('规划器参数尚未加载，正在等待初始化。')
            response = self.edit('initialize', settings)
            if not response.accepted:
                raise RuntimeError('无法初始化界面参数：' + response.message)
            state = json.loads(response.state_json)
        self.initialized = True
        self.events.put(('ready', state))
        self.notice('已加载规划参数；可直接修改，停止输入后自动应用。')

    def on_map(self, message):
        self.occupancy = message

    def on_task(self, message):
        try:
            data = json.loads(message.data)
            self.events.put(('task', data))
        except (TypeError, ValueError):
            pass

    def on_state(self, message):
        try:
            self.accept_state(json.loads(message.data))
        except (TypeError, ValueError):
            pass

    def accept_state(self, state):
        with self._state_lock:
            signature = region_signature(state)
            if self._signature is not None and (
                    signature != self._signature or
                    (state.get('active') and not self.state.get('active'))):
                self.clear_path(state['frame_id'])
            self.state, self._signature = state, signature
        self.events.put(('state', state))

    def clear_path(self, frame='map'):
        path = Path()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()
        self.path.publish(path)

    def notice(self, text):
        self.events.put(('notice', text))

    def submit(self, operation, **payload):
        if self._closing.is_set():
            return False
        if operation == 'cancel':
            self.request_cancel()
            return True
        if operation not in ('initialize', 'status') and not self.initialized:
            self.notice('正在读取加载的规划参数，请稍候。')
            return False
        if not self._worker_lock.acquire(blocking=False):
            self.notice('正在处理上一项操作，请稍候。')
            return False

        def work():
            try:
                if operation == 'initialize':
                    self.initialize()
                    return
                if operation == 'start' and not self.execution_enabled:
                    raise ValueError('预览模式不能启动车辆任务。')
                if operation in ('preview', 'start') and payload:
                    configured = self.edit('configure', payload)
                    self.events.put(('configuration', {
                        'request': payload, 'state': json.loads(configured.state_json)}))
                    if not configured.accepted:
                        raise ValueError('参数未通过：' + configured.message)
                    payload_for_operation = {}
                else:
                    payload_for_operation = payload
                if operation == 'preview':
                    self.preview()
                else:
                    reply = self.edit(operation, payload_for_operation)
                    if operation == 'configure':
                        self.events.put(('configuration', {
                            'request': payload, 'state': json.loads(reply.state_json)}))
                    self.notice(('操作成功：' + OPERATION_LABELS.get(operation, operation)
                                 if reply.accepted
                                 else '未通过：' + reply.message))
            except Exception as exc:
                self.notice(str(exc))
            finally:
                self._worker_lock.release()
        self._worker = threading.Thread(target=work, daemon=True)
        self._worker.start()
        return True

    def request_cancel(self):
        # Task cancellation must not queue behind a potentially slow preview.
        if not self.execution_enabled:
            self.notice('预览模式不能取消车辆任务。')
            return
        if not self.cancel_task.service_is_ready():
            self.notice('取消服务不可用，请检查统一任务管理器。')
            return
        future = self.cancel_task.call_async(CancelTask.Request())
        self.notice('正在请求取消任务……')

        timer_alive = [True]

        def stop_timer():
            if timer_alive[0]:
                timer_alive[0] = False
                timer.cancel()
                self.destroy_timer(timer)

        def expired():
            stop_timer()
            if not future.done():
                self.notice('取消回复超时；请检查任务状态和停车确认。')

        timer = self.create_timer(6.0, expired)

        def done(result):
            stop_timer()
            try:
                reply = result.result()
                self.notice('已请求取消；请等待停车确认。' if reply.accepted
                            else '取消未通过：' + reply.message)
            except Exception as exc:
                self.notice('取消请求异常：' + str(exc))
        future.add_done_callback(done)

    def wait(self, future, timeout, *, cleanup=False):
        deadline = time.monotonic() + timeout
        while not future.done():
            if not cleanup and self._closing.is_set():
                raise InterruptedError('界面正在退出。')
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise TimeoutError('等待 ROS 响应超时；请检查节点和日志。')
            time.sleep(0.02)
        result = future.result()
        if result is None:
            raise RuntimeError('ROS 返回了空响应。')
        return result

    def call(self, client, request, *, cleanup=False):
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('服务不可用：' + client.srv_name)
        return self.wait(client.call_async(request), 6.0, cleanup=cleanup)

    def edit(self, operation, payload=None):
        reply = self.call(self.region, RegionCommand.Request(
            operation=operation, request_json=json.dumps(payload or {}, allow_nan=False)))
        self.accept_state(json.loads(reply.state_json))
        return reply

    def preview(self):
        state = json.loads(self.edit('status').state_json)
        if state.get('active') or not state.get('valid'):
            raise ValueError('请先完成有效选区；活动任务或停车未确认时不能预览。')
        frozen = copy.deepcopy(state)
        signature = region_signature(frozen)
        if not self.compute.wait_for_server(timeout_sec=2.0):
            raise RuntimeError('预览规划器尚未就绪。')
        if not self.set_params.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('预览规划参数服务尚未就绪。')
        owner = uuid.uuid4().hex
        lease = False
        pending = handle = result_future = None
        terminal = False
        self.clear_path(frozen['frame_id'])
        try:
            # Retain ownership even if a parameter service response is delayed.
            pending_lease = self.set_params.call_async(SetParametersAtomically.Request(
                parameters=[Parameter(key, value=value).to_parameter_msg() for key, value in {
                    'task_parameter_owner': owner, **frozen['planner']}.items()]))
            response = self.wait(pending_lease, 6.0)
            if not response.result.successful:
                raise RuntimeError('预览参数被拒绝：' + response.result.reason)
            lease = True
            names = ['operation_width', 'swath_spacing', 'default_headland_width',
                     'min_turning_radius']
            values = self.call(self.get_params, GetParameters.Request(names=names)).values
            settings = dict(zip(names, map(parameter_value_to_python, values)))
            self.events.put(('settings', settings))
            goal = ComputeCoveragePath.Goal(frame_id=frozen['frame_id'],
                                            generate_headland=True, generate_route=True,
                                            generate_path=True)
            for ring in frozen['polygons']:
                closed = ring if ring[0] == ring[-1] else [*ring, ring[0]]
                goal.polygons.append(Coordinates(coordinates=[
                    Coordinate(axis1=float(x), axis2=float(y)) for x, y in closed]))
            self.notice('正在生成覆盖路径……')
            pending = self.compute.send_goal_async(goal)
            handle = self.wait(pending, 10.0)
            if not handle.accepted:
                terminal = True
                raise RuntimeError('预览规划器拒绝了请求。')
            result_future = handle.get_result_async()
            wrapped = self.wait(result_future, self.preview_timeout)
            terminal = True
            current = json.loads(self.edit('status').state_json)
            result = wrapped.result
            if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or result.error_code or
                    not result.nav_path.poses):
                raise RuntimeError(f'规划失败：状态 {wrapped.status}，错误码 '
                                   f'{result.error_code}。请检查区域转弯空间和规划器日志。')
            if result.nav_path.header.frame_id != frozen['frame_id']:
                raise RuntimeError('规划结果坐标系错误，未显示路径。')
            if not all(math.isfinite(v) for pose in result.nav_path.poses for v in (
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)):
                raise RuntimeError('规划结果包含非法坐标，未显示路径。')
            with self._state_lock:
                if (current.get('active') or self.state.get('active') or
                        signature != self._signature):
                    raise RuntimeError('区域或任务状态已改变，旧预览已丢弃；请重新规划。')
                self.path.publish(result.nav_path)
            self.notice(f'规划成功：{len(result.nav_path.poses)} 个路径点。' +
                        map_center_summary(result.nav_path, self.occupancy))
        finally:
            # No result/cancel acknowledgement is treated as uncertain ownership.
            if pending is not None and not terminal:
                try:
                    handle = handle or self.wait(pending, 5.0, cleanup=True)
                    if not handle.accepted:
                        terminal = True
                    else:
                        handle.cancel_goal_async()
                        result_future = result_future or handle.get_result_async()
                        self.wait(result_future, 5.0, cleanup=True)
                        terminal = True
                except Exception:
                    self.notice('预览动作终止未确认；请重启预览启动流程后再规划。')
            # A delayed lease response may acquire after an earlier timeout.
            try:
                if not lease and 'pending_lease' in locals():
                    lease = self.wait(pending_lease, 5.0, cleanup=True).result.successful
                if lease and (pending is None or terminal):
                    request = SetParametersAtomically.Request(parameters=[
                        Parameter('task_parameter_owner', value='').to_parameter_msg(),
                        Parameter('task_parameter_release', value=owner).to_parameter_msg()])
                    reply = self.call(self.set_params, request, cleanup=True)
                    if not reply.result.successful:
                        raise RuntimeError(reply.result.reason)
            except Exception as exc:
                self.notice('预览参数释放未确认，请重启预览节点：' + str(exc))

    def close(self):
        self._closing.set()
        if self._worker is not None:
            self._worker.join(timeout=30.0)

    def destroy_node(self):
        self.compute.destroy()
        return super().destroy_node()


class CoverageWindow:
    def __init__(self, controls):
        import tkinter as tk
        from tkinter import messagebox, ttk

        self.controls = controls
        self.root = tk.Tk()
        self.root.title('覆盖选区与路径预览' +
                        ('（整车任务模式）' if controls.execution_enabled else '（仅预览）'))
        self.root.geometry('680x640')
        self.root.minsize(590, 560)
        self.notice = tk.StringVar(value='正在连接选区服务……')
        self.state = tk.StringVar(value='')
        self.task = tk.StringVar(value='')
        self.settings = tk.StringVar(value='预览使用当前选区宽度和方向；参数将在规划前读取。')
        self._field_dirty = False
        self._syncing_fields = False
        self._apply_after = None
        self._mutating_widgets = []
        ttk.Label(self.root, text='覆盖选区编辑与路径预览',
                  font=('', 14, 'bold')).pack(pady=10)
        ttk.Label(self.root, text='在 RViz 选择 Publish Point，按边界顺序点击；\n'
                  '闭合外边界后，可添加孔洞，再生成覆盖路径。').pack(pady=4)
        buttons = ttk.Frame(self.root)
        buttons.pack(pady=6)
        for i, (label, operation) in enumerate([
                ('闭合外边界', 'close_outer'), ('开始孔洞', 'begin_hole'),
                ('闭合孔洞', 'close_hole'), ('撤销', 'undo'),
                ('清空', 'clear'), ('刷新状态', 'status')]):
            button = ttk.Button(buttons, text=label, command=lambda op=operation:
                                controls.submit(op))
            button.grid(row=i // 3, column=i % 3, padx=5, pady=5)
            if operation != 'status':
                self._mutating_widgets.append(button)
        configuration = ttk.Frame(self.root)
        configuration.pack(pady=4)
        self.width = tk.StringVar(value='')
        self.angle = tk.StringVar(value='')
        for label, variable in [('作业宽度（米）', self.width), ('条带方向（弧度）', self.angle)]:
            ttk.Label(configuration, text=label).pack(side='left', padx=3)
            entry = ttk.Entry(configuration, textvariable=variable, width=7)
            entry.pack(side='left')
            entry.bind('<Return>', lambda _: self.apply_fields(explicit=True))
            self._mutating_widgets.append(entry)
            variable.trace_add('write', self.fields_changed)
        apply_button = ttk.Button(configuration, text='应用',
                                  command=lambda: self.apply_fields(explicit=True))
        apply_button.pack(side='left', padx=5)
        preview_button = ttk.Button(self.root, text='生成覆盖路径（仅预览，不控制车辆）',
                                    command=lambda: self.request_action('preview'))
        preview_button.pack(pady=8)
        self._mutating_widgets.extend([apply_button, preview_button])
        if controls.execution_enabled:
            task_buttons = ttk.Frame(self.root)
            task_buttons.pack(pady=4)

            def start():
                if messagebox.askyesno('启动覆盖任务',
                                       '确认通过统一任务管理启动覆盖？车辆可能开始运动。',
                                       parent=self.root):
                    self.request_action('start')
            start_button = ttk.Button(task_buttons, text='启动覆盖任务', command=start)
            start_button.pack(side='left', padx=5)
            self._mutating_widgets.append(start_button)
            ttk.Button(task_buttons, text='取消任务并停车',
                       command=lambda: controls.submit('cancel')).pack(side='left', padx=5)
        for variable in (self.notice, self.state, self.task, self.settings):
            ttk.Label(self.root, textvariable=variable, wraplength=640,
                      justify='left').pack(padx=12, pady=6)
        ttk.Label(self.root, text='启动窗口不会启动任务。预览成功不代表实车执行成功。\n'
                  '路径预览不自动避开地图障碍；中心栅格检查不代替车体碰撞检查。',
                  wraplength=640, justify='left').pack(side='bottom', padx=12, pady=10)
        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self.update_widgets()
        self.root.after(100, self.poll)

    def field_values(self):
        width, angle = float(self.width.get()), float(self.angle.get())
        if (not math.isfinite(width) or width <= 0 or not math.isfinite(angle) or
                not -math.pi <= angle <= math.pi):
            raise ValueError('作业宽度必须大于零，方向应为 [-π, π] 内的弧度。')
        return {'operation_width': width, 'default_swath_angle': angle}

    def sync_fields(self, state):
        self._syncing_fields = True
        try:
            self.width.set(str(state['planner']['operation_width']))
            self.angle.set(str(state['planner']['default_swath_angle']))
        finally:
            self._syncing_fields = False

    def fields_changed(self, *_):
        if self._syncing_fields:
            return
        self._field_dirty = True
        if self._apply_after is not None:
            self.root.after_cancel(self._apply_after)
        self._apply_after = self.root.after(500, self.apply_fields)

    def apply_fields(self, explicit=False):
        if self._apply_after is not None:
            self.root.after_cancel(self._apply_after)
            self._apply_after = None
        if not self._field_dirty:
            return
        try:
            values = self.field_values()
        except ValueError as exc:
            if explicit:
                self.notice.set(str(exc))
            return
        if (not self.controls.initialized or self.controls._worker_lock.locked() or
                (self.controls.state or {}).get('active')):
            self._apply_after = self.root.after(300, self.apply_fields)
            return
        self.controls.submit('configure', **values)

    def request_action(self, operation):
        try:
            values = self.field_values()
        except ValueError as exc:
            self.notice.set(str(exc))
            return
        if self._apply_after is not None:
            self.root.after_cancel(self._apply_after)
            self._apply_after = None
        # Submit the visible values before planning/start even if debounce is pending.
        self.controls.submit(operation, **values)

    def update_widgets(self):
        idle = (self.controls.initialized and not self.controls._worker_lock.locked() and
                not (self.controls.state or {}).get('active'))
        for widget in self._mutating_widgets:
            widget.configure(state='normal' if idle else 'disabled')

    def poll(self):
        while True:
            try:
                kind, value = self.controls.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'notice':
                self.notice.set(value)
            elif kind == 'state':
                rings = value.get('polygons') or [[]]
                mode = {'OUTER_DRAFT': '绘制外边界', 'OUTER_CLOSED': '外边界已闭合',
                        'HOLE_DRAFT': '绘制孔洞'}.get(value['mode'], value['mode'])
                self.state.set(f"状态：{mode}；外边界 {len(rings[0])} 点；"
                               f"孔洞 {len(rings)-1} 个；有效："
                               + ('是' if value['valid'] else '否'))
                if self.controls.initialized:
                    self.state.set(self.state.get() +
                                   f"；已应用宽度 {value['planner']['operation_width']} 米，"
                                   f"方向 {value['planner']['default_swath_angle']} 弧度")
                    if value.get('active'):
                        self._field_dirty = False
                    if not self._field_dirty:
                        self.sync_fields(value)
            elif kind == 'ready':
                self._field_dirty = False
                self.sync_fields(value)
            elif kind == 'configuration':
                try:
                    if self.field_values() == value['request']:
                        self._field_dirty = False
                        self.sync_fields(value['state'])
                except ValueError:
                    pass
            elif kind == 'task':
                phases = {'idle': '空闲', 'dependency_check': '检查依赖', 'preparing': '准备',
                          'planning': '规划', 'validating': '校验路径', 'approaching': '前往起点',
                          'following': '执行覆盖路径', 'finalizing': '收尾', 'stopping': '停车中',
                          'finished': '已结束', 'navigating': '导航中', 'inspecting': '巡检中'}
                phase = phases.get(value.get('phase'), value.get('phase', '等待状态'))
                result = {None: '—', 'succeeded': '成功', 'failed': '失败',
                          'canceled': '已取消'}.get(value.get('result'), value.get('result'))
                stopped = '是' if value.get('cleanup_confirmed') else '否'
                self.task.set(f'任务阶段：{phase}；结果：{result}；停车清理确认：{stopped}')
            elif kind == 'settings':
                names = {'operation_width': '作业宽度', 'swath_spacing': '条带间距',
                         'default_headland_width': '地头宽度', 'min_turning_radius': '转弯半径'}
                self.settings.set('本次预览参数：' + '；'.join(
                    f'{names.get(name, name)} {item} 米' for name, item in value.items()))
        self.update_widgets()
        if rclpy.ok() and not self.controls._closing.is_set():
            self.root.after(100, self.poll)
        else:
            self.root.quit()


def main(args=None):
    # Keep the ROS context alive while a closing window drains its preview.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    controls = CoverageControls()
    previous = {signum: signal.signal(signum, lambda *_: controls._closing.set())
                for signum in (signal.SIGINT, signal.SIGTERM)}
    executor = SingleThreadedExecutor()
    executor.add_node(controls)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    window = None
    try:
        window = CoverageWindow(controls)
        window.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        controls.close()
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        controls.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if window is not None:
            window.root.destroy()
        for signum, handler in previous.items():
            signal.signal(signum, handler)
