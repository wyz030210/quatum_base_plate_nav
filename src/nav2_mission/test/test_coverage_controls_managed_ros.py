"""GUI commands use the real supervisor and coverage bridge."""
import threading

from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from nav2_mission.coverage_control_gui import CoverageControls
from test_coverage_controls_ros import until
import test_coverage_task_ros as coverage_test

coverage_system = coverage_test.coverage_system


def test_control_window_runs_and_cancels_through_unified_manager(coverage_system):
    _, status, _, boundary = coverage_system
    gui = CoverageControls(parameter_overrides=[Parameter('enable_execution', value=True)])
    executor = SingleThreadedExecutor()
    executor.add_node(gui)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def operation(name, **payload):
        gui.submit(name, **payload)
        until(lambda: gui._worker is None or not gui._worker.is_alive())

    try:
        until(lambda: gui.region.service_is_ready() and gui.cancel_task.service_is_ready())
        assert gui.edit('configure', {
            'operation_width': 0.7, 'default_swath_angle': 0.3}).accepted
        until(lambda: gui.initialized and not gui._worker_lock.locked())
        boundary['hold'] = 'follow'
        for x, y in [(0., 0.), (8., 0.), (8., 8.), (0., 8.)]:
            operation('point', x=x, y=y)
        operation('close_outer')
        operation('configure', operation_width=0.7, default_swath_angle=0.3)
        operation('start')
        until(lambda: any(stage == 'follow' for stage, _ in boundary['goals']))
        assert status()['active']
        operation('clear')
        assert gui.state['polygons'][0]
        assert gui.state['active']
        gui.submit('cancel')
        until(lambda: 'follow' in boundary['cancels'])
        assert status()['active']
        boundary['release_terminal'].set()
        until(lambda: not status()['active'])
        final = status()
        assert final['result'] == 'canceled' and final['cleanup_confirmed']
    finally:
        boundary['release_terminal'].set()
        gui.close()
        assert executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        gui.destroy_node()
