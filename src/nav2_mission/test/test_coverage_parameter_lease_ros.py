"""Parameter transactions against the real local OpenNav coverage server."""
import subprocess
import threading
import time

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.srv import ChangeState
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python


@pytest.fixture
def planner(tmp_path):
    rclpy.init()
    node = Node('planner_parameter_user')
    executable = get_package_prefix('opennav_coverage') + '/lib/opennav_coverage/opennav_coverage'
    log = open(tmp_path / 'planner.log', 'w+')
    process = subprocess.Popen([executable, '--ros-args', '-p', 'bond_heartbeat_period:=0.0'],
                               stdout=log, stderr=log)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def call(service, endpoint, request):
        client = node.create_client(service, '/coverage_server/' + endpoint)
        assert client.wait_for_service(timeout_sec=8)
        future = client.call_async(request)
        deadline = time.monotonic() + 8
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert future.done()
        result = future.result()
        node.destroy_client(client)
        return result

    try:
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            assert call(ChangeState, 'change_state', request).success

        def set_values(**values):
            return call(SetParametersAtomically, 'set_parameters_atomically',
                        SetParametersAtomically.Request(parameters=[
                            Parameter(key, value=value).to_parameter_msg()
                            for key, value in values.items()])).result

        def get_values(*names):
            return [parameter_value_to_python(value) for value in call(
                GetParameters, 'get_parameters', GetParameters.Request(names=list(names))).values]

        yield set_values, get_values, node
    finally:
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()


def test_active_task_lease_rejects_ordinary_edits_until_matching_owner_release(planner):
    set_values, get_values, _ = planner
    acquired = set_values(task_parameter_owner='1234567890abcdef1234567890abcdef',
                          operation_width=0.7, default_swath_angle=0.3)
    assert acquired.successful, acquired.reason
    assert not set_values(operation_width=1.2).successful
    assert not set_values(robot_width=0.9).successful
    assert not set_values(validate_path_bounds=False).successful
    assert not set_values(task_parameter_owner='').successful
    assert not set_values(task_parameter_owner='other').successful
    assert get_values('operation_width', 'default_swath_angle') == [0.7, 0.3]
    released = set_values(task_parameter_owner='',
                          task_parameter_release='1234567890abcdef1234567890abcdef')
    assert released.successful, released.reason
    assert set_values(operation_width=0.8).successful


def test_real_compute_and_concurrent_parameter_edit_both_finish_under_lease(planner):
    set_values, get_values, node = planner
    owner = '1234567890abcdef1234567890abcdef'
    acquired = set_values(task_parameter_owner=owner, operation_width=0.7,
                          swath_spacing=1.5, min_turning_radius=0.95,
                          default_headland_width=1.5, default_swath_angle=0.3)
    assert acquired.successful, acquired.reason
    client = ActionClient(node, ComputeCoveragePath, '/compute_coverage_path')
    try:
        assert client.wait_for_server(timeout_sec=5)
        goal = ComputeCoveragePath.Goal(frame_id='map', polygons=[Coordinates(coordinates=[
            Coordinate(axis1=x, axis2=y) for x, y in
            ((0., 0.), (80., 0.), (80., 80.), (0., 80.), (0., 0.))])])
        response = client.send_goal_async(goal)
        deadline = time.monotonic() + 8
        while not response.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert response.done() and response.result().accepted
        result = response.result().get_result_async()
        time.sleep(0.1)
        assert not result.done(), 'fixture must overlap the real planning operation'
        # The parameter service and compute action must never deadlock each other.
        edit = set_values(validate_path_bounds=False)
        assert not edit.successful and 'leased' in edit.reason
        deadline = time.monotonic() + 12
        while not result.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert result.done(), 'real planner did not return after concurrent edit'
        wrapped = result.result()
        assert wrapped.status == 4 and wrapped.result.error_code == 0
        assert wrapped.result.nav_path.poses
        assert get_values('operation_width') == [0.7]
        assert set_values(task_parameter_owner='', task_parameter_release=owner).successful
    finally:
        client.destroy()
