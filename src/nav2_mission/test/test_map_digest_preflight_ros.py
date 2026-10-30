"""Site map digest can be observed without vehicle, controller or motion nodes."""
import json
import subprocess
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node
from std_msgs.msg import String


def test_map_digest_preflight_is_nonmotion_and_does_not_auto_trust(tmp_path):
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n2 2\n255\n' + bytes([0, 254, 205, 254]))
    config = tmp_path / 'map.yaml'
    config.write_text('image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
                      'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    map_server = subprocess.Popen([
        '/opt/ros/humble/lib/nav2_map_server/map_server', '--ros-args',
        '-p', 'yaml_filename:=' + str(config)], stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL)
    provider = subprocess.Popen([
        'ros2', 'run', 'nav2_mission', 'localization_provider', '--ros-args',
        '-p', 'source:=amcl', '-p', 'map_id:="' + '0' * 64 + '"',
        '-p', 'localization_profile:=preflight'], stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL)
    rclpy.init()
    observer = Node('map_preflight_observer')
    states = []
    observer.create_subscription(String, '/task_control/localization',
                                 lambda msg: states.append(json.loads(msg.data)), 10)
    client = observer.create_client(ChangeState, '/map_server/change_state')
    try:
        assert client.wait_for_service(timeout_sec=8)
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(observer, future, timeout_sec=8)
            assert future.done() and future.result().success
        deadline = time.monotonic() + 8
        while not any(state.get('observed_map_id') for state in states):
            assert time.monotonic() < deadline, states
            rclpy.spin_once(observer, timeout_sec=0.1)
        state = next(state for state in states if state.get('observed_map_id'))
        assert len(state['observed_map_id']) == 64
        assert state['observed_map_id'] != '0' * 64
        assert not state['valid']
    finally:
        observer.destroy_node()
        rclpy.shutdown()
        for process in (provider, map_server):
            process.terminate()
        for process in (provider, map_server):
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
