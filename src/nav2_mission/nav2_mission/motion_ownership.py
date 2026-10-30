"""Goal UUID ownership plus conservative Humble action-client topology checks.

ROS status has no parent UUID. Navigator FollowPath attribution is an inference
valid only for the verified single bt_navigator client topology, never access control.
"""
from collections import Counter

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from rclpy.action.graph import get_action_client_names_and_types_by_node
from rclpy.qos import qos_profile_action_status_default
from unique_identifier_msgs.msg import UUID


_ACTIVE = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
           GoalStatus.STATUS_CANCELING}
_TERMINAL = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
             GoalStatus.STATUS_ABORTED}


class MotionOwnership:
    def __init__(self, node, navigate, follow, coverage, *,
                 navigator='/bt_navigator_navigate_to_pose_rclcpp_node',
                 through_navigator='/bt_navigator_navigate_through_poses_rclcpp_node',
                 navigator_server='/bt_navigator', through='/navigate_through_poses',
                 bridge='/coverage_navpath_bridge'):
        self.node = node
        self.navigate, self.follow, self.coverage = navigate, follow, coverage
        self.owned = {name: set() for name in (navigate, follow, coverage, through)}
        self.observed = {name: {} for name in self.owned}
        self.pending_navigation = set()
        self.inferred = set()
        own_name = node.get_fully_qualified_name()
        self.allowed = {navigate: {own_name, bridge, navigator_server},
                        follow: {navigator, through_navigator, bridge},
                        coverage: {own_name}, through: set()}
        self.through = through
        self.navigator = navigator
        self.clients = {name: node.create_client(CancelGoal, name + '/_action/cancel_goal')
                        for name in self.owned}
        self.canceling = set()
        self.subscriptions = [node.create_subscription(
            GoalStatusArray, name + '/_action/status',
            lambda message, action=name: self.receive(action, message),
            qos_profile_action_status_default) for name in self.owned]
        self.topology = {}
        self.topology_fault = ''

    def register(self, action, goal_id, *, pending_navigation=False):
        self.owned[action].add(goal_id)
        if pending_navigation:
            self.pending_navigation.add(goal_id)

    def navigation_terminal(self, goal_id):
        self.pending_navigation.discard(goal_id)

    def inspect_topology(self):
        nodes = self.node.get_node_names_and_namespaces()
        counts = Counter((namespace.rstrip('/') + '/' + name) for name, namespace in nodes)
        topology = {name: set() for name in self.owned}
        try:
            for name, namespace in set(nodes):
                identity = namespace.rstrip('/') + '/' + name
                for action, _ in get_action_client_names_and_types_by_node(
                        self.node, name, namespace):
                    if action in topology:
                        topology[action].add(identity)
            faults = []
            for action, clients in topology.items():
                unexpected = clients - self.allowed[action]
                duplicates = {name for name in clients if counts[name] != 1}
                if unexpected or duplicates:
                    faults.append(f'{action}: unexpected/duplicate clients '
                                  f'{sorted(unexpected | duplicates)}')
            self.topology = topology
            self.topology_fault = '; '.join(faults)
        except Exception as exc:
            self.topology_fault = f'action graph unavailable: {exc}'
        return self.topology_fault

    def receive(self, action, message):
        self.inspect_topology()
        for item in message.status_list:
            goal_id = bytes(item.goal_info.goal_id.uuid).hex()
            state = int(item.status)
            if (action == self.follow and state in _ACTIVE and
                    goal_id not in self.owned[action] and goal_id not in self.inferred and
                    not self.topology_fault and
                    self.navigator in self.topology.get(self.follow, set()) and
                    not any(value in _ACTIVE for value in self.observed[self.through].values()) and
                    (self.pending_navigation or any(
                        key in self.owned[self.navigate] and value in _ACTIVE
                        for key, value in self.observed[self.navigate].items()))):
                self.inferred.add(goal_id)
            # Retain active observations until an explicit terminal. Missing or
            # old status cannot establish termination, including after restart.
            self.observed[action][goal_id] = state
            if state in _TERMINAL:
                self.canceling.discard((action, goal_id))

    def unknown(self):
        return [(action, goal_id) for action, states in self.observed.items()
                for goal_id, state in states.items() if state in _ACTIVE and
                goal_id not in self.owned[action] and
                not (action == self.follow and goal_id in self.inferred)]

    def error(self):
        fault = self.inspect_topology()
        unknown = self.unknown()
        if fault:
            return 'external navigation conflict: ' + fault
        if unknown:
            return 'external navigation conflict: active unknown goals ' + str(unknown)
        return ''

    def cancel_unknown(self):
        for action, goal_id in self.unknown():
            if (action, goal_id) in self.canceling or not self.clients[action].service_is_ready():
                continue
            request = CancelGoal.Request()
            request.goal_info.goal_id = UUID(uuid=list(bytes.fromhex(goal_id)))
            future = self.clients[action].call_async(request)
            self.canceling.add((action, goal_id))
            # Cancellation acceptance is not a terminal status.
            future.add_done_callback(lambda _: None)

    def drained(self):
        return not any(state in _ACTIVE for states in self.observed.values()
                       for state in states.values())

    def snapshot(self):
        return {'unknown_active_goals': [list(value) for value in self.unknown()],
                'navigator_child_attribution': 'verified client topology inference',
                'inferred_navigator_follow_ids': sorted(self.inferred),
                'clients': {name: sorted(value) for name, value in self.topology.items()}}
