"""Command-line client for immutable ROS task requests."""
import argparse
import json
from pathlib import Path

import rclpy
from nav2_mission_interfaces.srv import CancelTask, GetTaskStatus, RegionCommand, StartTask
from rclpy.node import Node


def main(args=None):
    parser = argparse.ArgumentParser(description='Unified vehicle task start/cancel/status')
    parser.add_argument('--timeout', type=float, default=5.0)
    commands = parser.add_subparsers(dest='command', required=True)
    start = commands.add_parser('start')
    start.add_argument('task_type')
    payload = start.add_mutually_exclusive_group(required=True)
    payload.add_argument('--request', help='JSON request object')
    payload.add_argument('--file', type=Path, help='JSON request file')
    cancel = commands.add_parser('cancel')
    cancel.add_argument('task_id', nargs='?', default='')
    commands.add_parser('status')
    region = commands.add_parser('region')
    region.add_argument('operation', choices=['status', 'configure', 'point', 'undo', 'clear',
                                           'close_outer', 'begin_hole', 'close_hole', 'start'])
    region.add_argument('--request', default='{}', help='operation parameters as JSON object')
    options = parser.parse_args(args)
    if options.command == 'start':
        service = StartTask
        request = StartTask.Request()
        request.task_type = options.task_type
        request.request_json = (options.file.read_text(encoding='utf-8') if options.file else
                                options.request)
        # Invalid JSON is rejected before transport, server validates the task schema.
        json.loads(request.request_json)
    elif options.command == 'region':
        service = RegionCommand
        request = RegionCommand.Request(operation=options.operation, request_json=options.request)
        json.loads(request.request_json)
    elif options.command == 'cancel':
        service = CancelTask
        request = CancelTask.Request(task_id=options.task_id)
    else:
        service = GetTaskStatus
        request = GetTaskStatus.Request()
    rclpy.init(args=[])
    node = Node('task_control_cli')
    try:
        client = node.create_client(service, '/task_control/' + options.command)
        if not client.wait_for_service(timeout_sec=options.timeout):
            parser.exit(2, 'Task control service unavailable\n')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=options.timeout)
        if not future.done():
            parser.exit(2, 'Task control response timed out; query status before retrying start\n')
        reply = future.result()
        if options.command == 'status':
            print(reply.status_json)
        else:
            result = {'accepted': reply.accepted, 'task_id': reply.task_id,
                      'message': reply.message}
            if options.command == 'region':
                result['state'] = json.loads(reply.state_json)
            print(json.dumps(result))
            if not reply.accepted:
                raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
