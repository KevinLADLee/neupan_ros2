#!/usr/bin/env python3
"""Release a shared simulation once all native planners and start services are ready."""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node('neupan_start_fleet')
    try:
        names = node.declare_parameter('robot_names', ['']).value
        timeout = node.declare_parameter('startup_timeout', 40.0).value
        if not names or any(not name for name in names) or len(set(names)) != len(names) or timeout <= 0:
            raise ValueError('Expected unique robot_names and positive startup_timeout')
        ready, terminal, subscriptions = set(), set(), []
        clients = [node.create_client(Trigger, f'/{name}/start') for name in names]

        def diagnostics(message, name, simulator=False):
            for status in message.status:
                values = {item.key: item.value for item in status.values}
                if simulator and values.get('result') in ('goal_reached', 'collision'):
                    terminal.add(name)
                if not simulator and values.get('solved') == 'true':
                    ready.add(name)

        for name in names:
            subscriptions.append(node.create_subscription(
                DiagnosticArray, f'/{name}/neupan_diagnostics',
                lambda msg, key=name: diagnostics(msg, key), 10))
            subscriptions.append(node.create_subscription(
                DiagnosticArray, f'/{name}/neupan_sim/diagnostics',
                lambda msg, key=name: diagnostics(msg, key, True),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

        def wait_for(predicate):
            deadline = time.monotonic() + timeout
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return
            raise RuntimeError(f'Fleet startup timeout; planners not ready: {set(names) - ready - terminal}')

        wait_for(lambda: set(names) <= ready | terminal and all(c.service_is_ready() for c in clients))
        futures = [client.call_async(Trigger.Request()) for client in clients]
        wait_for(lambda: all(f.done() for f in futures))
        if not all(f.result() and f.result().success for f in futures):
            raise RuntimeError('A simulator refused to start')
        node.get_logger().info(f'Shared simulation started: {", ".join(names)}')
        return 0
    except Exception as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
