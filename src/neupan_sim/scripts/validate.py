#!/usr/bin/env python3
"""Validate isolated replicas or a shared robot world; synchronize, report and clean up."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

from validation_config import load_cases, load_shared_cases, finite_float, verdict


def executable(package, name):
    return str(Path(get_package_prefix(package)) / 'lib' / package / name)


class Validation(Node):
    def __init__(self, cases, output, shared=False):
        super().__init__('neupan_validation')
        self.cases, self.output, self.shared = cases, output, shared
        self.processes, self.logs, self.subscriptions_ = [], [], []
        self.states, self.ready, self.unsolved, self.start_clients = {}, set(), {}, {}
        self.started = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        transforms = []
        for i, case in enumerate(cases):
            name = case['name']
            self.unsolved[name] = 0
            self.subscriptions_.append(self.create_subscription(
                DiagnosticArray, f'/{name}/neupan_sim/diagnostics',
                lambda msg, key=name: self.sim_diagnostics(key, msg), qos))
            self.subscriptions_.append(self.create_subscription(
                DiagnosticArray, f'/{name}/neupan_diagnostics',
                lambda msg, key=name: self.planner_diagnostics(key, msg), 10))
            self.start_clients[name] = self.create_client(Trigger, f'/{name}/start')
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'validation_world'
            transform.child_frame_id = case['simulator']['world_frame']
            transform.transform.translation.x = 0.0 if shared else (i % 4) * 15.0
            transform.transform.translation.y = 0.0 if shared else -(i // 4) * 13.0
            transform.transform.rotation.w = 1.0
            if not shared or i == 0:
                transforms.append(transform)
        self.tf = StaticTransformBroadcaster(self)
        self.tf.sendTransform(transforms)

    def sim_diagnostics(self, name, message):
        for status in message.status:
            self.states[name] = {item.key: item.value for item in status.values}

    def planner_diagnostics(self, name, message):
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            if values.get('solved') == 'true':
                self.ready.add(name)
            if self.started and values.get('solved') == 'false' and status.message != 'arrived':
                self.unsolved[name] += 1

    def spawn(self, command, log_name):
        log = (self.output / log_name).open('w')
        self.logs.append(log)
        child_env = dict(os.environ, ROS_LOG_DIR=str(self.output / 'ros_logs'))
        self.processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True, env=child_env))

    def launch_cases(self):
        simulator_files = []
        for case in self.cases:
            name = case['name']
            planner_path = self.output / f'{name}.planner.yaml'
            sim_path = self.output / f'{name}.sim.yaml'
            planner_path.write_text(yaml.safe_dump(case['planner']))
            sim_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': case['simulator']}}))
            simulator_files.append(str(sim_path))
            if not self.shared:
                self.spawn([executable('neupan_sim', 'neupan_sim_node'), '--ros-args',
                            '-r', f'__ns:=/{name}', '--params-file', str(sim_path)], f'{name}.sim.log')
            self.spawn([executable('neupan_ros', 'neupan_node'), '--ros-args',
                        '-r', f'__ns:=/{name}', '-p', f'config_file:={planner_path}',
                        '-p', f'dune_checkpoint:={case["checkpoint"]}',
                        '-p', f'planning_frame:={case["simulator"]["world_frame"]}', '-p', f'base_frame:={name}/base_link',
                        '-p', 'control_rate:=20.0', '-p', 'obstacle_source:=pointcloud',
                        '-p', 'pose_timeout:=1.0', '-p', 'pointcloud_timeout:=1.0'], f'{name}.planner.log')

        if self.shared:
            fleet_path = self.output / 'fleet.yaml'
            fleet_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
                'robot_names': [c['name'] for c in self.cases], 'simulator_files': simulator_files}}}))
            self.spawn([executable('neupan_sim', 'neupan_fleet_sim'), '--ros-args',
                        '--params-file', str(fleet_path)], 'fleet.sim.log')

    def wait_for(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if any(p.poll() is not None for p in self.processes):
                raise RuntimeError('A child process exited; see per-case logs')
            if predicate():
                return
        raise TimeoutError('Validation startup or execution exceeded wall-clock deadline')

    def run(self, startup_timeout):
        self.wait_for(lambda: all(c['name'] in self.ready or
                      self.states.get(c['name'], {}).get('result') in ('goal_reached', 'collision')
                      for c in self.cases) and
                      all(c.service_is_ready() for c in self.start_clients.values()), startup_timeout)
        requests = [c.call_async(Trigger.Request()) for c in self.start_clients.values()]
        self.wait_for(lambda: all(f.done() for f in requests), 10)
        if not all(f.result() and f.result().success for f in requests):
            raise RuntimeError('A simulator refused to start')
        self.started = True
        print(f'Started {len(self.cases)} robots; shared_world={self.shared}', flush=True)
        maximum_time = max(c['simulator'].get('simulation_timeout', 30) for c in self.cases)
        self.wait_for(lambda: len(self.states) == len(self.cases) and all(
            s.get('result') in ('goal_reached', 'collision', 'timed_out') for s in self.states.values()),
            maximum_time * 4 + 15)

    def report(self, error=None):
        rows = []
        for case in self.cases:
            name = case['name']
            state = self.states.get(name, {})
            result = state.get('result', 'not_started')
            if error and result in ('running', 'not_started'):
                result = 'infrastructure_error'
            rows.append(dict(case=name, robot=case['robot'], scenario=case['scenario'],
                             result=result, passed=not error and verdict(result, case['expected']) and
                             (not self.shared or state.get('peer_count') == str(len(self.cases) - 1)),
                             elapsed_time=finite_float(state.get('elapsed_time')),
                             path_length=finite_float(state.get('path_length')),
                             minimum_clearance=finite_float(state.get('minimum_clearance')),
                             goal_distance=finite_float(state.get('goal_distance')),
                             unsolved_samples=self.unsolved[name],
                             peer_count=int(state['peer_count']) if 'peer_count' in state else None,
                             total_peer_hits=int(state['total_peer_hits']) if 'total_peer_hits' in state else None))
        passed = not error and all(row['passed'] for row in rows)
        report = dict(passed=passed, error=error, cases=rows,
                      mode='shared' if self.shared else 'replicas',
                      note='Shared world with mutual sensing and collisions.' if self.shared else
                           'Independent scene copies; wall-clock timing includes concurrent CPU load.')
        (self.output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        with (self.output / 'summary.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        for row in rows:
            print(f'{row["case"]}: {row["result"]}, clearance={row["minimum_clearance"]}', flush=True)
        print(f'Report: {self.output / "summary.json"}', flush=True)
        return passed

    def close(self):
        for p in self.processes:
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for p in self.processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                p.wait()
        for log in self.logs:
            log.close()


def write_rviz(cases, output):
    displays = []
    for case in cases:
        name = case['name']
        displays.append({'Class': 'rviz_default_plugins/MarkerArray', 'Name': name, 'Enabled': True,
                         'Topic': {'Value': f'/{name}/neupan_sim/markers', 'Depth': 5,
                                   'Durability Policy': 'Transient Local', 'Reliability Policy': 'Reliable'}})
    document = {'Visualization Manager': {'Global Options': {'Fixed Frame': 'validation_world'},
                'Displays': displays, 'Views': {'Current': {'Class': 'rviz_default_plugins/TopDownOrtho',
                'Scale': 35 if cases[0]['simulator']['world_frame'] == 'shared_map' else 12,
                'X': 0 if cases[0]['simulator']['world_frame'] == 'shared_map' else 20,
                'Y': 0 if cases[0]['simulator']['world_frame'] == 'shared_map' else -10}}}}
    path = output / 'validation.rviz'
    path.write_text(yaml.safe_dump(document))
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', help='Suite YAML; defaults to replicas or shared suite according to --shared')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--case', action='append', dest='selected')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--shared', action='store_true', help='One world with interacting robot bodies')
    parser.add_argument('--scenario', help='Shared scenario name (all its robots participate)')
    parser.add_argument('--startup-timeout', type=float, default=30.0)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        parser.error('Output directory must be new or empty; existing reports are never overwritten')
    if args.shared and args.selected:
        parser.error('--case is for replicas; shared scenarios run all their robots')
    if args.scenario and not args.shared:
        parser.error('--scenario requires --shared')
    suite = args.suite or str(Path(get_package_share_directory('neupan_sim')) / 'config' /
                             ('shared_validation.yaml' if args.shared else 'validation.yaml'))
    cases = (load_shared_cases(suite, get_package_share_directory, args.scenario) if args.shared
             else load_cases(suite, get_package_share_directory, args.selected))
    output.mkdir(parents=True, exist_ok=True)
    (output / 'resolved.json').write_text(json.dumps(cases, indent=2))
    rviz_path = write_rviz(cases, output)
    rclpy.init(args=[])
    node = Validation(cases, output, args.shared)
    error = None
    try:
        node.launch_cases()
        if args.rviz:
            node.spawn([executable('rviz2', 'rviz2'), '-d', str(rviz_path)], 'rviz.log')
        node.run(args.startup_timeout)
    except (Exception, KeyboardInterrupt) as failure:
        error = str(failure) or type(failure).__name__
        print(error, file=sys.stderr)
    finally:
        try:
            passed = node.report(error)
        finally:
            node.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
