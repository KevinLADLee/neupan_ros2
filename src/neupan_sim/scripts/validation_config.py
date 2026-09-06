"""Resolve independent validation cases without importing ROS."""
import copy
import math
from pathlib import Path
import re

import yaml


def merge(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def safe_name(name):
    if (not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]*', name)
            or '__' in name):
        raise ValueError(f'Invalid case identifier: {name!r}')
    return name


def footprint(robot):
    if robot.get('vertices') is not None:
        return [float(value) for point in robot['vertices'] for value in point]
    length, width, axle = float(robot['length']), float(robot['width']), float(robot.get('wheelbase', 0))
    x, y = -(length - axle) / 2, -width / 2
    return [x, y, x + length, y, x + length, y + width, x, y + width]


def load_cases(path, resolve_package, selected=None):
    path = Path(path).resolve()
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or not document.get('robots') or not document.get('scenarios'):
        raise ValueError('Suite needs nonempty robots and scenarios mappings')

    def resolve(value):
        if value.startswith('package://'):
            package, suffix = value[len('package://'):].split('/', 1)
            result = Path(resolve_package(package)) / suffix
        else:
            result = path.parent / value
        if not result.is_file():
            raise ValueError(f'Missing validation input: {result}')
        return str(result.resolve())

    cases = []
    for robot_name, spec in document['robots'].items():
        safe_name(robot_name)
        planner = yaml.safe_load(Path(resolve(spec['planner'])).read_text())
        planner = merge(planner, document.get('planner_overrides', {}))
        planner = merge(planner, spec.get('overrides', {}))
        model = resolve(spec['checkpoint'])
        for scenario_name, scenario in document['scenarios'].items():
            safe_name(scenario_name)
            name = f'{robot_name}_{scenario_name}'
            if selected and name not in selected:
                continue
            config = merge(planner, scenario.get('planner_overrides', {}))
            sim = merge(document.get('simulator', {}), scenario.get('simulator', {}))
            # Geometry and limits are derived once, never maintained in two YAMLs.
            robot = config['robot']
            sim.update(robot_vertices=footprint(robot),
                       speed_limits=robot['max_speed'], acceleration_limits=robot['max_acce'],
                       scenario_name=name, start_paused=True,
                       world_frame=f'{name}/map', base_frame=f'{name}/base_link',
                       laser_frame=f'{name}/laser_link')
            cases.append(dict(name=name, robot=robot_name, scenario=scenario_name,
                              planner=config, checkpoint=model, simulator=sim,
                              expected=scenario.get('expected', ['goal_reached'])))
    if len({c['name'] for c in cases}) != len(cases):
        raise ValueError('Case identifiers must be unique')
    if not cases or (selected and set(selected) - {c['name'] for c in cases}):
        raise ValueError('No cases selected, or unknown case names')
    return cases


def verdict(result, expected):
    return result in expected


def finite_float(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError):
        return None


def load_shared_cases(path, resolve_package, scenario_name=None):
    """Resolve one shared world; every robot is part of the same test outcome."""
    path = Path(path).resolve()
    document = yaml.safe_load(path.read_text())
    scenarios = document['scenarios']
    if scenario_name is None:
        if len(scenarios) != 1:
            raise ValueError('Select one shared scenario with --scenario')
        scenario_name = next(iter(scenarios))
    safe_name(scenario_name)
    if scenario_name not in scenarios:
        raise ValueError(f'Unknown shared scenario: {scenario_name}')
    scenario = scenarios[scenario_name]
    members = scenario['robots']
    if len(members) < 2:
        raise ValueError('A shared scenario needs at least two robots')
    world = merge(document.get('simulator', {}), scenario.get('simulator', {}))

    def resolve(value):
        if value.startswith('package://'):
            package, suffix = value[len('package://'):].split('/', 1)
            result = Path(resolve_package(package)) / suffix
        else:
            result = path.parent / value
        if not result.is_file():
            raise ValueError(f'Missing validation input: {result}')
        return str(result.resolve())

    cases = []
    for name, member in members.items():
        safe_name(name)
        profile = member['profile']
        spec = document['robots'][profile]
        config = yaml.safe_load(Path(resolve(spec['planner'])).read_text())
        for override in (document.get('planner_overrides', {}), spec.get('overrides', {}),
                         scenario.get('planner_overrides', {}), member.get('planner_overrides', {})):
            config = merge(config, override)
        if config['robot'].get('kinematics', 'diff') != 'diff':
            raise ValueError('Shared simulation currently supports differential drive only')
        sim = merge(world, dict(initial_pose=member['initial_pose'], path_waypoints=member['path_waypoints']))
        robot = config['robot']
        sim.update(robot_vertices=footprint(robot), speed_limits=robot['max_speed'],
                   acceleration_limits=robot['max_acce'], scenario_name=f'{scenario_name}/{name}',
                   start_paused=True, world_frame='shared_map', base_frame=f'{name}/base_link',
                   laser_frame=f'{name}/laser_link')
        cases.append(dict(name=name, robot=profile, scenario=scenario_name, planner=config,
                          checkpoint=resolve(spec['checkpoint']), simulator=sim,
                          expected=['goal_reached']))
    return cases
