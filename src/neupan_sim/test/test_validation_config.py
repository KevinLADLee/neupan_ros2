import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml

spec = importlib.util.spec_from_file_location('validation_config', Path(__file__).parents[1] / 'scripts/validation_config.py')
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)


class ValidationConfigTests(unittest.TestCase):
    def test_geometry_precedence_and_axle_offset(self):
        self.assertEqual(config.footprint({'vertices': [[0, 0], [1, 0], [0, 1]], 'length': 99}),
                         [0, 0, 1, 0, 0, 1])
        self.assertEqual(config.footprint({'length': 2, 'width': 1, 'wheelbase': .4}),
                         [-.8, -.5, 1.2, -.5, 1.2, .5, -.8, .5])

    def test_cartesian_product_is_isolated_and_models_are_required(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'model.bin').write_bytes(b'model')
            (root / 'planner.yaml').write_text(yaml.safe_dump({'robot': {
                'length': .5, 'width': .4, 'max_speed': [1., 1.], 'max_acce': [1., 2.]}}))
            profile = {'planner': 'planner.yaml', 'checkpoint': 'model.bin'}
            suite = {'robots': {'one': profile, 'two': profile},
                     'scenarios': {'clear': {'simulator': {}}, 'wall': {'simulator': {'segments': [1., 0., 1., 1.]}}}}
            path = root / 'suite.yaml'
            path.write_text(yaml.safe_dump(suite))
            cases = config.load_cases(path, lambda _: root)
            self.assertEqual(len(cases), 4)
            self.assertEqual(len({case['simulator']['base_frame'] for case in cases}), 4)
            self.assertNotIn('segments', cases[0]['simulator'])
            self.assertEqual(len(config.load_cases(path, lambda _: root, ['one_clear'])), 1)
            with self.assertRaises(ValueError):
                config.load_cases(path, lambda _: root, ['missing'])
            (root / 'model.bin').unlink()
            with self.assertRaisesRegex(ValueError, 'Missing validation input'):
                config.load_cases(path, lambda _: root)

    def test_result_does_not_pass_collision_or_infrastructure_errors(self):
        for result in ('collision', 'timed_out', 'infrastructure_error', 'running'):
            self.assertFalse(config.verdict(result, ['goal_reached']))
        self.assertTrue(config.verdict('timed_out', ['timed_out']))
        self.assertIsNone(config.finite_float('inf'))
        self.assertIsNone(config.finite_float(None))


if __name__ == '__main__':
    unittest.main()
