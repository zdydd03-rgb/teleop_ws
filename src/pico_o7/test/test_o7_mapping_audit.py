"""Offline mapping checks. SDK/ROS modules are never imported or instantiated."""
import ast
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent/'rm_teleop/rm_teleop'


def extract_definition(path, name, env=None):
    """Compile only the inspected pure definition, never the module's imports/main."""
    tree = ast.parse(Path(path).read_text())
    node = next(n for n in ast.walk(tree) if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name == name)
    namespace = dict(__name__=__name__, dataclass=dataclass, math=math)
    namespace.update(env or {})
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class MappingAuditChecks(unittest.TestCase):
    def test_actual_adapter_preserves_array_order_and_return_code(self):
        write = extract_definition(SRC/'rm_api2_motion_adapter.py', 'write_o7_hil_positions')
        calls = []
        arm = SimpleNamespace(rm_write_registers=lambda params, data: (calls.append((vars(params), data)), 1)[1])
        adapter = SimpleNamespace(_require_arm=lambda:arm,
            module=SimpleNamespace(rm_peripheral_read_write_params_t=lambda **kw:SimpleNamespace(**kw)))
        # These are mock-only test markers, not hardware/calibration targets.
        markers = [11,22,33,44,55,66,77]
        self.assertEqual(write(adapter, markers), 1)
        self.assertEqual(calls, [(dict(port=1,address=0,device=39,num=7), [b for v in markers for b in (0,v)])])
        for invalid in ([1]*6, [1]*8, [True]*7, [1.]*7, [-1]*7, [256]*7):
            with self.assertRaises(ValueError):write(adapter,invalid)
        self.assertEqual(len(calls),1)

    def test_current_shared_hil_rejects_other_axes(self):
        Config = extract_definition(SRC/'o7_single_session_hil.py','StreamConfig')
        Config(duration_sec=2.,joint=2,open_pose_confirmed=True).validate()
        for i in [0,1,3,4,5,6]:
            with self.assertRaises(ValueError):Config(joint=i,open_pose_confirmed=True).validate()
        with self.assertRaises(ValueError):Config(open_pose_confirmed=False).validate()

    def test_auto_generator_only_varies_index(self):
        triangle = extract_definition(SRC/'o7_auto_stream_hil.py','triangle_positions')
        for t in [0.,.2,.5,1.,1.5,2.]:
            values=triangle(t)
            self.assertTrue(220<=values[2]<=255)
            self.assertTrue(all(values[i]==255 for i in [0,1,3,4,5,6]))
        self.assertEqual(triangle(0.)[2],255)
        self.assertEqual(triangle(1.)[2],220)

    def test_evidence_does_not_promote_unconfirmed_sdk_calls(self):
        logs=json.loads((ROOT/'calibration/hil_log_audit.json').read_text())['retained']
        self.assertEqual([x['position_calls']for x in logs],[10,61])
        self.assertEqual(sum(x['sdk_acknowledged_count']for x in logs),0)
        self.assertEqual(sum(x['sdk_unconfirmed_count']for x in logs),71)
        for run in logs:
            self.assertEqual(run['changed_target_indices'],[2])
            self.assertEqual(run['sdk_acknowledged_target_values_by_channel'],[[]]*7)

    def test_user_endpoints_not_claimed_as_measured_calibration(self):
        table=json.loads((ROOT/'calibration/o7_axis_mapping.json').read_text())
        self.assertEqual(len(table['axes']),7)
        for row in table['axes']:
            self.assertEqual(row['declared_device_closed_value'],0)
            self.assertEqual(row['declared_device_open_value'],255)
            for key in ['confirmed_model_to_device_sign','confirmed_model_zero_device_value',
                        'measured_device_position_min','measured_device_position_max',
                        'mechanical_endpoint_min','mechanical_endpoint_max',
                        'gain_device_units_per_rad','offset_device_units','current_session_confirmed_hold_value']:
                self.assertIsNone(row[key])
            if row['device_index']==2:
                self.assertTrue(row['physical_axis_observed'])
                self.assertEqual([row['isolated_motion_test_command_min'],row['isolated_motion_test_command_max']],[220,255])
            else:
                self.assertFalse(row['physical_axis_observed'])
                self.assertIsNone(row['isolated_motion_test_command_min'])
                self.assertEqual(row['retained_command_values'],[255])
        with (ROOT/'calibration/o7_axis_mapping.csv').open() as stream:
            rows=list(csv.DictReader(stream))
        self.assertTrue(all(r['confirmed_model_to_device_sign']==''for r in rows))

    def test_permutation_keeps_names_units_and_separates_zero(self):
        table=json.loads((ROOT/'calibration/o7_axis_mapping.json').read_text())
        order=table['model']['independent_order'];indices=table['csv_to_device_order_indices']
        self.assertEqual(indices,[2,1,3,4,5,6,0])
        self.assertEqual([order[i]for i in indices],[r['model_joint']for r in table['axes']])
        for row in table['axes']:
            self.assertEqual(row['model_unit'],'rad');self.assertEqual(row['model_zero_rad'],0.)
            self.assertIn('not a calibrated',row['model_zero_definition'])
        pitch=table['axes'][2]
        self.assertAlmostEqual(pitch['model_solver_upper_rad'],1.8317/1.3462,places=14)
        self.assertLess(pitch['model_solver_upper_rad'],pitch['model_vendor_upper_rad'])

if __name__=='__main__':unittest.main(verbosity=2)
