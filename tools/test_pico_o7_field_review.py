import csv
import json
from pathlib import Path
import tempfile
import unittest

from pico_o7_field_review import report, window_metrics


class ReviewTests(unittest.TestCase):
    def test_only_sent_intervals_but_failed_cycles_are_retained(self):
        common=dict(timestamp_ns='100',command_duration_ms='1',command_interval_sec='0.005',
                    control_rx_interval_sec='0.005',feedback_age_sec='0.004',fault_latched='False',
                    stop_command_sent='False',reason='sent',command_sent='True',command_attempted='True')
        failed=dict(common, timestamp_ns='101',command_sent='False',reason='MOCK_FAILURE',command_interval_sec='10')
        result=window_metrics([common,failed],[],0,200)
        self.assertEqual(result['canfd_sent'],1)
        self.assertEqual(result['canfd_failed'],1)
        self.assertEqual(result['canfd_interval_sec']['max'],.005)
        self.assertEqual(result['canfd_not_sent_reasons'],{'MOCK_FAILURE':1})
        self.assertIsNone(result['actual_modbus_write_hz'])

    def test_exact_write_timing_and_raw_code_and_disabled_window(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); (root/'capture').mkdir()
            (root/'capture/samples.csv').write_text('record_type,timestamp_ns\n')
            def call(seq,start):
                return dict(event='O7_HAND_WRITE_FINISH',timestamp_ns=start+100,command_sequence=seq,
                            call_start_timestamp_ns=start,call_start_monotonic_ns=start,
                            duration_ms=224.,return_code=1,error=None,target_positions=[255]*7)
            events=[dict(event='O7_HAND_WRITE_START',timestamp_ns=100),call(1,100),
                    dict(event='O7_HAND_STATUS',timestamp_ns=300,enabled=False,reason='operator_disabled'),
                    dict(event='O7_HAND_WRITE_START',timestamp_ns=500),call(2,500)]
            with (root/'capture/events.csv').open('w') as stream:
                writer=csv.DictWriter(stream,fieldnames=['payload_json']);writer.writeheader()
                writer.writerows({'payload_json':json.dumps(e)} for e in events)
            result=report(root)
            self.assertEqual(result['overall']['raw_return_codes'],{'1':2})
            self.assertEqual(result['overall']['modbus_duration_ms']['avg'],224.)
            self.assertEqual(result['blocked_status_observations'][0]['new_native_call_starts'],1)
            self.assertEqual(result['unmatched_start_count'],0)
            self.assertEqual(result['physical_motion'],'REQUIRES_OPERATOR_OBSERVATION')
            self.assertTrue((root/'pico_o7_review.json').exists())

    def test_inflight_finish_after_disable_is_not_new_submission(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); (root/'capture').mkdir()
            (root/'capture/samples.csv').write_text('record_type,timestamp_ns\n')
            events=[dict(event='O7_HAND_WRITE_START',timestamp_ns=100),
                    dict(event='O7_HAND_STATUS',timestamp_ns=200,enabled=False,reason='operator_disabled'),
                    dict(event='O7_HAND_WRITE_FINISH',timestamp_ns=400,command_sequence=1,
                         call_start_timestamp_ns=101,call_start_monotonic_ns=101,
                         duration_ms=224.,return_code=1,error=None,target_positions=[255]*7)]
            with (root/'capture/events.csv').open('w') as stream:
                writer=csv.DictWriter(stream,fieldnames=['payload_json']);writer.writeheader()
                writer.writerows({'payload_json':json.dumps(e)} for e in events)
            result=report(root)
            self.assertEqual(result['blocked_status_observations'][0]['new_native_call_starts'],0)


if __name__=='__main__': unittest.main()
