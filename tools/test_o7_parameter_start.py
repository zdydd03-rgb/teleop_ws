"""Launch-parameter hand execution tests, with no ROS/SDK connection."""
import ast
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch

from test_pico_o7_combined import CANDIDATE, PROJECT, realtime, Mapper


class ParameterStartTests(unittest.TestCase):
    def setUp(self):
        self.reason=[''];self.events=[];self.calls=[];self.threads=[];self.controllers=[]

    def tearDown(self):
        for controller in self.controllers:controller.shutdown()
        for thread in self.threads:
            thread.join(1);self.assertFalse(thread.is_alive())

    def controller(self, selected=True):
        from types import SimpleNamespace
        def write(target):self.calls.append(list(target));return 0
        with patch.object(realtime.HandController,'_solve_loop',lambda self:None):
            adapter=SimpleNamespace(write_o7_hil_positions=write)
            controller=realtime.HandController(Mapper(),lambda:adapter,
                lambda:self.reason[0],self.events.append,start_from_parameter=selected,
                initializer=lambda *a:None if self.reason[0]=='existing API2 session not connected' else dict(success=True))
        self.controllers.append(controller)
        thread=threading.Thread(target=controller.write_loop)
        thread.start();self.threads.append(thread)
        return controller

    def wait(self,predicate):
        deadline=time.monotonic()+1
        while not predicate() and time.monotonic()<deadline:time.sleep(.005)
        self.assertTrue(predicate())

    def offer(self,controller,fraction):
        state=controller.state;now=time.monotonic()
        with state.lock:
            state.last_received=now
            q={name:lo+fraction*(hi-lo) for name,(lo,hi) in state.mapper.bounds.items()}
            state.pending=(state.epoch,now,dict(seq=1,valid=True,q_by_name=q))

    def test_parameter_false_does_not_start_or_write(self):
        c=self.controller(False);time.sleep(.03)
        self.assertFalse(c.state.enabled);self.assertFalse(c.state.startup_pending)
        self.assertEqual(self.calls,[])

    def test_parameter_true_waits_for_connection_then_starts_without_service(self):
        self.reason[0]='existing API2 session not connected'
        c=self.controller()
        self.wait(lambda:c.state.startup_pending)
        self.assertFalse(c.state.enabled);self.assertTrue(c.state.startup_pending)
        self.reason[0]=''
        self.wait(lambda:c.state.enabled)
        self.assertFalse(c.state.startup_pending)
        self.assertEqual(self.calls,[])  # Safe readiness is not a position command.
        self.wait(lambda:any(e['event']=='O7_HAND_PARAMETER_START' for e in self.events))

    def test_combined_waits_only_for_shared_connection(self):
        self.reason[0]='existing API2 session not connected'
        c=self.controller();time.sleep(.03)
        self.assertFalse(c.state.enabled);self.assertTrue(c.state.startup_pending)
        self.reason[0]='';self.wait(lambda:c.state.enabled)

    def test_startup_discards_pre_readiness_solution(self):
        self.reason[0]='existing API2 session not connected'
        c=self.controller();self.offer(c,.5)
        self.reason[0]='';self.wait(lambda:c.state.enabled)
        time.sleep(.03)
        self.assertIsNone(c.state.pending);self.assertEqual(self.calls,[])
        self.offer(c,.5);self.wait(lambda:len(self.calls)==1)

    def test_acknowledged_writes_continue_on_new_targets(self):
        c=self.controller();self.wait(lambda:c.state.enabled)
        self.offer(c,.5);self.wait(lambda:len(self.calls)==1)
        time.sleep(.26);self.offer(c,.7);self.wait(lambda:len(self.calls)==2)
        self.wait(lambda:c.state.return_codes=={'0':2})
        self.assertTrue(c.state.enabled)
        self.assertTrue(all(e['return_code']==0 for e in self.events if e['event']=='O7_HAND_WRITE_FINISH'))

    def test_safety_stop_requires_restart_not_service_or_auto_retry(self):
        c=self.controller();self.wait(lambda:c.state.enabled)
        self.reason[0]='node_closing';self.wait(lambda:not c.state.enabled)
        self.reason[0]='';time.sleep(.03)
        self.assertFalse(c.state.enabled);self.assertFalse(c.state.startup_pending)
        self.assertEqual(self.calls,[])

    def test_shutdown_cancels_pending_start(self):
        self.reason[0]='existing API2 session not connected';c=self.controller()
        c.shutdown();self.reason[0]='';time.sleep(.02)
        self.assertFalse(c.state.startup_pending);self.assertFalse(c.state.enabled)
        self.assertEqual(self.calls,[])

    def test_node_master_parameter_and_removed_control_services(self):
        tree=ast.parse((CANDIDATE/'shared_session_node.py').read_text())
        names={n.name for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
        self.assertIn('_hand_enable',names);self.assertIn('_hand_disable',names)
        self.assertIn('_hand_status',names)
        branch=next(n for n in ast.walk(tree) if isinstance(n,ast.If)
                    and ast.unparse(n.test)=='self.enable_o7_retargeting')
        constructor=next(n for n in ast.walk(branch) if isinstance(n,ast.Call)
                         and isinstance(n.func,ast.Name) and n.func.id=='HandController')
        self.assertTrue(next(k.value.value for k in constructor.keywords if k.arg=='start_from_parameter'))

    def test_removed_cli_controls(self):
        field=(PROJECT.parents[1]/'tools/o7_field.sh').read_text()
        cli=(CANDIDATE/'session_tools.py').read_text()
        for old in ('hand-enable','hand-disable','--enable-hand-writes'):
            self.assertNotIn(old,field);self.assertNotIn(old,cli)


if __name__=='__main__':unittest.main(verbosity=2)
