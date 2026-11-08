"""Recovery contract checks using a temporary checkpoint and mocked ROS/Docker."""
from contextlib import ExitStack
from pathlib import Path
import json
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch
import manage
import recover_finish


class RecoveryTests(unittest.TestCase):
    def exercise(self,frontend_present=False,changed_count=False):
        with tempfile.TemporaryDirectory() as d, ExitStack() as stack:
            base=Path(d);run_id='scpgo-corridor02-geometry-20261108-001';run=base/'results/scpgo'/run_id
            for folder in ['checks','config','raw','backend']:(run/folder).mkdir(parents=True)
            (run/'manage.py').write_text('original frozen source\n')
            (run/'backend/poses.txt').write_text('saved evidence\n')
            frozen={'manage.py':manage.sha(run/'manage.py')}
            (run/'config/manifest.json').write_text(json.dumps({'id':run_id,'image':'mock-image','frozen':frozen,'binaries':{}}))
            observer={'counts':{'lidar':2,'imu':10,'odom':2,'body':2},'exact_stamp_pairs':2,'errors':{},
                      'odom_without_body':0,'body_without_odom':0,'last':{'body':{'stamp_ns':1001}}}
            original=json.dumps({'observer':observer,'checked_wall_time_ns':time.time_ns()})
            (run/'checks/input-complete.json').write_text(original)
            current=json.loads(json.dumps(observer))
            if changed_count:current['counts']['body']=3
            (run/'checks/interface-observer.jsonl').write_text(json.dumps(current)+'\n')
            (run/'checks/end-before-stop.json').write_text(json.dumps({'saved_sha256':{'backend/poses.txt':manage.sha(run/'backend/poses.txt')}}))
            for name,value in [('BASE',base),('HERE',base/'tool'),('ID',manage.ID),('RUN',manage.RUN)]:stack.enter_context(patch.object(manage,name,value))
            stack.enter_context(patch.object(manage,'call',return_value=json.dumps([{'State':{'Running':True},'Mounts':[{'Destination':'/run/scpgo','Source':str(run)}],'Image':'mock-image'}])))
            stack.enter_context(patch.object(manage,'node_present',side_effect=lambda name:frontend_present if name=='/laserMapping' else True))
            stop=stack.enter_context(patch.object(manage,'stop_node'))
            command=stack.enter_context(patch.object(recover_finish.subprocess,'run',return_value=subprocess.CompletedProcess([],0)))
            if frontend_present or changed_count:
                with self.assertRaises(AssertionError):recover_finish.recover(run_id)
                stop.assert_not_called();command.assert_not_called()
                self.assertFalse((run/'checks/closed.json').exists())
            else:
                recover_finish.recover(run_id)
                result=json.loads((run/'checks/closed.json').read_text())
                self.assertEqual(result['saved_files_changed_at_stop'],[])
                self.assertEqual([c.args[0] for c in stop.call_args_list],['/scpgo/alaserPGO','/scpgo_interface_watch','/scpgo_record','/scpgo_input'])
            self.assertEqual((run/'checks/input-complete.json').read_text(),original)
            self.assertEqual(manage.sha(run/'manage.py'),frozen['manage.py'])

    def test_resumes_without_rewriting_original_checkpoint_or_source(self):self.exercise()
    def test_live_frontend_is_not_stopped_by_recovery(self):self.exercise(frontend_present=True)
    def test_changed_input_is_not_closed(self):self.exercise(changed_count=True)


if __name__=='__main__':unittest.main()
