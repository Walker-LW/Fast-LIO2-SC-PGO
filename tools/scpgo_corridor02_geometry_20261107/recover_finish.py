"""Resume only the known failed shutdown; preserve the prepared run's code/evidence."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import manage


def recover(run_id):
    manage.select_run_id(run_id)
    run=manage.RUN
    assert run.is_dir() and (run/'checks/input-complete.json').is_file(), 'No completed input checkpoint'
    assert not (run/'checks/closed.json').exists(), 'Already closed; do not repeat recovery'
    manifest=json.loads((run/'config/manifest.json').read_text())
    assert manifest['id']==run_id
    info=json.loads(manage.call(['docker','inspect',run_id]))[0]
    assert info['State']['Running'], 'Container not running; preserve state and inspect'
    mounts={x['Destination']:x['Source'] for x in info['Mounts']}
    assert Path(mounts['/run/scpgo'])==run and info['Image']==manifest['image']
    for rel,digest in manifest['frozen'].items():
        assert manage.sha(run/rel)==digest, 'Frozen source/config changed: '+rel
    for rel,digest in manifest['binaries'].items():assert manage.sha(manage.WS/rel)==digest
    assert not manage.node_present('/laserMapping'), 'Frontend still registered; recovery is only for an already stopped frontend'
    assert manage.node_present('/scpgo_input'), 'Keep-alive player missing; preserve state'
    checkpoint=json.loads((run/'checks/input-complete.json').read_text())
    previous=checkpoint['observer']
    current=json.loads((run/'checks/interface-observer.jsonl').read_text().splitlines()[-1])
    for key in ['lidar','imu','odom','body']:
        assert current['counts'][key]==previous['counts'][key], ('Input changed after frontend shutdown',key)
    assert not current['errors'] and current['exact_stamp_pairs']==previous['exact_stamp_pairs']
    assert current['odom_without_body']==current['body_without_odom']==0
    assert current['last']['body']['stamp_ns']==previous['last']['body']['stamp_ns']
    stamp=str(time.time_ns())
    recovery=dict(reason='Original stop check used rosnode info exit code; frontend already exited cleanly but was misclassified.',
        recovery_started_wall_time_ns=time.time_ns(),original_input_check_reused=True,
        frozen_run_sources_preserved=True,recovery_script_sha256=manage.sha(Path(__file__)),
        additional_tail_since_original_input_check_s=(time.time_ns()-checkpoint['checked_wall_time_ns'])/1e9,
        note='Keep-alive continued during diagnosis. Extra repeated backend events are retained and must not be treated as independent loops.')
    manage.write(run/'checks'/('recovery-finish-'+stamp+'.json'),recovery)
    # Reuse the existing successful input checkpoint. Never re-run input-complete.
    saved=run/'checks/end-before-stop.json'
    if saved.exists():
        for rel,digest in json.loads(saved.read_text())['saved_sha256'].items():assert manage.sha(run/rel)==digest
    else:
        subprocess.run(manage.ros('python3','/run/scpgo/runtime.py','save-check'),check=True)
    print('RECOVERY_PRECHECK_PASS: frontend absent, input checkpoint retained, frozen sources unchanged',flush=True)
    for node in ['/scpgo/alaserPGO','/scpgo_interface_watch','/scpgo_record']:
        manage.stop_node(node)
    for _ in range(30):
        if not list((run/'raw').glob('*.active')):break
        time.sleep(1)
    assert not list((run/'raw').glob('*.active')), 'Recorder not closed; preserve clock and container'
    manage.stop_node('/scpgo_input')
    subprocess.run(['docker','stop','-t','20',run_id],check=True)
    before=json.loads(saved.read_text())
    changed=[rel for rel,digest in before['saved_sha256'].items() if manage.sha(run/rel)!=digest]
    manage.write(run/'checks/closed.json',dict(no_active_bag=True,saved_files_changed_at_stop=changed,
        closed_wall_time_ns=time.time_ns(),recovery=recovery,
        note='Recovery closes existing capture only; backend shutdown/factor-consumption warnings remain subject to audit.'))
    assert not changed, 'Files changed at shutdown; keep all outputs and inspect before export'
    print('RECOVERED_CLOSED_AND_FROZEN_PASS')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--id',required=True)
    recover(p.parse_args().id)
