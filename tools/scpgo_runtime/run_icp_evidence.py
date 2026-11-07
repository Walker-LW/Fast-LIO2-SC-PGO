import json
from pathlib import Path
import subprocess
import sys
import numpy as np

run,out=map(Path,sys.argv[1:3])
prepared=json.loads((out/'prepared.json').read_text())
poses=np.loadtxt(run/'backend/odom_poses.txt').reshape(-1,3,4)
results=[]
for event in prepared['selected_events']:
    index=event['event']
    result=subprocess.run([str(out/'icp_evidence_probe'),str(out/('event-%03d-source.pcd'%index)),
        str(out/('event-%03d-target.pcd'%index))],text=True,capture_output=True)
    (out/('event-%03d-probe-stderr.txt'%index)).write_text(result.stderr)
    assert result.returncode==0,result.stderr
    measured=json.loads(result.stdout)
    assert abs(measured['fitness']-event['log_score'])<=max(abs(event['log_score'])*0.005,0.00001)
    previous,current=event['previous'],event['current']
    a,b=np.eye(4),np.eye(4)
    a[:3,:]=poses[previous]; b[:3,:]=poses[current]
    relative=np.linalg.inv(a)@b
    z=np.array(measured['original_backend_between_measurement']).reshape(4,4)
    error=np.linalg.inv(z)@relative
    # Translation norm gives a conservative Cauchy weight upper bound:
    # ||SE3 Log translation|| >= ||group translation||, all variances = 0.5.
    error_translation=float(np.linalg.norm(error[:3,3]))
    measured.update(event=event,measurement_translation_m=float(np.linalg.norm(z[:3,3])),
        odom_relative_translation_m=float(np.linalg.norm(relative[:3,3])),
        between_error_group_translation_m=error_translation,
        cauchy_block_weight_upper_bound_at_saved_odom=1/(1+error_translation**2/0.5),
        applicability='Weight bound is relevant only to accepted factors; diagnostic at saved original poses, not logged solver weights.')
    results.append(measured)
report=dict(status='RECORDED_ICP_INPUT_REPRODUCTION_PASS',results=results,
    limitations=['Only the predeclared three-event sample was replayed offline.',
        'Saved odometry matrices are rounded; relative translations and residuals are approximate.',
        'No GT, actual factor dump, or original online robust-weight log is available.',
        'No algorithm, parameter or original result changed.'])
(out/'icp-probe-results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
