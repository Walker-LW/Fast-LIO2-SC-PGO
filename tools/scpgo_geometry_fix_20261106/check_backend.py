import os,subprocess,time,socket,json,re
from pathlib import Path
import rosparam
from roslib.packages import find_node

out=Path('/fix/config-checks');out.mkdir(exist_ok=True)
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11311',ROS_IP='127.0.0.1',ROS_LOG_DIR='/tmp/roslogs',ROS_HOME='/tmp/geometry-ros')
log=(out/'master.log').open('w')
master=subprocess.Popen(['roscore'],stdout=log,stderr=subprocess.STDOUT)
try:
    for _ in range(100):
        try:
            with socket.create_connection(('127.0.0.1',11311),timeout=.2):break
        except OSError:time.sleep(.1)
    else:raise RuntimeError('Master did not start')
    expected='/ws/devel_scpgo_geometry/lib/aloam_velodyne/alaserPGO'
    nodes=find_node('aloam_velodyne','alaserPGO');assert nodes==[expected],nodes
    cfg='/ws/src/M3DGR/deployment-scpgo/runtime/corridor01-indoor.yaml'
    profile=rosparam.load_file(cfg)[0][0]
    approved=dict(profile,validate_config_only=True,save_directory='/fix/config-checks/must-not-be-created/')
    rosparam.upload_params('/scpgo',approved)
    proc=subprocess.run([expected,'__ns:=/scpgo'],capture_output=True,text=True,timeout=30)
    (out/'binary.log').write_text(proc.stdout+proc.stderr)
    assert proc.returncode==0 and 'Configuration valid;' in proc.stdout
    defaults=json.loads(Path('/defaults/parameter-defaults.json').read_text())
    for key,value in defaults.items():
        m=re.search(r'backend parameter '+key+r'=([^\s\x1b]+)',proc.stdout)
        assert m and float(m[1])==float(profile.get(key,value)),(key,m[1] if m else None)
    assert 'keyframe_meter_gap=1 keyframe_deg_gap=10 sc_dist_thres=0.2 sc_max_radius=40 mapviz_filter_size=0.2' in proc.stdout
    proc=subprocess.run(['roslaunch','/defaults/backend-parameterized.launch','run_dir:=/fix/config-checks/must-not-be-created','validate_only:=true'],capture_output=True,text=True,timeout=60)
    (out/'launch.log').write_text(proc.stdout+proc.stderr)
    assert proc.returncode==0 and 'Configuration valid;' in proc.stdout
    assert not (out/'must-not-be-created').exists()
    report={'status':'PASS','resolved_binary':nodes,'approved_parameters_preserved':True,'actual_launch_validation':True,'no_output_directory_created':True,'data_replayed':False}
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
finally:
    master.terminate()
    try:master.wait(timeout=15)
    except subprocess.TimeoutExpired:master.kill();master.wait()
    log.close()
