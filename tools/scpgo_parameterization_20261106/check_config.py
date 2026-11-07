"""Exercise the actual binary's startup-only validation; no data or SLAM workers."""
import os, subprocess, time, socket, json, re
from pathlib import Path
import rosparam

out = Path('/checks')
env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:11311', ROS_IP='127.0.0.1', ROS_LOG_DIR='/tmp/roslogs')
os.environ.update(env)
masterlog = (out/'config-master.log').open('w')
master = subprocess.Popen(['roscore'], env=env, stdout=masterlog, stderr=subprocess.STDOUT)
results=[]
try:
    for _ in range(100):
        try:
            with socket.create_connection(('127.0.0.1',11311),timeout=.2): break
        except OSError: time.sleep(.1)
    else: raise RuntimeError('ROS master did not start')
    approved = rosparam.load_file('/tools/corridor01-indoor.yaml')[0][0]
    defaults = json.loads(Path('/tools/parameter-defaults.json').read_text())
    cases = [('approved', approved, 0), ('defaults', {}, 0),
             ('override', dict(approved, icp_voxel_size=.2, sc_num_rings=12, sc_num_sectors=72, sc_lidar_height=0., sc_tree_period=5),0),
             ('invalid_voxel', dict(approved, icp_voxel_size=0.),2),
             ('invalid_period', dict(approved, sc_tree_period=0),2),
             ('invalid_radius', dict(approved, sc_max_radius=-1.),2)]
    for name,values,expected in cases:
        subprocess.run(['rosparam','delete','/scpgo'],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        values=dict(values,validate_config_only=True,save_directory='/checks/protected-output/')
        rosparam.upload_params('/scpgo',values)
        p=subprocess.run(['/ws/devel_scpgo_params/lib/aloam_velodyne/alaserPGO','__ns:=/scpgo'],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=30)
        (out/(name+'.log')).write_text(p.stdout)
        assert p.returncode==expected,(name,p.returncode,p.stdout)
        if expected==0:
            assert 'Configuration valid;' in p.stdout
            for key in defaults: assert 'backend parameter '+key+'=' in p.stdout,key
            if name=='defaults':
                for key,value in defaults.items():
                    match=re.search(r'backend parameter '+key+r'=([^\s\x1b]+)',p.stdout)
                    assert float(match.group(1))==value,(key,match.group(1),value)
            if name=='approved':
                assert 'keyframe_meter_gap=1 keyframe_deg_gap=10 sc_dist_thres=0.2 sc_max_radius=40 mapviz_filter_size=0.2' in p.stdout
            if name=='override': assert 'backend parameter icp_voxel_size=0.2' in p.stdout
        assert not (out/'protected-output').exists(), 'Validation must not create/clean output'
        results.append({'case':name,'exit_code':p.returncode,'pass':True})
    p=subprocess.run(['roslaunch','/tools/backend-parameterized.launch',
                      'run_dir:=/checks/protected-run','validate_only:=true'],
                     env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=60)
    (out/'launch-validation.log').write_text(p.stdout)
    assert p.returncode==0 and 'Configuration valid;' in p.stdout,p.stdout
    assert not (out/'protected-run').exists()
    results.append({'case':'actual_launch_and_executable_resolution','exit_code':p.returncode,'pass':True})
    (out/'config-checks.json').write_text(json.dumps({'cases':results,'no_output_directory_created':True,'no_lidar_or_imu_published':True},indent=2)+'\n')
    print(json.dumps(results))
finally:
    master.terminate()
    try: master.wait(timeout=15)
    except subprocess.TimeoutExpired: master.kill(); master.wait()
    masterlog.close()
