"""Check actual ROS graph and parameters before permitting replay."""
import json
from pathlib import Path
import yaml
import rosgraph
master=rosgraph.Master('/preplay_check')
pubs,subs,_=master.getSystemState()
pubs=dict(pubs);subs=dict(subs)
nodes=set(n for entries in [pubs,subs] for names in entries.values() for n in names)
for name in ['/laserMapping','/record_pose','/record_cloud']:
    if name not in nodes: raise RuntimeError('Missing node '+name)
if not any(n=='/corridor_rviz' or n.startswith('/corridor_rviz_') for n in nodes):
    raise RuntimeError('Missing RViz subscriber')
for topic in ['/Odometry','/cloud_registered','/path']:
    if '/laserMapping' not in pubs.get(topic,[]): raise RuntimeError('Missing algorithm publisher '+topic)
for topic,node in [('/Odometry','/record_pose'),('/cloud_registered','/record_cloud'),
                   ('/livox/avia/lidar','/laserMapping'),('/livox/avia/imu','/laserMapping')]:
    if node not in subs.get(topic,[]): raise RuntimeError('Missing subscription '+topic+' '+node)
policy=json.loads((Path(__file__).parent/'config/output-policy.json').read_text())
expected={'/preprocess/blind':float(policy.get('blind',1.0)),'/publish/path_en':True,'/pcd_save/pcd_save_en':policy['online_pcd'],
          '/pcd_save/interval':-1,
          '/use_sim_time':True,'/publish/dense_publish_en':True,
          '/common/lid_topic':'/livox/avia/lidar','/common/imu_topic':'/livox/avia/imu'}
for key,value in expected.items():
    actual=master.getParam(key)
    if actual!=value: raise RuntimeError('Parameter mismatch '+key+' '+repr(actual))
reference=Path(__file__).parent/'config/reference-blind1-runtime.yaml'
if reference.exists():
    baseline=yaml.safe_load(reference.read_text())
    differences=[]
    keys=['common','cube_side_length','feature_extract_enable','filter_size_map','filter_size_surf',
          'mapping','max_iteration','point_filter_num','publish','runtime_pos_log_enable','use_sim_time','pcd_save']
    for key in keys:
        actual=master.getParam('/'+key)
        if actual!=baseline[key]:differences.append(dict(parameter=key,reference=baseline[key],actual=actual))
    pre=dict(baseline['preprocess']);pre['blind']=float(policy['blind'])
    if master.getParam('/preprocess')!=pre: differences.append(dict(parameter='preprocess_except_planned_blind'))
    result=dict(status='MATCH_EXCEPT_BLIND' if not differences else 'MISMATCH',differences=differences,
                planned_change={'preprocess/blind':[baseline['preprocess']['blind'],float(policy['blind'])]},
                limits='Checks selected runtime configuration groups, not hardware load, repeatability or GT accuracy.')
    with (Path(__file__).parent/'checks/reference-config-comparison.json').open('x') as f:json.dump(result,f,indent=2)
    if differences:raise RuntimeError('Unexpected configuration differences; inspect reference-config-comparison.json')
    print('PASS: reference blind=1 runtime groups match except planned blind change.')
print('PASS: nodes, actual subscriptions/publishers and parameters. Input remains paused until user starts it.')
print('NOTE: /tf publisher is created on first odometry publication; verify TF after input starts and in recorded output.')
print('Coverage observer: '+('present (optional)' if '/coverage_observer' in nodes else 'omitted (optional); review input/output coverage after recording'))
