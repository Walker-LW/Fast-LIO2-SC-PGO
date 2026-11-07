"""Static ROS config checks and synthetic file export checks; no replay/nodes."""
import ast,json,pathlib,subprocess,tempfile,sys,types
import numpy as np
import rosbag,rospy,roslaunch.config,yaml
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs.point_cloud2 import create_cloud
root=pathlib.Path('/tools')
for path in root.glob('*.py'): ast.parse(path.read_text())
with tempfile.TemporaryDirectory() as directory:
    r=pathlib.Path(directory)
    text=(root/'prepare.sh').read_text().split("<<'XML'\n",1)[1].split('\nXML',1)[0]
    launch=r/'full.launch';launch.write_text(text)
    cfg=roslaunch.config.load_config_default([str(launch)],11311)
    for key,value in {'/preprocess/blind':1.,'/pcd_save/pcd_save_en':False,'/publish/path_en':True,'/use_sim_time':True}.items():
        assert cfg.params[key].value==value
    assert len(cfg.nodes)==1 and cfg.nodes[0].name=='laserMapping'
    launch.write_text(text.replace('name="pcd_save/pcd_save_en" value="false"','name="pcd_save/pcd_save_en" value="true"'))
    cfg=roslaunch.config.load_config_default([str(launch)],11311)
    assert cfg.params['/pcd_save/pcd_save_en'].value is True
    launch.write_text(launch.read_text().replace('name="preprocess/blind" type="double" value="1.0"','name="preprocess/blind" type="double" value="4.0"'))
    cfg=roslaunch.config.load_config_default([str(launch)],11311)
    assert cfg.params['/preprocess/blind'].value==4.0
    assert yaml.safe_load((root/'view.rviz').read_text())['Visualization Manager']['Global Options']['Fixed Frame']=='camera_init'
    for name in ['raw','derived','checks']:(r/name).mkdir()
    fields=[PointField(n,i*4,PointField.FLOAT32,1) for i,n in enumerate(['x','y','z','intensity'])]
    with rosbag.Bag(str(r/'raw/cloud_0.bag'),'w') as bag:
        for i in range(2):
            msg=create_cloud(Header(stamp=rospy.Time(10+i),frame_id='camera_init'),fields,[(float(i),2.,3.,4.),(float(i)+.01,2.,3.,5.)])
            bag.write('/cloud_registered',msg,msg.header.stamp)
    (r/'derived/trajectory.tum').write_text('10.000000000 0 2 3 0 0 0 1\n11.000000000 1 2 3 0 0 0 1\n')
    (r/'checks/full-run.json').write_text(json.dumps({'input_bag_start':10.}))
    for name in ['export_pcd.py','downsample_pcd.py','trajectory.py']:
        subprocess.run(['python3',str(root/name),str(r)],check=True)
    result=json.loads((r/'checks/pcd-voxel-0p05m-first.json').read_text())
    assert result['input_points']==4 and result['output_points']==2
    for name in ['export_pcd.py','downsample_pcd.py','trajectory.py']:
        assert subprocess.run(['python3',str(root/name),str(r)],stdout=subprocess.PIPE,stderr=subprocess.PIPE).returncode!=0
    # Native-style extra fields must not prevent comparison of the four shared fields.
    (r/'PCD').mkdir()
    content=(r/'derived/final_registered.pcd').read_bytes()
    head,payload=content.split(b'DATA binary\n')
    original=np.frombuffer(payload,dtype='<f4').reshape(-1,4)
    native=np.zeros((len(original),8),dtype='<f4');native[:,:4]=original
    head=head.replace(b'FIELDS x y z intensity',b'FIELDS x y z intensity normal_x normal_y normal_z curvature')
    head=head.replace(b'SIZE 4 4 4 4',b'SIZE 4 4 4 4 4 4 4 4').replace(b'TYPE F F F F',b'TYPE F F F F F F F F').replace(b'COUNT 1 1 1 1',b'COUNT 1 1 1 1 1 1 1 1')
    (r/'PCD/scans.pcd').write_bytes(head+b'DATA binary\n'+native.tobytes())
    subprocess.run(['python3',str(root/'compare_pcd.py'),str(r)],check=True)
    assert json.loads((r/'checks/native-vs-exported-pcd.json').read_text())['status']=='MATCH_XYZI'
    # Regression: no /tf publisher before first odometry is allowed; mandatory publishers remain checked.
    (r/'config').mkdir()
    params={key:value.value for key,value in cfg.params.items()}
    class Master:
        def getSystemState(self):
            return ([('/Odometry',['/laserMapping']),('/path',['/laserMapping']),('/cloud_registered',['/laserMapping'])],
                    [('/Odometry',['/record_pose']),('/cloud_registered',['/record_cloud','/corridor_rviz']),
                     ('/livox/avia/lidar',['/laserMapping']),('/livox/avia/imu',['/laserMapping'])],[])
        def getParam(self,key):
            if key in params:return params[key]
            prefix=key+'/'
            result={}
            for name,value in params.items():
                if not name.startswith(prefix):continue
                parts=name[len(prefix):].split('/');target=result
                for part in parts[:-1]:target=target.setdefault(part,{})
                target[parts[-1]]=value
            if not result:raise KeyError(key)
            return result
    previous=sys.modules.get('rosgraph')
    sys.modules['rosgraph']=types.SimpleNamespace(Master=lambda name:Master())
    try:
        for blind,enabled in [(1.,False),(1.,True),(4.,True)]:
            params['/pcd_save/pcd_save_en']=enabled
            params['/preprocess/blind']=blind
            (r/'config/output-policy.json').write_text(json.dumps({'online_pcd':enabled,'blind':blind}))
            exec(compile((root/'ready.py').read_text(),str(root/'ready.py'),'exec'),{'__file__':str(r/'ready.py')})
        keys=['common','cube_side_length','feature_extract_enable','filter_size_map','filter_size_surf','mapping','max_iteration','point_filter_num','publish','runtime_pos_log_enable','use_sim_time','pcd_save','preprocess']
        reference={key:Master().getParam('/'+key) for key in keys};reference['preprocess']['blind']=1.
        (r/'config/reference-blind1-runtime.yaml').write_text(yaml.safe_dump(reference))
        exec(compile((root/'ready.py').read_text(),str(root/'ready.py'),'exec'),{'__file__':str(r/'ready.py')})
        assert json.loads((r/'checks/reference-config-comparison.json').read_text())['status']=='MATCH_EXCEPT_BLIND'
    finally:
        if previous is not None:sys.modules['rosgraph']=previous
print('PASS: Python syntax in installed image; launch overrides; RViz YAML; synthetic PCD export, 5cm original-point selection, trajectory XYZ roundtrip and overwrite refusal. No ROS nodes or replay started.')
