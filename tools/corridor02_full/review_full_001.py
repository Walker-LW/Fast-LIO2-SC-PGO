import pathlib, json, re, hashlib, math
import rosbag
r=pathlib.Path('/results/baseline/corridor02-full-b1-20261102-001')
for f in ['checks/actions.log','config/runtime.yaml','checks/record-odom-console.log','checks/record-cloud-console.log']:
    print('\nFILE',f); print((r/f).read_text(errors='replace'))
for f in ['checks/algorithm-console.log','checks/live-progress.log','checks/player-console.log','checks/rviz-console.log']:
    s=(r/f).read_text(errors='replace')
    print('\nTAIL',f); print(s[-5000:])
    if 'algorithm' in f:
        print('WARNING_LINES'); print('\n'.join(x for x in s.splitlines() if any(k in x for k in ['No Effective','No point','WARN','ERROR','died','exit code'])))
for folder in ['raw','derived']:
    for p in sorted((r/folder).iterdir()):
        h=hashlib.sha256()
        with p.open('rb') as f:
            for chunk in iter(lambda:f.read(8*1024*1024),b''):h.update(chunk)
        print('FILE_HASH',p.name,p.stat().st_size,h.hexdigest())
with rosbag.Bag(str(r/'raw/odometry.bag')) as b:
    poses=[]; tf=[]
    for topic,m,t in b.read_messages(topics=['/Odometry','/tf']):
        if topic=='/tf':
            tf.extend(x.header.stamp.to_nsec() for x in m.transforms if x.header.frame_id=='camera_init' and x.child_frame_id=='body')
        else:
            p=m.pose.pose.position
            poses.append((m.header.stamp.to_nsec(),(p.x,p.y,p.z)))
    missing=sorted(set(t for t,p in poses)-set(tf))
    steps=[]
    for (t,p),(u,q) in zip(poses,poses[1:]):
        steps.append(dict(start_relative=(t-1737472022243104700)/1e9,end_relative=(u-1737472022243104700)/1e9,dt=(u-t)/1e9,step=math.sqrt(sum((a-c)**2 for a,c in zip(p,q)))))
    print('TOP_STEPS',json.dumps(sorted(steps,key=lambda x:x['step'],reverse=True)[:12]))
    print('TF_MISSING',json.dumps([(t-1737472022243104700)/1e9 for t in missing]))
with (r/'derived/final_registered.pcd').open('rb') as f:
    header=b''
    while not header.endswith(b'DATA binary\n'):
        line=f.readline(); assert line
        header+=line
    print('PCD_HEADER',header.decode()); print('PCD_PAYLOAD_BYTES',(r/'derived/final_registered.pcd').stat().st_size-len(header))
