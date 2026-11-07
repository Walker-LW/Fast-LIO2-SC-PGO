"""Inspect sensor imagery and independent wheel odometry; no ROS playback."""
from pathlib import Path
import csv
import json
import sys
import cv2
import numpy as np
import rosbag

bag_path,out=map(Path,sys.argv[1:3])
targets=[0,5,20,60,120,180,240,300,360,380,395,403]
topics=['/camera/color/image_raw/compressed','/cv_camera/image_raw/compressed']
selected={t:{} for t in topics}
metadata={}
wheel=[]
with rosbag.Bag(str(bag_path)) as bag:
    start=bag.get_start_time()
    metadata=dict(input=str(bag_path),bag_start=start,bag_end=bag.get_end_time(),
        duration=bag.get_end_time()-start,topics={k:dict(type=v.msg_type,count=v.message_count)
            for k,v in bag.get_type_and_topic_info().topics.items()},
        selection='Nearest bag receive time to predeclared offsets, no appearance-based selection.',
        targets_seconds=targets,limitation='Camera and wheel are auxiliary evidence, not continuous GT or automatic loop labels.')
    for topic,msg,received in bag.read_messages(topics=topics+['/odom']):
        elapsed=received.to_sec()-start
        if topic=='/odom':
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            wheel.append([msg.header.stamp.to_sec(),elapsed,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        else:
            nearest=min(targets,key=lambda t:abs(elapsed-t))
            old=selected[topic].get(nearest)
            if old is None or abs(elapsed-nearest)<old[0]:
                selected[topic][nearest]=(abs(elapsed-nearest),elapsed,msg.header.stamp.to_sec(),bytes(msg.data))
frames=[]
for topic,items in selected.items():
    label='front' if topic==topics[0] else 'omni'
    thumbs=[]
    for target in targets:
        distance,elapsed,stamp,payload=items[target]
        path=out/('%s-%03d.jpg'%(label,target))
        path.write_bytes(payload)
        image=cv2.imdecode(np.frombuffer(payload,dtype=np.uint8),cv2.IMREAD_COLOR)
        assert image is not None
        width,height=640,360
        scale=min(width/image.shape[1],height/image.shape[0])
        small=cv2.resize(image,(int(image.shape[1]*scale),int(image.shape[0]*scale)))
        canvas=np.zeros((height+35,width,3),dtype=np.uint8)
        x=(width-small.shape[1])//2; y=(height-small.shape[0])//2
        canvas[y:y+small.shape[0],x:x+small.shape[1]]=small
        cv2.putText(canvas,'%s t=%.3fs (bag receive)'%(label,elapsed),(12,height+25),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),1)
        thumbs.append(canvas)
        frames.append(dict(topic=topic,target_seconds=target,receive_relative_seconds=elapsed,
            header_stamp=stamp,file=path.name,width=image.shape[1],height=image.shape[0]))
    sheet=np.vstack([np.hstack(thumbs[i:i+3]) for i in range(0,len(thumbs),3)])
    cv2.imwrite(str(out/(label+'-contact-sheet.jpg')),sheet)
with (out/'wheel-trajectory.csv').open('x',newline='') as stream:
    writer=csv.writer(stream); writer.writerow(['stamp','receive_relative_s','x','y','z','qx','qy','qz','qw']); writer.writerows(wheel)
wheel=np.asarray(wheel)
metadata['wheel_summary']=dict(messages=len(wheel),first=wheel[0].tolist(),last=wheel[-1].tolist(),
    endpoint_distance_m=float(np.linalg.norm(wheel[-1,2:5]-wheel[0,2:5])),
    path_length_m=float(np.linalg.norm(np.diff(wheel[:,2:5],axis=0),axis=1).sum()),
    note='Wheel odometry is not GT; no inference of actual endpoint solely from this trace.')
# Render a schematic wheel XY trace as an aid to visual inspection, without alignment.
xy=wheel[:,2:4]; low=xy.min(axis=0); high=xy.max(axis=0)
scale=900/max(float(np.max(high-low)),1)
screen=np.empty_like(xy); screen[:,0]=50+(xy[:,0]-low[0])*scale; screen[:,1]=950-(xy[:,1]-low[1])*scale
canvas=np.full((1020,1050,3),255,dtype=np.uint8)
cv2.polylines(canvas,[screen.astype(np.int32)],False,(140,80,20),2)
for index,color,text in [(0,(0,160,0),'START'),(-1,(0,0,220),'END')]:
    x,y=screen[index].astype(int); cv2.circle(canvas,(x,y),7,color,-1); cv2.putText(canvas,text,(x+8,y),cv2.FONT_HERSHEY_SIMPLEX,0.6,color,2)
cv2.putText(canvas,'Raw /odom XY; wheel estimate, NOT ground truth',(20,25),cv2.FONT_HERSHEY_SIMPLEX,0.65,(0,0,0),1)
cv2.imwrite(str(out/'wheel-trajectory.jpg'),canvas)
metadata['frames']=frames
(out/'input-inspection.json').write_text(json.dumps(metadata,indent=2)+'\n')
print(json.dumps({k:v for k,v in metadata.items() if k not in ['frames','topics']},indent=2))
