from pathlib import Path
import rosbag,rospy,cv2,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

out=Path('/out');times=np.loadtxt('/run/scpgo/backend/times.txt')
with rosbag.Bag('/datasets/Corridor01.bag') as bag:
    t0=bag.get_start_time();meta=[]
    fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    for ax,key in zip(axes.flat,[0,375,378,382]):
        stamp=float(times[key]);items=list(bag.read_messages(topics=['/camera/color/image_raw/compressed'],start_time=rospy.Time.from_sec(stamp-.15),end_time=rospy.Time.from_sec(stamp+.15)))
        assert items,key
        _,msg,t=min(items,key=lambda item:abs(item[1].header.stamp.to_sec()-stamp))
        img=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_COLOR);assert img is not None
        name=f'camera-keyframe-{key}.jpg';(out/name).write_bytes(bytes(msg.data))
        ax.imshow(cv2.cvtColor(img,cv2.COLOR_BGR2RGB));ax.set_title(f'Keyframe {key}; bag +{msg.header.stamp.to_sec()-t0:.2f} s');ax.axis('off')
        meta.append({'keyframe':key,'keyframe_stamp':stamp,'camera_stamp':msg.header.stamp.to_sec(),'delta_s':msg.header.stamp.to_sec()-stamp,'bag_relative_s':msg.header.stamp.to_sec()-t0,'file':name})
    fig.suptitle('Independent camera evidence: proximity in a repeated corridor does not label LiDAR correspondences',fontsize=11)
    fig.savefig(out/'camera-return-check.png',dpi=160);plt.close(fig)
    (out/'camera-timestamps.json').write_text(json.dumps(meta,indent=2)+'\n')
print(json.dumps(meta))
