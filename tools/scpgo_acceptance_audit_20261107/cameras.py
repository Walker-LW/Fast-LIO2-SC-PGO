from pathlib import Path
import rosbag,rospy,cv2,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

out=Path('/out');times=np.loadtxt('/run/scpgo/backend/times.txt')
pairs=[(0,32),(147,310),(41,151),(135,287)]
meta=[]
with rosbag.Bag('/datasets/Corridor01.bag') as bag:
 t0=bag.get_start_time()
 for a,b in pairs:
  fig,axes=plt.subplots(2,2,figsize=(12,9),constrained_layout=True)
  for row,topic in enumerate(['/camera/color/image_raw/compressed','/cv_camera/image_raw/compressed']):
   for col,key in enumerate([a,b]):
    stamp=float(times[key]);items=list(bag.read_messages(topics=[topic],start_time=rospy.Time.from_sec(stamp-.2),end_time=rospy.Time.from_sec(stamp+.2)))
    assert items,(topic,key)
    _,m,t=min(items,key=lambda x:abs(x[1].header.stamp.to_sec()-stamp))
    img=cv2.imdecode(np.frombuffer(m.data,dtype=np.uint8),cv2.IMREAD_COLOR);assert img is not None
    name=f'kf{key}-camera{row}.jpg';(out/name).write_bytes(bytes(m.data))
    ax=axes[row,col];ax.imshow(cv2.cvtColor(img,cv2.COLOR_BGR2RGB));ax.axis('off');ax.set_title(f"KF {key}, bag +{m.header.stamp.to_sec()-t0:.2f}s; camera {row}")
    meta.append({'pair':[a,b],'keyframe':key,'topic':topic,'stamp':m.header.stamp.to_sec(),'delta_s':m.header.stamp.to_sec()-stamp,'file':name})
  fig.suptitle(f'Candidate ({a},{b}): independent scene evidence; no pose alignment')
  fig.savefig(out/f'pair-{a}-{b}-cameras.png',dpi=150);plt.close(fig)
(out/'camera-timestamps.json').write_text(json.dumps(meta,indent=2)+'\n')
print('EXTRACTED',len(meta),'frames')
