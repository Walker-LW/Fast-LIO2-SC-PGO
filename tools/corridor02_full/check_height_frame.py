from pathlib import Path
import json
import numpy as np
r=Path('/experiment')
out=r/'checks/height-frame-20261102-001'
out.mkdir()
a=np.loadtxt(str(r/'derived/trajectory.tum'))
states=np.loadtxt(str(r/'Log/mat_pre.txt'))
g0=states[0,-3:];g1=states[-1,-3:]
up=-g0/np.linalg.norm(g0)
h=(a[:,1:4]-a[0,1:4]).dot(up)
tilt=lambda g:float(np.degrees(np.arccos(np.clip(-g[2]/np.linalg.norm(g),-1,1))))
result=dict(initial_logged_gravity=g0.tolist(),final_logged_gravity=g1.tolist(),
            initial_gravity_tilt_from_negative_Z_deg=tilt(g0),final_gravity_tilt_from_negative_Z_deg=tilt(g1),
            raw_trajectory_z_minmax=a[:,3].min().item(),raw_trajectory_z_max=a[:,3].max().item(),
            initial_gravity_projected_height_min=float(h.min()),initial_gravity_projected_height_max=float(h.max()),
            initial_gravity_projected_height_span=float(np.ptp(h)),
            initial_gravity_projected_endpoint_height_change=float(h[-1]),
            limits='Projection on fixed initial logged gravity only; not GT altitude, not corrected trajectory. Gravity state estimated and can change. Same-floor statement does not supply exact ground profile.')
with (out/'report.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
