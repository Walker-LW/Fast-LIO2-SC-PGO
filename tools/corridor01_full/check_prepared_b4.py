"""Expand the actual prepared launch offline; never start nodes."""
from pathlib import Path
import roslaunch.config,yaml
r=Path('/run-b4')
cfg=roslaunch.config.load_config_default([str(r/'config/full.launch')],11311)
tree={}
for key,param in cfg.params.items():
    parts=key.strip('/').split('/');node=tree
    for part in parts[:-1]:node=node.setdefault(part,{})
    node[parts[-1]]=param.value
baseline=yaml.safe_load((r/'config/reference-blind1-runtime.yaml').read_text())
keys=['common','cube_side_length','feature_extract_enable','filter_size_map','filter_size_surf','mapping','max_iteration','point_filter_num','publish','runtime_pos_log_enable','use_sim_time','pcd_save']
for key in keys:assert tree[key]==baseline[key],key
pre=dict(baseline['preprocess']);pre['blind']=4.
assert tree['preprocess']==pre
assert tree['pcd_save']['pcd_save_en'] is True
assert len(cfg.nodes)==1 and cfg.nodes[0].name=='laserMapping'
print('PASS: actual prepared blind=4 launch expands; selected configuration groups match saved blind=1 runtime except blind 1 -> 4. Online PCD true. No nodes started.')
