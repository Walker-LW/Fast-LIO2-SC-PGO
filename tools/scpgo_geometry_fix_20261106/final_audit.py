from pathlib import Path
import hashlib,json,shutil

out=Path(__file__).resolve().parent
ws=Path('/home/lw/slam-research/fast_lio_ws')
src=ws/'src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
installation=json.loads((out/'installation.json').read_text())
for p,h in installation['protected_binaries'].items():assert sha(Path(p))==h,p
assert sha(src/'src/laserPosegraphOptimization.cpp')==installation['source_sha256']
profile=ws/'src/M3DGR/deployment-scpgo/runtime/corridor01-indoor.yaml'
assert sha(profile)==installation['profile_sha256']
baseline=out.parent/'scpgo_parameterization_20261106/modified'
for name,path in [('BackendParameters.h','include/aloam_velodyne/BackendParameters.h'),('Scancontext.cpp','include/scancontext/Scancontext.cpp'),('Scancontext.h','include/scancontext/Scancontext.h')]:
    assert sha(src/path)==sha(baseline/name),name
meta=json.loads((out/'real/manifest.json').read_text())
run=Path('/home/lw/slam-research/results/scpgo/scpgo-corridor01-20261106-002')
for name,h in meta['scan_sha256'].items():assert sha(run/'backend/Scans'/name)==h,name
for p,h in meta['input_sha256'].items():assert sha(Path(p.replace('/run/scpgo',str(run))))==h,p
results=json.loads((out/'results-summary.json').read_text())
check=json.loads((out/'config-checks/report.json').read_text());assert check['status']=='PASS'
binary=ws/'devel_scpgo_geometry/lib/aloam_velodyne/alaserPGO'
report={'status':'GEOMETRY_FIX_BUILT_AND_VERIFIED','source_sha256':installation['source_sha256'],'binary':str(binary),'binary_sha256':sha(binary),
        'protected_binaries_unchanged':True,'profile_and_parameter_headers_unchanged':True,'real_input_files_unchanged':True,
        'synthetic':results['synthetic'],'config_check':check,
        'offline_pairs':meta['count'],'offline_icp_runs':meta['count']*4,'offline_summary':results['summary'],
        'no_full_slam_replay':True,'corridor01_ghosting_resolution_demonstrated':False,
        'limits':meta['limitations'],'left_unchanged':['ICP voxel 0.4 m and all existing numerical settings','candidate search and yaw initialization','factor noise and robust kernel','shutdown behavior','local2global fixed 16 threads'],
        'debug_topics':'Names unchanged; copies published in camera_init are world-coordinate visualizations. Internal ICP source/current and target/historical-root remain local. Do not feed these published world copies directly into the old score-reproduction script.',
        'validation_harness_setup_notes':['First startup-check attempt required ROS_HOME in /tmp for read-only container.','Comparator normalized YAML scientific notation strings to float; production YAML unchanged.']}
(out/'final-report.json').write_text(json.dumps(report,indent=2)+'\n')
archive=Path(installation['backup'])
for name in ['laserPosegraphOptimization.cpp','change-manifest.json','final-report.json','results-summary.json','backend-build.log']:
    shutil.copy2(out/name,archive/name)
print(json.dumps({k:report[k] for k in ['status','binary','binary_sha256','protected_binaries_unchanged','profile_and_parameter_headers_unchanged','real_input_files_unchanged','offline_pairs','offline_icp_runs']},indent=2))
