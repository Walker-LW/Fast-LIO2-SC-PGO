from pathlib import Path
import hashlib,json,subprocess,shutil
root=Path(__file__).resolve().parent
base=Path('/home/lw/slam-research')
record=base/'results/validation/scpgo-parameterization-20261106-001'
repo=base/'fast_lio_ws/src/M3DGR'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
protected=json.loads((record/'protected-binaries.json').read_text())
assert all(sha(Path(p))==v for p,v in protected.items())
pkg=repo/'baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO'
for name,rel in [('laserPosegraphOptimization.cpp','src/laserPosegraphOptimization.cpp'),('Scancontext.h','include/scancontext/Scancontext.h'),('Scancontext.cpp','include/scancontext/Scancontext.cpp'),('BackendParameters.h','include/aloam_velodyne/BackendParameters.h')]:
    assert sha(root/'modified'/name)==sha(pkg/rel)
checks=json.loads((record/'config-checks.json').read_text())
assert len(checks['cases'])==7 and all(c['pass'] for c in checks['cases'])
binary=base/'fast_lio_ws/devel_scpgo_params/lib/aloam_velodyne/alaserPGO'
report={'status':'BUILD_AND_CONFIGURATION_CHECKS_PASS','source_commit':subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),
        'new_binary':str(binary),'new_binary_sha256':sha(binary),'protected_old_binaries_unchanged':True,
        'profile':str(repo/'deployment-scpgo/runtime/corridor01-indoor.yaml'),
        'additional_parameter_count':38,'checks':checks,
        'slam_replayed':False,'map_accuracy_evaluated':False,
        'limitations':['Defaults checked at startup; trajectory/ICP equivalence has not been measured.',
                       'Upstream geometry, synchronization and shutdown behavior retained.',
                       'New profile changes radius and keyframe spacing together; it is not a single-factor experiment.']}
(record/'final-report.json').write_text(json.dumps(report,indent=2)+'\n')
shutil.copyfile(record/'final-report.json',root/'final-report.json')
shutil.copyfile(record/'config-checks.json',root/'config-checks.json')
shutil.copyfile(root/'README.md',repo/'deployment-scpgo/runtime/PARAMETERIZATION.md')
diff=subprocess.check_output(['git','-C',str(repo),'diff','--ignore-space-at-eol','--','baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO'],text=True)
(root/'parameterization.diff').write_text(diff)
print(json.dumps(report,indent=2))
