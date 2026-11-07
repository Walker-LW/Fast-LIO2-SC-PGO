from pathlib import Path
import hashlib,json
p=Path('/fix');out=p/'synthetic'
text=(p/'laserPosegraphOptimization.cpp').read_text()
def extract(signature):
    start=text.index(signature);brace=text.index('{',start);end=brace+1;depth=1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]
names=['gtsam::Pose3 Pose6DtoGTSAMPose3(', 'pcl::PointCloud<PointType>::Ptr local2global(', 'void loopFindNearKeyframesCloud(', 'std::optional<gtsam::Pose3> doICPVirtualRelative(']
functions='\n\n'.join(extract(n) for n in names)
(out/'extracted-functions.cpp').write_text(functions)
(out/'harness.cpp').write_text((out/'prefix.cpp').read_text()+'\n'+functions+'\n'+(out/'cases.cpp').read_text())
(out/'manifest.json').write_text(json.dumps({'design':'Same six cases and tolerances as preceding geometry validation; no settings changed. Captured debug clouds remain world-coordinate, as before.','candidate_sha256':hashlib.sha256((p/'laserPosegraphOptimization.cpp').read_bytes()).hexdigest(),'functions':names},indent=2)+'\n')
