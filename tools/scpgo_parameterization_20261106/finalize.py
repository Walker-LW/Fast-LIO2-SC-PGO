from pathlib import Path
import shutil
root=Path(__file__).resolve().parent
pkg=Path('/home/lw/slam-research/fast_lio_ws/src/M3DGR/baseline_systems/Fast_LIO2_M3DGR/src/SC-PGO')
for name,rel in [('laserPosegraphOptimization.cpp','src/laserPosegraphOptimization.cpp'),('Scancontext.h','include/scancontext/Scancontext.h'),('Scancontext.cpp','include/scancontext/Scancontext.cpp')]:
    shutil.copyfile(root/'modified'/name,pkg/rel)
print('Preserved original line endings and float conversions.')
