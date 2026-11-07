"""Parse configuration in the installed ROS environment; start no nodes."""
from pathlib import Path
import ast
import tempfile
import roslaunch.config
import yaml

root = Path('/tools')
for name in ('watch.py', 'verify.py'):
    ast.parse((root/name).read_text())
text = (root/'prepare.sh').read_text().split("<<'XML'\n", 1)[1].split('\nXML', 1)[0]
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory)/'full.launch'
    path.write_text(text)
    config = roslaunch.config.load_config_default([str(path)], 11311)
    expected = {'/preprocess/blind': 1.0, '/publish/path_en': True,
                '/pcd_save/pcd_save_en': False, '/use_sim_time': True}
    for key, value in expected.items():
        assert config.params[key].value == value, key
    assert len(config.nodes) == 1 and config.nodes[0].name == 'laserMapping'
view = yaml.safe_load((root/'view.rviz').read_text())
assert view['Visualization Manager']['Global Options']['Fixed Frame'] == 'camera_init'
print('PASS: installed Python syntax, launch expansion and override parameters, RViz YAML. No nodes started.')
