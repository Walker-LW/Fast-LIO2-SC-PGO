"""Create Corridor01-specific copies; never alter Corridor02 tools or archives."""
from pathlib import Path
root=Path(__file__).resolve().parent
old=root.parent/'corridor02_full'
for name in ['run.sh','prepare.sh','watch.py','verify.py','export_pcd.py','view.rviz']:
    text=(old/name).read_text(encoding='utf-8')
    text=text.replace('corridor02-full-b1','corridor01-full-b1').replace('Corridor02','Corridor01')
    if name=='prepare.sh':
        text=text.replace('"$here/environment-audit-20261102-003.log"', '"$here/input-audit-20261103-001.json"')
        text=text.replace('f6ac8ab3e82fa80bab74f20eaeb437bbe68cb2e2e0f4c90ba25248d724743452',
            '"$(python3 -c \'import json,sys; print(json.load(open(sys.argv[1]))["sha256"])\' "$here/input-audit-20261103-001.json")"')
        text=text.replace('cp -- "$here/export_pcd.py" "$run/"',
            'cp -- "$here/export_pcd.py" "$here/downsample_pcd.py" "$here/trajectory.py" "$here/ready.py" "$here/end_status.py" "$run/"')
        text=text.replace('printf \'Prepared %s',
            'sha256sum "$run/downsample_pcd.py" "$run/trajectory.py" "$run/ready.py" "$run/end_status.py" >> "$run/config/tools.sha256"\nprintf \'Prepared %s')
    if name=='run.sh':
        start=text.index('    for node in /laserMapping')
        end=text.index('    touch "$run/checks/ready.ok"',start)
        text=text[:start]+'    ros python3 "$inside/ready.py"\n'+text[end:]
        text=text.replace('interactive rviz -d "$inside/config/view.rviz"',
                          'interactive rviz -d "$inside/config/view.rviz" __name:=corridor_rviz')
        text=text.replace("  *) echo 'Unknown action'; exit 1 ;;",'''  downsample)
    once
    ros python3 "$inside/downsample_pcd.py" "$inside" 2>&1 | tee "$run/checks/downsample-console.log"
    ;;
  trajectory)
    once
    ros python3 "$inside/trajectory.py" "$inside" 2>&1 | tee "$run/checks/trajectory-export-console.log"
    ;;
  end-status)
    ros python3 "$inside/end_status.py" "$inside"
    ;;
  *) echo 'Unknown action'; exit 1 ;;''')
    (root/name).write_text(text,encoding='utf-8',newline='\n')
text=(old/'downsample_pcd.py').read_text(encoding='utf-8')
text=text.replace("import hashlib, json, math", "import hashlib, json, math, sys")
text=text.replace("r=Path('/results/baseline/corridor02-full-b1-20261102-001')", "r=Path(sys.argv[1])")
(root/'downsample_pcd.py').write_text(text,encoding='utf-8',newline='\n')
print('Created Corridor01 tool copies only. No experiment prepared or started.')
