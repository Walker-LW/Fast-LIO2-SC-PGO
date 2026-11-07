from pathlib import Path
import json
base=Path('/results/baseline')
a=base/'corridor01-full-b1-20261103-001/config'
b=base/'corridor01-full-b4-20261105-001/config'
names=['commit.txt','image.txt','executable.sha256','source.sha256','input.sha256']
result={name:dict(equal=(a/name).read_bytes()==(b/name).read_bytes(),blind1=(a/name).read_text(),blind4=(b/name).read_text()) for name in names}
with (base/'corridor01-blind-ab-20261105-001/identity.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({name:item['equal'] for name,item in result.items()},indent=2))
