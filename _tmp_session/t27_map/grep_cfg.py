import re
p='/home/duzhong/dzrobot/config/robot.yaml'
t=open(p,encoding='utf-8').read().splitlines()
pat=re.compile(r'(?i)(pid|\bkp|\bki|\bkd|deadzone|scan_|fire_only|aspect_|shoot_distance|target_confirm|cls_switch)')
hits=[(i+1,l.strip()) for i,l in enumerate(t) if pat.search(l) and not l.strip().startswith('#')]
print('HITS',len(hits))
for i,l in hits[:80]: print(i,'|',l[:140])
