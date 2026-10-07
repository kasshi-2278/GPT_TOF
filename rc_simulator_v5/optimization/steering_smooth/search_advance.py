from optimization.evaluate_three_laps import ROOT
from optimization.evaluate_smooth import evaluate_smooth
import json,itertools
base=(ROOT/'optimization/steering_smooth/candidates/smooth_003.py').read_text();rows=[]
params=list(itertools.product((45,50,55),(70,80,90),(150,160)))
for i,(start,handle,turn_start) in enumerate(params,51):
 s=base.replace('if min(left,right)<45',f'if min(left,right)<{start}').replace('desired=70 if right<left else -70',f'desired={handle} if right<left else -{handle}').replace("'turn_start': 150",f"'turn_start': {turn_start}")
 p=ROOT/f'optimization/steering_smooth/candidates/smooth_{i:03}.py';p.write_text(s)
 r=evaluate_smooth(p,f'smooth_{i:03}');r.update(escape_start=start,escape_handle=handle,turn_start=turn_start)
 rows.append(r);print(i,r['completed_three_laps'],r['total_time_s'],r['collisions'],r['danger'],r['weave'],round(r['straight_fraction'],3),r['smooth_score'],flush=True)
 (ROOT/'optimization/steering_smooth/advance_search.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
