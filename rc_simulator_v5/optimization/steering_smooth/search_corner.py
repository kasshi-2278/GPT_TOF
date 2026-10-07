from optimization.evaluate_three_laps import ROOT
from optimization.evaluate_smooth import evaluate_smooth
import json,itertools
base=(ROOT/'optimization/steering_smooth/Previous_Best.py').read_text();rows=[]
params=[(start,handle,rate,185) for start,handle,rate in itertools.product((145,150,155,160),(90,95,100),(120,140))]
params += [(150,100,140,release) for release in (175,180,190,195)]
for i,(start,handle,rate,release) in enumerate(params,23):
 s=base.replace("'turn_start': 150",f"'turn_start': {start}").replace("'turn_handle': 100",f"'turn_handle': {handle}").replace("'steer_rate': 140",f"'steer_rate': {rate}").replace("'turn_release': 185",f"'turn_release': {release}")
 p=ROOT/f'optimization/steering_smooth/candidates/smooth_{i:03}.py';p.write_text(s)
 r=evaluate_smooth(p,f'smooth_{i:03}')
 r.update(turn_start=start,turn_handle=handle,steer_rate=rate,turn_release=release)
 rows.append(r);print(i,r['completed_three_laps'],r['total_time_s'],r['collisions'],r['danger'],r['weave'],round(r['straight_fraction'],3),r['smooth_score'],flush=True)
 (ROOT/'optimization/steering_smooth/corner_search.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
