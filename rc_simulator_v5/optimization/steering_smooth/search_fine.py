from optimization.evaluate_three_laps import ROOT
from optimization.evaluate_smooth import evaluate_smooth
import json,itertools,ast
from pathlib import Path
base=(ROOT/'optimization/steering_smooth/Previous_Best.py').read_text()
addon=next(ast.literal_eval(n.value) for n in ast.parse((ROOT/'optimization/steering_smooth/search_profiles.py').read_text()).body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='addon' for t in n.targets))
params=[dict(rate=rate,jerk=jerk,cap=cap,gain=.45 if cap==14 else .6,center=.05,filter=.08) for rate,jerk,cap in itertools.product((135,138,140,145),(4000,8000),(14,20))]
params += [dict(rate=140,jerk=8000,cap=cap,gain=.45 if cap==14 else .6,center=center,filter=.08) for center,cap in itertools.product((.04,.045,.055,.06),(14,20))]
params += [dict(rate=rate,jerk=4000,cap=cap,gain=.45 if cap==14 else .6,center=.05,filter=tau) for rate,cap,tau in itertools.product((135,140),(14,20),(.07,.09))]
rows=[]
for i,p0 in enumerate(params,69):
 s=base.replace('(20,.6) if self.stable_echoes else (14,.45)',f"({p0['cap']},{p0['gain']}) if self.stable_echoes else (14,.45)").replace("'center_gain': 0.05",f"'center_gain': {p0['center']}").replace("'filter_s': 0.08",f"'filter_s': {p0['filter']}")
 s+=addon.replace('SMOOTH_RATE=RATE',f"SMOOTH_RATE={p0['rate']}").replace('SMOOTH_JERK=JERK',f"SMOOTH_JERK={p0['jerk']}").replace('SMOOTH_FILTER=FILTER','SMOOTH_FILTER=0').replace('NEUTRAL_BAND=BAND','NEUTRAL_BAND=0')
 p=ROOT/f'optimization/steering_smooth/candidates/smooth_{i:03}.py';p.write_text(s)
 r=evaluate_smooth(p,f'smooth_{i:03}');r.update(p0)
 rows.append(r);print(i,r['completed_three_laps'],r['total_time_s'],r['collisions'],r['danger'],r['weave'],round(r['straight_fraction'],3),r['smooth_score'],flush=True)
 (ROOT/'optimization/steering_smooth/fine_search.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
