"""Reproduce ten sensor-noise/start-offset checks for the divider policy."""
import csv
import json
from optimization.evaluate_three_laps import ROOT,evaluate_three

CASES=[dict(seed=7,noise=0,dx=0,dy=0,dh=0),dict(seed=8,noise=0,dx=0,dy=0,dh=0),
       dict(seed=11,noise=.5,dx=2,dy=0,dh=2),dict(seed=12,noise=.5,dx=-2,dy=0,dh=-2),
       dict(seed=13,noise=1,dx=3,dy=2,dh=3),dict(seed=14,noise=1,dx=-3,dy=-2,dh=-3),
       dict(seed=15,noise=1.5,dx=2,dy=-2,dh=5),dict(seed=16,noise=1.5,dx=-2,dy=2,dh=-5),
       dict(seed=17,noise=2,dx=3,dy=0,dh=3),dict(seed=18,noise=2,dx=-3,dy=0,dh=-3)]

if __name__=='__main__':
    rows=[]
    for i,case in enumerate(CASES,1):
        result=evaluate_three(ROOT/'examples/sonic_drive_three_laps.py',f'flow_program_final_{i:02}',external=True,**case)
        rows.append(result)
        print(i,result['completed_three_laps'],result['total_time_s'],result['collisions'],result['danger'],flush=True)
    folder=ROOT/'optimization/divider_flow';folder.mkdir(parents=True,exist_ok=True)
    (folder/'program_final_validation.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    with (folder/'program_final_validation.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
