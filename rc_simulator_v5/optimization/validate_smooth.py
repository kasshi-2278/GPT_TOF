"""Ten actual-worker runs of the selected steering and straight-score policy."""
import csv,json
from optimization.evaluate_smooth import evaluate_smooth
from optimization.evaluate_three_laps import ROOT
from optimization.validate_divider_flow import CASES

if __name__=='__main__':
    rows=[]
    for i,case in enumerate(CASES,1):
        result=evaluate_smooth(ROOT/'examples/sonic_drive_smooth.py',f'smooth_final_{i:02}',external=True,**case)
        rows.append(result)
        print(i,result['completed_three_laps'],result['total_time_s'],result['collisions'],result['danger'],result['smooth_score'],flush=True)
    folder=ROOT/'optimization/steering_smooth'
    (folder/'final_validation.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    with (folder/'final_validation.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n');writer.writeheader();writer.writerows(rows)
