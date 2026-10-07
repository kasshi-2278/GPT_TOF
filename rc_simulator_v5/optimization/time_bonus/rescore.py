"""Re-rank recorded runs without pretending to run new driving tests."""
from pathlib import Path
import csv,json,hashlib
from optimization.time_score import RULES,add_time_score

ROOT=Path(__file__).resolve().parents[2]

def main():
    source=ROOT/'optimization/steering_smooth'
    out=ROOT/'optimization/time_bonus';out.mkdir(parents=True,exist_ok=True)
    rows=[add_time_score(r) for r in json.loads((source/'history.json').read_text())]
    validations=[add_time_score(r) for r in json.loads((source/'final_validation.json').read_text())]
    safe=[r for r in rows if r['completed_three_laps'] and not r['collisions'] and not r['danger'] and not r['program_error']]
    safe.sort(key=lambda r:(-r['race_score'],r['name']))
    for filename,records in [('history',rows),('final_validation',validations)]:
        (out/(filename+'.json')).write_text(json.dumps(records,ensure_ascii=False,indent=2))
        fields=['name','completed_three_laps','total_time_s','collisions','danger','weave','original_score','straight_bonus','smooth_score','time_bonus','race_score','sha256']
        with (out/(filename+'.csv')).open('w',encoding='utf-8-sig',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore',lineterminator='\n');writer.writeheader();writer.writerows(records)
    (out/'rules.json').write_text(json.dumps(RULES,ensure_ascii=False,indent=2))
    summary=dict(recorded_runs=len(rows),new_driving_runs=0,best_candidate=safe[0]['name'],
                 best_original_score=safe[0]['original_score'],best_straight_score=safe[0]['smooth_score'],
                 time_bonus=safe[0]['time_bonus'],best_race_score=safe[0]['race_score'],
                 selected_program='examples/sonic_drive_smooth.py',
                 selected_source_sha256=hashlib.sha256((ROOT/'examples/sonic_drive_smooth.py').read_bytes()).hexdigest(),
                 previous_selected_candidate='smooth_079',selection_unchanged=safe[0]['name']=='smooth_079',
                 validation_completion=sum(r['completed_three_laps'] for r in validations),validation_tests=len(validations),
                 validation_collisions=sum(r['collisions'] for r in validations),rules=RULES)
    (out/'selection.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
