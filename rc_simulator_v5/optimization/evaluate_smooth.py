"""Unchanged simulation/three-lap evaluator plus the fixed straightness reward."""
import csv,json
from optimization.evaluate_three_laps import ROOT,evaluate_three
from optimization.straight_score import RULES,add_straight_score

def evaluate_smooth(path,name,**kwargs):
    result=evaluate_three(path,name,logs=True,**kwargs)
    with (ROOT/'optimization/three_laps/logs'/(name+'.csv')).open(encoding='utf-8-sig') as stream:
        result=add_straight_score(result,csv.DictReader(stream))
    output=ROOT/'optimization/steering_smooth';(output/'results').mkdir(parents=True,exist_ok=True)
    (output/'results'/(name+'.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2))
    return result

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('program');p.add_argument('--name',default='smooth_test');p.add_argument('--external',action='store_true')
    args=p.parse_args();print(json.dumps(evaluate_smooth(args.program,args.name,external=args.external),ensure_ascii=False,indent=2))
