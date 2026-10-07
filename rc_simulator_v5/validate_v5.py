"""Reproducible V5 tests; subprocesses run the packaged, trusted example programs.

python validate_v5.py               # unit + fault/compatibility tests
python validate_v5.py --trials      # also run sensor-only driving trials
"""
from __future__ import annotations
import argparse
import json
import platform
from pathlib import Path
import time
import unittest
from config import VehicleConfig
from run_program import trial, save_csv
from tests import test_programs, test_regression

ROOT=Path(__file__).resolve().parent

class RecordingResult(unittest.TextTestResult):
    def startTestRun(self):
        super().startTestRun(); self.passed=[]
    def addSuccess(self,test):
        super().addSuccess(test); self.passed.append(test.id())


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--trials',action='store_true')
    parser.add_argument('--trials-only',action='store_true',help='Run trials, reusing the existing unit-test report')
    parser.add_argument('--case',action='append',default=[],help='Select a case by name; repeatable')
    args=parser.parse_args()
    reports=ROOT/'reports'; reports.mkdir(exist_ok=True)
    if args.trials_only:
        report_path=reports/'v5_validation.json'
        if not report_path.exists(): parser.error('Run python validate_v5.py once before --trials-only')
        payload=json.loads(report_path.read_text(encoding='utf-8'))
        success=payload['failures']==0 and payload['errors']==0
    else:
        suite=unittest.TestSuite([test_programs.suite(),test_regression.suite()])
        with (reports/'v5_test_console.txt').open('w',encoding='utf-8') as output:
            result=unittest.TextTestRunner(stream=output,verbosity=2,resultclass=RecordingResult).run(suite)
        payload=dict(scope='V5 external Python loader, five sensor inputs, virtual GPIO/PWM, timeout handling, and shared physics regression',
                     python=platform.python_version(),platform=platform.platform(),
                     tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
                     passed_test_ids=result.passed,
                     failed_tests=[str(t) for t,_ in result.failures+result.errors],trials=[])
        success=result.wasSuccessful()
        print('Tests:',result.testsRun,'Failures:',len(result.failures),'Errors:',len(result.errors),flush=True)
    (reports/'v5_validation.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
    if success and (args.trials or args.trials_only):
        cases=[('sensor_nominal','sensor_drive.py',{}),
               ('sensor_noise_2cm','sensor_drive.py',dict(config=VehicleConfig(sensor_noise_std_cm=2),seed=29)),
               ('sensor_start_1','sensor_drive.py',dict(spawn='start1')),
               ('sensor_start_2','sensor_drive.py',dict(spawn='start2')),
               ('sensor_start_3','sensor_drive.py',dict(spawn='start3')),
               ('sensor_heading_offset','sensor_drive.py',dict(dx=-8,dh=8)),
               ('legacy_photo_transcription','sonic_drive_photo_transcribed.py',{}),
               ('legacy_stop_example','sonic_drive_legacy_example.py',{}),
               ('initial_reactive_candidate','sensor_reactive_conservative.py',{})]
        for name,filename,options in cases:
            if args.case and name not in args.case: continue
            report,sim=trial(ROOT/'examples'/filename,seconds=120,**options)
            report['case']=name
            payload['trials']=[r for r in payload['trials'] if r.get('case')!=name]+[report]
            (reports/'v5_validation.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
            save_csv(reports/(name+'.csv'),sim.logs)
            (reports/(name+'.log')).write_text(sim.program_log(),encoding='utf-8')
            # Large raw PWM event traces are regenerated via run_program.py as needed.
            print(name,'time=',report['simulated_time_s'],'distance=',report['distance_m'],
                  'collision=',report['collision'],'stop=',report['first_safety_stop_s'],
                  'error=',report['program_error'])
    payload['limitations']=[
        'Tests run on the platform shown above, not on the user PC or physical Raspberry Pi car.',
        'No real original sonic_drive.py file was supplied; the legacy photo example is a transcription.',
        'Ideal rays, assumed sensor mounts, kinematic bicycle model, generic ESC; no ultrasound beam/cone, specular echoes or dropouts.',
        'No proof of all-course safety, coverage, or lap completion. Stop behavior is reported explicitly.',
        'User programs execute with the current user privileges; a separate process is NOT a security sandbox.'
    ]
    (reports/'v5_validation.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if success else 1

if __name__=='__main__':
    raise SystemExit(main())
