"""Headless replay using the SAME simulation core as the Tk GUI.

python run_program.py examples/sensor_drive.py --trust-code --seconds 120
"""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import sys
from config import VehicleConfig, PHYSICS_HZ
from simulation import Simulation
from world import World, DEFAULT_MAP

def trial(path, *, seconds=120, interface='auto', world=None, config=None, spawn='demo', seed=7,
          dx=0.0,dy=0.0,dh=0.0):
    sim=Simulation(world or World.load(),config=config,seed=seed,spawn_name=spawn)
    sim.load_program(path,interface)
    sim.car.x+=dx; sim.car.y+=dy; sim.car.heading+=dh
    if sim.world.collision(sim.car.polygon(sim.config)):
        sim.close(); raise ValueError('開始位置が壁に重なっています。')
    minimum=float('inf'); first_stop=None; moving=0
    try:
        for _ in range(round(seconds*PHYSICS_HZ)):
            if sim.program_error or sim.collision_id or sim.finished or sim.program_stopped: break
            sim.step(block=True)
            if abs(sim.car.speed)>.2: moving+=1
            if sim.time>1 and sim.requested_accel==0 and '安全停止' in sim.state and first_stop is None:
                first_stop=sim.time
            if sim.steps%24==0:
                minimum=min(minimum,sim.world.clearance(sim.car.polygon(sim.config)))
        report=dict(program=sim.program_spec.path.name,interface=sim.program_spec.interface,
                    program_sha256=sim.program_spec.sha256,source='external_python',
                    control_inputs=['Fr','FrLh','RrLh','FrRh','RrRh','simulation_time_or_dt'],
                    ground_truth_pose_sent_to_program=False,guide_controller_instantiated=sim.controller is not None,
                    requested_duration_s=seconds,simulated_time_s=round(sim.time,3),
                    moving_time_s=round(moving/PHYSICS_HZ,3),distance_m=round(sim.distance/100,3),
                    collision=sim.collision_id,program_error=sim.program_error,
                    normal_program_exit=sim.finished,state=sim.state,
                    first_safety_stop_s=None if first_stop is None else round(first_stop,3),
                    sampled_min_clearance_cm=round(minimum,3) if minimum<float('inf') else None,
                    clearance_sampling_hz=5,collision_sampling_hz=PHYSICS_HZ,
                    pwm_event_count=len(sim.pwm_events),spawn=spawn,seed=seed,
                    noise_std_cm=sim.config.sensor_noise_std_cm,
                    limits='Idealized 2D sensors/vehicle; not physical safety validation; no lap or coverage criterion.')
        return report,sim
    finally:
        sim.close()


def save_csv(path, records):
    if not records: return
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(records[0])); writer.writeheader();writer.writerows(records)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('program',type=Path)
    parser.add_argument('--trust-code',action='store_true',help='Acknowledge that trusted local Python is executed, NOT sandboxed')
    parser.add_argument('--interface',choices=['auto','function','legacy'],default='auto')
    parser.add_argument('--seconds',type=float,default=120)
    parser.add_argument('--map',type=Path,default=DEFAULT_MAP)
    parser.add_argument('--spawn',choices=['demo','start1','start2','start3'],default='demo')
    parser.add_argument('--noise-cm',type=float,default=0)
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'reports'/'user_run')
    args=parser.parse_args()
    if not args.trust_code:
        parser.error('信頼できる.pyだけを実行してください。確認した場合に --trust-code を付けます。')
    if not 0<args.seconds<=3600:
        parser.error('--seconds は0より大きく3600以下です。')
    try:
        report,sim=trial(args.program,seconds=args.seconds,interface=args.interface,world=World.load(args.map),
                         spawn=args.spawn,config=VehicleConfig(sensor_noise_std_cm=args.noise_cm))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        save_csv(args.output.with_suffix('.csv'),sim.logs)
        if sim.pwm_events: save_csv(args.output.with_name(args.output.name+'_pwm').with_suffix('.csv'),sim.pwm_events)
        args.output.with_suffix('.log').write_text(sim.program_log(),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
        return 2 if report['collision'] or report['program_error'] else 0
    except (OSError,ValueError,SyntaxError) as exc:
        print(str(exc),file=sys.stderr); return 1

if __name__=='__main__':
    raise SystemExit(main())
