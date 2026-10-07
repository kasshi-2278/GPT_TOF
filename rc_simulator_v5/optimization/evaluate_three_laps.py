"""Continuous three-lap tests with an external read-only timing observer."""
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
from optimization.evaluate import ROOT,Metrics,user_score
from config import VehicleConfig,PHYSICS_HZ
from simulation import Simulation
from world import World,DEFAULT_MAP
from lap_timer import LapTimer

TIMEOUT_S=1800

def evaluate_three(path,name,*,external=False,seed=7,noise=0,dx=0,dy=0,dh=0,logs=True):
    path=Path(path).resolve()
    sim=Simulation(World.load(),VehicleConfig(sensor_noise_std_cm=noise),seed=seed,
                   mode='program' if external else 'manual')
    if external:sim.load_program(path)
    sim.car.x+=dx;sim.car.y+=dy;sim.car.heading+=dh
    if sim.world.collision(sim.car.polygon(sim.config)):
        sim.close();raise ValueError('Invalid evaluation spawn')
    if not external:
        spec=importlib.util.spec_from_file_location('three_'+name,path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        controller=module.Controller()
    timer=LapTimer(sim,3);metrics=Metrics(sim);rows=[];error=None
    try:
        for _ in range(round(TIMEOUT_S*PHYSICS_HZ)):
            if not external and sim.steps%6==0:
                sim.read_sensors();sim._sampled_step=sim.steps
                accel,handle,state=controller.step(dict(sim.sensors),.05)
                command=(accel,handle)
            sim.step(block=True) if external else sim.step(command)
            if sim.steps%6==0 or sim.collision_id or sim.program_error or sim.finished:
                metrics.sample(sim);timer.sample(sim)
                row=sim.record();row['lap_count']=len(timer.lap_times)
                row['lap_elapsed_s']=sim.time-timer.last_crossing_time
                row['clearance_cm']=sim.world.clearance(sim.car.polygon(sim.config))
                row['gates_passed_this_lap']=timer.gate
                row['state']=sim.state if external else state
                rows.append(row)
            if timer.completed or sim.collision_id or sim.program_error or sim.finished or metrics.stop_duration>=5:break
    except Exception as exc:error=repr(exc)
    finally:sim.close()
    result=dict(name=name,program=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                external_worker=external,seed=seed,noise_cm=noise,dx=dx,dy=dy,dh=dh,
                completed_three_laps=timer.completed,laps_completed=len(timer.lap_times),
                lap_times_s=[round(t,6) for t in timer.lap_times],
                crossing_times_s=[round(t,6) for t in timer.crossing_times],
                total_time_s=round(timer.total_time,6) if timer.completed else None,
                elapsed_s=round(sim.time,6),collisions=int(bool(sim.collision_id)),collision=sim.collision_id,
                danger=metrics.danger,sharp_steer=metrics.sharp,weave=metrics.weave,stops=metrics.stops,
                min_clearance_cm=round(metrics.min_clearance,6),program_error=error or sim.program_error,
                score=round(user_score(timer.completed,int(bool(sim.collision_id)),metrics.danger,metrics.sharp,metrics.weave,metrics.stops,sim.time),6),
                final_pose=[sim.car.x,sim.car.y,sim.car.heading],final_state=sim.state if external else locals().get('state',''),
                map_sha256=hashlib.sha256(DEFAULT_MAP.read_bytes()).hexdigest(),
                timing='Ordered original evaluator regions + southbound crossing of spawn y within +/-60cm, >=3000cm per lap; continuous car/controller state; no resets.')
    output=ROOT/'optimization/three_laps';(output/'results').mkdir(parents=True,exist_ok=True);(output/'logs').mkdir(exist_ok=True)
    (output/'results'/f'{name}.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    if logs and rows:
        with (output/'logs'/f'{name}.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    if external:(output/'logs'/f'{name}.log').write_text(sim.program_log())
    return result

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('program',type=Path);p.add_argument('--name',default='three_laps');p.add_argument('--external',action='store_true')
    a=p.parse_args();print(json.dumps(evaluate_three(a.program,a.name,external=a.external),ensure_ascii=False,indent=2))
