"""Fixed external evaluator. Pose is NEVER passed to the driving policy.

Use the unchanged simulator collision/physics model; the evaluator alone sees
pose and wall clearance to measure outcomes. No guide route is loaded into a
controller. Internal execution is a fast manual-input harness; final checks
also use the standard isolated program worker.
"""
from pathlib import Path
import csv
import hashlib
import importlib.util
import json
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config import VehicleConfig, PHYSICS_HZ
from simulation import Simulation
from world import World, DEFAULT_MAP

TIMEOUT_S = 600.0
RULES = {
    'timeout_s': TIMEOUT_S,
    'completion': 'Ordered regions: lower right, middle left, middle right, upper right, upper left, then return within 60cm of initial demo position facing south; distance >=3000cm. Evaluator only, no route data used.',
    'danger': 'True body-to-obstacle clearance <10cm, one episode; rearm at >=15cm.',
    'sharp_steer': 'Physical steering rate >60deg/s for >=0.10s; one episode, rearm after >=0.20s below threshold.',
    'weave': 'Physical steer crosses opposite +/-3deg bands within 2s of previous crossing; one reversal event.',
    'unnecessary_stop': 'After 10cm travel: speed<1cm/s for >=1s, front>25cm and both diagonal distances>15cm; one episode, rearm after >=1s moving.',
    'collision': 'Unchanged simulator collision latch; first collision ends the run and counts once.',
    'time': 'Actual simulated time until lap, collision, error, 5s of standstill, or 600s timeout. No additional timeout penalty; uses the user score formula exactly.',
    'safety_selection': 'Zero collisions and no errors first, then completion, then danger, weave, sharp steering, stops, time. Raw score best is retained separately.',
    'sensor_model': 'Existing ideal rays and sequential virtual HC-SR04 Echo emulation. No beam/interference/dropout model added.',
}

class Metrics:
    def __init__(self, sim):
        self.start = (sim.car.x, sim.car.y)
        self.gate = 0
        self.completed = False
        self.danger = self.sharp = self.weave = self.stops = 0
        self.near = False
        self.sharp_active = False
        self.sharp_duration = self.calm_duration = 0.0
        self.last_steer = sim.car.steer
        self.last_time = 0.0
        self.last_sign = 0
        self.sign_time = None
        self.stop_duration = self.moving_duration = 0.0
        self.stop_active = False
        self.min_clearance = float('inf')

    def sample(self, sim):
        t = sim.time
        dt = t - self.last_time
        clearance = sim.world.clearance(sim.car.polygon(sim.config))
        self.min_clearance = min(self.min_clearance, clearance)
        if clearance < 10 and not self.near:
            self.danger += 1
            self.near = True
        if clearance >= 15:
            self.near = False
        rate = abs(sim.car.steer - self.last_steer) / max(dt, 1e-9)
        if rate > 60:
            self.sharp_duration += dt
            self.calm_duration = 0
            if self.sharp_duration >= .1 - 1e-8 and not self.sharp_active:
                self.sharp += 1
                self.sharp_active = True
        else:
            self.calm_duration += dt
            self.sharp_duration = 0
            if self.calm_duration >= .2:
                self.sharp_active = False
        sign = 1 if sim.car.steer >= 3 else -1 if sim.car.steer <= -3 else 0
        if sign and sign != self.last_sign:
            if self.last_sign and self.sign_time is not None and t-self.sign_time <= 2:
                self.weave += 1
            self.last_sign = sign
            self.sign_time = t
        stopped = abs(sim.car.speed) < 1 and sim.distance >= 10
        if stopped:
            self.stop_duration += dt
            self.moving_duration = 0
            s = sim.sensors
            if self.stop_duration >= 1 and not self.stop_active and s['Fr'] > 25 and min(s['FrLh'],s['FrRh']) > 15:
                self.stops += 1
                self.stop_active = True
        else:
            self.stop_duration = 0
            self.moving_duration += dt
            if self.moving_duration >= 1:
                self.stop_active = False
        x,y = sim.car.x,sim.car.y
        gates = [x>850 and y<200, x<320 and 200<y<330,
                 x>800 and 350<y<470, x>800 and y>500,
                 x<180 and y>480]
        if self.gate < len(gates) and gates[self.gate]:
            self.gate += 1
        self.completed = self.gate == 5 and math.dist((x,y),self.start)<60 and sim.distance>=3000 and abs(abs(sim.car.heading)-180)<60
        self.last_time = t
        self.last_steer = sim.car.steer

def rank(r):
    return (-(r['collisions'] + bool(r['program_error'])), r['completed'],
            -r['danger'], -r['weave'], -r['sharp_steer'], -r['stops'], r['score'])

def user_score(completed,collisions,danger,sharp_steer,weave,stops,time_s):
    return 1000*int(completed)-500*collisions-20*danger-5*sharp_steer-10*weave-100*stops-time_s

def evaluate(path, name, *, external=False, seed=7, noise=0, dx=0, dy=0, dh=0, logs=True):
    path = Path(path).resolve()
    world = World.load()
    cfg = VehicleConfig(sensor_noise_std_cm=noise)
    sim = Simulation(world, cfg, seed=seed, mode='program' if external else 'manual')
    sim.car.x += dx; sim.car.y += dy; sim.car.heading += dh
    if world.collision(sim.car.polygon(cfg)):
        raise ValueError('Invalid evaluation spawn')
    if external:
        sim.load_program(path)
        sim.car.x += dx; sim.car.y += dy; sim.car.heading += dh
    else:
        spec = importlib.util.spec_from_file_location('candidate_'+name.replace('-','_'),path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        controller = module.Controller()
    metrics = Metrics(sim)
    rows = []
    error = None
    try:
        for _ in range(round(TIMEOUT_S*PHYSICS_HZ)):
            if not external and sim.steps % 6 == 0:
                sim.read_sensors()
                sim._sampled_step = sim.steps  # Avoid taking a duplicate noise sample.
                accel,handle,state = controller.step(dict(sim.sensors),.05)
                if not all(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)<=100 for v in (accel,handle)):
                    raise ValueError('Invalid controller output')
                command = (accel,handle)
            sim.step(block=True) if external else sim.step(command)
            if sim.steps % 6 == 0 or sim.collision_id or sim.program_error or sim.finished:
                metrics.sample(sim)
                row = sim.record()
                row['clearance_cm'] = world.clearance(sim.car.polygon(cfg))
                row['gates_passed'] = metrics.gate
                row['state'] = sim.state if external else state
                rows.append(row)
            if sim.collision_id or sim.program_error or sim.finished or metrics.completed:
                break
            # A stopped, latched controller cannot improve with more identical ticks.
            # Completion remains a separate safety/selection constraint, so a
            # short stopped run cannot be mistaken for a successful lap.
            if metrics.stop_duration >= 5:
                break
    except Exception as exc:
        error = repr(exc)
    finally:
        sim.close()
    elapsed = sim.time
    charged = elapsed
    score = user_score(metrics.completed,int(bool(sim.collision_id)),metrics.danger,metrics.sharp,metrics.weave,metrics.stops,charged)
    result = dict(name=name,program=str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
                  sha256=hashlib.sha256(path.read_bytes()).hexdigest(),external_worker=external,
                  completed=metrics.completed,collisions=int(bool(sim.collision_id)),collision=sim.collision_id,
                  danger=metrics.danger,sharp_steer=metrics.sharp,weave=metrics.weave,stops=metrics.stops,
                  time_s=round(charged,3),elapsed_s=round(elapsed,3),score=round(score,3),
                  gates_passed=metrics.gate,distance_m=round(sim.distance/100,3),
                  min_clearance_cm=round(metrics.min_clearance,3),
                  program_error=error or sim.program_error,seed=seed,noise_cm=noise,dx=dx,dy=dy,dh=dh,
                  final_pose=[sim.car.x,sim.car.y,sim.car.heading],final_sensors=sim.sensors,final_state=sim.state if external else locals().get('state',''),
                  map_sha256=hashlib.sha256(DEFAULT_MAP.read_bytes()).hexdigest())
    output = ROOT/'optimization'
    (output/'results').mkdir(parents=True,exist_ok=True)
    (output/'logs').mkdir(parents=True,exist_ok=True)
    (output/'results'/f'{name}.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    if logs and rows:
        with (output/'logs'/f'{name}.csv').open('w',newline='',encoding='utf-8-sig') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    if external:
        (output/'logs'/f'{name}.log').write_text(sim.program_log(),encoding='utf-8')
    return result

if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('program',type=Path)
    parser.add_argument('--name',default='manual_check')
    parser.add_argument('--external',action='store_true')
    args=parser.parse_args()
    print(json.dumps(evaluate(args.program,args.name,external=args.external),ensure_ascii=False,indent=2))
