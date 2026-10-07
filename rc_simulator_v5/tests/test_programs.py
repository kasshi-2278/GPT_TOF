from __future__ import annotations
import ast
import copy
import dataclasses
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from program_bridge import ProgramProcess, ProgramSpec, ProgramError
from simulation import Simulation
from config import PHYSICS_HZ, VehicleConfig
from world import World
from virtual_hardware import load_hardware, ESCModel, decode_pwm

ROOT=Path(__file__).resolve().parents[1]
VALUES=dict(Fr=50,FrLh=100,RrLh=150,FrRh=200,RrRh=250)

class ProgramTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.path=Path(self.tmp.name)
        self.to_close=[]
    def tearDown(self):
        for obj in self.to_close: obj.close()
        self.tmp.cleanup()
    def write(self,source,name='test_program.py'):
        p=self.path/name; p.write_text(source,encoding='utf-8'); return p
    def runner(self,source,interface='auto',**kwargs):
        p=self.write(source)
        spec=ProgramSpec.inspect(p,interface)
        worker=ProgramProcess(spec,**kwargs); self.to_close.append(worker); return worker
    def sim(self,source=None,example=None,world=None,config=None):
        s=Simulation(world or World.load(),config)
        self.to_close.append(s)
        p=ROOT/'examples'/example if example else self.write(source)
        s.load_program(p); return s

    def test_decide_one_arg(self):
        w=self.runner('def decide(s): return (s["Fr"]*.5,-20,"ok")')
        r=w.step(VALUES,0,.05); self.assertEqual((r['accel'],r['handle']),(25,-20))
    def test_decide_dt(self):
        w=self.runner('def decide(s,dt): return dt*100,0')
        self.assertEqual(w.step(VALUES,0,.05)['accel'],5)
    def test_controller_state_and_reset_process(self):
        src='class Controller:\n def __init__(self): self.a=0\n def step(self,s,dt):\n  self.a+=1\n  return self.a,0\n'
        s=self.sim(src); s.run_seconds(.1); self.assertEqual(s.car.accel,2)
        proc=s.external.proc; s.reset(); self.assertIsNotNone(proc.poll())
        s.run_seconds(.05); self.assertEqual(s.car.accel,1)
    def test_controller_decide_method(self):
        w=self.runner('class Controller:\n def decide(self,s): return 2,3')
        self.assertEqual(w.step(VALUES,0,.05)['handle'],3)
    def test_dict_output(self):
        w=self.runner('def decide(s): return {"Accel":30,"Handle":-50,"comment":"日本語"}')
        self.assertEqual(w.step(VALUES,0,.05)['comment'],'日本語')
    def test_input_has_only_five_sensors(self):
        source='def decide(s,dt):\n assert set(s)=={"Fr","FrLh","RrLh","FrRh","RrRh"}\n assert isinstance(dt,float)\n return 5,0'
        w=self.runner(source); w.step(VALUES,0,.05)
        self.assertEqual(set(w.last_payload_keys),{'op','sensors','time','dt'})
    def test_no_route_data_required(self):
        world=World.load(); del world.data['demo_route']
        world=World(world.data)
        s=self.sim('def decide(s,dt): return 10,0',world=world)
        self.assertIsNone(s.controller); s.run_seconds(1)
        self.assertIsNone(s.program_error); self.assertGreater(s.distance,5)
    def test_no_coordinate_shortcut_in_sample(self):
        tree=ast.parse((ROOT/'examples'/'sensor_drive.py').read_text())
        names={n.id for n in ast.walk(tree) if isinstance(n,ast.Name)}
        self.assertFalse({'world','pose','heading','route','x_cm','y_cm'} & names)
    def test_nan_output_stops(self):
        s=self.sim('def decide(s): return float("nan"),0'); s.run_seconds(1)
        self.assertIsNotNone(s.program_error); self.assertEqual(s.distance,0); self.assertEqual(s.car.accel,0)
    def test_infinite_output_stops(self):
        w=self.runner('def decide(s): return 0,float("inf")')
        with self.assertRaises(ProgramError): w.step(VALUES,0,.05)
    def test_out_of_range_stops(self):
        s=self.sim('def decide(s): return 101,0'); s.run_seconds(1)
        self.assertIsNotNone(s.program_error); self.assertEqual(s.car.speed,0)
    def test_bool_output_rejected(self):
        w=self.runner('def decide(s): return True,0')
        with self.assertRaises(ProgramError): w.step(VALUES,0,.05)
    def test_wrong_return_type(self):
        w=self.runner('def decide(s): return "forward"')
        with self.assertRaises(ProgramError): w.step(VALUES,0,.05)
    def test_missing_pose_argument_not_supplied(self):
        w=self.runner('def decide(s,x,y,heading): return 1,0')
        with self.assertRaisesRegex(ProgramError,'必須引数'): w.step(VALUES,0,.05)
    def test_no_api_detected(self):
        w=self.runner('value=1')
        with self.assertRaisesRegex(ProgramError,'decide'): w.step(VALUES,0,.05)
    def test_syntax_error_before_execution(self):
        p=self.write('def decide(: pass')
        with self.assertRaises(SyntaxError): ProgramSpec.inspect(p)
    def test_missing_dependency_error(self):
        w=self.runner('import no_such_package_rc_v5\ndef decide(s): return 1,0')
        with self.assertRaisesRegex(ProgramError,'ModuleNotFoundError'): w.step(VALUES,0,.05)
    def test_exception_contains_file_and_line(self):
        w=self.runner('def decide(s):\n    raise RuntimeError("テスト例外")\n')
        with self.assertRaises(ProgramError) as err: w.step(VALUES,0,.05)
        self.assertIn('test_program.py',str(err.exception)); self.assertIn('line 2',str(err.exception))
    def test_watchdog_busy_loop(self):
        w=self.runner('def decide(s):\n while True: pass',timeout=.15)
        start=time.monotonic()
        with self.assertRaises(ProgramError): w.step(VALUES,0,.05)
        self.assertLess(time.monotonic()-start,3); self.assertIsNotNone(w.proc.poll())
    def test_watchdog_import_loop(self):
        w=self.runner('while True: pass',startup_timeout=.15)
        with self.assertRaises(ProgramError): w.step(VALUES,0,.05)
        self.assertIsNotNone(w.proc.poll())
    def test_gui_style_nonblocking_timeout(self):
        s=self.sim('def decide(s):\n while True: pass'); s.program_timeout=.15
        durations=[]; start=time.monotonic()
        while not s.program_error and time.monotonic()-start<3:
            t=time.monotonic(); s.step(block=False); durations.append(time.monotonic()-t); time.sleep(.002)
        self.assertIsNotNone(s.program_error)
        self.assertLess(max(durations),.3)
        self.assertEqual(s.car.speed,0)
    def test_reload_edited_file_no_bytecode_cache(self):
        s=self.sim('def decide(s): return 11,0'); s.run_seconds(.05)
        s.program_spec.path.write_text('def decide(s): return 22,0',encoding='utf-8')
        s.reload_program(); s.run_seconds(.05); self.assertEqual(s.car.accel,22)
    def test_source_edit_requires_reload(self):
        s=self.sim('def decide(s): return 11,0')
        s.program_spec.path.write_text('def decide(s): return 22,0',encoding='utf-8')
        s.run_seconds(1); self.assertIsNotNone(s.program_error); self.assertEqual(s.distance,0)
    def test_same_directory_import(self):
        (self.path/'my_helper.py').write_text('VALUE=37',encoding='utf-8')
        w=self.runner('from my_helper import VALUE\ndef decide(s): return VALUE,0')
        self.assertEqual(w.step(VALUES,0,.05)['accel'],37)
    def test_japanese_path_with_spaces(self):
        p=self.write('def decide(s): return 25,0','走行 テスト.py')
        s=Simulation(World.load()); self.to_close.append(s); s.load_program(p); s.run_seconds(.1)
        self.assertEqual(s.car.accel,25)
    def test_print_output_separate_from_protocol(self):
        w=self.runner('def decide(s):\n print("テスト出力",flush=True)\n return 25,0')
        r=w.step(VALUES,0,.05); time.sleep(.03)
        self.assertEqual(r['accel'],25); self.assertIn('テスト出力',w.log_text())
    def test_output_flood_is_bounded(self):
        w=self.runner('def decide(s):\n print("x"*2100000)\n return 10,0',timeout=3)
        self.assertEqual(w.step(VALUES,0,.05)['accel'],10)
        self.assertLessEqual(len(w.messages),800)
    def test_explicit_stop_kills_worker_and_brakes(self):
        s=self.sim('def decide(s): return 50,0'); s.run_seconds(.3)
        proc=s.external.proc; self.assertGreater(s.car.speed,0)
        s.stop_program(); self.assertEqual(s.car.speed,0); self.assertEqual(s.car.accel,0)
        self.assertIsNotNone(proc.poll())
    def test_collision_detected_not_hidden_by_guide(self):
        s=self.sim('def decide(s): return 50,0'); s.run_seconds(20)
        self.assertIsNotNone(s.collision_id); self.assertEqual(s.car.speed,0); self.assertIsNone(s.external)
    def test_identical_sensors_ignore_world_coordinates(self):
        # Translating the world and car preserves sensor-only commands exactly.
        src='def decide(s,dt): return 10,(s["FrLh"]-s["FrRh"])*.01'
        world=World.load(); d=copy.deepcopy(world.data); dx,dy=1500,1200
        for w in d['walls']:
            for key in ('a','b'): w[key]=[w[key][0]+dx,w[key][1]+dy]
        for p in d['posts']: p['center']=[p['center'][0]+dx,p['center'][1]+dy]
        for z in d['zones']: z['polygon']=[[a+dx,b+dy] for a,b in z['polygon']]
        del d['demo_route']
        d['extents_cm']=[d['extents_cm'][0]+dx,d['extents_cm'][1]+dy,d['extents_cm'][2]+dx,d['extents_cm'][3]+dy]
        for p in [d['start'],*d['spawns'].values()]: p['x']+=dx; p['y']+=dy
        a=self.sim(src,world=world); b=self.sim(src,world=World(d))
        a.run_seconds(.5); b.run_seconds(.5)
        self.assertAlmostEqual(a.car.accel,b.car.accel)
        self.assertAlmostEqual(a.car.handle,b.car.handle,places=8)
        self.assertAlmostEqual(a.car.x+dx,b.car.x,places=6)
    def test_nonblocking_sampling_does_not_consume_noise_repeatedly(self):
        s=self.sim('def decide(s): return 0,0',config=VehicleConfig(sensor_noise_std_cm=1))
        for _ in range(10):
            if s.step(block=False): break
        sampled=dict(s.sensors)
        if s.steps==0:
            s.step(block=False); self.assertEqual(sampled,s.sensors)
    def test_guide_still_explicit_only(self):
        s=Simulation(World.load()); self.to_close.append(s)
        self.assertEqual(s.mode,'program'); self.assertIsNone(s.controller)
        s.mode='guide'; s.reset(); s.run_seconds(1)
        self.assertIsNotNone(s.controller); self.assertGreater(s.distance,0)
    def test_ui_cleanup_map_preserved(self):
        d=World.load().data
        self.assertNotIn('orange',[z['id'] for z in d['zones']])
        for z in d['zones']:
            if z['id'] in ('cyan','green','gray'): self.assertEqual(z['label'],'')


class LegacyTests(ProgramTests):
    # Only define new tests here; loader below avoids inheriting the base tests twice.
    def legacy_source(self,body):
        return 'import time\nimport Adafruit_PCA9685\npwm=Adafruit_PCA9685.PCA9685(address=0x40)\npwm.set_pwm_freq(60)\n'+body
    def test_legacy_auto_detection(self):
        spec=ProgramSpec.inspect(ROOT/'examples'/'sonic_drive_legacy_example.py')
        self.assertEqual(spec.interface,'legacy')
    def test_virtual_echo_measures_all_five(self):
        text=(ROOT/'examples'/'sonic_drive_legacy_example.py').read_text()
        source=text[:text.index('try:\n')]+'''\nfor name,trig,echo in [('Fr',15,33),('FrLh',13,40),('RrLh',35,37),('FrRh',32,31),('RrRh',36,38)]:
    value=Mesure(GPIO,time,trig,echo)
    print('MEASURE',name,round(value,3),flush=True)
time.sleep(.1)
'''
        w=self.runner(source)
        for i in range(30):
            r=w.step(VALUES,i/120,1/120)
            if r['kind']=='finished': break
        time.sleep(.03)
        data={}
        for line in w.log_text().splitlines():
            if line.startswith('MEASURE '):
                _,name,value=line.split(); data[name]=float(value)
        self.assertEqual(set(data),set(VALUES))
        for k in VALUES: self.assertLess(abs(data[k]-VALUES[k]),.5)
    def test_legacy_pwm_does_move_vehicle(self):
        s=self.sim(self.legacy_source('pwm.set_pwm(7,0,425)\npwm.set_pwm(1,0,488)\ntime.sleep(1)'))
        initial_heading=s.car.heading
        s.run_seconds(.8)
        self.assertIsNone(s.program_error); self.assertGreater(s.distance,10)
        self.assertLess((s.car.heading-initial_heading+180)%360-180,0)
    def test_legacy_sleep_uses_simulation_time(self):
        s=self.sim(self.legacy_source('pwm.set_pwm(7,0,425)\ntime.sleep(.2)\npwm.set_pwm(7,0,375)\ntime.sleep(.2)'))
        s.run_seconds(.35)
        changed=[e['time_s'] for e in s.pwm_events if e['off']==375]
        self.assertEqual(len(changed),1); self.assertAlmostEqual(changed[0],.200001,places=5)
    def test_reverse_brake_neutral_reverse_sequence(self):
        body='pwm.set_pwm(7,0,297)\ntime.sleep(.2)\npwm.set_pwm(7,0,375)\ntime.sleep(.02)\npwm.set_pwm(7,0,297)\ntime.sleep(.3)'
        s=self.sim(self.legacy_source(body)); s.run_seconds(.45)
        self.assertEqual([e['esc_stage'] for e in s.pwm_events],['BRAKE','NEUTRAL','REVERSE'])
        self.assertEqual(s.pwm_events[0]['applied_accel'],0)
        self.assertLess(s.pwm_events[-1]['applied_accel'],0)
        self.assertLess(s.car.speed,0)
    def test_reverse_without_neutral_is_brake(self):
        s=self.sim(self.legacy_source('pwm.set_pwm(7,0,297)\ntime.sleep(.5)'))
        s.run_seconds(.4); self.assertEqual(s.distance,0)
    def test_legacy_unknown_channel_errors(self):
        s=self.sim(self.legacy_source('pwm.set_pwm(3,0,400)\ntime.sleep(.1)'))
        s.run_seconds(1); self.assertIsNotNone(s.program_error); self.assertEqual(s.distance,0)
    def test_legacy_wrong_pin_errors(self):
        s=self.sim('import RPi.GPIO as GPIO\nGPIO.setmode(GPIO.BOARD)\nGPIO.setup(99,GPIO.OUT)')
        s.run_seconds(1); self.assertIn('99',s.program_error)
    def test_legacy_out_of_calibration_pwm_errors(self):
        s=self.sim(self.legacy_source('pwm.set_pwm(1,0,900)'))
        s.run_seconds(1); self.assertIsNotNone(s.program_error)
    def test_legacy_bcm_pins(self):
        s=self.sim('import time\nimport RPi.GPIO as GPIO\nGPIO.setmode(GPIO.BCM)\nGPIO.setup(22,GPIO.OUT,initial=GPIO.LOW)\nGPIO.setup(13,GPIO.IN)\nGPIO.output(22,1)\ntime.sleep(.00001)\nGPIO.output(22,0)\ntime.sleep(.02)')
        s.run_seconds(.1); self.assertIsNone(s.program_error); self.assertTrue(s.finished)
    def test_legacy_normal_exit_stops(self):
        s=self.sim(self.legacy_source('pwm.set_pwm(7,0,425)\ntime.sleep(.1)'))
        s.run_seconds(.5); self.assertTrue(s.finished); self.assertEqual(s.car.speed,0)
    def test_legacy_frequency_preserves_pulse_duration(self):
        src=self.legacy_source('pwm.set_pwm_freq(120)\npwm.set_pwm(7,0,850)\ntime.sleep(.1)')
        s=self.sim(src); s.run_seconds(.08)
        self.assertAlmostEqual(s.car.accel,40)
    def test_legacy_echo_pause_resume_grants(self):
        s=self.sim(example='sonic_drive_legacy_example.py'); s.run_seconds(.3)
        at=s.time; pose=(s.car.x,s.car.y)
        time.sleep(.03); self.assertEqual(s.time,at); self.assertEqual((s.car.x,s.car.y),pose)
        s.run_seconds(.3); self.assertIsNone(s.program_error); self.assertGreater(s.time,at)
    def test_photo_transcription_runs_with_pwm_events(self):
        s=self.sim(example='sonic_drive_photo_transcribed.py'); s.run_seconds(5)
        self.assertIsNone(s.program_error); self.assertGreater(len(s.pwm_events),10)
    def test_hardware_duplicate_pins_rejected(self):
        d=load_hardware(ROOT/'hardware.json'); d['sensor_pins_board']['FrLh'][0]=15
        p=self.path/'bad.json'; p.write_text(json.dumps(d))
        with self.assertRaises(ValueError): load_hardware(p)


def suite():
    result=unittest.TestSuite()
    for cls in (ProgramTests,LegacyTests):
        for name in cls.__dict__:
            if name.startswith('test_'): result.addTest(cls(name))
    return result

if __name__=='__main__':
    outcome=unittest.TextTestRunner(verbosity=2).run(suite())
    raise SystemExit(not outcome.wasSuccessful())
