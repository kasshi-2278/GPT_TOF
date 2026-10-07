"""Exercise real Tk controls. Needs a display (or Xvfb); Pillow only for PNGs.

python gui_smoke_v5.py [--capture]
"""
import argparse
import json
from pathlib import Path
import tempfile
import time
import tkinter as tk
from unittest.mock import patch
from rc_simulator_v5 import App, MODES

ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--capture',action='store_true');args=parser.parse_args()
    root=tk.Tk();app=App(root,animate=True);root.geometry('1440x980+0+0')
    checks=[]
    def pump_until(condition, timeout=4):
        end=time.monotonic()+timeout
        while time.monotonic()<end and not condition():
            root.update();time.sleep(.003)
        if not condition(): raise AssertionError('GUI operation timed out: '+str(condition))
    try:
        root.update()
        assert app.sim.mode=='program' and app.sim.external is None and app.sim.controller is None
        checks.append('default_external_python_no_guide_or_worker_before_start')
        with patch('tkinter.filedialog.askopenfilename',return_value=str(ROOT/'examples'/'minimal_decide.py')),patch('tkinter.messagebox.askyesno',return_value=True):
            app.py_load_button.invoke()
        assert app.sim.program_spec.path.name=='minimal_decide.py'
        checks.append('real_PY_load_button_and_dialog_flow')
        app.toggle();pump_until(lambda:app.sim.time>.5)
        assert app.sim.car.speed>0 and not app.sim.program_error
        checks.append('nonblocking_start_and_physics_movement')
        app.toggle();t=app.sim.time
        for _ in range(10):root.update();time.sleep(.01)
        assert app.sim.time==t
        checks.append('pause_freezes_virtual_time')
        app.single_step();pump_until(lambda:app.pending_steps==0)
        assert abs(app.sim.time-t-.05)<1e-6
        checks.append('single_step_advances_50ms')
        app.reset();assert app.sim.time==0 and app.sim.external is None
        checks.append('reset_closes_worker_and_rewinds')
        app.select_python(ROOT/'examples'/'sonic_drive_legacy_example.py',confirm=False)
        app.toggle();pump_until(lambda:app.sim.time>.4)
        assert app.sim.program_spec.interface=='legacy' and len(app.sim.pwm_events)>2
        app.show_program_log();root.update()
        assert app.log_window.winfo_exists()
        app.log_window.destroy();app.log_window=None
        checks.append('legacy_GPIO_PWM_and_live_log_window')
        proc=app.sim.external.proc;app.stop_python()
        assert proc.poll() is not None and app.sim.car.speed==0
        checks.append('stop_terminates_worker_and_vehicle')
        with tempfile.TemporaryDirectory() as folder:
            file=Path(folder)/'hang.py';file.write_text('def decide(s):\n while True: pass\n')
            app.select_python(file,confirm=False);app.sim.program_timeout=.2;app.toggle()
            pump_until(lambda:app.sim.program_error is not None,timeout=3)
            assert app.sim.car.speed==0
            checks.append('unresponsive_python_stops_without_freezing_Tk')
        app.sim.program_timeout=1.0
        app.select_python(ROOT/'examples'/'sensor_drive.py',confirm=False)
        app.mode_var.set(next(k for k,v in MODES.items() if v=='guide'));app.change_mode()
        assert app.sim.controller is not None
        app.mode_var.set(next(k for k,v in MODES.items() if v=='program'));app.change_mode()
        assert app.sim.controller is None and not app.show_route.get()
        checks.append('guide_explicit_only_and_no_guide_on_switch_back')
        root.geometry('1120x700+0+0');root.update()
        assert app.py_load_button.winfo_rootx()+app.py_load_button.winfo_width()<1120
        checks.append('loader_visible_at_minimum_window_size')
        root.geometry('1440x980+0+0');root.update();app.fit()
        if args.capture:
            from PIL import ImageGrab
            app.rate_var.set('5.0');app.toggle()
            pump_until(lambda:app.sim.time>=16,timeout=9)
            assert app.running and not app.sim.program_error and app.sim.mode=='program'
            app.refresh();root.update()
            ImageGrab.grab().crop((0,0,1440,980)).save(ROOT/'preview_v5.png')
            app.follow.set(True);app.options_changed();app.refresh();root.update()
            ImageGrab.grab().crop((0,0,1440,980)).save(ROOT/'preview_v5_close.png')
            checks.append('screenshots_from_running_external_sensor_controller')
        payload={'gui_checks_passed':len(checks),'checks':checks,'tk_version':tk.TkVersion,
                 'mode':app.sim.mode,'program':app.sim.program_spec.path.name,
                 'control_uses_guide':app.sim.controller is not None,'platform':'Linux/Xvfb (not Windows/macOS tested)'}
        (ROOT/'reports'/'gui_v5.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(payload,ensure_ascii=False,indent=2))
    finally:
        app.close()

if __name__=='__main__':main()
