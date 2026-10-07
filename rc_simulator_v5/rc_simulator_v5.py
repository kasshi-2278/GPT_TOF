#!/usr/bin/env python3
"""RC simulator V5: load external Python and drive from ultrasonic sensors.

Run: python rc_simulator_v5.py
Test (no GUI): python validate_v5.py
Tkinter is included in many desktop Python distributions. Pygame is not used.
"""
from __future__ import annotations
import argparse
import csv
import math
import sys
import time
import traceback
import webbrowser
from pathlib import Path

from config import VehicleConfig, PHYSICS_HZ
from geometry import direction, local_to_world, rectangle, wall_polygon, clamp
from simulation import Simulation
from world import World, DEFAULT_MAP
from sonar_mapping import SonarMap
from lap_timer import LapTimer
from vehicle_view import VehicleView

ROOT=Path(__file__).resolve().parent
SENSOR_COLORS={'Fr':'#bf8314','FrLh':'#078c98','RrLh':'#337bba','FrRh':'#c76536','RrRh':'#b84973'}
MODES={'外部Python（超音波のみ）':'program','手動運転':'manual','ガイド走行（比較用・自己位置使用）':'guide','写真の判断部（比較用）':'photo'}
INTERFACES={'自動判定':'auto','関数 / Controller':'function','実車互換 GPIO/PWM':'legacy'}

class Camera:
    def __init__(self):
        self.scale=1.0; self.cx=515.0; self.cy=305.0; self.width=1000; self.height=700
    def fit(self,extent,width,height):
        self.width=max(1,width); self.height=max(1,height)
        a,b,c,d=extent
        self.scale=min(self.width/(c-a+155),self.height/(d-b+170))
        self.cx=(a+c)/2+10; self.cy=(b+d)/2-8
    def screen(self,x,y):
        return (self.width/2+(x-self.cx)*self.scale,self.height/2-(y-self.cy)*self.scale)
    def world(self,x,y):
        return (self.cx+(x-self.width/2)/self.scale,self.cy-(y-self.height/2)/self.scale)
    def zoom(self,factor,x,y):
        before=self.world(x,y)
        self.scale=clamp(self.scale*factor,0.2,6.0)
        after=self.world(x,y)
        self.cx += before[0]-after[0]; self.cy += before[1]-after[1]

class App:
    def __init__(self,root,map_path=DEFAULT_MAP,animate=True):
        import tkinter as tk
        from tkinter import ttk, font
        self.tk=tk; self.ttk=ttk; self.root=root
        self.root.title('RC シミュレーター V5｜外部Python・超音波センサー走行')
        self.root.geometry(f'{min(1440,max(1120,self.root.winfo_screenwidth()-80))}x{min(900,max(700,self.root.winfo_screenheight()-80))}')
        self.root.minsize(1120,700)
        self.root.configure(bg='#142337')
        families=set(font.families(root))
        self.family=next((f for f in ['Yu Gothic UI','Meiryo','Noto Sans CJK JP','Hiragino Sans','Arial'] if f in families),'TkDefaultFont')
        self.root.option_add('*Font',(self.family,10))
        self.font=(self.family,10); self.smallfont=(self.family,9)
        self.map_path=Path(map_path)
        self.sim=Simulation(World.load(self.map_path))
        self.sim.load_program(ROOT/'examples'/'sonic_drive_three_laps.py')
        self.closed=False; self.pending_steps=0; self.log_window=None
        self.vehicle_view=None; self.lap_timer=LapTimer(self.sim,3); self.sonar_map=SonarMap()
        self.camera=Camera(); self.running=False; self.accumulator=0.0
        self.keys=set(); self.last_time=time.perf_counter(); self.animate=animate
        self.pan_anchor=None; self.wall_anchor=None; self.last_panel_time=0.0
        self.show_route=tk.BooleanVar(value=False)
        self.show_sensors=tk.BooleanVar(value=True)
        self.show_labels=tk.BooleanVar(value=True)
        self.follow=tk.BooleanVar(value=False)
        self.edit=tk.BooleanVar(value=False)
        self.mode_var=tk.StringVar(value=list(MODES)[0])
        self.rate_var=tk.StringVar(value='1.0')
        self.interface_var=tk.StringVar(value='自動判定')
        self.program_var=tk.StringVar(value='sonic_drive_three_laps.py  |  関数 / Controller')
        self.spawn_var=tk.StringVar(value='デモ開始')
        self.notice=tk.StringVar(value='ホイール：拡大縮小  /  右ドラッグ：移動  /  F：全体  /  Space：開始・停止')
        self.vars={}
        self._style()
        self._build()
        self._bind()
        self.root.update_idletasks()
        self.fit()
        self.refresh()
        if animate: self.root.after(16,self.tick)

    def _style(self):
        style=self.ttk.Style(self.root)
        if 'clam' in style.theme_names(): style.theme_use('clam')
        style.configure('TButton',padding=(10,6),font=self.font,background='#eef2f5',foreground='#1a3148')
        style.map('TButton',background=[('active','#d9e4ec')])
        style.configure('Accent.TButton',background='#22796c',foreground='white')
        style.map('Accent.TButton',background=[('active','#2f9687')])
        style.configure('TCombobox',padding=4,font=self.font)

    def _button(self,frame,text,command,style='TButton'):
        b=self.ttk.Button(frame,text=text,command=command,style=style)
        b.pack(side='left',padx=3,pady=5); return b

    def _build(self):
        tk=self.tk; ttk=self.ttk
        header=tk.Frame(self.root,bg='#142337',height=69); header.pack(fill='x')
        tk.Label(header,text='RC SIMULATOR',font=(self.family,20,'bold'),bg='#142337',fg='white').pack(side='left',padx=(20,14),pady=13)
        tk.Label(header,text='V5  /  外部Python → 超音波 → 操舵・駆動',font=(self.family,11),bg='#142337',fg='#b9cddb').pack(side='left')
        tk.Label(header,text='1030 cm  ·  2D / cm',font=(self.family,11,'bold'),bg='#142337',fg='#a7d4cb').pack(side='right',padx=23)
        bar=tk.Frame(self.root,bg='#e6ebef'); bar.pack(fill='x')
        self.start_button=self._button(bar,'▶ 開始',self.toggle,'Accent.TButton')
        self._button(bar,'リセット',self.reset)
        self._button(bar,'1ステップ',self.single_step)
        self.mode_box=ttk.Combobox(bar,textvariable=self.mode_var,values=list(MODES),state='readonly',width=30)
        self.mode_box.pack(side='left',padx=8)
        self.mode_box.bind('<<ComboboxSelected>>',lambda e:self.change_mode())
        tk.Label(bar,text='時間倍率',bg='#e6ebef').pack(side='left',padx=(6,3))
        rate=ttk.Combobox(bar,textvariable=self.rate_var,values=['0.25','0.5','1.0','2.0','5.0'],state='readonly',width=5)
        rate.pack(side='left',padx=3)
        self._button(bar,'全体 F',self.fit)
        self._button(bar,'CSV保存',self.export_csv)
        # A second, quiet toolbar remains readable on smaller windows.
        tools=tk.Frame(self.root,bg='#f1f4f6'); tools.pack(fill='x')
        for text,var in [('ガイド経路',self.show_route),('センサー線',self.show_sensors),('注記',self.show_labels),('車両追従 C',self.follow),('壁を追加（ドラッグ）',self.edit)]:
            tk.Checkbutton(tools,text=text,variable=var,command=self.options_changed,bg='#f1f4f6',activebackground='#f1f4f6',font=self.smallfont).pack(side='left',padx=5)
        self._button(tools,'マップ読込',self.load_map)
        self._button(tools,'マップ保存',self.save_map)
        self._button(tools,'元図面',self.open_source)
        tk.Label(tools,text='開始点',bg='#f1f4f6',font=self.smallfont).pack(side='left',padx=4)
        spawn=ttk.Combobox(tools,textvariable=self.spawn_var,values=['デモ開始','スタート1','スタート2','スタート3'],state='readonly',width=10)
        spawn.pack(side='left',padx=4)
        spawn.bind('<<ComboboxSelected>>',lambda e:self.change_spawn())
        pybar=tk.Frame(self.root,bg='#dbeae5'); pybar.pack(fill='x')
        self.py_load_button=self._button(pybar,'PY読込',self.load_python,'Accent.TButton')
        self._button(pybar,'再読込',self.reload_python)
        self._button(pybar,'Python停止',self.stop_python)
        self._button(pybar,'実行ログ',self.show_program_log)
        self._button(pybar,'超音波・判断 V',self.show_vehicle_view)
        interface=ttk.Combobox(pybar,textvariable=self.interface_var,values=list(INTERFACES),state='readonly',width=22)
        interface.pack(side='left',padx=6)
        interface.bind('<<ComboboxSelected>>',lambda e:self.interface_changed())
        tk.Label(pybar,textvariable=self.program_var,bg='#dbeae5',fg='#164d41',font=self.smallfont,anchor='w').pack(side='left',fill='x',expand=True,padx=8)
        main=tk.Frame(self.root,bg='#d9e1e7'); main.pack(fill='both',expand=True)
        aside=tk.Frame(main,bg='#142337',width=304); aside.pack(side='right',fill='y'); aside.pack_propagate(False)
        pc=tk.Canvas(aside,bg='#142337',highlightthickness=0)
        scrollbar=ttk.Scrollbar(aside,orient='vertical',command=pc.yview); scrollbar.pack(side='right',fill='y')
        pc.configure(yscrollcommand=scrollbar.set); pc.pack(side='left',fill='both',expand=True)
        self.panel=tk.Frame(pc,bg='#142337')
        panel_window=pc.create_window(0,0,window=self.panel,anchor='nw')
        self.panel.bind('<Configure>',lambda e:pc.configure(scrollregion=pc.bbox('all')))
        pc.bind('<Configure>',lambda e:pc.itemconfigure(panel_window,width=e.width))
        self.canvas=tk.Canvas(main,bg='#f8f8f5',highlightthickness=0,takefocus=True)
        self.canvas.pack(side='left',fill='both',expand=True,padx=(0,1))
        p=self.panel
        def heading(text):
            tk.Label(p,text=text,font=(self.family,10,'bold'),bg='#142337',fg='#8fa9bc',anchor='w').pack(fill='x',padx=18,pady=(6,2))
        def label(key,style=None,initial=''):
            var=tk.StringVar(value=initial); self.vars[key]=var
            tk.Label(p,textvariable=var,font=style or (self.family,10),bg='#142337',fg='#f1f5f7',anchor='w',justify='left',wraplength=270).pack(fill='x',padx=18,pady=2)
        heading('SIMULATION')
        label('status',(self.family,16,'bold'),'PAUSED')
        label('time'); label('position')
        heading('LAP TIMES / ３周')
        label('lap_times')
        heading('ULTRASONIC  /  cm')
        for name in ['Fr','FrLh','RrLh','FrRh','RrRh']:
            row=tk.Frame(p,bg='#142337'); row.pack(fill='x',padx=18,pady=2)
            tk.Label(row,text='●',fg=SENSOR_COLORS[name],bg='#142337',font=(self.family,13)).pack(side='left')
            sensor_label={'Fr':'正面','FrLh':'左45°','RrLh':'左90°','FrRh':'右45°','RrRh':'右90°'}[name]
            tk.Label(row,text=sensor_label,width=6,anchor='w',fg='#d8e3eb',bg='#142337',font=(self.family,11)).pack(side='left',padx=(4,0))
            var=tk.StringVar(); self.vars[name]=var
            tk.Label(row,textvariable=var,anchor='e',fg='white',bg='#142337',font=(self.family,12,'bold')).pack(side='right')
        heading('CONTROL')
        label('commands'); label('motion'); label('pwm')
        heading('STATE')
        label('state')
        heading('PYTHON / INPUT')
        label('python')
        label('input_scope')
        heading('このマップの前提')
        tk.Label(p,text='赤・白の板と支柱：当たり判定あり\n色付き領域：路面表示のみ\n斜め部分・開口部：一部推定\n車体・センサー配置：未実測',font=self.smallfont,bg='#142337',fg='#c0d0dd',anchor='w',justify='left').pack(fill='x',padx=18,pady=2)
        tk.Label(p,text='外部Pythonには距離5個と時刻のみ渡します。\n座標・経路は画面/比較用です。\n信頼できる.pyのみ実行してください。',font=(self.family,8,'bold'),bg='#20364a',fg='#f4d58d',anchor='w',justify='left',wraplength=250,padx=8,pady=5).pack(fill='x',padx=12,pady=6)
        footer=tk.Frame(self.root,bg='#e6ebef'); footer.pack(fill='x')
        tk.Label(footer,textvariable=self.notice,bg='#e6ebef',fg='#284052',font=self.smallfont,anchor='w').pack(fill='x',padx=12,pady=6)

    def _bind(self):
        c=self.canvas
        c.bind('<Configure>',self.on_resize)
        c.bind('<MouseWheel>',lambda e:self.zoom(1.18 if e.delta>0 else 1/1.18,e.x,e.y))
        c.bind('<Button-4>',lambda e:self.zoom(1.18,e.x,e.y))
        c.bind('<Button-5>',lambda e:self.zoom(1/1.18,e.x,e.y))
        c.bind('<ButtonPress-3>',self.pan_start); c.bind('<B3-Motion>',self.pan_move)
        c.bind('<ButtonRelease-3>',lambda e:setattr(self,'pan_anchor',None))
        c.bind('<ButtonPress-1>',self.left_start); c.bind('<B1-Motion>',self.left_move); c.bind('<ButtonRelease-1>',self.left_end)
        c.bind('<Shift-Button-1>',self.place_car)
        c.bind('<Shift-Button-3>',self.delete_wall)
        self.root.bind('<KeyPress>',self.key_down)
        self.root.bind('<KeyRelease>',lambda e:self.keys.discard(e.keysym.lower()))
        self.root.bind('<FocusOut>',lambda e:self.keys.clear())
        self.root.protocol('WM_DELETE_WINDOW',self.close)

    def key_down(self,event):
        key=event.keysym.lower()
        self.keys.add(key)
        if event.widget.winfo_class() in ['TCombobox','Entry','TEntry']: return
        if key=='space': self.toggle(); return 'break'
        if key=='r': self.reset()
        elif key=='f': self.fit()
        elif key=='c': self.follow.set(not self.follow.get()); self.options_changed()
        elif key=='v': self.show_vehicle_view()
        elif key=='m':
            values=list(MODES); self.mode_var.set(values[(values.index(self.mode_var.get())+1)%len(values)]); self.change_mode()
        elif key=='escape':
            self.running=False; self.keys.clear(); self.start_button.configure(text='▶ 開始')
        elif key in ['plus','equal','minus']:
            value=clamp(float(self.rate_var.get())+(-0.25 if key=='minus' else 0.25),0.25,5)
            self.rate_var.set(str(value))

    def on_resize(self,event):
        self.camera.width=event.width; self.camera.height=event.height
        # A resize returns to overview instead of clipping coordinates.
        self.fit()

    def fit(self):
        self.follow.set(False)
        self.root.update_idletasks()
        self.camera.fit(self.sim.world.extents,self.canvas.winfo_width(),self.canvas.winfo_height())
        self.draw_static(); self.draw_dynamic()

    def zoom(self,factor,x,y):
        self.follow.set(False); self.camera.zoom(factor,x,y)
        self.draw_static(); self.draw_dynamic()

    def pan_start(self,event):
        self.follow.set(False); self.pan_anchor=(event.x,event.y,self.camera.cx,self.camera.cy)
    def pan_move(self,event):
        if self.pan_anchor:
            x,y,cx,cy=self.pan_anchor
            self.camera.cx=cx-(event.x-x)/self.camera.scale
            self.camera.cy=cy+(event.y-y)/self.camera.scale
            self.draw_static(); self.draw_dynamic()

    def options_changed(self):
        if self.edit.get(): self.running=False; self.start_button.configure(text='▶ 開始')
        if self.follow.get():
            self.camera.scale=max(self.camera.scale,2.5)
            self.camera.cx=self.sim.car.x; self.camera.cy=self.sim.car.y
        self.draw_static(); self.draw_dynamic()

    def toggle(self):
        if self.sim.collision_id or self.sim.finished or self.sim.program_error or self.sim.program_stopped:
            self.notice.set('走行は終了しています。リセットして再開してください。'); return
        if self.sim.mode=='program' and self.sim.program_spec is None:
            self.notice.set('先にPY読込で.pyファイルを選択してください。'); return
        self.pending_steps=0
        self.edit.set(False); self.running=not self.running
        self.accumulator=0; self.last_time=time.perf_counter()
        self.start_button.configure(text='Ⅱ 一時停止' if self.running else '▶ 開始')
        self.canvas.focus_set(); self.refresh()

    def reset(self):
        self.pending_steps=0
        self.running=False; self.start_button.configure(text='▶ 開始'); self.accumulator=0
        try: self.sim.reset()
        except ValueError as exc: self.error('初期位置を設定できません',exc); return
        self.notice.set('リセットしました。追加した壁は保持されます。Shift＋右クリックで追加壁を削除できます。')
        self.refresh()

    def single_step(self):
        self.running=False; self.start_button.configure(text='▶ 開始')
        if self.sim.mode=='program' and self.sim.program_spec is None:
            self.notice.set('先にPY読込を選択してください。'); return
        self.pending_steps=PHYSICS_HZ//20
        self._step_once_poll()

    def _step_once_poll(self):
        if self.closed or not self.pending_steps: return
        for _ in range(self.pending_steps):
            if not self.sim.step(self.manual_command(),block=False): break
            self.pending_steps-=1
            if self.sim.collision_id or self.sim.finished or self.sim.program_error or self.sim.program_stopped:
                self.pending_steps=0; break
        self.refresh()
        if self.pending_steps:
            self.root.after(5,self._step_once_poll)

    def change_mode(self):
        self.pending_steps=0
        self.running=False; self.start_button.configure(text='▶ 開始')
        self.sim.mode=MODES[self.mode_var.get()]
        try: self.sim.reset()
        except ValueError as exc:
            self.sim.mode='program'; self.mode_var.set(list(MODES)[0]); self.sim.reset()
            self.error('このモードは利用できません',exc)
        self.accumulator=0
        if self.sim.mode!='guide': self.show_route.set(False)
        self.draw_static()
        self.notice.set('モードを変更し、初期位置に戻しました。手動は W/S または ↑/↓、A/D または ←/→。')
        self.refresh()

    def change_spawn(self):
        choices={'デモ開始':'demo','スタート1':'start1','スタート2':'start2','スタート3':'start3'}
        self.sim.spawn_name=choices[self.spawn_var.get()]
        self.reset()
        self.notice.set('開始点を変更しました。初期向きはデモ用に設定した仮の向きです。')

    def manual_command(self):
        forward=bool(self.keys & {'w','up'}); back=bool(self.keys & {'s','down'})
        left=bool(self.keys & {'a','left'}); right=bool(self.keys & {'d','right'})
        return (40.0*(forward-back),75.0*(left-right))

    def left_start(self,event):
        self.canvas.focus_set()
        if self.edit.get():
            self.running=False; self.start_button.configure(text='▶ 開始')
            self.wall_anchor=self.camera.world(event.x,event.y)
    def left_move(self,event):
        if self.wall_anchor:
            self.canvas.delete('draft')
            self.canvas.create_line(*self.camera.screen(*self.wall_anchor),event.x,event.y,fill='#a85292',width=4,dash=(5,3),tags='draft')
    def left_end(self,event):
        if self.wall_anchor:
            a=self.wall_anchor; b=self.camera.world(event.x,event.y); self.wall_anchor=None
            self.canvas.delete('draft')
            if self.sim.world.add_wall(a,b):
                if self.sim.world.collision(self.sim.car.polygon(self.sim.config)):
                    self.sim.world.data['walls'].pop(); self.sim.world.rebuild()
                    self.notice.set('車両と重なる壁は追加できません。')
                else: self.notice.set('追加壁を保存する場合は「マップ保存」。ガイド経路は自動再計画しません。')
            if self.sim.mode=='program': self.sim.invalidate_program()
            self.sim.read_sensors(); self.draw_static(); self.refresh()

    def place_car(self,event):
        if self.running: return 'break'
        x,y=self.camera.world(event.x,event.y); c=self.sim.car
        old=(c.x,c.y)
        c.x=x; c.y=y
        if self.sim.world.collision(c.polygon(self.sim.config)):
            c.x,c.y=old; self.notice.set('壁・支柱に重なっているため置けません。')
        else:
            c.speed=0; c.accel=0; c.handle=0; c.steer=0
            self.sim.close_program()
            self.mode_var.set('手動運転'); self.sim.mode='manual'
            self.sim.collision_id=None; self.sim.finished=False; self.sim.program_error=None; self.sim.program_stopped=False; self.sim.state='手動：配置変更'
            self.sim.trail.clear(); self.notice.set('車両を移動し手動モードにしました。ガイド走行はモード選択時に初期位置へ戻ります。')
        self.sim.read_sensors(); self.refresh(); return 'break'

    def delete_wall(self,event):
        if self.running: return 'break'
        if self.sim.world.remove_added(self.camera.world(event.x,event.y)):
            if self.sim.mode=='program': self.sim.invalidate_program()
            self.sim.read_sensors(); self.draw_static(); self.refresh()
        return 'break'

    def error(self,title,error):
        from tkinter import messagebox
        messagebox.showerror(title,str(error),parent=self.root)

    def load_map(self):
        from tkinter import filedialog
        self.running=False; self.start_button.configure(text='▶ 開始')
        path=filedialog.askopenfilename(parent=self.root,title='マップJSONを開く',initialdir=str(ROOT/'maps'),filetypes=[('Map JSON','*.json')])
        if not path: return
        try:
            new=Simulation(World.load(path),self.sim.config)
            if self.sim.program_spec:
                new.load_program(self.sim.program_spec.path,self.sim.program_spec.interface)
            self.sim.close()
            self.sim=new; self.map_path=Path(path); self.mode_var.set(list(MODES)[0]); self.spawn_var.set('デモ開始'); self.fit(); self.refresh()
        except (OSError,ValueError,KeyError,TypeError) as exc: self.error('マップを読み込めません',exc)

    def save_map(self):
        from tkinter import filedialog
        path=filedialog.asksaveasfilename(parent=self.root,title='マップを別名で保存',defaultextension='.json',initialfile='course_custom.json',filetypes=[('Map JSON','*.json')])
        if path:
            try: self.sim.world.save(path); self.notice.set(f'マップを保存しました：{Path(path).name}')
            except (OSError,ValueError) as exc: self.error('保存できません',exc)

    def export_csv(self):
        from tkinter import filedialog
        if not self.sim.logs: self.notice.set('先に走行するか、1ステップ実行してください。'); return
        path=filedialog.asksaveasfilename(parent=self.root,title='走行ログを保存',defaultextension='.csv',initialfile='rc_run.csv',filetypes=[('CSV','*.csv')])
        if path:
            try:
                with open(path,'w',encoding='utf-8-sig',newline='') as stream:
                    writer=csv.DictWriter(stream,fieldnames=list(self.sim.logs[0]))
                    writer.writeheader(); writer.writerows(self.sim.logs)
                self.notice.set(f'ログを保存しました：{Path(path).name}')
            except OSError as exc: self.error('ログを保存できません',exc)

    def open_source(self):
        webbrowser.open((ROOT/'source'/'course_drawing.jpeg').as_uri())

    def wline(self,points,**options):
        xy=[value for p in points for value in self.camera.screen(*p)]
        return self.canvas.create_line(*xy,**options)
    def wpoly(self,points,**options):
        xy=[value for p in points for value in self.camera.screen(*p)]
        return self.canvas.create_polygon(*xy,**options)
    def wtext(self,x,y,text,**options):
        return self.canvas.create_text(*self.camera.screen(x,y),text=text,font=self.smallfont,**options)

    def draw_static(self):
        cv=self.canvas; cv.delete('static'); cv.delete('dynamic')
        world=self.sim.world; ex=world.extents
        gridleft=int(math.floor(ex[0]/50)*50); gridright=int(math.ceil(ex[2]/50)*50)
        for x in range(gridleft,gridright+1,50):
            self.wline([(x,ex[1]),(x,ex[3])],fill='#e6e8e6',width=1,tags='static')
        for y in range(int(ex[1]),int(ex[3])+1,50):
            self.wline([(ex[0],y),(ex[2],y)],fill='#e6e8e6',width=1,tags='static')
        for z in world.data.get('zones',[]):
            self.wpoly(z['polygon'],fill=z.get('color','#dddddd'),outline='',tags='static')
            if z['id'].startswith('P'):
                poly=z['polygon']; self.wline(poly+[poly[0]],fill={'P1':'#65845a','P2':'#bb6267','P3':'#526fa6'}.get(z['id'],'#999999'),width=2,dash=(5,2),tags='static')
        if self.show_route.get() and self.sim.mode=='guide' and world.data.get('demo_route'):
            route=world.data['demo_route']['points']
            self.wline(route+[route[0]],fill='#4a9c89',width=2,dash=(5,4),tags='static')
            self.wtext(660,580,'ガイド経路（デモ）',fill='#378775',tags='static')
        palette={'red':'#a84f43','white':'#eceae1','added':'#aa5d99'}
        for w in world.data['walls']:
            # Display thickness >= 3px for legibility; physical thickness is NOT altered.
            self.wline([w['a'],w['b']],fill='#4f514f',width=max(4,w['thickness_cm']*self.camera.scale+2),capstyle='round',tags='static')
            self.wline([w['a'],w['b']],fill=palette.get(w.get('color'),w.get('color','#666666')),width=max(2,w['thickness_cm']*self.camera.scale),capstyle='round',tags='static')
        for p in world.data['posts']:
            sx,sy=self.camera.screen(*p['center']); r=max(3.4,p['radius_cm']*self.camera.scale)
            cv.create_oval(sx-r,sy-r,sx+r,sy+r,fill='#e8c276',outline='#554a35',width=1.4,tags='static')
        # Diagram arrows are decoration, never invisible collision objects.
        self.wline([(459,500),(459,573)],fill='#1e5066',width=3,arrow='last',tags='static')
        for marker in world.data.get('start_markers',[]):
            x=marker['x']
            self.wline([(x,marker['y0']),(x,marker['y1'])],fill='#87999c',dash=(3,4),width=1,tags='static')
            if self.show_labels.get(): self.wtext(x+12,(marker['y0']+marker['y1'])/2,marker['name'],fill='#637a7f',tags='static',angle=90)
        if self.show_labels.get():
            for z in world.data['zones']:
                if z.get('label'):
                    poly=z['polygon']; x=sum(p[0] for p in poly)/len(poly); y=sum(p[1] for p in poly)/len(poly)
                    self.wtext(x,y,z['label'],fill='#344c53',tags='static')
            self.wtext(525,520,'機構未確定',fill='#517e8a',tags='static')
        self.wpoly([(-15,-7),(-15,7),(-2,0)],fill='#415b64',tags='static')
        self.wtext(-15,-18,'原点',fill='#657373',tags='static')
        cv.tag_lower('static')

    def draw_dynamic(self):
        cv=self.canvas; cv.delete('dynamic')
        s=self.sim; c=s.car; cfg=s.config
        if len(s.trail)>1:
            self.wline(list(s.trail),fill='#4a719a',width=2,tags='dynamic')
        if self.show_sensors.get():
            # Re-evaluate ideal geometry for display only. Does not consume RNG or update controller readings.
            for name,(f,l,angle) in cfg.sensor_specs().items():
                origin=local_to_world(c.x,c.y,c.heading,f,l)
                d,hit=s.world.raycast(origin,c.heading+angle,cfg.sensor_range_cm)
                col=SENSOR_COLORS[name]
                self.wline([origin,hit],fill=col,width=1.5,tags='dynamic')
                hx,hy=self.camera.screen(*hit)
                cv.create_oval(hx-2,hy-2,hx+2,hy+2,fill=col,outline='',tags='dynamic')
        self.wpoly(c.polygon(cfg),fill='#356da0' if not s.collision_id else '#c35349',outline='#173a58',width=1.5,tags='dynamic')
        # Rear wheels align with body; front wheels rotate with physical left-positive steer.
        for axle in [cfg.rear_axle_cm,cfg.front_axle_cm]:
            for side in [-1,1]:
                wx,wy=local_to_world(c.x,c.y,c.heading,axle,side*(cfg.width_cm/2+1.0))
                heading=c.heading-c.steer if axle==cfg.front_axle_cm else c.heading
                self.wpoly(rectangle(wx,wy,heading,cfg.tire_diameter_cm,2.8),fill='#253440',outline='#172128',tags='dynamic')
        arrow_start=local_to_world(c.x,c.y,c.heading,-cfg.length_cm*.15,0)
        arrow_end=local_to_world(c.x,c.y,c.heading,cfg.length_cm*.40,0)
        self.wline([arrow_start,arrow_end],fill='white',width=1.8,arrow='last',tags='dynamic')
        if self.show_route.get() and s.mode=='guide':
            px,py=self.camera.screen(*s.controller.target)
            cv.create_oval(px-3,py-3,px+3,py+3,outline='#167f6a',width=2,tags='dynamic')
        # Scale bar remains in screen coordinates, at every zoom level.
        scale_cm=100 if self.camera.scale<2 else 20
        xx,yy=20,self.camera.height-22
        cv.create_line(xx,yy,xx+scale_cm*self.camera.scale,yy,fill='#395666',width=3,tags='dynamic')
        cv.create_text(xx,yy-11,text=f'{scale_cm} cm',anchor='w',font=self.smallfont,fill='#395666',tags='dynamic')
        cv.tag_raise('dynamic')

    def update_panel(self):
        s=self.sim; c=s.car; v=self.vars
        status='PY ERROR' if s.program_error else 'STOPPED' if s.program_stopped else 'COLLISION' if s.collision_id else 'FINISHED' if s.finished else 'RUNNING' if self.running else 'PAUSED'
        v['status'].set(status)
        v['time'].set(f'{s.time:7.2f} s  |  {s.distance/100:5.2f} m' + (f'  |  {s.laps} 周' if s.laps is not None else ''))
        v['position'].set(f'x {c.x:6.1f} / y {c.y:6.1f} cm   方位 {c.heading:+.1f}°')
        lap_text=[f'{i}周目  {value:.2f} s' for i,value in enumerate(self.lap_timer.lap_times,1)]
        if self.lap_timer.completed:lap_text.append(f'３周合計  {self.lap_timer.total_time:.2f} s')
        else:lap_text.append(f'{len(lap_text)+1}周目 計測中  {max(0,s.time-self.lap_timer.last_crossing_time):.2f} s')
        v['lap_times'].set('\n'.join(lap_text))
        for name in SENSOR_COLORS: v[name].set(f'{s.sensors.get(name,0):6.1f}')
        v['commands'].set(f'Accel {c.accel:+6.1f}     Handle {c.handle:+6.1f}')
        v['motion'].set(f'速度 {c.speed:5.1f} cm/s   操舵 {c.steer:+5.1f}°')
        t,p=c.pwm(); v['pwm'].set(f'PWM : throttle {t}  /  steer {p}')
        v['state'].set(str(s.state).split(' | target_deg=')[0])
        spec=s.program_spec
        if s.mode=='program' and spec:
            label='実車互換 GPIO/PWM' if spec.interface=='legacy' else '関数 / Controller'
            v['python'].set(f'{spec.path.name}\n{label}')
            self.program_var.set(f'{spec.path.name}  |  {label}')
            v['input_scope'].set('Fr / FrLh / RrLh / FrRh / RrRh\n座標・方位・ガイド経路は\n入力に含めていません')
            if spec.interface=='legacy':
                t,p=s.last_raw_pwm
                v['pwm'].set(f'出力PWM: {t:.0f} / {p:.0f}\nESC: {s.esc.stage}（仮モデル）')
        else:
            v['python'].set('外部Pythonは実行していません')
            v['input_scope'].set('比較用ガイド：自己位置を使用' if s.mode=='guide' else '手動 / 内蔵の比較用制御')


    def refresh(self):
        self.lap_timer.sample(self.sim)
        self.sonar_map.sample(self.sim)
        if self.follow.get():
            self.camera.cx=self.sim.car.x; self.camera.cy=self.sim.car.y
            self.draw_static()
        self.draw_dynamic(); self.update_panel()

    def load_python(self):
        from tkinter import filedialog
        self.running=False; self.pending_steps=0; self.start_button.configure(text='▶ 開始')
        path=filedialog.askopenfilename(parent=self.root,title='信頼できる走行Pythonを選択',initialdir=str(ROOT/'examples'),filetypes=[('Python program','*.py')])
        if path: self.select_python(path,confirm=True)

    def select_python(self,path,confirm=True):
        from tkinter import messagebox
        if confirm and not messagebox.askyesno('Pythonの実行確認',
            '選択したPythonは開始時にPC上で実行されます。\n'
            '別プロセスですが、ファイルやネットへのアクセスを制限するサンドボックスではありません。\n'
            '自作など、信頼できるファイルだけを選んでください。\n\n'+str(path)+'\n\nこのファイルを使用しますか？',parent=self.root):
            return False
        self.running=False; self.pending_steps=0; self.start_button.configure(text='▶ 開始')
        try:
            spec=self.sim.load_program(path,INTERFACES[self.interface_var.get()])
        except (OSError,ValueError,SyntaxError) as exc:
            self.error('Pythonを読み込めません',exc); return False
        self.mode_var.set(list(MODES)[0]); self.show_route.set(False)
        self.accumulator=0; self.last_time=time.perf_counter()
        self.notice.set(f'選択済み：{spec.path.name}。開始で実行します。元のファイルは変更していません。')
        self.draw_static(); self.refresh()
        return True

    def reload_python(self):
        if self.sim.program_spec is None:
            self.load_python(); return
        self.select_python(self.sim.program_spec.path,confirm=True)

    def interface_changed(self):
        if self.sim.program_spec:
            self.select_python(self.sim.program_spec.path,confirm=True)

    def stop_python(self):
        self.running=False; self.pending_steps=0; self.accumulator=0
        self.sim.stop_program(); self.start_button.configure(text='▶ 開始')
        self.notice.set('Pythonを終了し車両を停止しました。リセット/再読込で再実行できます。')
        self.refresh()

    def show_vehicle_view(self):
        if self.vehicle_view is not None and self.vehicle_view.window.winfo_exists():
            self.vehicle_view.window.lift();return
        self.vehicle_view=VehicleView(self)

    def show_program_log(self):
        if self.log_window is not None and self.log_window.winfo_exists():
            self.log_window.lift(); return
        from tkinter.scrolledtext import ScrolledText
        window=self.tk.Toplevel(self.root); self.log_window=window
        window.title('Python実行ログ / エラー'); window.geometry('860x470')
        self.tk.Label(window,text='print出力（最大2MB、表示は直近800行） / エラーのファイル名・行番号',anchor='w').pack(fill='x',padx=8,pady=5)
        text=ScrolledText(window,font=(self.family,10),wrap='word'); text.pack(fill='both',expand=True,padx=8,pady=5)
        cache=['']
        def refresh_log():
            if self.closed or not window.winfo_exists(): return
            content=self.sim.program_log()
            if self.sim.program_error: content+='\n\n'+self.sim.program_error
            if content!=cache[0]:
                text.configure(state='normal'); text.delete('1.0','end'); text.insert('end',content or '出力はまだありません。')
                text.configure(state='disabled'); text.see('end'); cache[0]=content
            window.after(250,refresh_log)
        refresh_log()

    def close(self):
        if self.closed: return
        self.closed=True; self.running=False; self.pending_steps=0
        if self.vehicle_view is not None:self.vehicle_view.close()
        self.sim.close(); self.root.destroy()

    def tick(self):
        if self.closed: return
        now=time.perf_counter(); elapsed=min(.1,now-self.last_time); self.last_time=now
        try:
            self.sim.check_program_health()
            if self.running:
                self.accumulator=min(.5,self.accumulator+elapsed*float(self.rate_var.get()))
                until=now+.006
                while self.accumulator>=1/PHYSICS_HZ and time.perf_counter()<until:
                    if not self.sim.step(self.manual_command(),block=False): break
                    self.accumulator-=1/PHYSICS_HZ
                    if self.sim.collision_id or self.sim.finished or self.sim.program_error or self.sim.program_stopped:
                        self.running=False; self.accumulator=0; self.start_button.configure(text='▶ 開始'); break
            if self.sim.program_error:
                self.notice.set(self.sim.state+'  — 詳細は「実行ログ」')
            if now-self.last_panel_time>=1/30:
                self.refresh(); self.last_panel_time=now
        except Exception as exc:
            self.running=False; self.start_button.configure(text='▶ 開始')
            if self.sim.mode=='program': self.sim.fail_program(exc)
            self.notice.set(f'停止しました：{exc}'); traceback.print_exc()
        if not self.closed: self.root.after(4,self.tick)

def main():
    parser=argparse.ArgumentParser(description='RC simulator V5 / external sensor-only Python')
    parser.add_argument('--map',type=Path,default=DEFAULT_MAP)
    args=parser.parse_args()
    try:
        import tkinter as tk
        root=tk.Tk()
        App(root,args.map)
        root.mainloop()
    except ImportError:
        print('Tkinter が見つかりません。Tk/Tcl付きのPythonを使用してください。',file=sys.stderr)
        print('確認: python -m tkinter   /   GUIなし検証: python validate_v5.py',file=sys.stderr)
        return 1
    except Exception as exc:
        print(f'起動できません: {exc}',file=sys.stderr)
        traceback.print_exc()
        return 1
    return 0

if __name__=='__main__':
    sys.exit(main())
