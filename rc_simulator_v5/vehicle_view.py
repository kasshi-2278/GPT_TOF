"""Read-only ultrasonic observations and controller decision displays."""
import math
import re
from geometry import local_to_world

COLORS={'Fr':'#dfaf38','FrLh':'#27b8c6','RrLh':'#699fe0',
        'FrRh':'#e89667','RrRh':'#df82bb'}
LABELS={'Fr':'正面','FrLh':'左45°','RrLh':'左90°','FrRh':'右45°','RrRh':'右90°'}


def sensor_segments(config,readings,display_range=150):
    """Mounts and observed five ranges in the car's forward/left frame."""
    result=[]
    for name,(forward,left,angle) in config.sensor_specs().items():
        distance=readings.get(name)
        if distance is None or not math.isfinite(distance) or distance<0:continue
        shown=min(distance,display_range)
        a=math.radians(angle)
        result.append((name,(forward,left),(forward+shown*math.cos(a),left-shown*math.sin(a)),distance))
    return result


def gap_sector(state):
    """Decode actual controller output; old programs have no synthetic sector."""
    if '安全停止' in str(state):return None
    low=re.search(r'\| gap_min_deg=([-+0-9.]+)',str(state))
    high=re.search(r'\| gap_max_deg=([-+0-9.]+)',str(state))
    if not low or not high:return None
    try:a,b=float(low.group(1)),float(high.group(1))
    except ValueError:return None
    return (a,b) if -60<=a<=b<=60 else None


class VehicleView:
    def __init__(self,app):
        tk=app.tk;ttk=app.ttk
        self.app=app
        self.window=tk.Toplevel(app.root)
        self.window.title('超音波センサー・進行判断')
        self.window.geometry('900x630');self.window.minsize(650,430)
        self.window.protocol('WM_DELETE_WINDOW',self.close)
        self.window.bind('<KeyPress>',app.key_down)
        self.window.bind('<KeyRelease>',lambda e:app.keys.discard(e.keysym.lower()))
        self.window.bind('<FocusOut>',lambda e:app.keys.clear())
        self.info=tk.StringVar()
        tk.Label(self.window,textvariable=self.info,anchor='w',bg='#142337',fg='white',
                 font=(app.family,11),padx=12,pady=8).pack(fill='x')
        self.tabs=ttk.Notebook(self.window);self.tabs.pack(fill='both',expand=True,padx=6,pady=6)
        self.sonar=tk.Canvas(self.tabs,bg='#142337',highlightthickness=0)
        self.tabs.add(self.sonar,text='超音波５方向・判断')
        self.mapping=tk.Frame(self.tabs)
        tools=tk.Frame(self.mapping);tools.pack(fill='x')
        tk.Button(tools,text='検出点を消去',command=self.clear_map).pack(side='left',padx=5,pady=3)
        tk.Button(tools,text='検出点をCSV保存',command=self.save_map).pack(side='left',padx=5,pady=3)
        self.map_canvas=tk.Canvas(self.mapping,bg='#142337',highlightthickness=0)
        self.map_canvas.pack(fill='both',expand=True)
        self.tabs.add(self.mapping,text='障害物マッピング')
        self.note=tk.StringVar()
        tk.Label(self.window,textvariable=self.note,anchor='w',font=(app.family,9),
                 padx=10,pady=5).pack(fill='x')
        self.cache=None;self.after_id=None
        self.tabs.bind('<<NotebookTabChanged>>',lambda e:setattr(self,'cache',None))
        self.refresh()

    def close(self):
        if self.after_id is not None:
            self.window.after_cancel(self.after_id);self.after_id=None
        if self.window.winfo_exists():self.window.destroy()
        if self.app.vehicle_view is self:self.app.vehicle_view=None

    def refresh(self):
        if self.app.closed or not self.window.winfo_exists():return
        sim=self.app.sim;car=sim.car
        self.info.set(f'{sim.time:.2f} s　速度 {car.speed:.1f} cm/s　操舵 {car.steer:+.1f}°　走行判断：超音波５方向')
        selected=self.tabs.select()
        self.app.sonar_map.sample(sim)
        canvas=self.sonar if selected==str(self.sonar) else self.map_canvas
        signature=(id(sim.world),len(sim.world.ray_edges),sim.time,car.x,car.y,car.heading,
                   selected,canvas.winfo_width(),canvas.winfo_height(),tuple(sim.sensors.items()))
        if signature!=self.cache:
            self.cache=signature
            if canvas is self.sonar:self.draw_sonar(sim)
            else:self.draw_map(sim)
        self.after_id=self.window.after(100,self.refresh)

    def draw_sonar(self,sim):
        cv=self.sonar;cv.delete('all')
        w=max(1,cv.winfo_width());h=max(1,cv.winfo_height())
        origin=(w/2,h*.68);scale=min(w*.43/150,h*.58/150)
        def screen(p):return origin[0]-p[1]*scale,origin[1]-p[0]*scale
        sector=gap_sector(sim.state)
        if sector is not None:
            a,b=sector
            angles=[a+(b-a)*i/24 for i in range(25)]
            points=[screen((0,0))]+[screen((85*math.cos(math.radians(v)),85*math.sin(math.radians(v)))) for v in angles]
            cv.create_polygon(*[v for p in points for v in p],fill='#244b3c',outline='#418a64')
        for radius in (50,100,150):
            r=radius*scale
            cv.create_oval(origin[0]-r,origin[1]-r,origin[0]+r,origin[1]+r,
                           outline='#344b5c',dash=(4,4))
            cv.create_text(origin[0]+8,origin[1]-r+8,text=f'{radius} cm',fill='#92a8b8',anchor='w')
        cv.create_text(w/2,20,text='前方 ↑',fill='white',font=(self.app.family,13,'bold'))
        for name,start,end,distance in sensor_segments(sim.config,sim.sensors):
            a=screen(start);b=screen(end)
            cv.create_line(*a,*b,fill=COLORS[name],width=3,arrow='last')
            cv.create_oval(b[0]-4,b[1]-4,b[0]+4,b[1]+4,fill=COLORS[name] if distance<sim.config.sensor_range_cm else '',outline=COLORS[name])
            x=max(80,min(w-80,b[0]));y=max(40,min(h-60,b[1]-14))
            cv.create_text(x,y,text=f'{LABELS[name]} {distance:.1f} cm'+(' 未検出' if distance>=sim.config.sensor_range_cm else ''),fill=COLORS[name],font=(self.app.family,10,'bold'))
        length=sim.config.length_cm/2;width=sim.config.width_cm/2
        pts=[screen(p) for p in ((length,width),(length,-width),(-length,-width),(-length,width))]
        cv.create_polygon(*[v for p in pts for v in p],fill='#f2bf46',outline='white',width=2)
        cv.create_line(*screen((-length/2,0)),*screen((length*.8,0)),fill='#142337',width=2,arrow='last')
        self.draw_decision(cv,sim,screen)
        cv.create_text(w/2,h-20,text='　 /　 '.join(f'{LABELS[n]} {sim.sensors.get(n,0):.1f}' for n in COLORS),
                       fill='white',font=(self.app.family,10))
        self.note.set('色の線：５つの測距値（150cmまで）　緑矢印：目標方向　緑の帯：FTG選択領域（未観測部分あり）　白点線：操舵指令')

    def clear_map(self):
        self.app.sonar_map.clear();self.cache=None

    def save_map(self):
        from tkinter import filedialog
        path=filedialog.asksaveasfilename(parent=self.window,title='超音波検出点を保存',
            defaultextension='.csv',initialfile='ultrasonic_map.csv',filetypes=[('CSV','*.csv')])
        if path:self.app.sonar_map.save_csv(path)

    def draw_map(self,sim):
        cv=self.map_canvas;cv.delete('all');model=self.app.sonar_map
        w=max(1,cv.winfo_width());h=max(1,cv.winfo_height())
        coords=list(model.track)+[(p[2],p[3]) for p in model.points]+[(sim.car.x,sim.car.y)]
        left=min(p[0] for p in coords)-40;right=max(p[0] for p in coords)+40
        bottom=min(p[1] for p in coords)-40;top=max(p[1] for p in coords)+40
        scale=min(max(1,w-60)/(right-left),max(1,h-70)/(top-bottom))
        cx=(left+right)/2;cy=(bottom+top)/2
        def screen(x,y):return w/2+(x-cx)*scale,h/2-(y-cy)*scale
        for x in range(math.ceil(left/100)*100,math.floor(right/100)*100+1,100):
            cv.create_line(*screen(x,bottom),*screen(x,top),fill='#263e50')
        for y in range(math.ceil(bottom/100)*100,math.floor(top/100)*100+1,100):
            cv.create_line(*screen(left,y),*screen(right,y),fill='#263e50')
        track=[v for p in model.track for v in screen(*p)]
        if len(track)>=4:cv.create_line(*track,fill='#698292',width=1)
        # Merge display pixels only; CSV retains all original measured returns.
        occupied=set()
        for _,name,x,y,*_ in model.points:
            sx,sy=screen(x,y);key=(name,round(sx/3),round(sy/3))
            if key in occupied:continue
            occupied.add(key)
            cv.create_oval(sx-2,sy-2,sx+2,sy+2,fill=COLORS[name],outline='')
        car=sim.car;cfg=sim.config
        corners=[screen(*local_to_world(car.x,car.y,car.heading,f,l)) for f,l in
                 ((cfg.length_cm/2,cfg.width_cm/2),(cfg.length_cm/2,-cfg.width_cm/2),
                  (-cfg.length_cm/2,-cfg.width_cm/2),(-cfg.length_cm/2,cfg.width_cm/2))]
        cv.create_polygon(*[v for p in corners for v in p],fill='#f2bf46',outline='white')
        cv.create_line(*screen(car.x,car.y),*screen(*local_to_world(car.x,car.y,car.heading,30,0)),fill='white',arrow='last')
        self.draw_decision(cv,sim,lambda p:screen(*local_to_world(car.x,car.y,car.heading,*p)))
        cv.create_text(12,15,anchor='w',text=f'検出点 {len(model.points):,} / 30,000　格子100 cm　最大距離の値は除外',fill='white')
        for i,name in enumerate(COLORS):
            cv.create_text(12+i*105,h-15,anchor='w',text='● '+LABELS[name],fill=COLORS[name])
        self.note.set('測距値＋表示専用の車体位置・向き。コースの壁座標は使いません。点は反射方向の推定です。')

    def draw_decision(self,cv,sim,screen):
        """Read output telemetry; never infer a target from map geometry."""
        state=str(sim.state)
        match=re.search(r'\| target_deg=([-+0-9.]+)',state)
        stopped='安全停止' in state or sim.program_stopped or bool(sim.collision_id)
        if match and not stopped:
            angle=math.radians(float(match.group(1)))
            end=(85*math.cos(angle),85*math.sin(angle))
            cv.create_line(*screen((0,0)),*screen(end),fill='#67ff91',width=4,arrow='last',arrowshape=(14,17,6))
            ex,ey=screen(end)
            cv.create_text(ex,ey-16,text=f'目標 {float(match.group(1)):+.1f}°',fill='#67ff91',font=(self.app.family,10,'bold'))
        # Curvature commanded by Handle, rather than the actual wheel lag.
        curvature=math.tan(math.radians(sim.car.handle*sim.config.max_steer_deg/100))/sim.config.wheelbase_cm
        pts=[]
        for i in range(21):
            distance=i*4
            forward=math.sin(curvature*distance)/curvature if abs(curvature)>1e-8 else distance
            left=(1-math.cos(curvature*distance))/curvature if abs(curvature)>1e-8 else 0
            pts.extend(screen((forward,left)))
        if not stopped:cv.create_line(*pts,fill='white',width=2,dash=(5,4),arrow='last')
        label=state.split(' | target_deg=')[0]
        if stopped:label='停止判断：'+label
        cv.create_text(12,42,anchor='w',text=label,fill='#ffb481' if stopped else '#67ff91',font=(self.app.family,10,'bold'))
