"""Read-only observer windows. View rays never enter the drive controller."""
import math
from geometry import local_to_world

COLORS={'Fr':'#dfaf38','FrLh':'#27b8c6','RrLh':'#699fe0',
        'FrRh':'#e89667','RrRh':'#df82bb'}
LABELS={'Fr':'正面','FrLh':'左45°','RrLh':'左90°','FrRh':'右45°','RrRh':'右90°'}


def perspective_columns(world,car,config,count=100,fov_deg=90,maximum=800):
    """Read-only visual ray casts; separate from the five HC-SR04 samples."""
    origin=local_to_world(car.x,car.y,car.heading,config.length_cm/2-2,0)
    metadata={o['id']:o.get('color','white') for group in ('walls','posts','zones')
              for o in world.data.get(group,[])}
    result=[]
    for i in range(count):
        angle=-fov_deg/2+fov_deg*(i+.5)/count
        distance,hit=world.raycast(origin,car.heading+angle,maximum)
        color='white'
        if distance<maximum:
            hx,hy=hit
            for ident,poly,box in world.polygons:
                if box[0]-.01<=hx<=box[2]+.01 and box[1]-.01<=hy<=box[3]+.01:
                    color=metadata.get(ident,'white');break
            else:
                for ident,center,radius in world.circles:
                    if math.dist(hit,center)<=radius+.01:
                        color=metadata.get(ident,'red');break
        result.append((max(1,distance*math.cos(math.radians(angle))),color,distance<maximum))
    return result


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


class VehicleView:
    def __init__(self,app):
        tk=app.tk;ttk=app.ttk
        self.app=app
        self.window=tk.Toplevel(app.root)
        self.window.title('車からの見え方')
        self.window.geometry('900x630');self.window.minsize(650,430)
        self.window.protocol('WM_DELETE_WINDOW',self.close)
        self.window.bind('<KeyPress>',app.key_down)
        self.window.bind('<KeyRelease>',lambda e:app.keys.discard(e.keysym.lower()))
        self.window.bind('<FocusOut>',lambda e:app.keys.clear())
        self.info=tk.StringVar()
        tk.Label(self.window,textvariable=self.info,anchor='w',bg='#142337',fg='white',
                 font=(app.family,11),padx=12,pady=8).pack(fill='x')
        self.tabs=ttk.Notebook(self.window);self.tabs.pack(fill='both',expand=True,padx=6,pady=6)
        self.front=tk.Canvas(self.tabs,bg='#dce4e8',highlightthickness=0)
        self.sonar=tk.Canvas(self.tabs,bg='#142337',highlightthickness=0)
        self.tabs.add(self.front,text='前方の車視点');self.tabs.add(self.sonar,text='超音波５方向')
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
        canvas=self.front if selected==str(self.front) else self.sonar if selected==str(self.sonar) else self.map_canvas
        signature=(id(sim.world),len(sim.world.ray_edges),sim.time,car.x,car.y,car.heading,
                   selected,canvas.winfo_width(),canvas.winfo_height(),tuple(sim.sensors.items()))
        if signature!=self.cache:
            self.cache=signature
            if canvas is self.front:self.draw_front(sim)
            elif canvas is self.sonar:self.draw_sonar(sim)
            else:self.draw_map(sim)
        self.after_id=self.window.after(100,self.refresh)

    def draw_front(self,sim):
        cv=self.front;cv.delete('all')
        w=max(1,cv.winfo_width());h=max(1,cv.winfo_height());horizon=h*.46
        cv.create_rectangle(0,0,w,horizon,fill='#dce4e8',outline='')
        cv.create_rectangle(0,horizon,w,h,fill='#666d70',outline='')
        focal=w/2
        palette={'red':(191,65,57),'white':(237,238,230),'green':(53,139,116),
                 'added':(163,82,146),'post':(200,204,202)}
        columns=perspective_columns(sim.world,sim.car,sim.config)
        for i,(depth,color,hit) in enumerate(columns):
            if not hit:continue
            top=max(0,horizon-25*focal/depth)
            bottom=min(h,horizon+15*focal/depth)
            rgb=palette.get(color,(216,219,214));shade=max(.35,1-depth/1100)
            fill='#'+''.join(f'{int(v*shade):02x}' for v in rgb)
            cv.create_rectangle(i*w/len(columns),top,(i+1)*w/len(columns)+1,bottom,fill=fill,outline='')
        cv.create_polygon(w*.27,h,w*.38,h*.9,w*.62,h*.9,w*.73,h,fill='#194d62',outline='#6fa0b0')
        cv.create_line(w/2-8,horizon,w/2+8,horizon,fill='#112b3b')
        cv.create_line(w/2,horizon-8,w/2,horizon+8,fill='#112b3b')
        cv.create_text(w/2,20,text=f"正面 {sim.sensors.get('Fr',0):.1f} cm",fill='#112b3b',font=(self.app.family,13,'bold'))
        self.note.set('観察用の仮想視点。壁高さ40cm・視点高さ15cmは表示用の設定です。')

    def draw_sonar(self,sim):
        cv=self.sonar;cv.delete('all')
        w=max(1,cv.winfo_width());h=max(1,cv.winfo_height())
        origin=(w/2,h*.68);scale=min(w*.43/150,h*.58/150)
        def screen(p):return origin[0]-p[1]*scale,origin[1]-p[0]*scale
        for radius in (50,100,150):
            r=radius*scale
            cv.create_oval(origin[0]-r,origin[1]-r,origin[0]+r,origin[1]+r,
                           outline='#344b5c',dash=(4,4))
            cv.create_text(origin[0]+8,origin[1]-r+8,text=f'{radius} cm',fill='#92a8b8',anchor='w')
        cv.create_text(w/2,20,text='前方 ↑',fill='white',font=(self.app.family,13,'bold'))
        for name,start,end,distance in sensor_segments(sim.config,sim.sensors):
            a=screen(start);b=screen(end)
            cv.create_line(*a,*b,fill=COLORS[name],width=3,arrow='last')
            cv.create_oval(b[0]-4,b[1]-4,b[0]+4,b[1]+4,fill=COLORS[name],outline='')
            x=max(80,min(w-80,b[0]));y=max(40,min(h-60,b[1]-14))
            cv.create_text(x,y,text=f'{LABELS[name]} {distance:.1f} cm',fill=COLORS[name],font=(self.app.family,10,'bold'))
        length=sim.config.length_cm/2;width=sim.config.width_cm/2
        pts=[screen(p) for p in ((length,width),(length,-width),(-length,-width),(-length,width))]
        cv.create_polygon(*[v for p in pts for v in p],fill='#f2bf46',outline='white',width=2)
        cv.create_line(*screen((-length/2,0)),*screen((length*.8,0)),fill='#142337',width=2,arrow='last')
        cv.create_text(w/2,h-20,text='　 /　 '.join(f'{LABELS[n]} {sim.sensors.get(n,0):.1f}' for n in COLORS),
                       fill='white',font=(self.app.family,10))
        self.note.set('走行プログラムと同じ５つの測距値を表示。線の表示範囲は150cmです。')

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
        cv.create_text(12,15,anchor='w',text=f'検出点 {len(model.points):,} / 30,000　格子100 cm　最大距離の値は除外',fill='white')
        for i,name in enumerate(COLORS):
            cv.create_text(12+i*105,h-15,anchor='w',text='● '+LABELS[name],fill=COLORS[name])
        self.note.set('測距値＋表示専用の車体位置・向き。コースの壁座標は使いません。点は反射方向の推定です。')
