"""HC-SR04-only gap and local pure pursuit, selected for continuous three laps.

Five distances in cm and control interval in seconds only. Local wall-relative
lookahead targets are rebuilt from ultrasonic echoes. No map, route, pose,
goal or simulator imports. Positive Handle turns left.
The original conservative corner and fault handling remains the fallback.
"""
import math

BASE_PARAMS = {'filter_s': 0.08, 'target_cm': 44, 'angle_gain': 3, 'distance_gain': 1.2, 'fit_angle_limit': 55, 'gap_handle': 8, 'side_gain': 0, 'side_buffer': 25, 'diag_gain': 0, 'diag_buffer': 50, 'turn_start': 150, 'turn_release': 185, 'min_turn_s': 0.6, 'turn_handle': 100, 'protect_diag': 25, 'near_turn_limit': 100, 'emergency_front': 16, 'emergency_diag': 12, 'emergency_side': 8, 'deadband': 2, 'steer_rate': 140, 'turn_slowdown': 0.5, 'slow_distance': 100, 'base_accel': 35, 'decel_rate': 100, 'accel_rate': 20, 'mode': 'original', 'center_gain': 0.05, 'small_turn': 25}

def clamp(x,a,b):
    return max(a,min(b,x))

class PreviousController:
    def __init__(self):
        self.p=BASE_PARAMS
        self.filtered=None
        self.handle=0.0
        self.accel=0.0
        self.turn=0
        self.turn_time=0.0
        self.stopped=False
        self.last_angle=0.0
        self.wall_gap_time=0.0
        self.previous_side=None
        self.stop_reason='安全停止：測距/周期異常'

    def step(self,sensors,dt):
        names=('Fr','FrLh','RrLh','FrRh','RrRh')
        if not isinstance(dt,(float,int)) or not math.isfinite(dt) or not 0<dt<=.3:
            self.stopped=True
        if any(n not in sensors or not isinstance(sensors[n],(float,int)) or not math.isfinite(sensors[n]) or not 0<sensors[n]<=450 for n in names):
            self.stopped=True
        if self.stopped:
            self.accel=0
            return 0,self.handle,self.stop_reason
        raw=[sensors[n] for n in names]
        if raw[0]<self.p['emergency_front'] or min(raw[1],raw[3])<self.p['emergency_diag'] or min(raw[2],raw[4])<self.p['emergency_side']:
            self.stopped=True
            self.stop_reason='安全停止：接近限界'
            self.accel=0
            return 0,self.handle,'安全停止：接近限界'
        if self.filtered is None:
            self.filtered=raw[:]
        alpha=1-math.exp(-dt/self.p['filter_s'])
        self.filtered=[old+alpha*(new-old) for old,new in zip(self.filtered,raw)]
        # A nearer obstacle is never delayed by the smoothing filter.
        f,fl,sl,fr,sr=self.filtered
        if self.p.get('bridge_gaps',False):
            if self.previous_side is not None and sr>self.previous_side+self.p.get('gap_jump_cm',35):
                self.wall_gap_time+=dt
                if self.wall_gap_time<self.p.get('gap_hold_s',3):
                    sr=self.previous_side
                    # Across a missing wall, keep its prior local direction;
                    # do not fit endpoints on two unrelated walls.
                    fr=(sr+6*math.tan(math.radians(self.last_angle)))/(1/math.sqrt(2)-math.tan(math.radians(self.last_angle))/math.sqrt(2))
            else:
                self.wall_gap_time=0
            self.previous_side=sr
        forward=min(f,raw[0])
        fa=13+fr/math.sqrt(2)
        ra=10+fr/math.sqrt(2)
        fb=7
        rb=10+sr
        slope=(ra-rb)/max(6,fa-fb)
        angle=math.degrees(math.atan(slope))
        clearance=rb-slope*fb-10
        reliable=fr<245 and sr<245 and abs(angle)<self.p['fit_angle_limit'] and fr<self.p.get('gap_ratio',99)*max(sr,10)
        if reliable:
            self.last_angle=angle
            demand=-self.p['angle_gain']*angle-self.p['distance_gain']*(clearance-self.p['target_cm'])
        elif self.p.get('gap_hold',False) and fr>self.p.get('gap_ratio',2.4)*max(sr,10):
            demand=-self.p['angle_gain']*clamp(self.last_angle,-8,8)
        elif sr<180:
            demand=-self.p['distance_gain']*(sr-self.p['target_cm'])
        else:
            demand=-self.p['gap_handle']
        if self.p.get('mode','wall')=='center':
            demand=self.p.get('center_gain',.35)*(sl-sr)
            if sr>180 and sl>180:
                demand=0
        if self.p.get('mode')=='original':
            # Preserve the original ordered decision structure, but make the
            # near-wall demands continuous and cycle-time independent.
            demand=0.0
            if sl<self.p['side_buffer'] or fl<self.p['diag_buffer']:
                demand=-self.p.get('small_turn',30)
            elif sr<self.p['side_buffer'] or fr<self.p['diag_buffer']:
                demand=self.p.get('small_turn',30)
            elif abs(sl-sr)>12:
                demand=self.p.get('center_gain',.15)*(sl-sr)
        # Side protection overrides attraction to a wall across a corner.
        demand+=self.p['side_gain']*max(0,(self.p['side_buffer']-sr)/self.p['side_buffer'])
        demand-=self.p['side_gain']*max(0,(self.p['side_buffer']-sl)/self.p['side_buffer'])
        demand+=self.p['diag_gain']*max(0,(self.p['diag_buffer']-fr)/self.p['diag_buffer'])
        demand-=self.p['diag_gain']*max(0,(self.p['diag_buffer']-fl)/self.p['diag_buffer'])
        if self.turn:
            self.turn_time+=dt
            if forward>self.p['turn_release'] and self.turn_time>self.p['min_turn_s']:
                self.turn=0
        elif forward<self.p['turn_start']:
            self.turn=1 if .7*fl+.3*sl >= .7*fr+.3*sr else -1
            self.turn_time=0
        if self.turn:
            demand=self.turn*self.p['turn_handle']
            state='左回避' if self.turn>0 else '右回避'
        else:
            state='右壁追従'
        # Do not steer toward a dangerously close diagonal even during turns.
        if fr<self.p['protect_diag'] and demand<0:
            demand=max(demand,-self.p['near_turn_limit'])
        if fl<self.p['protect_diag'] and demand>0:
            demand=min(demand,self.p['near_turn_limit'])
        demand=clamp(demand,-100,100)
        if abs(demand)<self.p['deadband']:
            demand=0
        self.handle+=clamp(demand-self.handle,-self.p['steer_rate']*dt,self.p['steer_rate']*dt)
        turn_factor=1-self.p['turn_slowdown']*min(1,abs(self.handle)/100)
        front_factor=clamp((forward-self.p['emergency_front'])/self.p['slow_distance'],.15,1)
        target=self.p['base_accel']*min(turn_factor,front_factor)
        self.accel+=clamp(target-self.accel,-self.p['decel_rate']*dt,self.p['accel_rate']*dt)
        return self.accel,self.handle,state


# Divider avoidance; decisions use only the five ultrasonic distances.
class DividerController(PreviousController):
    def __init__(self):
        super().__init__()
        self.p=dict(self.p)
        self.parallel_time=0.0
        self.guard_active=False
        self.guard_target_cm=25
        self.guard_angle_gain=2.5
        self.guard_distance_gain=0.3
        self.guard_limit=10
        self.guard_trust_deg=15.0
        self.guard_confirm_s=.25
        self.recent_diagonal=[250.0,250.0]
        self.startup_echoes=[]
        self.stable_echoes=False

    def step(self,sensors,dt):
        previous_handle=self.handle
        names=('Fr','FrLh','RrLh','FrRh','RrRh')
        valid=all(isinstance(sensors.get(n),(int,float)) and math.isfinite(sensors[n]) and 0<sensors[n]<=450 for n in names)
        if valid and len(self.startup_echoes)<6:
            self.startup_echoes.append([sensors[n] for n in names])
            if len(self.startup_echoes)==6:
                # Initial low-speed measurement qualification, in centimetres.
                # Discontinuous or noisy echoes keep the conservative policy.
                residuals=[abs(self.startup_echoes[i][j]-2*self.startup_echoes[i-1][j]+self.startup_echoes[i-2][j])
                           for i in range(2,6) for j in range(5)]
                self.stable_echoes=max(residuals)<.25
        if valid:
            isolated=(sensors['FrLh']<35 and sensors['RrLh']>100) or (sensors['FrRh']<35 and sensors['RrRh']>100)
            self.p['emergency_diag']=22 if isolated else 12
        if valid and self.guard_active and self.filtered is not None and sensors["Fr"]<180:
            self.turn=1 if sensors["FrLh"]>=sensors["FrRh"] else -1
            self.turn_time=0
            self.guard_active=False
        accel,handle,state=super().step(sensors,dt)
        if self.stopped:
            return accel,handle,state
        f,fl,sl,fr,sr=self.filtered
        root_half=math.sqrt(.5)
        span=6+fr*root_half
        slope=(fr*root_half-sr)/span
        angle=math.degrees(math.atan(slope))
        clearance=sr-7*slope
        agrees=fr<240 and sr<150 and abs(angle)<self.guard_trust_deg and span>15
        corridor=sl<80 and sr<80 and fl<180 and fr<180 and f>180
        if self.turn or not agrees or f<150:
            self.guard_active=False
        self.parallel_time=self.parallel_time+dt if agrees and corridor and abs(previous_handle)<20 else 0
        if self.parallel_time>=self.guard_confirm_s:
            self.guard_active=True
        if not self.turn and self.guard_active:
            desired=-self.guard_angle_gain*angle-self.guard_distance_gain*(clearance-self.guard_target_cm)
            desired=max(-self.guard_limit,min(self.guard_limit,desired))
            self.handle=previous_handle+max(-100*dt,min(100*dt,desired-previous_handle))
            handle=self.handle
            state='仕切り壁予防：右壁の向き・離隔を保持'
        # A disappearing echo must not erase a recently seen wall end.
        left=min(sensors['FrLh'],self.recent_diagonal[0]+25*dt)
        right=min(sensors['FrRh'],self.recent_diagonal[1]+25*dt)
        self.recent_diagonal=[left,right]
        # Start lateral escape and braking before the endpoint reaches the body.
        if min(left,right)<45 and sl<100 and sr<100:
            desired=70 if right<left else -70
            self.handle=previous_handle+max(-140*dt,min(140*dt,desired-previous_handle))
            handle=self.handle
            cap,gain=(20,0.6) if self.stable_echoes else (14,.45)
            target=max(4,min(cap,(min(left,right)-12)*gain))
            self.accel=min(self.accel,target)
            accel=self.accel
            state='仕切り壁回避：斜め接近を保持して減速'
        return accel,handle,state


# Local ultrasonic gaps and pure pursuit replace crawling in parallel corridors.
GAP_PARAMS={
    'lookahead_cm':85, 'bubble_cm':18, 'gap_weight':.15,
    'steer_rate':100, 'filter_s':.2, 'base_accel':100, 'local_limit':50,
}


class CorridorController(DividerController):
    def __init__(self):
        super().__init__()
        self.p=dict(self.p)
        self.q=dict(GAP_PARAMS)
        self.narrow_mode=False
        self.local_target=0.0

    def gap_target(self,raw,reference):
        """Inflate observed echoes by vehicle half width + clearance margin.

        Five rays give a sparse local obstacle set, not a lidar scan. Unmeasured
        sectors are not certified free; keep small changes and frontal braking.
        """
        self.selected_gap=None
        f,fl,sl,fr,sr=raw
        h=math.sqrt(.5)
        points=[(12+f,0),(13+fl*h,10+fl*h),(7,10+sl),
                (13+fr*h,-10-fr*h),(7,-10-sr)]
        limit=self.q['lookahead_cm']
        free=[]
        for degree in range(-60,61,5):
            a=math.radians(degree);c,s=math.cos(a),math.sin(a)
            safe=True
            for x,y in points:
                along=x*c+y*s;across=-x*s+y*c
                if 0<along<limit+14 and abs(across)<self.q['bubble_cm']:
                    safe=False;break
            if safe:free.append(degree)
        if not free:return None
        groups=[]
        for a in free:
            if not groups or a-groups[-1][-1]>5:groups.append([a])
            else:groups[-1].append(a)
        # Prefer a wide contiguous gap that contains the current local target.
        group=min(groups,key=lambda g:min(abs(a-reference) for a in g)-self.q['gap_weight']*len(g))
        self.selected_gap=(group[0],group[-1])
        return clamp(reference,group[0],group[-1])

    def _step_drive(self,sensors,dt):
        previous_handle=self.handle
        previous_accel=self.accel
        accel,handle,state=super().step(sensors,dt)
        if self.stopped:return accel,handle,state
        raw=[sensors[n] for n in ('Fr','FrLh','RrLh','FrRh','RrRh')]
        f,fl,sl,fr,sr=self.filtered
        h=math.sqrt(.5)
        rs=(fr*h-sr)/(6+fr*h)
        ls=(fl*h-sl)/(6+fl*h)
        parallel=(abs(math.degrees(math.atan(rs)))<20 and abs(math.degrees(math.atan(ls)))<20)
        narrow=(sl<45 and sr<45 and min(sl,sr)>16 and min(fl,fr)>25 and f>180)
        if narrow:self.narrow_mode=True
        left_fit=fl<150 and sl<45 and abs(math.degrees(math.atan(ls)))<25
        right_fit=fr<150 and sr<45 and abs(math.degrees(math.atan(rs)))<25
        continued=self.narrow_mode and f>180 and min(sl,sr)>16 and (left_fit or right_fit) and not self.turn
        if not continued:self.narrow_mode=False
        if not narrow and not continued:
            self.local_target=math.degrees(math.atan(math.tan(math.radians(self.handle*32/100))*self.q['lookahead_cm']/34))
            return accel,handle,state
        length=self.q['lookahead_cm']
        if continued and not narrow:
            y=(sl-24+(length+7)*ls) if left_fit else (24-sr-(length+7)*rs)
            reference=math.degrees(math.atan2(y,length))
            label='壁の切れ目：局所ギャップ追従保持'
        else:
            y=(sl-sr)/2
            if parallel:y+=(ls-rs)*(length+7)/2
            reference=math.degrees(math.atan2(y,length))
            label='直線ギャップ中央＋ピュアパシュート'
        heading=self.gap_target(raw,reference)
        if heading is None:
            self.accel=0;self.stopped=True
            return 0,self.handle,'安全停止：超音波ギャップ不足'
        alpha=1-math.exp(-dt/self.q['filter_s'])
        self.local_target+=alpha*(heading-self.local_target)
        target_y=length*math.tan(math.radians(self.local_target))
        curvature=2*target_y/(length*length+target_y*target_y)
        demand=math.degrees(math.atan(17*curvature))/32*100
        demand=clamp(demand,-self.q['local_limit'],self.q['local_limit'])
        self.handle=previous_handle+clamp(demand-previous_handle,-self.q['steer_rate']*dt,self.q['steer_rate']*dt)
        target=self.q['base_accel']*(1-.4*min(1,abs(self.handle)/100))
        target*=clamp((min(f,raw[0])-16)/100,.15,1)
        target*=clamp((min(sl,sr)-8)/16,.25,1)
        self.accel=previous_accel+clamp(target-previous_accel,-100*dt,20*dt)
        return self.accel,self.handle,label

    def step(self,sensors,dt):
        accel,handle,state=self._step_drive(sensors,dt)
        # Display-only output telemetry. No simulator information is consumed.
        target=0.0 if self.stopped else self.local_target
        state=state.replace('仕切り壁回避：斜め接近を保持して減速','仕切り端：検出側から離れて通過')
        gap=getattr(self,'selected_gap',None)
        detail=f' | gap_min_deg={gap[0]:.1f} | gap_max_deg={gap[1]:.1f}' if gap is not None else ''
        return accel,handle,state+f' | target_deg={target:.3f}'+detail


class Controller(CorridorController):
    GLOBAL_LOOKAHEAD=85
    CORRECTION_LIMIT=1
    def _step_drive(self,sensors,dt):
        self.selected_gap=None
        previous_handle=self.handle
        accel,handle,state=super()._step_drive(sensors,dt)
        if self.stopped:return accel,handle,state
        raw=[sensors[n] for n in ('Fr','FrLh','RrLh','FrRh','RrRh')]
        reference=self.local_target
        original_length=self.q['lookahead_cm']
        try:
            self.q['lookahead_cm']=self.GLOBAL_LOOKAHEAD
            selected=self.gap_target(raw,reference)
        finally:
            self.q['lookahead_cm']=original_length
        protected=(self.turn or self.guard_active or '仕切り壁回避' in state or
                   abs(handle)>30 or raw[0]<180 or min(raw[1],raw[3])<45 or min(raw[2],raw[4])<25)
        if protected:
            self.selected_gap=None
            return accel,handle,'安全制御優先：'+state
        if selected is None:
            self.stopped=True;self.accel=0
            return 0,self.handle,'安全停止：Follow the Gapで通過候補なし'
        if abs(selected-reference)>3:
            target=reference+clamp(selected-reference,-self.CORRECTION_LIMIT,self.CORRECTION_LIMIT)
            def pursuit(angle):
                return math.degrees(math.atan(34*math.sin(math.radians(angle))/self.GLOBAL_LOOKAHEAD))/32*100
            demand=clamp(handle+pursuit(target)-pursuit(reference),-100,100)
            self.handle=previous_handle+clamp(demand-previous_handle,-100*dt,100*dt)
            handle=self.handle;self.local_target=target
            state='Follow the Gap：選んだ空間へ操舵'
        else:state='Follow the Gap：目標が空き空間内'
        return accel,handle,state


_GapController=Controller
class Controller(_GapController):
    SMOOTH_RATE=140
    SMOOTH_JERK=8000
    SMOOTH_FILTER=0
    NEUTRAL_BAND=0
    def __init__(self):
        super().__init__()
        self.output_handle=0.0
        self.output_rate=0.0
        self.profile_target=0.0
    def step(self,sensors,dt):
        accel,desired,state=super().step(sensors,dt)
        if self.stopped:
            self.output_handle=desired;self.output_rate=0;self.profile_target=desired
            return accel,desired,state
        urgent=(sensors['Fr']<90 or min(sensors['FrLh'],sensors['FrRh'])<25 or
                min(sensors['RrLh'],sensors['RrRh'])<15)
        if urgent:
            self.output_rate=(desired-self.output_handle)/dt
            self.output_handle=desired;self.profile_target=desired
            return accel,desired,'接近回避優先：'+state
        goal=0.0 if abs(desired)<self.NEUTRAL_BAND else desired
        alpha=1.0 if self.SMOOTH_FILTER==0 else 1-math.exp(-dt/self.SMOOTH_FILTER)
        self.profile_target+=alpha*(goal-self.profile_target)
        error=self.profile_target-self.output_handle
        rate=clamp(error/dt,-self.SMOOTH_RATE,self.SMOOTH_RATE)
        self.output_rate+=clamp(rate-self.output_rate,-self.SMOOTH_JERK*dt,self.SMOOTH_JERK*dt)
        proposed=self.output_handle+self.output_rate*dt
        if error*(self.profile_target-proposed)<=0:proposed=self.profile_target
        proposed=clamp(proposed,-100,100)
        self.output_rate=(proposed-self.output_handle)/dt
        self.output_handle=proposed;self.handle=proposed
        return accel,proposed,'平滑操舵：'+state
