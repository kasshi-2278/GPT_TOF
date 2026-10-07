"""Editable map loader shared by the GUI and headless tests."""
from __future__ import annotations
import json, math
from copy import deepcopy
from pathlib import Path
from geometry import *

DEFAULT_MAP=Path(__file__).resolve().parent/'maps'/'course_1030.json'

class World:
    def __init__(self, data: dict):
        self.data=deepcopy(data)
        self._validate()
        self.rebuild()

    @classmethod
    def load(cls, path=DEFAULT_MAP):
        with Path(path).open(encoding='utf-8') as stream: data=json.load(stream)
        return cls(data)

    def _point(self, p):
        if not isinstance(p,(list,tuple)) or len(p)!=2 or any(
            not isinstance(x,(float,int)) or not math.isfinite(x) or abs(x)>100000 for x in p):
            raise ValueError(f'Invalid coordinate: {p!r}')

    def _positive(self, value, title):
        if not isinstance(value,(float,int)) or not math.isfinite(value) or value<=0:
            raise ValueError(f'{title}: expected a positive finite number')

    def _validate(self):
        d=self.data
        if d.get('schema_version')!=1 or d.get('units')!='cm':
            raise ValueError('Map schema_version=1 and units=cm are required')
        extent=d.get('extents_cm',[])
        if len(extent)!=4: raise ValueError('extents_cm must contain four values')
        self._point(extent[:2]); self._point(extent[2:])
        if extent[0]>=extent[2] or extent[1]>=extent[3]: raise ValueError('Invalid map extents')
        ids=set()
        for group in ('walls','posts','zones'):
            if len(d.get(group,[]))>2000: raise ValueError('Too many map elements')
            for obj in d.get(group,[]):
                ident=obj.get('id')
                if not isinstance(ident,str) or not ident or ident in ids:
                    raise ValueError('Map IDs must be unique non-empty strings')
                ids.add(ident)
                if group=='walls':
                    self._point(obj['a']); self._point(obj['b'])
                    self._positive(obj['thickness_cm'],'Wall thickness')
                    if math.dist(obj['a'],obj['b'])<1e-6: raise ValueError('Zero-length wall')
                elif group=='posts':
                    self._point(obj['center']); self._positive(obj['radius_cm'],'Post radius')
                else:
                    poly=obj['polygon']
                    if len(poly)<3: raise ValueError('A zone needs at least 3 vertices')
                    for p in poly: self._point(p)
                    if obj.get('solid',False) and not is_convex(poly):
                        raise ValueError('Solid zones must be convex; split a concave region into convex parts')
        start=d['start']; self._point([start['x'],start['y']])
        if not math.isfinite(start['heading_deg']): raise ValueError('Invalid heading')
        route=d.get('demo_route',{})
        # A guide is optional: sensor-only Python runs without route data.
        if route:
            if not route.get('closed') or len(route.get('points',[]))<4:
                raise ValueError('If supplied, demo_route must be a closed route with >=4 points')
            if len(route['points'])>20000: raise ValueError('Demo route too large')
            for p in route['points']: self._point(p)
            for a,b in edges(route['points']):
                if math.dist(a,b)<1e-6: raise ValueError('Repeated consecutive route point')

    def rebuild(self):
        self.polygons=[]; self.circles=[]
        for w in self.data.get('walls',[]):
            if w.get('solid',True):
                poly=wall_polygon(w['a'],w['b'],w['thickness_cm'])
                self.polygons.append((w['id'],poly,bounds(poly)))
        for z in self.data.get('zones',[]):
            if z.get('solid',False):
                self.polygons.append((z['id'],z['polygon'],bounds(z['polygon'])))
        for p in self.data.get('posts',[]):
            if p.get('solid',True): self.circles.append((p['id'],tuple(p['center']),p['radius_cm']))
        self.ray_edges=[(a,b) for _,poly,_ in self.polygons for a,b in edges(poly)]
        self.extents=self.data['extents_cm']

    def raycast(self, origin, heading, maximum):
        ray=direction(heading)
        for _,poly,box in self.polygons:
            if box[0]<=origin[0]<=box[2] and box[1]<=origin[1]<=box[3] and contains(poly,origin):
                return 0.0,origin
        best=maximum
        for a,b in self.ray_edges:
            hit=ray_segment(origin,ray,a,b,best)
            if hit is not None: best=min(best,hit)
        for _,center,radius in self.circles:
            hit=ray_circle(origin,ray,center,radius,best)
            if hit is not None: best=min(best,hit)
        return best,(origin[0]+best*ray[0],origin[1]+best*ray[1])

    def collision(self, poly):
        box=bounds(poly)
        for ident,other,otherbox in self.polygons:
            if boxes_overlap(box,otherbox) and polygons_overlap(poly,other): return ident
        for ident,c,r in self.circles:
            if boxes_overlap(box,(c[0]-r,c[1]-r,c[0]+r,c[1]+r)) and polygon_circle_overlap(poly,c,r): return ident
        return None

    def clearance(self, poly):
        best=float('inf'); box=bounds(poly)
        for ident,other,ob in self.polygons:
            gap=math.hypot(max(ob[0]-box[2],box[0]-ob[2],0),max(ob[1]-box[3],box[1]-ob[3],0))
            if gap<best: best=min(best,polygon_distance(poly,other))
        for _,c,r in self.circles:
            if contains(poly,c): return 0.0
            best=min(best,max(0.0,min(point_segment_distance(c,a,b) for a,b in edges(poly))-r))
        return best

    def add_wall(self,a,b):
        self._point(a); self._point(b)
        if math.dist(a,b)<3: return False
        i=1; ids={w['id'] for w in self.data['walls']}
        while f'added_{i}' in ids: i+=1
        self.data['walls'].append(dict(id=f'added_{i}',a=list(a),b=list(b),thickness_cm=3.0,
                                      color='added',solid=True,confidence='user',note='GUIで追加した壁'))
        self.rebuild(); return True

    def remove_added(self, point):
        candidates=[w for w in self.data['walls'] if w['id'].startswith('added_')]
        if not candidates: return False
        w=min(candidates,key=lambda w: point_segment_distance(point,w['a'],w['b']))
        if point_segment_distance(point,w['a'],w['b'])>30: return False
        self.data['walls'].remove(w); self.rebuild(); return True

    def save(self,path):
        self._validate()
        Path(path).write_text(json.dumps(self.data,ensure_ascii=False,indent=2),encoding='utf-8')
