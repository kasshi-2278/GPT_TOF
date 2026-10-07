"""Dependency-free 2D geometry. All distances are centimeters."""
from __future__ import annotations
import math
from typing import Iterable

Point = tuple[float, float]
EPS = 1e-9

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))

def direction(heading: float) -> Point:
    """World x=right, y=up; heading 0=north, +90=east."""
    a = math.radians(heading)
    return math.sin(a), math.cos(a)

def local_to_world(cx: float, cy: float, heading: float,
                   forward: float, left: float) -> Point:
    dx, dy = direction(heading)
    return cx + forward*dx - left*dy, cy + forward*dy + left*dx

def rectangle(cx: float, cy: float, heading: float, length: float, width: float) -> list[Point]:
    return [local_to_world(cx, cy, heading, a*length/2, b*width/2)
            for a,b in [(1,1),(1,-1),(-1,-1),(-1,1)]]

def wall_polygon(a: Point, b: Point, thickness: float) -> list[Point]:
    dx,dy=b[0]-a[0],b[1]-a[1]
    size=math.hypot(dx,dy)
    if size < EPS: raise ValueError('Wall endpoints must differ')
    nx,ny=-dy/size*thickness/2,dx/size*thickness/2
    return [(a[0]+nx,a[1]+ny),(b[0]+nx,b[1]+ny),
            (b[0]-nx,b[1]-ny),(a[0]-nx,a[1]-ny)]

def bounds(poly: Iterable[Point]) -> tuple[float,float,float,float]:
    p=list(poly)
    return min(x for x,y in p),min(y for x,y in p),max(x for x,y in p),max(y for x,y in p)

def boxes_overlap(a, b) -> bool:
    return not (a[2]<b[0]-EPS or b[2]<a[0]-EPS or a[3]<b[1]-EPS or b[3]<a[1]-EPS)

def edges(poly):
    for i in range(len(poly)):
        yield poly[i],poly[(i+1)%len(poly)]

def cross(a: Point, b: Point) -> float:
    return a[0]*b[1]-a[1]*b[0]

def is_convex(poly: list[Point]) -> bool:
    signs=[]
    for i in range(len(poly)):
        a,b,c=poly[i-2],poly[i-1],poly[i]
        z=cross((b[0]-a[0],b[1]-a[1]),(c[0]-b[0],c[1]-b[1]))
        if abs(z)>EPS: signs.append(z>0)
    return bool(signs) and (all(signs) or not any(signs))

def contains(poly: list[Point], point: Point) -> bool:
    # Convex, boundaries included.
    pos=neg=False
    for a,b in edges(poly):
        z=cross((b[0]-a[0],b[1]-a[1]),(point[0]-a[0],point[1]-a[1]))
        pos |= z>EPS; neg |= z<-EPS
    return not (pos and neg)

def polygons_overlap(a: list[Point], b: list[Point]) -> bool:
    """SAT includes containment, tangency, and collinear contact."""
    if not boxes_overlap(bounds(a),bounds(b)): return False
    for poly in (a,b):
        for p,q in edges(poly):
            nx,ny=-(q[1]-p[1]),q[0]-p[0]
            pa=[x*nx+y*ny for x,y in a]; pb=[x*nx+y*ny for x,y in b]
            if max(pa)<min(pb)-EPS or max(pb)<min(pa)-EPS: return False
    return True

def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    dx,dy=b[0]-a[0],b[1]-a[1]
    length2=dx*dx+dy*dy
    t=clamp(((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length2,0,1) if length2 else 0
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)

def polygon_circle_overlap(poly: list[Point], center: Point, radius: float) -> bool:
    if contains(poly,center): return True
    return any(point_segment_distance(center,a,b)<=radius+EPS for a,b in edges(poly))

def polygon_distance(a: list[Point], b: list[Point]) -> float:
    if polygons_overlap(a,b): return 0.0
    return min(min(point_segment_distance(p,x,y) for p in a for x,y in edges(b)),
               min(point_segment_distance(p,x,y) for p in b for x,y in edges(a)))

def ray_segment(origin: Point, ray: Point, a: Point, b: Point, max_distance: float):
    s=(b[0]-a[0],b[1]-a[1]); q=(a[0]-origin[0],a[1]-origin[1])
    den=cross(ray,s)
    if abs(den)<EPS:
        if abs(cross(q,ray))>EPS: return None
        u=q[0]*ray[0]+q[1]*ray[1]
        v=(b[0]-origin[0])*ray[0]+(b[1]-origin[1])*ray[1]
        if max(u,v)<-EPS: return None
        d=max(0.0,min(u,v))
        return d if d<=max_distance+EPS else None
    t=cross(q,s)/den; u=cross(q,ray)/den
    if -EPS<=t<=max_distance+EPS and -EPS<=u<=1+EPS: return max(0.0,t)
    return None

def ray_circle(origin: Point, ray: Point, center: Point, radius: float, max_distance: float):
    ox,oy=origin[0]-center[0],origin[1]-center[1]
    c=ox*ox+oy*oy-radius*radius
    if c<=0: return 0.0
    b=ox*ray[0]+oy*ray[1]; discriminant=b*b-c
    if discriminant<0: return None
    d=-b-math.sqrt(discriminant)
    return d if 0<=d<=max_distance else None
