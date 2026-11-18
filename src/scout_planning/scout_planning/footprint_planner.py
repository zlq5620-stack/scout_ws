"""Heading-aware forward/rotate search; every motion checks an oriented rectangle.

Continuous poses are retained; discretization merges search states only. This is
an approximate, bounded hybrid search, not a completeness or optimality claim.
"""
import heapq
import math
import time
import numpy as np
from scout_avoidance.local_planner_core import MapCollision, wrap
from scipy.ndimage import distance_transform_edt


class FootprintPlanner:
    def __init__(self, values, resolution, origin, length=.61, width=.56, margin=.04):
        if not all(math.isfinite(v) and v > 0 for v in (length,width,margin)):
            raise ValueError('Invalid footprint dimensions')
        self.map=MapCollision(values,resolution,origin)
        self.hx=length/2+margin;self.hy=width/2+margin
        self.radius=math.hypot(self.hx,self.hy)
        self.step=max(.08,min(.15,resolution*2))
        self.angle_step=math.pi/8
        self.position_bin=self.step/2
        self.guide_cache=None

    def clear(self, poses, padding=0.):
        return self.map.clear_rectangle(poses,self.hx+padding,self.hy+padding)

    def segment(self, start, end):
        """Only straight translation OR in-place rotation; padded sampled sweep."""
        distance=math.hypot(end[0]-start[0],end[1]-start[1])
        angle=wrap(end[2]-start[2])
        n=max(1,int(math.ceil(distance/.02)),int(math.ceil(abs(angle)/math.radians(3))))
        t=np.linspace(0,1,n+1)
        poses=np.column_stack((start[0]+t*(end[0]-start[0]),start[1]+t*(end[1]-start[1]),start[2]+t*angle))
        if distance > 1e-8:
            # A straight rectangular sweep is exactly a longer rectangle.
            # This tests the WHOLE segment cheaply, not just its endpoints.
            lateral=-(end[0]-start[0])*math.sin(start[2])+(end[1]-start[1])*math.cos(start[2])
            if abs(angle)>1e-8 or abs(lateral)>1e-8:return None
            mid=[(start[0]+end[0])/2,(start[1]+end[1])/2,start[2]]
            return poses if self.map.clear_rectangle([mid],self.hx+distance/2+.001,self.hy+.001) else None
        # Between-sample corner displacement is bounded by translation + arc.
        padding=(distance+self.radius*abs(angle))/(2*n)+.001
        return poses if self.clear(poses,padding) else None

    def connector(self, start, goal):
        distance=math.hypot(goal[0]-start[0],goal[1]-start[1])
        heading=math.atan2(goal[1]-start[1],goal[0]-start[0]) if distance>1e-8 else start[2]
        points=[(start[0],start[1],heading),(goal[0],goal[1],heading),goal]
        current=start;result=[];cost=0.
        for end in points:
            checked=self.segment(current,end)
            if checked is None:return None
            result.extend(checked[1:]);cost+=math.hypot(end[0]-current[0],end[1]-current[1])+.12*abs(wrap(end[2]-current[2]))
            current=end
        return result,cost

    def search(self,start,goal,max_expansions=15000,time_budget=.7):
        if not all(math.isfinite(x) for x in (*start,*goal)):
            raise ValueError('Invalid start/goal')
        if not self.clear([start]):return None,'START_BLOCKED',0
        if not self.clear([goal]):return None,'GOAL_BLOCKED',0
        deadline=time.monotonic()+time_budget
        # Obstacle-aware 2D distance field guides the heading search around walls.
        # It is only a heuristic: full rectangle sweeps remain authoritative.
        ox,oy,a=self.map.origin;c,s=math.cos(a),math.sin(a)
        def cell(p):
            dx,dy=p[0]-ox,p[1]-oy
            return int((c*dx+s*dy)//self.map.resolution),int((-s*dx+c*dy)//self.map.resolution)
        goal_cell=cell(goal)
        if self.guide_cache is None or self.guide_cache[0]!=goal_cell:
            blocked=(self.map.values<0)|(self.map.values>=60)
            # Inscribed radius discourages guide paths through slots too thin.
            guide_blocked=distance_transform_edt(~blocked)*self.map.resolution < min(self.hx,self.hy)
            h,w=blocked.shape;field=np.full((h,w),np.inf)
            gx,gy=goal_cell
            if 0<=gx<w and 0<=gy<h:
                field[gy,gx]=0.;queue=[(0.,gx,gy)]
                while queue:
                    if time.monotonic()>deadline:return None,'SEARCH_TIME_LIMIT',0
                    cost,x,y=heapq.heappop(queue)
                    if cost>field[y,x]:continue
                    for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                        nx,ny=x+dx,y+dy
                        if not(0<=nx<w and 0<=ny<h) or guide_blocked[ny,nx]:continue
                        score=cost+math.hypot(dx,dy)*self.map.resolution
                        if score<field[ny,nx]:field[ny,nx]=score;heapq.heappush(queue,(score,nx,ny))
            self.guide_cache=(goal_cell,field)
        field=self.guide_cache[1]
        def key(p):return (round(p[0]/self.position_bin),round(p[1]/self.position_bin),round(wrap(p[2])/self.angle_step)%16)
        def distance(p):return math.hypot(p[0]-goal[0],p[1]-goal[1])
        def heuristic(p):
            x,y=cell(p)
            d=field[y,x] if 0<=y<field.shape[0] and 0<=x<field.shape[1] else math.inf
            return 1.4*max(distance(p),float(d)) if math.isfinite(d) else 1.4*distance(p)
        records=[(tuple(start),None,None,0.)];best={key(start):0.};heap=[(heuristic(start),0)];expanded=0
        while heap and expanded<max_expansions:
            if time.monotonic()>deadline:return None,'SEARCH_TIME_LIMIT',expanded
            _,idx=heapq.heappop(heap);p,parent,edge,g=records[idx]
            if g>best.get(key(p),math.inf)+1e-9:continue
            expanded+=1
            if distance(p)<=.30:
                link=self.connector(p,goal)
                if link is not None:
                    edges=[link[0]];cur=idx
                    while records[cur][1] is not None:
                        edges.append(records[cur][2]);cur=records[cur][1]
                    path=[tuple(start)]
                    for e in reversed(edges):path.extend(tuple(v) for v in e)
                    return path,'PATH_READY',expanded
            successors=[(p[0]+self.step*math.cos(p[2]),p[1]+self.step*math.sin(p[2]),p[2])]
            successors.extend((p[0],p[1],wrap(p[2]+sign*self.angle_step)) for sign in (-1,1))
            for end in successors:
                score=g+math.hypot(end[0]-p[0],end[1]-p[1])+.12*abs(wrap(end[2]-p[2]))
                k=key(end)
                if score>=best.get(k,math.inf)-1e-9:continue
                checked=self.segment(p,end)
                if checked is None:continue
                best[k]=score;records.append((end,idx,checked[1:],score));nxt=len(records)-1
                heapq.heappush(heap,(score+heuristic(end),nxt))
        return None,'EXPANSION_LIMIT' if heap else 'NO_PATH',expanded

