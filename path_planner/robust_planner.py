"""Robust geometric validation + offline time parameterization (standard library)."""
from __future__ import annotations
import bisect
import copy
import csv
import json
import math
from pathlib import Path
import sys
from dataclasses import dataclass, asdict
import path_planner as geo

@dataclass
class Dynamics:
    # Unmeasured commissioning assumptions, SI units. NOT identified vehicle limits.
    vmax: float = 1.0
    accel: float = 0.35
    brake: float = 0.45
    alat: float = 0.65
    mu: float = 0.35
    friction_use: float = 0.55
    wheelbase: float = 0.26
    wheel_angle_deg: float = 22.0
    steer_rate_deg_s: float = 35.0
    lat_jerk: float = 1.2
    lon_jerk: float = 1.0
    delay: float = 0.35
    look_min: float = 0.35
    look_max: float = 0.60
    look_time: float = 0.50
    body_radius: float = 0.25
    cone_radius: float = 0.10
    tracking_margin: float = 0.20
    smooth_reserve: float = 0.12
    smooth_sigma: float = 0.12
    finish_runout: float = 0.60
    residual_bias_dps: float = 0.005
    initial_heading_deg: float = 0.10
    initial_position_m: float = 0.03
    # Probabilities are deliberately not invented. Hard rejection replaces risk trading.
    touch_penalty_s: float = 1.0
    knock_penalty_s: float = 3.0
    missed_gap_penalty_s: float = 30.0
    turn_weight_s: float = 0.30
    clearance_weight_s: float = 1.0
    bounds: object = None  # [xmin,xmax,ymin,ymax], start-local coordinates


def validate_config(d):
    for key,value in asdict(d).items():
        if key == 'bounds': continue
        if not isinstance(value,(int,float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if d.look_min > d.look_max or d.wheel_angle_deg >= 80 or d.friction_use > 1:
        raise ValueError('invalid lookahead / steering / friction limits')
    if d.bounds is not None:
        if len(d.bounds)!=4 or not all(math.isfinite(x) for x in d.bounds):
            raise ValueError('bounds must be [xmin,xmax,ymin,ymax]')
        if d.bounds[0]>=d.bounds[1] or d.bounds[2]>=d.bounds[3]: raise ValueError('empty bounds')


def geometry(points):
    """Recompute distance, tangent and signed circumcircle curvature from coordinates."""
    out=copy.deepcopy(points)
    s=0.0
    for i,p in enumerate(out):
        if i: s+=math.hypot(p.x-out[i-1].x,p.y-out[i-1].y)
        p.s=s
        a,b=out[max(0,i-1)],out[min(len(out)-1,i+1)]
        p.heading=math.atan2(b.y-a.y,b.x-a.x)
    for i in range(1,len(out)-1):
        a,b,c=out[i-1:i+2]
        ux,uy=b.x-a.x,b.y-a.y
        vx,vy=c.x-b.x,c.y-b.y
        den=math.hypot(ux,uy)*math.hypot(vx,vy)*math.hypot(c.x-a.x,c.y-a.y)
        out[i].curvature=2*(ux*vy-uy*vx)/max(den,1e-12)
    out[0].curvature=out[1].curvature
    out[-1].curvature=out[-2].curvature
    return out


def smooth_path(sol,d):
    p=geo._resample_by_spacing(sol.points,0.025)
    # Pad using endpoint tangent: no zero padding / endpoint curling.
    n=int(math.ceil(3*d.smooth_sigma/0.025))
    weights=[math.exp(-0.5*(j*0.025/d.smooth_sigma)**2) for j in range(-n,n+1)]
    total=sum(weights)
    q=copy.deepcopy(p)
    for i in range(1,len(p)-1):
        x=y=0.0
        for j,w in zip(range(-n,n+1),weights):
            k=i+j
            if k<0:
                px=p[0].x+k*0.025*math.cos(p[0].heading)
                py=p[0].y+k*0.025*math.sin(p[0].heading)
            elif k>=len(p):
                px=p[-1].x+(k-len(p)+1)*0.025*math.cos(p[-1].heading)
                py=p[-1].y+(k-len(p)+1)*0.025*math.sin(p[-1].heading)
            else: px,py=p[k].x,p[k].y
            x+=w*px; y+=w*py
        q[i].x=x/total; q[i].y=y/total
    # Timing line is x=0; brake to rest AFTER crossing it.
    end=q[-1]
    for i in range(1,int(math.ceil(d.finish_runout/0.025))+1):
        dist=min(d.finish_runout,i*0.025)
        q.append(geo.PathPoint(end.x-dist,end.y,math.pi,0,0,'return'))
    return geometry(q)


def crossing(a,b,c,e):
    ux,uy=b.x-a.x,b.y-a.y
    vx,vy=e[0]-c[0],e[1]-c[1]
    den=ux*vy-uy*vx
    if abs(den)<1e-10: return False
    wx,wy=c[0]-a.x,c[1]-a.y
    t=(wx*vy-wy*vx)/den
    u=(wx*uy-wy*ux)/den
    return 0<=t<1 and 0<u<1


def validate_path(points,sol,d,min_radius):
    required=max(sol.config.clearance-d.smooth_reserve,d.body_radius+d.cone_radius+d.tracking_margin)
    clearance=min(geo.point_segment_distance(x,y,a.x,a.y,b.x,b.y) for a,b in zip(points,points[1:]) for x,y in sol.obstacles)
    if clearance < required-1e-6: raise ValueError(f'clearance {clearance:.3f} < {required:.3f}')
    peak=max(abs(p.curvature) for p in points)
    hard_radius=max(min_radius,d.wheelbase/math.tan(math.radians(d.wheel_angle_deg)))
    if peak>1/hard_radius+1e-5: raise ValueError(f'radius {1/peak:.3f} < hard radius {hard_radius:.3f}')
    outbound=[p for p in points if p.segment=='outbound']
    if not geo.corridor_ok(outbound,[(0.,0.)]+sol.obstacles,sol.config.corridor_limit):
        raise ValueError('smoothed outbound left corridor')
    # The official task requires EVERY gap, exactly once on the outbound leg.
    counts=[]
    for a,b in zip(sol.obstacles,sol.obstacles[1:]):
        count=sum(crossing(p,q,a,b) for p,q in zip(outbound,outbound[1:]))
        counts.append(count)
        if count!=1: raise ValueError(f'gap crossings {counts}: expected exactly one')
    return_points=[p for p in points if p.segment=='return']
    side=-sol.sides[-1]
    outer=max(side*y for _,y in sol.obstacles)+required
    if any(side*p.y<outer for p in return_points):
        raise ValueError('return lane is not outside the barrel row')
    # Check barrel-side passage at each local tangent-normal cross section.
    for (ox,oy),side,gate in zip(sol.obstacles,sol.sides,sol.gates):
        ux,uy=math.cos(gate.tangent),math.sin(gate.tangent)
        passages=[]
        for a,b in zip(outbound,outbound[1:]):
            pa=(a.x-ox)*ux+(a.y-oy)*uy
            pb=(b.x-ox)*ux+(b.y-oy)*uy
            if pa<=0<pb:
                f=-pa/(pb-pa)
                x=a.x+f*(b.x-a.x)-ox; y=a.y+f*(b.y-a.y)-oy
                passages.append(side*(-uy*x+ux*y))
        if len(passages)!=1 or passages[0]<required-1e-5:
            raise ValueError('wrong/multiple barrel-side passage')
    if d.bounds:
        xmin,xmax,ymin,ymax=d.bounds
        m=d.body_radius+d.tracking_margin
        if any(not(xmin+m<=p.x<=xmax-m and ymin+m<=p.y<=ymax-m) for p in points):
            raise ValueError('vehicle envelope leaves field bounds')
    return {'minimum_clearance_m':clearance,'minimum_radius_m':1/max(peak,1e-12),
            'gap_crossings':counts,'field_bounds_checked':d.bounds is not None}


def profile(points,d):
    n=len(points); s=[p.s for p in points]; k=[p.curvature for p in points]
    friction=d.mu*9.81*d.friction_use
    alat=min(d.alat,friction/math.sqrt(2))
    ax=min(d.accel,friction/math.sqrt(2)); brake=min(d.brake,friction/math.sqrt(2))
    delta=[math.atan(d.wheelbase*x) for x in k]
    dk=[]; dd=[]
    for i in range(n):
        lo=max(0,i-1); hi=min(n-1,i+1); ds=max(s[hi]-s[lo],1e-9)
        dk.append((k[hi]-k[lo])/ds); dd.append((delta[hi]-delta[lo])/ds)
    caps=[min(d.vmax,math.sqrt(alat/max(abs(k[i]),1e-9)),
              math.radians(d.steer_rate_deg_s)/max(abs(dd[i]),1e-9),
              (d.lat_jerk/max(abs(dk[i]),1e-9))**(1/3)) for i in range(n)]
    # Preview all restrictions over the response-distance window, including S transitions.
    preview=d.vmax*d.delay
    v=[min(caps[max(0,i-1):min(n,bisect.bisect_right(s,s[i]+preview)+1)]) for i in range(n)]
    v[0]=min(v[0],0.15); v[-1]=0
    for i in range(n-2,-1,-1): v[i]=min(v[i],math.sqrt(v[i+1]**2+2*brake*(s[i+1]-s[i])))
    for i in range(1,n): v[i]=min(v[i],math.sqrt(v[i-1]**2+2*ax*(s[i]-s[i-1])))
    def measures(v):
        dt=[2*(s[i+1]-s[i])/max(v[i]+v[i+1],1e-12) for i in range(n-1)]
        a=[(v[i+1]-v[i])/dt[i] for i in range(n-1)]
        jy=max([abs(a[i+1]-a[i])/(0.5*(dt[i+1]+dt[i])) for i in range(len(a)-1)]+[0])
        al=[v[i]**2*k[i] for i in range(n)]
        jl=max(abs(al[i+1]-al[i])/dt[i] for i in range(n-1))
        return dt,a,jy,jl
    # Gaussian smoothing of squared speed with odd reflection at endpoints.
    # A global multiplicative envelope keeps the smoothed curve below every cap
    # without reintroducing sharp min() corners at active constraints.
    w=[x*x for x in v]; m=32
    kernel=[math.exp(-0.5*(j/10.)**2) for j in range(-m,m+1)]
    norm=sum(kernel); smooth=[]
    for i in range(n):
        value=0.
        for j,weight in zip(range(-m,m+1),kernel):
            q=i+j
            z=2*w[0]-w[-q] if q<0 else (2*w[-1]-w[2*n-2-q] if q>=n else w[q])
            value+=weight*z
        smooth.append(max(0.,value/norm))
    factor=min([1.]+[w[i]/smooth[i] for i in range(n-1) if smooth[i]>1e-12])
    v=[math.sqrt(x*factor) for x in smooth]; v[-1]=0.
    # Enforce acceleration by global scaling, preserving smooth derivatives.
    dt,a,jx,jy=measures(v)
    factor=min(1.,math.sqrt(ax/max(max(a),1e-12)),math.sqrt(brake/max(-min(a),1e-12)))
    v=[x*factor for x in v]
    dt,a,jx,jy=measures(v)
    scale=min(1.,(d.lon_jerk/max(jx,1e-12))**(1/3),(d.lat_jerk/max(jy,1e-12))**(1/3))
    v=[x*scale*0.999 for x in v]
    dt,a,jx,jy=measures(v)
    times=[0.]
    for t in dt: times.append(times[-1]+t)
    look=[min(d.look_max,max(d.look_min,d.look_min+d.look_time*x)) for x in v]
    # Limit arc angle subtended by lookahead. Do not blindly enlarge L in a tight bend.
    for i in range(n):
        ahead=min(n,bisect.bisect_right(s,s[i]+d.look_max)+1)
        pk=max(abs(x) for x in k[i:ahead])
        look[i]=max(d.look_min,min(look[i],0.45/max(pk,1e-9)))
    return {'speed':v,'time':times,'lookahead':look,'accel':a+[0.],
            'max_lateral_accel':max(v[i]**2*abs(k[i]) for i in range(n)),
            'max_longitudinal_jerk':jx,'max_lateral_jerk':jy,'time_scale':scale,
            'max_steer_rate_deg_s':max(abs(delta[i+1]-delta[i])/dt[i]*180/math.pi for i in range(n-1))}


def drift_check(points,obstacles,d,prof):
    """Conservative dead-reckoning error budget: integral |v|*(yaw0+b*t) dt.
    Excludes wheel-scale error and slip: these need separate physical calibration.
    """
    uncertainty=[d.initial_position_m]
    budget=d.initial_position_m
    residual=[]
    for i in range(1,len(points)):
        angle=math.radians(d.initial_heading_deg+d.residual_bias_dps*prof['time'][i])
        budget+=(points[i].s-points[i-1].s)*angle
        uncertainty.append(budget)
        if d.bounds:
            xmin,xmax,ymin,ymax=d.bounds
            margin=d.body_radius+d.tracking_margin+budget
            q=points[i]
            if not(xmin+margin<=q.x<=xmax-margin and ymin+margin<=q.y<=ymax-margin):
                raise ValueError('field boundary consumed by localization uncertainty')
        a,b=points[i-1],points[i]
        distance=min(geo.point_segment_distance(x,y,a.x,a.y,b.x,b.y) for x,y in obstacles)
        residual.append(distance-d.body_radius-d.cone_radius-d.tracking_margin-budget)
    if min(residual)<0:
        raise ValueError(f'gyro/initial-pose uncertainty consumes clearance: {min(residual):.3f} m; recalibrate or replan')
    return {'min_clearance_after_error_budget_m':min(residual),
            'final_position_error_bound_m':budget,
            'assumed_residual_bias_dps':d.residual_bias_dps}


def export(sol,d,prof,out):
    out=Path(out); out.mkdir(parents=True,exist_ok=True)
    geo.write_outputs(sol,str(out))
    points=sol.points
    with (out/'trajectory.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f); w.writerow(['s_m','x_m','y_m','heading_rad','curvature_1pm','speed_mps','time_s','lookahead_m','accel_mps2'])
        for i,p in enumerate(points): w.writerow([p.s,p.x,p.y,p.heading,p.curvature,prof['speed'][i],prof['time'][i],prof['lookahead'][i],prof['accel'][i]])
    # Export ALL validated samples: no unvalidated 240-point decimation.
    lines=['/* Generated SI trajectory; start-local X forward, Y left. */','#ifndef ROBUST_TRAJECTORY_H','#define ROBUST_TRAJECTORY_H',
           'typedef struct { float x,y,s,k,v,look; } RobustPoint;', 'static const RobustPoint ROBUST_PATH[] = {']
    for i,p in enumerate(points):
        vals=[p.x,p.y,p.s,p.curvature,prof['speed'][i],prof['lookahead'][i]]
        lines.append('    {'+', '.join(f'{v:.7f}f' for v in vals)+'},')
    lines+=['};','#define ROBUST_PATH_N ((int)(sizeof(ROBUST_PATH)/sizeof(ROBUST_PATH[0])))',
            f'#define ROBUST_WHEELBASE {d.wheelbase:.7f}f', f'#define ROBUST_ROAD_ANGLE_RAD {math.radians(d.wheel_angle_deg):.7f}f',
            f'#define ROBUST_ACCEL {d.accel:.7f}f',f'#define ROBUST_BRAKE {d.brake:.7f}f','#endif']
    (out/'robust_trajectory.h').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    # Legacy XY-only output is explicitly unsuitable for activating speed/lookahead.
    lines=['/* XY only: speed scheduling requires robust_trajectory.h + firmware bridge. */','static const float PATH_PTS[][2] = {']
    lines += [f'    {{{p.x:.7f}f, {p.y:.7f}f}},' for p in points]
    lines+=['};']
    (out/'path_array.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    payload=json.loads((out/'path.json').read_text(encoding='utf-8'))
    payload['dynamics']=asdict(d)
    for i,p in enumerate(payload['path']):
        p.update(speed_mps=prof['speed'][i],lookahead_m=prof['lookahead'][i],time_s=prof['time'][i])
    (out/'path.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')


def main(argv=None):
    for stream in (sys.stdout,sys.stderr):
        if hasattr(stream,'reconfigure'): stream.reconfigure(encoding='utf-8')
    parser=geo.build_arg_parser()
    parser.set_defaults(radius=1.20,min_radius=1.0,clearance=0.60,uturn_radius=1.40)
    parser.add_argument('--dynamics',default=str(Path(__file__).with_name('vehicle_config.json')))
    parser.add_argument('--radii',default=None,help='candidate radii, none below --radius; e.g. 1.2,1.4,1.6')
    args=parser.parse_args(argv)
    try:
        d=Dynamics(**json.loads(Path(args.dynamics).read_text(encoding='utf-8-sig')))
        validate_config(d)
        obs,start,sides=geo.load_input(args)
        if not obs or not all(math.isfinite(v) for xy in obs for v in xy): raise ValueError('invalid barrel coordinates')
        if not all(math.isfinite(v) for v in (start.x,start.y,start.theta)): raise ValueError('invalid start')
        for name in ('radius','min_radius','clearance','max_offset','uturn_radius','heading_step','heading_span'):
            if not math.isfinite(getattr(args,name)) or getattr(args,name)<=0: raise ValueError(f'invalid {name}')
        local=geo.transform_to_start_frame(obs,start)
        if any(math.hypot(a[0]-b[0],a[1]-b[1])<2*(d.body_radius+d.cone_radius+d.tracking_margin) for a,b in zip(local,local[1:])):
            raise ValueError('adjacent barrels too close for robust footprint')
        if any(b[0]<=a[0] for a,b in zip([(0.,0.)]+local,local)):
            raise ValueError('barrels must advance in start-frame x; rotate start heading for another row orientation')
        sides=sides or [1 if (i%2==0)==(args.first_side=='left') else -1 for i in range(len(obs))]
        if any(sides[i]==sides[i+1] for i in range(len(sides)-1)): raise ValueError('race mode requires alternating sides')
        radii=[float(r) for r in args.radii.split(',')] if args.radii else [args.radius,args.radius+0.2,args.radius+0.4]
        if not all(math.isfinite(r) and r>=max(args.radius,args.min_radius) for r in radii): raise ValueError('candidate radius below hard bound')
        candidates=[]; reports=[]
        for radius in radii:
            print(f'Checking radius {radius:.2f} m ...',flush=True)
            cfg=geo.PlannerConfig(clearance=max(args.clearance,d.body_radius+d.cone_radius+d.tracking_margin)+d.smooth_reserve,
                max_offset=args.max_offset,radius=radius,min_radius=radius,uturn_radius=max(args.uturn_radius,radius),
                sides=sides,first_side=sides[0],corridor_limit=args.corridor,heading_step_deg=args.heading_step,
                heading_span_deg=args.heading_span,offset_penalty=0.,exit_lengths=tuple(sorted(set([args.exit,1.,1.4,1.8,2.2]))))
            # Reserve a straight launch and 3% radius for smoothing/chord effects.
            launch=0.50
            shifted=[(x-launch,y) for x,y in local]
            sol=geo.plan_for_radius(shifted,geo.obstacle_tangents(shifted),sides,cfg,radius*1.03)
            if sol is not None:
                for p in sol.points: p.x+=launch; p.s+=launch
                prefix=[geo.PathPoint(i*0.025,0,0,i*0.025,0,'outbound') for i in range(20)]
                sol.points=prefix+sol.points
                # The shifted solver returns to x=0.5: extend to the original timing line.
                end=sol.points[-1]
                sol.points += [geo.PathPoint(launch-i*0.025,end.y,math.pi,end.s+i*0.025,0,'return') for i in range(1,21)]
                sol.obstacles=list(local)
                for gate in sol.gates:
                    gate.obstacle=(gate.obstacle[0]+launch,gate.obstacle[1])
                    for candidate in gate.candidates: candidate.pose.x+=launch
                sol.start=geo.Pose(0,0,0)
            if sol is None or sol.summary.get('has_loop'):
                reports.append({'radius':radius,'accepted':False,'reason':'no loop-free geometric candidate'}); continue
            try:
                sol.points=smooth_path(sol,d)
                check=validate_path(sol.points,sol,d,max(args.radius,args.min_radius))
                prof=profile(sol.points,d)
                check.update(drift_check(sol.points,sol.obstacles,d,prof))
                variation=sum(abs(b.curvature-a.curvature) for a,b in zip(sol.points,sol.points[1:]))
                clearance_margin=check['minimum_clearance_m']-d.body_radius-d.cone_radius
                score=prof['time'][-1]+d.turn_weight_s*variation+d.clearance_weight_s*(d.touch_penalty_s+d.knock_penalty_s)*d.tracking_margin/max(clearance_margin,0.01)
                check.update(predicted_total_time_s=prof['time'][-1],score=score,score_type='heuristic seconds, NOT calibrated expected race penalty',radius_candidate_m=radius,
                    max_lateral_accel_mps2=prof['max_lateral_accel'],max_lateral_jerk_mps3=prof['max_lateral_jerk'],
                    max_longitudinal_jerk_mps3=prof['max_longitudinal_jerk'],max_steer_rate_deg_s=prof['max_steer_rate_deg_s'],
                    peak_speed_mps=max(prof['speed']),time_dilation_factor=prof['time_scale'])
                sol.summary.update(check); sol.summary['length_m']=sol.points[-1].s
                sol.summary['min_clearance_m']=check['minimum_clearance_m']
                sol.summary['radius_used_m']=check['minimum_radius_m']
                sol.summary.update(end_x_m=sol.points[-1].x,end_y_m=sol.points[-1].y,path_points=len(sol.points),
                    clearance_target_m=max(args.clearance,d.body_radius+d.cone_radius+d.tracking_margin))
                for label in ('outbound','uturn','return'):
                    sol.summary[label+'_length_m']=sum(b.s-a.s for a,b in zip(sol.points,sol.points[1:]) if b.segment==label)
                # Compute score timing at the x=0 return crossing separately from stop/runout.
                line=next((i for i,p in enumerate(sol.points) if p.segment=='return' and p.x<=0),len(sol.points)-1)
                sol.summary['predicted_finish_line_time_s']=prof['time'][line]
                sol.warnings.append('Vehicle dynamics are unmeasured assumptions; simulation does not certify real tracking.')
                if d.bounds is None: sol.warnings.append('Field bounds unspecified: boundary compliance NOT certified.')
                sol.summary['warnings']=sol.warnings
                candidates.append((score,sol,prof)); reports.append(dict(check,accepted=True))
            except ValueError as exc: reports.append({'radius':radius,'accepted':False,'reason':str(exc)})
        if not candidates:
            raise ValueError('No robust candidate. Constraints were NOT relaxed. '+json.dumps(reports,ensure_ascii=False))
        _,sol,prof=min(candidates,key=lambda c:c[0])
        export(sol,d,prof,args.out)
        (Path(args.out)/'candidate_report.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(sol.summary,ensure_ascii=False,indent=2))
        print('Validated nominal trajectory exported. Read ROBUSTNESS.md before firmware integration.')
        return 0
    except (ValueError,RuntimeError,OSError,TypeError) as exc:
        print('Planning rejected: '+str(exc),file=sys.stderr)
        return 3

if __name__=='__main__': raise SystemExit(main())
