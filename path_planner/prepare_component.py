"""Keil prebuild: shared vehicle JSON + independent test JSON. No MCU JSON parser."""
from pathlib import Path
import os
import json, math, sys, copy, hashlib
import robust_planner as r
import path_planner as g
BASE=Path(__file__).resolve().parent
FW=BASE.parent/'firmware/demo/DEMO_component'
os.environ.setdefault('MPLCONFIGDIR',str(FW/'generated_path/.mplconfig'))

def write(p,s):
    b=s.encode('utf-8')
    if not p.exists() or p.read_bytes()!=b:p.write_bytes(b)

def main():
    v=json.loads((BASE/'vehicle_config.json').read_text(encoding='utf-8-sig'))
    c=json.loads((BASE/'component_tests.json').read_text(encoding='utf-8-sig'))
    d=r.Dynamics(**v);r.validate_config(d)
    if c['module'] not in range(1,6):raise ValueError('module must be 1..5')
    def positive(x):
        if not isinstance(x,(int,float)) or not math.isfinite(x) or x<=0:raise ValueError('expected finite positive test parameter')
    for group in ('pwm','speed','steering','circle','path'):
        for k,x in c[group].items():
            if k not in ('offsets_deg','cones_m','turn_deg','target_yaw_deg'):positive(x)
    for x in (c['settle_hold_s'],c['settle_timeout_s']):positive(x)
    if not 10<=c['pwm']['high_percent']<=40:raise ValueError('module 1 default PWM must be 10..40 percent')
    if not 2<=c['pwm']['hold_high_s']<=5:raise ValueError('module 1 default hold must be 2..5 seconds')
    if c['pwm']['max_steer_deg']>d.servo_max_deg:raise ValueError('module 1 steering limit exceeds servo_max_deg')
    if any(x>d.vmax for x in [c['speed']['target_mps'],c['circle']['speed1_mps'],c['circle']['speed2_mps'],c['path']['vmax_mps']]):raise ValueError('test speed exceeds shared vmax')
    offsets=c['steering']['offsets_deg']
    if not offsets or len(offsets)>16 or offsets[-1]!=0:raise ValueError('steering offsets require 1..16 values, last=0')
    if any(not math.isfinite(x) or abs(x)>d.servo_max_deg for x in offsets+[c['circle']['turn_deg']]):raise ValueError('steering offset exceeds shared servo_max_deg')
    if not 5<=abs(c['circle']['target_yaw_deg'])<=90:raise ValueError('circle target_yaw_deg must have magnitude 5..90')
    if len(c['path']['cones_m'])!=2:raise ValueError('path module expects exactly two cones')
    if d.pwm_max_percent>51:raise ValueError('existing E35 fast PWM telemetry supports at most 51 percent')
    inc=FW/'user/inc'
    h=['/* Generated: shared vehicle_config.json */','#ifndef VEHICLE_PARAMS_H','#define VEHICLE_PARAMS_H']
    for k,val in [('PWM_START_MILLI_PERCENT',round(d.pwm_start_percent*1000)),('PWM_MAX_MILLI_PERCENT',round(d.pwm_max_percent*1000)),('SERVO_MAX_MILLIDEG',round(d.servo_max_deg*1000))]:h.append(f'#define VEHICLE_{k} ({val})')
    for name,k in [('PWM_START_PERCENT','pwm_start_percent'),('PWM_MAX_PERCENT','pwm_max_percent'),('SERVO_MAX_DEG','servo_max_deg'),('SPEED_KP','speed_kp'),('SPEED_KI','speed_ki'),('SPEED_KD','speed_kd')]:h.append(f'#define VEHICLE_{name} ({float(v[k])}f)')
    h+=['#endif',''];write(inc/'vehicle_params.h','\n'.join(h))
    mapping={'MODULE':c['module'],'PWM_HIGH':c['pwm']['high_percent'],'PWM_HOLD_HIGH':c['pwm']['hold_high_s'],'STRAIGHT_KP':c['pwm']['heading_kp_per_deg'],'STRAIGHT_KD':c['pwm']['yaw_rate_kd_per_dps'],'STRAIGHT_WHEEL_K':c['pwm']['wheel_diff_k_per_mps'],'STRAIGHT_MAX_DEG':c['pwm']['max_steer_deg'],'SPEED_TARGET':c['speed']['target_mps'],'SPEED_HOLD':c['speed']['hold_s'],'SPEED_ACCEL':c['speed']['accel_mps2'],'SPEED_DECEL':c['speed']['decel_mps2'],'STEER_RATE':c['steering']['rate_deg_s'],'STEER_HOLD':c['steering']['hold_s'],'CIRCLE_STRAIGHT':c['circle']['straight_s'],'CIRCLE_TURN':c['circle']['turn_deg'],'CIRCLE_TARGET_YAW':c['circle']['target_yaw_deg'],'CIRCLE_V1':c['circle']['speed1_mps'],'CIRCLE_V2':c['circle']['speed2_mps'],'CIRCLE_HOLD1':c['circle']['hold1_s'],'CIRCLE_HOLD2':c['circle']['hold2_s'],'SETTLE_HOLD':c['settle_hold_s'],'SETTLE_TIMEOUT':c['settle_timeout_s']}
    h=['/* Generated: component_tests.json */','#ifndef COMPONENT_CONFIG_H','#define COMPONENT_CONFIG_H']
    for k,x in mapping.items():h.append(f'#define C_{k} ({int(x) if k=="MODULE" else str(float(x))+"f"})')
    h+=['#define C_STEER_COUNT '+str(len(offsets)), 'static const float COMP_STEER_OFFSETS[] = {'+','.join(str(float(x))+'f' for x in offsets)+'};','#endif','']
    write(inc/'component_config.h','\n'.join(h))
    # Regenerate navigation macros for all modules; real path required only in mode 5.
    out=FW/'generated_path';out.mkdir(exist_ok=True)
    if c['module']==5:
        cones=c['path']['cones_m']; local=copy.deepcopy(d);local.vmax=c['path']['vmax_mps']
        local.steer_rate_deg_s=min(local.steer_rate_deg_s,c['steering']['rate_deg_s']*d.wheel_angle_deg/d.servo_max_deg)
        cfg=out/'dynamics.json';write(cfg,json.dumps(vars(local),indent=2))
        args=['--points',' '.join(f'{x},{y}' for x,y in cones),'--out',str(out),'--dynamics',str(cfg),'--radius',str(c['path']['radius_m'])]
        if r.main(args):raise ValueError('two cone planning rejected')
        payload=json.loads((out/'path.json').read_text(encoding='utf-8'))
        pts=[g.PathPoint(p['x'],p['y'],math.radians(p['heading_deg']),p['s_m'],p['curvature_1pm'],'outbound') for p in payload['path'] if p['segment']=='outbound']
        end=pts[-1]
        for dist in __import__('numpy').arange(.025,c['path']['exit_straight_m']+.001,.025):pts.append(g.PathPoint(end.x+dist*math.cos(end.heading),end.y+dist*math.sin(end.heading),end.heading,0,0,'outbound'))
        pts=r.geometry(pts)
        required=max(.60,d.body_radius+d.cone_radius+d.tracking_margin)
        if min(g.point_segment_distance(x,y,a.x,a.y,b.x,b.y) for a,b in zip(pts,pts[1:]) for x,y in cones)<required:raise ValueError('exit straight violates cone clearance')
        if max(abs(p.curvature) for p in pts)>1/max(c['path']['radius_m'],d.wheelbase/math.tan(math.radians(d.wheel_angle_deg)))+1e-5:raise ValueError('exit curvature violates radius')
        if d.bounds and any(not(d.bounds[0]+d.body_radius+d.tracking_margin<=p.x<=d.bounds[1]-d.body_radius-d.tracking_margin and d.bounds[2]+d.body_radius+d.tracking_margin<=p.y<=d.bounds[3]-d.body_radius-d.tracking_margin) for p in pts):raise ValueError('exit violates bounds')
        prof=r.profile(pts,local);r.drift_check(pts,cones,local,prof)
        curve_accel=any(abs(p.curvature)>.05 and a>.01 for p,a in zip(pts,prof['accel']))
        if not curve_accel:raise ValueError('no curve acceleration: adjust path/speed constraints')
        write(out/'component_summary.json',json.dumps({'length_m':pts[-1].s,'duration_s':prof['time'][-1],'peak_speed_mps':max(prof['speed']),'curve_acceleration_samples':int(sum(abs(p.curvature)>.05 and a>.01 for p,a in zip(pts,prof['accel']))),'cones_m':cones,'note':'Forward-only segment extracted from finite candidate planner; no global optimality claim.'},indent=2))
        # Reuse the planner exporter without its turnaround and return leg.
        from types import SimpleNamespace
        lines=['#ifndef ROBUST_TRAJECTORY_H','#define ROBUST_TRAJECTORY_H','typedef struct { float x,y,s,k,v,look; } RobustPoint;','static const RobustPoint ROBUST_PATH[] = {']
        lines += ['{'+','.join(f'{x:.7f}f' for x in [p.x,p.y,p.s,p.curvature,prof['speed'][i],prof['lookahead'][i]])+'},' for i,p in enumerate(pts)]
        lines+=['};']
        import csv
        with (out/'component_trajectory.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['s_m','x_m','y_m','curvature_1pm','speed_mps','accel_mps2','time_s'])
            w.writerows([p.s,p.x,p.y,p.curvature,prof['speed'][i],prof['accel'][i],prof['time'][i]] for i,p in enumerate(pts))
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(2,1,figsize=(9,7));ax[0].plot([p.x for p in pts],[p.y for p in pts]);ax[0].scatter(*zip(*cones),c='r');ax[0].axis('equal');ax[0].grid();ax[1].plot([p.s for p in pts],prof['speed']);ax[1].set(xlabel='s (m)',ylabel='v (m/s)');ax[1].grid();fig.tight_layout();fig.savefig(out/'component_path.png');plt.close(fig)
    else:
        lines=['#ifndef ROBUST_TRAJECTORY_H','#define ROBUST_TRAJECTORY_H','typedef struct { float x,y,s,k,v,look; } RobustPoint;','static const RobustPoint ROBUST_PATH[]={{0,0,0,0,0,.35f},{1,0,1,0,0,.35f}};']
    lines+=['#define ROBUST_PATH_N ((int)(sizeof(ROBUST_PATH)/sizeof(ROBUST_PATH[0])))']
    for name,x in [('WHEELBASE',d.wheelbase),('ROAD_ANGLE_RAD',math.radians(d.wheel_angle_deg)),('ACCEL',d.accel),('BRAKE',d.brake),('LOOK_MIN',d.look_min),('LOOK_MAX',d.look_max),('LOOK_TIME',d.look_time)]:lines.append(f'#define ROBUST_{name} ({float(x)}f)')
    lines+=['#endif',''];write(inc/'robust_trajectory.h','\n'.join(lines))
    write(FW/'component_manifest.json',json.dumps({'module':c['module'],'vehicle':v,'tests':c,'generated_header_sha256':{n:hashlib.sha256((inc/n).read_bytes()).hexdigest() for n in ['vehicle_params.h','component_config.h','robust_trajectory.h']}},indent=2))
    print(f'COMPONENT READY: module {c["module"]}, wheelbase {d.wheelbase}, body radius {d.body_radius}')

if __name__=='__main__':
    try:main()
    except Exception as e:print('COMPONENT BUILD BLOCKED:',e,file=sys.stderr);sys.exit(1)
