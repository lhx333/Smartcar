"""Host test of the ACTUAL portable C controller; a toy plant, not vehicle identification."""
import ctypes,json,math
from pathlib import Path
from collections import deque
root=Path(__file__).resolve().parent
lib=ctypes.CDLL(str(root/'firmware_patch/controller_test.dll'))
lib.tick.argtypes=[ctypes.c_float]*6+[ctypes.POINTER(ctypes.c_float)]
lib.reset.argtypes=[]
path=json.loads((root/'out_robust/path.json').read_text(encoding='utf-8'))
barrels=[(x['x'],x['y']) for x in path['obstacles']]

def run(bias=0.,lag=.12,offset=.0):
    lib.reset();out=(ctypes.c_float*5)();dt=.005
    x=0.;y=offset;yaw=0.;nx=0.;ny=0.;est_yaw=0.;v=0.;delta=0.;rate=0.
    buf=deque([(0.,0.)]*max(1,int(lag/dt)))
    peak_error=0.;clearance=100.;status='timeout';finish=None
    for step in range(50000):
        lib.tick(nx,ny,est_yaw,rate+math.radians(bias),v,dt,out)
        peak_error=max(peak_error,float(out[2]))
        if not out[4]: status='tracking_fault';break
        buf.append((float(out[0]),float(out[1])));vref,turn=buf.popleft()
        v+=max(-.45*dt,min(.35*dt,(vref-v)*dt/.25))
        delta+=max(-math.radians(35)*dt,min(math.radians(35)*dt,(turn*math.radians(22)-delta)*dt/.10))
        rate=v*math.tan(delta)/.26
        yaw+=rate*dt;est_yaw+=(rate+math.radians(bias))*dt
        x+=v*math.cos(yaw)*dt;y+=v*math.sin(yaw)*dt
        nx+=v*math.cos(est_yaw)*dt;ny+=v*math.sin(est_yaw)*dt
        clearance=min(clearance,min(math.hypot(x-bx,y-by) for bx,by in barrels))
        if out[3]>path['summary']['length_m']-1 and x<=0 and finish is None:finish=step*dt
        if out[3]>path['summary']['length_m']-.06: status='complete';break
    return dict(bias_dps=bias,delay_s=lag,initial_true_y_offset_m=offset,status=status,
        duration_s=step*dt,finish_line_time_s=finish,peak_estimated_cross_track_m=peak_error,
        min_true_cone_center_distance_m=clearance,
        physical_contact=clearance<.35,robust_margin_violated=clearance<.55,
        successful_safe_finish=(status=='complete' and finish is not None and clearance>=.55),
        final_true_position=[x,y],final_estimated_position=[nx,ny])
if __name__=='__main__':
    results=[run(),run(lag=.25),run(bias=.005),run(bias=.01),run(bias=.05),run(bias=.1),run(offset=.10),run(bias=.20)]
    (root/'out_robust/simulation_report.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(json.dumps(results,indent=2))
