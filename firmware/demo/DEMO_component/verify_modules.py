"""Desktop contract checks on actual portable task code; restores selected JSON mode."""
from pathlib import Path
import subprocess,json,tempfile
ROOT=Path(__file__).resolve().parent
PLANNER=ROOT.parents[2]/'path_planner'
cfg=PLANNER/'component_tests.json'
original=cfg.read_bytes()
log=[]
source=r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "component.h"
int main(void){int i,seen[32]={0},positive=0,negative=0;float v=0;ComponentOutput o;component_reset();
for(i=0;i<24000;i++){
o=component_step(.005f,v,C_PWM_HIGH,C_PWM_HOLD_HIGH,C_CIRCLE_TARGET_YAW,C_CIRCLE_TURN,C_MODULE==4&&i>1500?C_CIRCLE_TARGET_YAW:0,C_CIRCLE_V1,C_CIRCLE_V2);assert(isfinite(o.speed)&&isfinite(o.turn)&&isfinite(o.pwm));assert(o.speed>=-.0001f&&o.pwm>=-.0001f);
if(o.phase>=0&&o.phase<32)seen[o.phase]=1;
if(C_MODULE==3){assert(o.pwm==0);positive|=o.turn>.9f;negative|=o.turn<-.9f;}
if(C_MODULE==1 && i==0)assert(fabsf(o.pwm-C_PWM_HIGH)<.001f);
if(C_MODULE==1 && i==999)assert(o.phase==1&&fabsf(o.pwm-C_PWM_HIGH)<.001f);
if(C_MODULE==1 && i==1000)assert(o.phase==9&&o.pwm==0);
v=o.open_loop?(o.pwm>0?.3f:0):o.speed;
if(o.done){assert(o.done==1);break;}
}
assert(i<23999);
if(C_MODULE==1){
assert(seen[1]&&seen[9]&&i>=1000);
component_reset();v=0;
for(i=0;i<401;i++){o=component_step(.005f,v,40.0f,2.0f,C_CIRCLE_TARGET_YAW,C_CIRCLE_TURN,0,C_CIRCLE_V1,C_CIRCLE_V2);v=o.pwm>0?.3f:0;
if(i==0||i==399)assert(o.phase==1&&fabsf(o.pwm-40.0f)<.001f);
if(i==400)assert(o.phase==9&&o.pwm==0);}
}
if(C_MODULE==2)assert(seen[1]&&seen[2]&&seen[3]&&seen[9]);
if(C_MODULE==3)assert(positive&&negative&&fabsf(o.turn)<.001f);
if(C_MODULE==4)assert(seen[1]&&seen[2]&&seen[3]&&seen[4]&&seen[5]&&seen[6]&&seen[9]);
printf("PASS module %d: phase sequence, finite outputs, completion, command contract\n",C_MODULE);return 0;}
'''
try:
    for mode in range(1,6):
        c=json.loads(original);c['module']=mode;cfg.write_text(json.dumps(c,indent=2),encoding='utf-8')
        result=subprocess.run(['python',str(PLANNER/'prepare_component.py')],capture_output=True,text=True,encoding='utf-8')
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        if mode==5:
            s=json.loads((ROOT/'generated_path/component_summary.json').read_text())
            assert s['curve_acceleration_samples']>0
            log.append('PASS module 5: forward path generated, geometric/dynamic checks passed, curve acceleration exists')
            continue
        with tempfile.TemporaryDirectory() as td:
            p=Path(td);(p/'main.c').write_text(source)
            subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(ROOT/'code'),'-I',str(ROOT/'user/inc'),str(p/'main.c'),str(ROOT/'code/component.c'),'-lm','-o',str(p/'verify.exe')],check=True)
            result=subprocess.run([str(p/'verify.exe')],capture_output=True,text=True)
            if result.returncode:raise RuntimeError(result.stdout+result.stderr)
            log.append(result.stdout.strip())
finally:
    cfg.write_bytes(original)
    subprocess.run(['python',str(PLANNER/'prepare_component.py')],check=True)
(ROOT/'verification_components.txt').write_text('\n'.join(log)+'\n',encoding='utf-8')
print('\n'.join(log))
