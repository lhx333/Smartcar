from pathlib import Path
import json,subprocess,tempfile
root=Path(__file__).resolve().parents[1]
planner=root.parents[2]/'path_planner'
config=planner/'component_tests.json'
original=config.read_bytes(); lines=[]
try:
    for mode in range(1,6):
        c=json.loads(original);c['module']=mode;config.write_text(json.dumps(c,indent=2),encoding='utf-8')
        r=subprocess.run(['python',str(planner/'prepare_component.py')],capture_output=True,text=True,encoding='utf-8')
        if r.returncode:raise RuntimeError(r.stdout+r.stderr)
        with tempfile.TemporaryDirectory() as td:
            binary=Path(td)/'startup.exe'
            subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(root/'verification/runtime_stubs'),'-I',str(root/'user/inc'),'-I',str(root/'code'),str(root/'verification/startup_gate.c'),str(root/'code/component.c'),'-lm','-o',str(binary)],check=True)
            r=subprocess.run([str(binary)],capture_output=True,text=True,check=True)
            lines.append(r.stdout.strip())
finally:
    config.write_bytes(original)
    subprocess.run(['python',str(planner/'prepare_component.py')],check=True)
(root/'verification/startup_gate_result.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print('\n'.join(lines))
