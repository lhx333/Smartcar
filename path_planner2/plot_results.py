"""Standalone scientific plots; optional matplotlib dependency."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent
out=root/'out_robust';d=json.loads((out/'path.json').read_text(encoding='utf-8'));p=d['path']
fig,axes=plt.subplots(3,1,figsize=(12,8),constrained_layout=True)
axes[0].plot([x['x'] for x in p],[x['y'] for x in p],lw=1.5,label='robust path')
for b in d['obstacles']:
 axes[0].add_patch(plt.Circle((b['x'],b['y']),.35,color='red',alpha=.18))
 axes[0].plot(b['x'],b['y'],'ro',ms=3)
axes[0].axvline(0,color='gray',ls='--');axes[0].axis('equal');axes[0].set(xlabel='X (m)',ylabel='Y (m)',title='Path and nominal physical exclusion circles (vehicle + cone)')
s=[x['s_m'] for x in p]
axes[1].plot(s,[x['speed_mps'] for x in p],label='speed (m/s)')
axes[1].plot(s,[x['lookahead_m'] for x in p],label='lookahead (m)',ls='--');axes[1].legend();axes[1].set(xlabel='Arc length (m)',title='Offline speed ceiling and lookahead ceiling')
axes[2].plot(s,[x['curvature_1pm'] for x in p],label='curvature (1/m)')
axes[2].plot(s,[x['speed_mps']**2*x['curvature_1pm'] for x in p],label='lateral acceleration (m/s²)');axes[2].legend();axes[2].set(xlabel='Arc length (m)')
for ax in axes:ax.grid(alpha=.25)
fig.savefig(out/'speed_and_path.png',dpi=160)
fig.savefig(out/'speed_and_path.svg')
print(out/'speed_and_path.png')
