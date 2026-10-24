"""Apply the reviewed step31 patch only to an unchanged, explicitly named baseline."""
import argparse,hashlib,json,shutil,datetime
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--firmware',required=True,help='DEMO directory containing user/');p.add_argument('--trajectory',default='out_robust');args=p.parse_args()
    base=Path(__file__).resolve().parent;patch=base/'firmware_patch';root=Path(args.firmware).resolve()/'user'
    manifest=json.loads((patch/'baseline_sha256.json').read_text())
    for rel,expected in manifest.items():
        f=root/rel
        if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest()!=expected:
            raise SystemExit('Baseline differs; refusing blind overwrite: '+str(f))
    traj=Path(args.trajectory)
    if not traj.is_absolute():traj=base/traj
    summary=json.loads((traj/'path.json').read_text(encoding='utf-8'))['summary']
    if summary['predicted_total_time_s']>110:
        raise SystemExit('Insufficient margin for the existing 120 s firmware timeout; review timing first.')
    header=traj/'robust_trajectory.h'
    if not header.is_file():raise SystemExit('Generate trajectory first')
    backup=root.parent/('robust_backup_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    backup.mkdir(exist_ok=False)
    items={'src/navigation.c':patch/'navigation.c','src/state_machine.c':patch/'state_machine.c',
           'inc/navigation.h':patch/'navigation.h','inc/robust_control.h':patch/'robust_control.h',
           'inc/robust_speed_pi.h':patch/'robust_speed_pi.h','inc/robust_trajectory.h':header}
    for rel in items:
        old=root/rel
        if old.exists():
            dst=backup/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(old,dst)
    for rel,source in items.items():shutil.copy2(source,root/rel)
    print('Applied. Backup:',backup)
    print('Rebuild Keil project. Calibrate wheel steering mapping, speed response and gyro bias before driving.')
if __name__=='__main__':main()
