"""Generate/check the offline plan and synchronize the selected Keil firmware.

Keil invokes this before compilation. The MCU runs C and does not read JSON.
Only generated headers are updated here; hand-written firmware is never copied
over during an ordinary build.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

BASE = Path(__file__).resolve().parent
DEFAULT_FIRMWARE = BASE.parent / 'firmware/demo/DEMO_new_path1'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_changed(path, data):
    content = data.encode('utf-8') if isinstance(data, str) else data
    if not path.exists() or path.read_bytes() != content:
        path.write_bytes(content)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware', type=Path, default=DEFAULT_FIRMWARE)
    parser.add_argument('--force-plan', action='store_true')
    args = parser.parse_args(argv)
    root = args.firmware.resolve()
    inc = root / 'user/inc'
    if not inc.is_dir() or not (root / 'mdk/rt1064.uvprojx').is_file():
        parser.error('firmware must contain user/inc and mdk/rt1064.uvprojx')
    try:
        import robust_planner as planner
        config_path = BASE / 'vehicle_config.json'
        config = json.loads(config_path.read_text(encoding='utf-8-sig'))
        dynamics = planner.Dynamics(**config)
        planner.validate_config(dynamics)
        inputs = {name: digest(BASE / name) for name in (
            'barrels_input.csv', 'vehicle_config.json', 'path_planner.py', 'robust_planner.py')}
        out = BASE / 'out_robust'
        stamp = out / 'firmware_plan_manifest.json'
        previous = json.loads(stamp.read_text(encoding='utf-8')) if stamp.exists() else {}
        products = ('robust_trajectory.h', 'path.json', 'trajectory.csv')
        cached = previous.get('inputs') == inputs and all(
            (out / name).is_file() and digest(out / name) == previous.get('outputs', {}).get(name)
            for name in products)
        if args.force_plan or not cached:
            result = subprocess.run([
                sys.executable, '-B', str(BASE / 'path_planner.py'),
                '--input', str(BASE / 'barrels_input.csv'),
                '--dynamics', str(config_path), '--out', str(out)], cwd=BASE)
            if result.returncode:
                raise ValueError('planning failed; firmware build is stopped to prevent a stale route')
            write_changed(stamp, json.dumps({
                'inputs': inputs, 'outputs': {name: digest(out / name) for name in products}
            }, indent=2) + '\n')
        payload = json.loads((out / 'path.json').read_text(encoding='utf-8'))
        summary = payload['summary']
        if payload['dynamics'] != config:
            raise ValueError('generated trajectory/config mismatch; regenerate the trajectory')
        # The existing firmware has a 120-second watchdog. Refuse a plan whose
        # nominal duration already consumes its commissioning margin.
        if summary['predicted_total_time_s'] > 110:
            raise ValueError('planned duration exceeds 110 s; review the 120 s run watchdog')
        values = [dynamics.pwm_start_percent, dynamics.pwm_max_percent, dynamics.servo_max_deg]
        if any(abs(value * 1000 - round(value * 1000)) > 1e-6 for value in values):
            raise ValueError('PWM/servo limits support at most three decimal places')
        start, maximum, angle = (round(value * 1000) for value in values)
        header = '\n'.join([
            '/* Generated from path_planner/vehicle_config.json. Do not edit. */',
            '#ifndef VEHICLE_PARAMS_H', '#define VEHICLE_PARAMS_H',
            f'#define VEHICLE_PWM_START_MILLI_PERCENT ({start})',
            f'#define VEHICLE_PWM_MAX_MILLI_PERCENT ({maximum})',
            f'#define VEHICLE_SERVO_MAX_MILLIDEG ({angle})',
            f'#define VEHICLE_SPEED_KP ({dynamics.speed_kp}f)',
            f'#define VEHICLE_SPEED_KI ({dynamics.speed_ki}f)',
            f'#define VEHICLE_SPEED_KD ({dynamics.speed_kd}f)',
            '#define VEHICLE_PWM_START_PERCENT (VEHICLE_PWM_START_MILLI_PERCENT / 1000.0f)',
            '#define VEHICLE_PWM_MAX_PERCENT (VEHICLE_PWM_MAX_MILLI_PERCENT / 1000.0f)',
            '#define VEHICLE_SERVO_MAX_DEG (VEHICLE_SERVO_MAX_MILLIDEG / 1000.0f)',
            '#if VEHICLE_PWM_START_MILLI_PERCENT <= 0 || VEHICLE_PWM_START_MILLI_PERCENT > VEHICLE_PWM_MAX_MILLI_PERCENT || VEHICLE_PWM_MAX_MILLI_PERCENT > 100000',
            '#error "Invalid configured PWM range"', '#endif',
            '#if VEHICLE_SERVO_MAX_MILLIDEG <= 0 || VEHICLE_SERVO_MAX_MILLIDEG > 20000',
            '#error "Configured servo limit exceeds calibrated travel"', '#endif', '#endif', ''
        ])
        write_changed(inc / 'vehicle_params.h', header)
        write_changed(inc / 'robust_trajectory.h', (out / 'robust_trajectory.h').read_bytes())
        report = {
            'config_source': str(config_path), 'inputs': inputs,
            'pwm_start_percent': dynamics.pwm_start_percent,
            'pwm_max_percent': dynamics.pwm_max_percent,
            'servo_max_deg': dynamics.servo_max_deg,
            'speed_pid': {'kp': dynamics.speed_kp, 'ki': dynamics.speed_ki, 'kd': dynamics.speed_kd},
            'wheel_angle_deg_assumed_at_full_turn': dynamics.wheel_angle_deg,
            'trajectory_summary': summary,
            'generated_headers': {name: digest(inc / name) for name in ('vehicle_params.h', 'robust_trajectory.h')},
        }
        write_changed(root / 'firmware_manifest.json', json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(f'Firmware ready: {summary["path_points"]} points, {summary["length_m"]:.3f} m; '
              f'launch {dynamics.pwm_start_percent:g}%, max {dynamics.pwm_max_percent:g}%, '
              f'servo +/-{dynamics.servo_max_deg:g} deg')
        return 0
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print('Firmware preparation failed: ' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
