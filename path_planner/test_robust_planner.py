import json,math,tempfile,unittest,contextlib,io
from pathlib import Path
import robust_planner as r
import path_planner as g
class RobustTests(unittest.TestCase):
    def test_dynamic_limits(self):
        d=r.Dynamics();data=json.loads(Path('out_robust/path.json').read_text(encoding='utf-8'))
        pts=[g.PathPoint(p['x'],p['y'],math.radians(p['heading_deg']),p['s_m'],p['curvature_1pm'],p['segment']) for p in data['path']]
        prof=r.profile(pts,d);v=prof['speed']
        self.assertEqual(v[-1],0)
        self.assertLessEqual(max(v),d.vmax)
        self.assertLessEqual(prof['max_lateral_accel'],d.alat+1e-6)
        self.assertLessEqual(prof['max_steer_rate_deg_s'],d.steer_rate_deg_s+1e-5)
        self.assertLessEqual(prof['max_longitudinal_jerk'],d.lon_jerk+1e-5)
        self.assertLessEqual(prof['max_lateral_jerk'],d.lat_jerk+1e-5)
        self.assertLessEqual(max(prof['accel']),d.accel+1e-6)
        self.assertGreaterEqual(min(prof['accel']),-d.brake-1e-6)
    def test_bias_is_rejected(self):
        data=json.loads(Path('out_robust/path.json').read_text(encoding='utf-8'))
        pts=[g.PathPoint(p['x'],p['y'],0,p['s_m'],p['curvature_1pm'],p['segment']) for p in data['path']]
        d=r.Dynamics(residual_bias_dps=.05)
        with self.assertRaises(ValueError):r.drift_check(pts,[(o['x'],o['y']) for o in data['obstacles']],d,r.profile(pts,d))
    def test_invalid_dynamics(self):
        for x in [float('nan'),0,-1,float('inf')]:
            with self.assertRaises(ValueError):r.validate_config(r.Dynamics(vmax=x))
        for settings in [dict(pwm_start_percent=26,pwm_max_percent=25),
                         dict(pwm_max_percent=101),dict(servo_max_deg=21),
                         dict(servo_max_deg=float('nan')),
                         dict(speed_kp=-1),dict(speed_ki=float('nan')),
                         dict(speed_kd=float('inf'))]:
            with self.assertRaises(ValueError):r.validate_config(r.Dynamics(**settings))
    def run_plan(self,points,*extra):
        with tempfile.TemporaryDirectory() as out,contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            result=r.main(['--points',points,'--out',out,'--radii','1.2',*extra])
            if result==0:
                data=json.loads((Path(out)/'path.json').read_text(encoding='utf-8'))
                self.assertTrue(all(x==1 for x in data['summary']['gap_crossings']))
                self.assertGreaterEqual(data['summary']['minimum_radius_m'],1.2)
            return result
    def test_invalid_gap_and_side(self):
        self.assertNotEqual(self.run_plan('3,0 3.5,0'),0)
        self.assertNotEqual(self.run_plan('3,0 6,0','--sides','L,L'),0)
        self.assertNotEqual(self.run_plan('nan,0 6,0'),0)
    def test_unequal_spacing_layouts(self):
        for points in ['3,0 6,0 9.5,0','3,0 5.6,0 9,0 12,0','3,0 6.5,0 10,0 13.2,0']:
            self.assertEqual(self.run_plan(points),0)
    def test_tight_field_rejected(self):
        with self.assertRaises(ValueError):r.validate_config(r.Dynamics(bounds=[1,0,0,1]))
    def test_host_controller_regression(self):
        import simulate_tracking as sim
        result=sim.run()
        self.assertTrue(result['successful_safe_finish'])
        self.assertLess(result['peak_estimated_cross_track_m'],.05)
        self.assertFalse(sim.run(bias=.05)['successful_safe_finish'])
if __name__=='__main__':unittest.main()
