"""Exercise the E35 GUI's actual control method without opening a serial port."""
import ast
from pathlib import Path

host = Path(__file__).resolve().parents[4] / "E35_machine/main.py"
tree = ast.parse(host.read_text(encoding="utf-8"), filename=str(host))
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "E35App")
methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ("control", "_consume", "_config_timeout")]
scope = {}
exec(compile(ast.Module(body=methods, type_ignores=[]), str(host), "exec"), scope)


class Field:
    def __init__(self, value):
        self.value = value
        self.state = None

    def get(self):
        return self.value

    def set(self, value):
        self.value = value

    def configure(self, **kwargs):
        self.state = kwargs.get("state")


class App:
    control = scope["control"]
    _consume = scope["_consume"]
    _config_timeout = scope["_config_timeout"]

    def __init__(self):
        self.latest_state = "INIT"
        self.pwm_percent = Field("25")
        self.pwm_hold_s = Field("5")
        self.start_mode = Field("模块1：按PWM/时间")
        self.circle_yaw=Field("45")
        self.circle_servo=Field("8")
        self.circle_v1=Field("0.4")
        self.circle_v2=Field("0.6")
        self.command_status = Field("")
        self.start = Field("")
        self.sent = []
        self.pending_config = None
        self.config_generation = 0
        self.buf = ""
        self.root = type("Root", (), {"after": lambda *_: None})()

    def _send(self, data, label):
        self.sent.append((data, label))


app = App()
app.control("START")
assert not app.sent
app.latest_state = "ARMED"
for pwm, hold in ((10, 2), (25, 5), (40, 5)):
    app.pwm_percent.value, app.pwm_hold_s.value = str(pwm), str(hold)
    app.control("START")
    assert app.sent[-1][0] == f"CONFIG,{pwm},{hold}\r\n".encode()
    app._consume(f"CONFIG,OK,{pwm},{hold}\r\n")
    assert app.sent[-1][0] == b"START\r\n"
before = len(app.sent)
for pwm, hold in ((9, 2), (41, 5), (20, 1), (20, 6), (20.5, 3)):
    app.pwm_percent.value, app.pwm_hold_s.value = str(pwm), str(hold)
    app.control("START")
assert len(app.sent) == before
app.start_mode.value = "其他模块：普通START"
app.control("START")
assert app.sent[-1][0] == b"START\r\n"
app.start_mode.value = "模块4：按航向角/速度"
for angle,v1,v2 in ((45,"0.4","0.6"),(-90,"1.0","0.2")):
    app.circle_yaw.value=str(angle);app.circle_v1.value=v1;app.circle_v2.value=v2
    app.control("START")
    a=f"{angle:+04d}";s="08";b=f"{int(float(v1)*10):02d}";c=f"{int(float(v2)*10):02d}"
    assert app.sent[-1][0] == f"C4,{a},{s},{b},{c}\r\n".encode()
    app._consume(f"C4,OK,{a},{s},{b},{c}\r\n")
    assert app.sent[-1][0] == b"START\r\n"
print("PASS E35 module-1 command: ARMED gate, 10..40%, 2..5s, legacy START")
