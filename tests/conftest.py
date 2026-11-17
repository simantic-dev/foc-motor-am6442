"""Runs the firmware on a simulated AM6442 (Pyrite's `sim`) against each motor in MOTORS, once
per motor, and hands the tests the console and the plant's 2 ms log of the true shaft."""
import json, os, pathlib, re, shutil, subprocess
import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent

# The firmware's gains are tuned for "nominal". The others are the same firmware on a
# different motor: a drive that only works on the motor it was tuned on fails here.
MOTORS = {
    "nominal": {},
    "hot_winding_R+50%": {"R": 0.3},
    "L+50%": {"L": 1.5e-4},
    "heavy_load_J_x2": {"J": 2e-4},
    "bus_sag_20V": {"VBUS": 20.0},
}
STEP_1000_MS, STEP_3000_MS = 50, 300  # fw/foc.c profile(), ms after alignment


class Run:
    def __init__(self, text, params):
        self.console, self.params = text, params
        self.t0 = float(re.search(r"\[(\d+\.\d+)s\] \(\w*uart0\) aligned", text)[1])  # closed loop starts
        self.rows = [dict(ms=int(m[1]), ref=int(m[2]), rpm=int(m[3]))
                     for m in re.finditer(r"t=\s*(\d+) ms ref=\s*(-?\d+) rpm speed=\s*(-?\d+) rpm", text)]
        self.plant = [dict(t=int(m[1]) / 1000 - self.t0, rpm=int(m[2]), id=float(m[3]), iq=float(m[4]),
                           load=float(m[5]))
                      for m in re.finditer(r"plant t=(\d+) ms shaft=(-?\d+) rpm id=(-?[\d.]+) A iq=(-?[\d.]+) A "
                                           r"load=([\d.]+)", text)]

    def field(self, pattern):
        m = re.search(pattern, self.console)
        assert m, f"console has no {pattern!r}:\n{self.console[-2000:]}"
        return m

    def window(self, a_ms, b_ms):
        """Plant samples between a_ms and b_ms after the closed loop started."""
        return [p for p in self.plant if a_ms / 1000 <= p["t"] < b_ms / 1000]


def simulate(params):
    sim = os.environ.get("SIM") or shutil.which("sim")
    assert sim, "no `sim` on PATH: install it (https://simantic.com/install) or set SIM=<path>"
    assert (ROOT / "fw/foc.release.out").exists(), "firmware not built: see README"
    p = subprocess.run([sim, "run", "--scenario", str(ROOT / "sim/motor.yaml"), "--ascii", "--timeout", "2.5"],
                       capture_output=True, text=True, timeout=300,
                       env={**os.environ, "PLANT": json.dumps(params)})
    assert p.returncode == 0, f"sim failed ({p.returncode}):\n{p.stdout}\n{p.stderr}"
    return Run((p.stdout + p.stderr).replace("\r", ""), params)


_cache = {}


@pytest.fixture(scope="session", params=list(MOTORS), ids=list(MOTORS))
def run(request):
    if request.param not in _cache:
        _cache[request.param] = simulate(MOTORS[request.param])
    return _cache[request.param]
