"""Regression tests of the FOC drive, run on a simulated AM6442 R5F against several motors.
Speeds and currents are the plant's (the truth), not the firmware's own view, unless named."""
import math
import re

import pytest

from conftest import STEP_1000_MS, STEP_3000_MS

MIN_DEAD_NS = 300    # your gate driver's minimum dead time
IQ_LIMIT_A = 5.0     # fw/foc.c IQ_MAX
LOAD_NM, KT = 0.05, 1.5 * 7 * 0.004


def detuned_by_inertia(request, run):
    """The speed gains are fixed for J = 1e-4 kg m^2; 2x inertia halves the speed-loop
    bandwidth, so the 1000 rpm step settles in ~280 ms instead of ~150 ms (4% overshoot,
    stable). Strict: gain scheduling or inertia estimation would make these pass."""
    if run.params.get("J", 1e-4) > 1.5e-4:
        request.applymarker(pytest.mark.xfail(strict=True, reason="speed gains tuned for nominal inertia"))


def load_start_ms(run):
    loaded = [p["t"] for p in run.plant if p["load"] > 0]
    top = max(p["rpm"] for p in run.plant)
    assert loaded, f"the load never came on: the plant applies it once the shaft reaches 2950 rpm, it peaked at {top} rpm"
    return loaded[0] * 1000


# --- bring-up ---------------------------------------------------------------------------------

def test_boots_and_reports_pass(run, request):
    detuned_by_inertia(request, run)
    assert "== foc-motor-am6442: FOC of a PMSM on the AM6442 R5F ==" in run.console
    assert "RESULT: PASS" in run.console


def test_current_sense_offsets_are_midscale(run):
    m = run.field(r"offsets  a=(\d+) b=(\d+)")
    assert abs(int(m[1]) - 2048) < 10 and abs(int(m[2]) - 2048) < 10


def test_encoder_alignment_is_field_oriented(run):
    """A wrong encoder zero shows up as d-axis current while the motor carries load: the
    current vector's angle off the q axis stays under 15 electrical degrees."""
    t = load_start_ms(run)
    s = run.window(t + 100, t + 250)
    id_, iq = sum(p["id"] for p in s) / len(s), sum(p["iq"] for p in s) / len(s)
    angle = math.degrees(math.atan2(abs(id_), iq))
    assert angle < 15, f"current vector {angle:.1f} deg off the q axis (id {id_:.2f} A, iq {iq:.2f} A)"


def test_bridge_has_dead_time(run):
    """Every phase's dead-band unit is on with at least MIN_DEAD_NS, read from the ePWM registers
    by the plant. MIN_DEAD_NS stands for the gate driver's datasheet minimum: set it to yours."""
    seen = re.findall(r"plant deadtime A=(\d+) B=(\d+) C=(\d+) ns", run.console)
    assert seen, "the plant never saw the bridge"
    assert all(int(v) >= MIN_DEAD_NS for v in seen[-1]), f"dead time per phase {seen[-1]} ns, need >= {MIN_DEAD_NS}"


# --- speed steps ------------------------------------------------------------------------------

def test_rise_time_0_to_1000_rpm(run):
    """The step uses the full current: 10-90% within 25% of the current-limited minimum."""
    s = run.window(STEP_1000_MS, STEP_3000_MS)
    t10 = next(p["t"] for p in s if p["rpm"] >= 100)
    t90 = next(p["t"] for p in s if p["rpm"] >= 900)
    j = run.params.get("J", 1e-4)
    best = j * (800 * 2 * math.pi / 60) / (KT * IQ_LIMIT_A)
    assert t90 - t10 < best * 1.25, f"10-90% rise took {(t90 - t10) * 1000:.1f} ms, physics allows {best * 1000:.1f} ms"


def test_overshoot_1000_rpm(run):
    peak = max(p["rpm"] for p in run.window(STEP_1000_MS, STEP_3000_MS))
    assert peak < 1000 * 1.10, f"overshoot to {peak} rpm"


def test_overshoot_3000_rpm(run):
    peak = max(p["rpm"] for p in run.window(STEP_3000_MS, load_start_ms(run)))
    assert peak < 3000 * 1.05, f"overshoot to {peak} rpm"


def test_settles_at_1000_rpm(run, request):
    detuned_by_inertia(request, run)
    s = run.window(STEP_1000_MS + 120, STEP_3000_MS)
    assert all(abs(p["rpm"] - 1000) <= 20 for p in s), [p["rpm"] for p in s]


def test_settles_at_3000_rpm(run):
    s = run.window(STEP_3000_MS + 250, load_start_ms(run))
    assert all(abs(p["rpm"] - 3000) <= 60 for p in s), [p["rpm"] for p in s]


# --- load step --------------------------------------------------------------------------------

def test_load_step_dip(run):
    t = load_start_ms(run)
    dip = min(p["rpm"] for p in run.window(t, t + 100))
    assert dip > 3000 * 0.95, f"load step pulled the shaft down to {dip} rpm"


def test_load_step_recovery(run):
    t = load_start_ms(run)
    s = run.window(t + 80, t + 250)
    assert s and all(abs(p["rpm"] - 3000) <= 60 for p in s), [p["rpm"] for p in s]


def test_load_torque_is_carried_by_iq(run):
    t = load_start_ms(run)
    iq = [p["iq"] for p in run.window(t + 150, t + 250)]
    want = LOAD_NM / KT
    assert iq and all(abs(v - want) < 0.25 for v in iq), f"iq {iq}, load needs {want:.2f} A"


# --- limits and timing ------------------------------------------------------------------------

def test_current_limit_is_respected(run):
    worst = max(abs(p["iq"]) for p in run.plant)
    assert worst <= IQ_LIMIT_A * 1.05, f"|iq| reached {worst} A"


def test_firmware_speed_matches_the_shaft(run):
    """The firmware's eQEP speed against the plant's true speed, in steady state."""
    fw = [r for r in run.rows if STEP_3000_MS + 250 <= r["ms"] < load_start_ms(run)]
    nearest = lambda ms: min(run.plant, key=lambda p: abs(p["t"] * 1000 - ms))["rpm"]
    pairs = [(r["rpm"], nearest(r["ms"])) for r in fw]
    assert pairs and all(abs(a - b) <= 30 for a, b in pairs), pairs


def test_isr_fits_the_pwm_period(run):
    m = run.field(r"isr worst (\d+) cycles = (\d+) ns at \d+ MHz \(budget (\d+) ns\)")
    assert int(m[2]) < int(m[3]) / 10, f"{m[1]} cycles ({m[2]} ns) is over 10% of the {m[3]} ns PWM period"
