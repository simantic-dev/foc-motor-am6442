# Surface PMSM on a 24 V three-phase bridge, as the AM6442 R5F's FOC loop (fw/foc.c) sees it.
# Ported from sim-fixtures tests/stm/nucleo-g474re/foc-can-demo/foc_plant.py to the AM263x
# AM64x ePWM registers. Host type: `peripheral` scenario part with a periodic timer that reads
# ePWM0/1/2's TBPRD and CMPA and drives ADC0 AIN0/AIN1 with ctx.set_property.
#
# Model (rotor dq frame, amplitude-invariant Park):
#   phase-to-neutral volts from the duties CMPA / TBPRD (up-down, high side on below CMPA);
#   L did/dt = vd - R id + we L iq,  L diq/dt = vq - R iq - we L id - we psi;
#   J dwm/dt = 1.5 p psi iq - B wm - TL;  we = p wm. Counter frozen: bridge off, currents 0.
# Sensing: 5 mOhm shunt, x10 amplifier, biased at mid-span of the 1.8 V ADC: 0.9 V + 0.05 V/A.
#
# MODELED SIMPLIFICATION: averaged switching (no ripple, no dead time); ideal sensors; the
# plant steps every PWM period (50 us) with 10 sub-steps.
# Parameters can be overridden with the PLANT environment variable, a JSON object such as
# '{"R": 0.3, "J": 2e-4}' (the tests run the same firmware against several motors).
import json
import math
import os

from pmsm import MOTOR

PARAMS = dict(R=0.2, L=1e-4, PSI=0.004, J=1e-4, B=1e-5, VBUS=24.0, LOAD_NM=0.05, LOAD_AFTER_S=0.2)
PARAMS.update(json.loads(os.environ.get("PLANT", "{}")))
R, L, PSI, J, B, VBUS = (PARAMS[k] for k in ("R", "L", "PSI", "J", "B", "VBUS"))
LOAD_NM, LOAD_AFTER_S = PARAMS["LOAD_NM"], PARAMS["LOAD_AFTER_S"]
LOAD_ARM_RPM = 2950  # the load comes on LOAD_AFTER_S after the shaft first reaches this
P = 7
V_PER_A, VBIAS = 0.05, 0.9
LOG_S = 0.002
EPWM = [0x23000000, 0x23010000, 0x23020000]
TBCTL, TBPRD, CMPA = 0x00, 0x0A, 0x12
STEP_US, SUB = 50, 10


class Peripheral:
    def __init__(self, ctx):
        self.ctx = ctx
        self.t = 0.0
        self.next_log = 0.0
        self.load_at = None
        ctx.info(f"plant params {json.dumps(PARAMS, sort_keys=True)}")
        ctx.schedule_periodic(STEP_US)

    def on_access(self, req):
        if req.is_init:
            self.ctx.schedule_periodic(STEP_US)
        req.value = 0

    def on_timer(self):
        rd = lambda base, off: self.ctx.bus_read(base + off, 2)
        on = all((rd(b, TBCTL) & 3) != 3 for b in EPWM)
        duty = [rd(b, CMPA) / (rd(b, TBPRD) or 1) for b in EPWM]
        m = MOTOR
        dt = STEP_US * 1e-6 / SUB
        mean = sum(duty) / 3
        vph = [VBUS * (d - mean) for d in duty]
        if self.load_at is None and MOTOR.wm * 60 / (2 * math.pi) >= LOAD_ARM_RPM:
            self.load_at = self.t + LOAD_AFTER_S
        tl = LOAD_NM if self.load_at is not None and self.t >= self.load_at else 0.0
        for _ in range(SUB):
            th = P * m.theta_m
            we = P * m.wm
            if on:
                c = [math.cos(th - k * 2 * math.pi / 3) for k in range(3)]
                s = [math.sin(th - k * 2 * math.pi / 3) for k in range(3)]
                vd = 2 / 3 * sum(v * ck for v, ck in zip(vph, c))
                vq = -2 / 3 * sum(v * sk for v, sk in zip(vph, s))
                m.id += (vd - R * m.id + we * L * m.iq) / L * dt
                m.iq += (vq - R * m.iq - we * L * m.id - we * PSI) / L * dt
            else:
                m.id = m.iq = 0.0
            te = 1.5 * P * PSI * m.iq
            load = tl if m.wm > 0 else 0.0
            m.wm += (te - B * m.wm - load) / J * dt
            m.theta_m += m.wm * dt
        self.t += STEP_US * 1e-6
        th = P * m.theta_m
        ia = m.id * math.cos(th) - m.iq * math.sin(th)
        ib = m.id * math.cos(th - 2 * math.pi / 3) - m.iq * math.sin(th - 2 * math.pi / 3)
        self.ctx.set_property("adc0", "ch0", VBIAS + V_PER_A * ia)
        self.ctx.set_property("adc0", "ch1", VBIAS + V_PER_A * ib)
        if self.t >= self.next_log:
            self.next_log += LOG_S
            self.ctx.info(f"plant t={self.t * 1000:.0f} ms shaft={m.wm * 60 / (2 * math.pi):.0f} rpm "
                          f"id={m.id:.2f} A iq={m.iq:.2f} A load={tl:.3f} Nm")
