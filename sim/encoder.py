# 500-line quadrature encoder (2000 counts per turn) on the plant's shaft (pmsm.MOTOR), into
# eQEP0 QEPA/QEPB. Host type: `gpio-device`, line 0 = A, line 1 = B. A periodic timer moves
# the emitted count one step toward the shaft's count, so edges are at least STEP_US apart
# (200 k counts/s = 6000 rpm). Forward rotation is A leading B.
import math

from pmsm import MOTOR

COUNTS = 2000
STEP_US = 5
STATES = [(0, 0), (1, 0), (1, 1), (0, 1)]


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.count = None
        ctx.schedule_periodic(STEP_US)

    def on_timer(self):
        target = math.floor(MOTOR.theta_m / (2 * math.pi) * COUNTS)
        if self.count is None:
            self.count = target
        elif target != self.count:
            self.count += 1 if target > self.count else -1
        else:
            return
        a, b = STATES[self.count % 4]
        self.ctx.gpio(0).set(bool(a))
        self.ctx.gpio(1).set(bool(b))

    def on_gpio(self, line, level):
        pass
