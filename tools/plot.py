"""Runs the drive in sim and plots speed and current against time: docs/drive.png.
   SIM=<path> uvx --with matplotlib --with pytest python tools/plot.py [PLANT-JSON]"""
import json, pathlib, sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tests"))
from conftest import STEP_1000_MS, STEP_3000_MS, simulate  # noqa: E402

params = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
run = simulate(params)
t = [p["t"] * 1000 for p in run.plant]
ref = [0 if x < STEP_1000_MS else 1000 if x < STEP_3000_MS else 3000 for x in t]
load_ms = next(p["t"] for p in run.plant if p["load"] > 0) * 1000

fig, (a, b) = plt.subplots(2, 1, sharex=True, figsize=(10, 6), height_ratios=[3, 2])
a.plot(t, ref, "k--", lw=1, label="setpoint")
a.plot(t, [p["rpm"] for p in run.plant], lw=1.6, label="shaft (plant)")
a.plot([r["ms"] for r in run.rows], [r["rpm"] for r in run.rows], "o", ms=3, label="firmware, from eQEP")
a.set_ylabel("speed (rpm)")
a.legend(loc="lower right")
b.plot(t, [p["iq"] for p in run.plant], lw=1.2, label="iq (torque)")
b.plot(t, [p["id"] for p in run.plant], lw=1.2, label="id (flux)")
b.axhline(5.0, color="grey", lw=0.8, ls=":")
b.set_ylabel("current (A)")
b.set_xlabel("time since closed loop (ms)")
b.legend(loc="upper right")
b.set_xlim(-380, 1100)
for ax in (a, b):
    ax.axvline(load_ms, color="red", lw=0.8)
    ax.grid(alpha=0.3)
a.text(load_ms + 5, 400, "0.05 Nm load", color="red")
isr = run.field(r"isr worst (\d+) cycles = (\d+) ns")
fig.suptitle(f"FOC on TI AM6442 R5F in Pyrite, 20 kHz current loop, worst ISR {isr[1]} cycles "
             f"({isr[2]} ns){'  ' + json.dumps(params) if params else ''}")
fig.tight_layout()
out = ROOT / "docs/drive.png"
fig.savefig(out, dpi=120)
print(out)
