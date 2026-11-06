#!/usr/bin/env python3
"""test_tilt.py - 25 deg pitch/roll hard cutoff (latch, no back-off), 15-25 deg stop (no back-off), noise, stale."""
import math, os, sys, random
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); sys.path.insert(0, ROOT)
import gb_obstacle as OB
OK = BAD = 0
def check(n, c, info=""):
    global OK, BAD
    if c: OK += 1
    else: BAD += 1; print("FAIL:", n, info)
G = 9.81
def acc(pitch, roll):
    """IMU accel for given robot pitch/roll (per-axis tan angles, matching the estimator)."""
    gy = math.tan(math.radians(pitch)); gx = -math.tan(math.radians(roll)); k = G / math.sqrt(1 + gx * gx + gy * gy)
    return gx * k, gy * k, k
def run(pitch, roll, secs=1.0, moving=True, noise=0.0, seed=1):
    rnd = random.Random(seed); im = OB.Imu(); kinds = []
    for i in range(int(48 * 0.5)): im.update(i / 48, -0.37, -0.39, 10.08, moving)          # level start (3 deg rest bias)
    for i in range(int(48 * secs)):
        ax, ay, az = acc(pitch, roll)
        ev = im.update(0.5 + i / 48, ax + rnd.gauss(0, noise), ay + rnd.gauss(0, noise), az + rnd.gauss(0, noise), moving)
        if ev: kinds.append(im.kind)
    return kinds, im
def final(k): return k[-1] if k else None
for ax_name, mk in (("pitch", lambda d: (d, 0)), ("roll", lambda d: (0, d))):
    for sgn in (1, -1):
        for deg, want in ((10, None), (14, None), (16, "tilt"), (18, "tilt"), (21, "tilt"), (24, "tilt"), (26, "tilt_hard"), (30, "tilt_hard"), (45, "tilt_hard")):
            k, im = run(*mk(sgn * deg))
            check("%s %+d deg -> %s" % (ax_name, sgn * deg, want), final(k) == want, (final(k), im.last))
            if want == "tilt": check("%s %+d never latches" % (ax_name, sgn * deg), "tilt_hard" not in k)
for p, r, want in ((20, 20, "tilt_hard"), (15, 15, "tilt"), (10, 10, None), (-21, 21, "tilt_hard"), (24, 5, "tilt"), (26, -26, "tilt_hard"), (5, -26, "tilt_hard")):
    k, im = run(p, r); check("combined p%+d r%+d -> %s" % (p, r, want), final(k) == want, (final(k), im.last))
# timing: hard cutoff fires within 0.6 s of a step to 26 / 30 deg
for deg, tmax in ((26, 0.6), (30, 0.5)):
    im = OB.Imu(); t_ev = None
    for i in range(24): im.update(i / 48, -0.37, -0.39, 10.08, True)
    for i in range(96):
        t = 0.5 + i / 48; ax, ay, az = acc(deg, 0)
        if im.update(t, ax, ay, az, True) and im.kind == "tilt_hard" and t_ev is None: t_ev = t - 0.5
    check("latch latency %d deg < %.1fs" % (deg, tmax), t_ev is not None and t_ev < tmax, t_ev)
# noise / spikes / at rest
k, _ = run(3, 2, secs=5, noise=0.8); check("rest + 0.8 m/s2 noise: no event", not k, k)
k, _ = run(10, 0, secs=5, noise=1.5); check("10 deg + 1.5 m/s2 vibration: no latch", "tilt_hard" not in k, k)
im = OB.Imu(); evs = []
for i in range(48): im.update(i / 48, -0.37, -0.39, 10.08, True)
for i in range(3):
    ax, ay, az = acc(40, 0); evs.append(im.update(1 + i / 48, ax, ay, az, True) and im.kind)
for i in range(48): evs.append(im.update(1.1 + i / 48, -0.37, -0.39, 10.08, True) and im.kind)
check("3-sample 40 deg spike: no tilt event", not any(e in ("tilt", "tilt_hard") for e in evs), evs)
k, _ = run(26, 0, moving=False); check("hard tilt at rest still latches", final(k) == "tilt_hard")
im = OB.Imu(); kind = None
for i in range(48): im.update(i / 48, 0.0, 0.0, -9.8, False); kind = im.kind or kind
check("upside down -> latch", kind == "tilt_hard")
# fusion: event naming, priority, stale IMU
f = OB.Fusion(); f.cmd = (0.1, 0)
for i in range(24): f.on_imu(i / 48, -0.37, -0.39, 10.08)
for i in range(48):
    ax, ay, az = acc(0, 30); f.on_imu(0.5 + i / 48, ax, ay, az)
e = f.pop_event(0.5 + 47 / 48); check("fusion: hard tilt event sensor", e and e[1] == "imu-tilt-hard", e)
f = OB.Fusion(); f.cmd = (0.1, 0)
for i in range(24): f.on_imu(i / 48, -0.37, -0.39, 10.08)
for i in range(48):
    ax, ay, az = acc(-20, 0); f.on_imu(0.5 + i / 48, ax, ay, az)
e = f.pop_event(0.5 + 47 / 48); check("fusion: 20 deg -> imu-tilt", e and e[1] == "imu-tilt", e)
f = OB.Fusion(); f.events = [(10.0, "imu", "jolt"), (10.1, "imu-tilt-hard", "tip")]
check("hard tilt wins priority", f.pop_event(10.2)[1] == "imu-tilt-hard")
f = OB.Fusion(); f.cmd = (0.0, 0.0)
for i in range(48 * 60): f.on_imu(i / 48, 0.0, 0.0, -9.8)
check("events bounded while idle", len(f.events) <= 60, len(f.events))
f = OB.Fusion(); f.odom_t = 100.0; f.imu_t = 99.0
f.on_lidar(OB.LidarView(100.0)); f.lidar.blind_f = 0.0
f.on_depth(OB.DepthView(100.0)); f.depth.valid_frac = 0.5
l, a, why, kind = f.limit(0.12, 0, 100.0); check("stale IMU caps 0.06, no stop", abs(l - OB.V_DEGRADED) < 1e-9, (l, why))
# wiring
gate = open(os.path.join(ROOT, "gb_safety", "gb_gate.py")).read() if os.path.exists(os.path.join(ROOT, "gb_safety", "gb_gate.py")) else None
if gate is not None:
    check("gate hard tilt -> tilt_latch", 'sensor == "imu-tilt-hard"' in gate and "def tilt_latch" in gate and "ESTOP_REQ" in gate)
    check("gate tilt no back-off", 'sensor.startswith("imu")' in gate and "no back-off" in gate)
    run_sh = open(os.path.join(ROOT, "gb_safety", "gb_safety_run.sh")).read()
    check("run.sh mirrors ESTOP_REQ to host latch", 'ESTOP_REQ' in run_sh and '"$HOST_ESTOP"' in run_sh)
print("test_tilt: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
