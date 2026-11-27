"""2026-11-24 navclr: shared body-edge clearance config (nav_clearance.yaml) + hbatomic heartbeat writer."""
import os, sys, tempfile, threading, time, re
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gb_obstacle as OB, gb_explore as X
P = F = 0
def check(name, ok):
    global P, F
    P += bool(ok); F += (not ok); print(("PASS " if ok else "FAIL ") + name)
c = X.NAV_CLR
check("config file found and loaded (not defaults)", c["src"].endswith("nav_clearance.yaml"))
check("body-edge values 0.20 / 0.35 / 0.50", (c["front_m"], c["side_m"], c["open_dir_min_m"]) == (0.20, 0.35, 0.50))
check("centre conversion front 0.13+0.20=0.33", abs(c["front_c"] - (OB.BODY_FRONT + 0.20)) < 1e-9 and abs(c["front_c"] - 0.33) < 1e-9)
check("centre conversion side 0.10+0.35=0.45", abs(c["side_c"] - 0.45) < 1e-9)
check("centre conversion open 0.13+0.50=0.63", abs(c["open_c"] - 0.63) < 1e-9)
check("explore AVOID_DIST is at least nav_clearance front_c (learned overlay may raise it, never lower it)", X.AVOID_DIST >= c["front_c"])
check("avoidance turns before the gate: AVOID_DIST > gate STOP_FWD 0.27", X.AVOID_DIST > OB.STOP_FWD)
check("gate hard stops unchanged 0.27/0.28/0.24", (OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R) == (0.27, 0.28, 0.24))
gate = open("gb_safety/gb_gate.py").read()
check("gb_gate does not read nav_clearance", "nav_clearance" not in gate)
d = tempfile.mkdtemp(); p = os.path.join(d, "nc.yaml")
open(p, "w").write("reference: body_edge\nfront_m: 0.25\nside_m: 0.30\nopen_dir_min_m: 0.60\n")
c2 = X.load_nav_clearance((p,))
check("loader reads an alternate file", (c2["front_c"], c2["side_c"], c2["open_c"]) == (0.38, 0.4, 0.73))
open(p, "w").write("reference: centre\nfront_m: 0.25\n")
try: X.load_nav_clearance((p,)); ok = False
except ValueError: ok = True
check("non-body_edge reference is rejected", ok)
check("missing file -> documented defaults", X.load_nav_clearance(("/nonexistent",))["front_m"] == 0.20)
s = {"clear_deg": 60.0, "clear_m": 0.55, "left_mean": 0.3, "right_mean": 0.9}
check("open_dir ignores a clearest sector < 0.63 m (falls back to side means -> right)", X.open_dir(s) == -1)
s["clear_m"] = 1.15
check("open_dir uses a clearest sector >= 0.63 m (-> left)", X.open_dir(s) == 1)
src = open("gb_explore.py").read()
check("wander side zone uses side_m and V_SIDE cap", 'NAV_CLR["side_m"]' in src and "v = min(v, OB.V_SIDE)" in src)
rg = open("gb_safety/gb_rgbguard.py").read()
check("rgbguard reads the same shared file", "nav_clearance.yaml" in rg and "_nav_clearance()" in rg)
hb = os.path.join(d, "host_hb"); X._hb_write(hb); stop = [False]; bad = [0]; n = [0]
def w():
    while not stop[0]: X._hb_write(hb)
t = threading.Thread(target=w); t.start(); e = time.time() + 1.5
while time.time() < e:
    try: float(open(hb).read()); n[0] += 1
    except (OSError, ValueError): bad[0] += 1
stop[0] = True; t.join()
check("hbatomic: no torn/missing heartbeat reads (%d reads)" % n[0], bad[0] == 0 and n[0] > 1000)

# 2026-11-25 learnapply: whitelist clamp, floor at nav_clearance front_c, fallback on bad file
_ld = tempfile.mkdtemp(); _lf = os.path.join(_ld, "applied.json")
open(_lf, "w").write('{"params": {"explore_cruise_lin": 0.07, "explore_turn_ang": 0.48, "avoid_dist_m": 0.47, "slowdown_dist_m": 0.65, "not_a_key": 99}}\n')
info = X.apply_learned_params(paths=(_lf,))
check("learnapply clamps below-min cruise 0.07 up to whitelist min 0.08; keeps 0.47/0.65/0.48",
      abs(X.MAX_LIN-0.08)<1e-9 and abs(X.MAX_ANG-0.48)<1e-9 and abs(X.AVOID_DIST-0.47)<1e-9 and abs(X.SLOWDOWN_DIST-0.65)<1e-9)
check("learnapply ignores unknown keys", "not_a_key" not in X.effective_explore_params())
open(_lf, "w").write('{"params": {"explore_cruise_lin": 0.20, "avoid_dist_m": 0.10, "slowdown_dist_m": 0.20}}\n')
info = X.apply_learned_params(paths=(_lf,))
check("learnapply clamps cruise 0.20 via whitelist max 0.1012 (hard cap 1.0 no longer clamps default cruise); avoid >= front_c and whitelist min",
      abs(X._clamp_learned("explore_cruise_lin", 0.20)-0.1012)<1e-9 and abs(X.HARD_MAX_LIN-1.0)<1e-9 and abs(X.MAX_LIN-0.1012)<1e-9 and X.AVOID_DIST >= c["front_c"] and X.AVOID_DIST >= 0.40 and X.SLOWDOWN_DIST >= X.AVOID_DIST)
open(_lf, "w").write("{not json")
info = X.apply_learned_params(paths=(_lf,))
X.apply_learned_params(paths=())  # explicit empty path list = defaults
check("learnapply falls back to defaults on bad JSON (same as no file)",
      abs(info["avoid_dist_m"]-c["front_c"])<1e-9 and abs(info["slowdown_dist_m"]-0.60)<1e-9 and bool(info.get("error")))
# restore whatever is actually installed (approved file or defaults)
X.apply_learned_params()
check("after restore, AVOID_DIST still >= front_c and > gate stop", X.AVOID_DIST >= c["front_c"] and X.AVOID_DIST > OB.STOP_FWD)
check("gate hard stops still unchanged after learnapply tests 0.27/0.28/0.24", (OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R) == (0.27, 0.28, 0.24))

print("RESULT: %d passed, %d failed" % (P, F))
