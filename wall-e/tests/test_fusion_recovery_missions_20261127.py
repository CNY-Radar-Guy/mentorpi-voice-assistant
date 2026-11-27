#!/usr/bin/env python3
"""2026-11-27 fusion stuck-recovery: END-TO-END mission tests. Runs the real gb_explore.node_main against fake ROS, a
simulated room and the REAL gb_gate.guard()/_fusion_nudge(). ZERO hardware: nothing is published to any robot.
Run: python3 tests/test_fusion_recovery_missions_20261127.py   (~2-3 min)"""
import os, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT); sys.path.insert(0, HERE)
import fr_fakeros as S

P = F = 0
def check(name, cond, extra=""):
    global P, F
    if cond: P += 1; print("PASS", name)
    else: F += 1; print("FAIL", name, extra)

CHAIR = [(0.22, 0.15, 0.012), (0.22, -0.15, 0.012), (-0.20, 0.15, 0.012), (-0.20, -0.15, 0.012)]

def fus(res, label=None):
    return [f for f in (res.get("fusion") or []) if label is None or f.get("label") == label]

def summary(name, res, sim, w, tok):
    print("  [%s] outcome=%s reason=%s fusion=%s pose=(%.2f,%.2f) min_gap=%.3f allowed=%d trips=%d tok_left=%s" % (
        name, res.get("outcome"), res.get("reason"), [(f["label"], f["event"]) for f in fus(res)], w.x, w.y,
        w.min_gap, sim.allowed, sim.trips, tok))

# A. explore/wander: wedged between chair legs -> squeeze out, resume wander
w = S.World(circles=CHAIR)
res, sim, lines, tok = S.run_mission(ROOT, {"mission": "wander", "duration_s": 20.0, "return": False}, w, timeout=60)
summary("wander chair trap", res, sim, w, tok)
check("A explore: fusion escape recovered", any(f["event"] == "recovered" for f in fus(res, "explore")), res.get("fusion"))
check("A explore: left the chair (> 0.25 m)", w.x > 0.25 or abs(w.y) > 0.3, (w.x, w.y))
check("A explore: mission resumed after escape (not aborted by fusion)", "fusion" not in str(res.get("reason")), res.get("reason"))
check("A explore: never touched a leg", w.min_gap > 0.0, w.min_gap)
check("A explore: relaxed commands only 0.06-0.07 m/s, |ang|<=0.3", sim.fusion_out and all(
    (v == 0 or 0.06 <= v <= 0.07) and abs(a) <= 0.3 + 1e-9 for v, a in sim.fusion_out), sim.fusion_out[:5])
check("A explore: token gone at mission end (clearances restored)", not tok)
check("A explore: restore logged", any("normal clearances 0.27/0.28/0.24 restored" in l for l in lines))

# B. go-to / destination: drive 1.0 m through a chair, then return home through it again
w = S.World(circles=[(0.45, 0.16, 0.012), (0.45, -0.16, 0.012)])   # 0.296 m inner gap: normal 0.15 corridor blocks
plan = {"mission": "distance", "distance_m": 1.0, "requested_m": 1.0, "requested_text": "1 meters", "clamped": False,
        "return": True, "park": True}
res, sim, lines, tok = S.run_mission(ROOT, plan, w, timeout=150)
summary("goto through chair + return", res, sim, w, tok)
check("B goto: out-leg fusion recovered", any(f["event"] == "recovered" for f in fus(res, "out")), res.get("fusion"))
check("B goto: return-home fusion recovered", any(f["event"] == "recovered" for f in fus(res, "return")), res.get("fusion"))
check("B goto: mission completed", res.get("outcome") == "done", (res.get("outcome"), res.get("reason")))
check("B goto: back near start", abs(w.x) < 0.25 and abs(w.y) < 0.25, (w.x, w.y))
check("B goto: never touched a leg", w.min_gap > 0.0, w.min_gap)
check("B goto: relaxed commands bounded", all((v == 0 or 0.06 <= v <= 0.07) and abs(a) <= 0.3 + 1e-9 for v, a in sim.fusion_out))
check("B goto: token gone at mission end", not tok)

# C. solid walls all around (real obstacle) -> never relaxed
box = [(0.22, -0.6, 0.22, 0.6), (-0.21, -0.6, -0.21, 0.6), (-0.21, 0.19, 0.22, 0.19), (-0.21, -0.19, 0.22, -0.19)]
w = S.World(segs=box)
res, sim, lines, tok = S.run_mission(ROOT, {"mission": "wander", "duration_s": 15.0, "return": False}, w, timeout=40)
summary("walled box", res, sim, w, tok)
check("C wall: gate never relaxed a stop", sim.allowed == 0, sim.allowed)
check("C wall: no fusion recovery claimed", not any(f["event"] == "recovered" for f in fus(res)), res.get("fusion"))
check("C wall: never touched the wall", w.min_gap > 0.0, w.min_gap)
check("C wall: robot did not drive out through the wall", abs(w.x) < 0.1 and abs(w.y) < 0.1, (w.x, w.y))

# D. chair trap but depth camera OFFLINE -> LiDAR-only (fail safe), never relaxed
w = S.World(circles=CHAIR)
res, sim, lines, tok = S.run_mission(ROOT, {"mission": "wander", "duration_s": 15.0, "return": False}, w, cam_mode="offline", timeout=40)
summary("chair trap camera offline", res, sim, w, tok)
check("D offline: gate never relaxed a stop", sim.allowed == 0, sim.allowed)
check("D offline: no fusion recovery", not any(f["event"] == "recovered" for f in fus(res)), res.get("fusion"))
check("D offline: stayed inside the chair (LiDAR-only rules)", abs(w.x) < 0.1, (w.x, w.y))
check("D offline: never touched a leg", w.min_gap > 0.0, w.min_gap)

# E. chair trap with STALE camera, go-to mission -> never relaxed
w = S.World(circles=CHAIR)
res, sim, lines, tok = S.run_mission(ROOT, plan, w, cam_mode="stale", timeout=40)
summary("goto chair trap camera stale", res, sim, w, tok)
check("E stale: gate never relaxed a stop", sim.allowed == 0, sim.allowed)
check("E stale: no fusion recovery", not any(f["event"] == "recovered" for f in fus(res)), res.get("fusion"))
check("E stale: never touched a leg", w.min_gap > 0.0, w.min_gap)

print("\n%d passed, %d failed" % (P, F))
sys.exit(1 if F else 0)
