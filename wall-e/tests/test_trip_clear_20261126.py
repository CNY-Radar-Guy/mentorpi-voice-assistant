#!/usr/bin/env python3
"""Unit tests for 2026-11-26 guard/bump trip auto-clear (no ROS, no motion)."""
from __future__ import annotations
import ast, json, math, os, sys, tempfile, time, pathlib

GATE = pathlib.Path("/home/pi/grokbot/releases/e48d3d8/gb_safety/gb_gate.py")
OBS = pathlib.Path("/home/pi/grokbot/releases/e48d3d8/gb_safety/gb_obstacle.py")
if not OBS.exists():
    OBS = pathlib.Path("/home/pi/docker/tmp/gb_code/e48d3d8/gb_obstacle.py")

fails = []
def check(name, ok, detail=None):
    if ok:
        print("  PASS", name)
    else:
        print("  FAIL", name, "" if detail is None else detail)
        fails.append(name)

src = GATE.read_text()
tree = ast.parse(src)
check("parses", True)
check("manual cap 0.288 preserved", 'GATE_MANUAL_MAX_LIN", "0.288"' in src)
check("auto cap 1.0 preserved", 'GATE_AUTO_MAX_LIN", "1.0"' in src)
check("0.27 stop threshold still present", "0.27" in src)
check("clear_trip defined", "def clear_trip(" in src)
check("_trip_clear_reason defined", "def _trip_clear_reason(" in src)
check("DISARM_EVENT defined", "DISARM_EVENT" in src)
check("startup-stale clear present", "startup-stale" in src)
check("idle-timeout clear present", "idle-timeout" in src)
check("disarm clear present", 'return "disarm"' in src)

wanted = {"clear_trip", "_trip_clear_reason", "commanded_motion"}
funcs = {}
for n in tree.body:
    if isinstance(n, ast.FunctionDef) and n.name == "commanded_motion":
        funcs[n.name] = n
    if isinstance(n, ast.ClassDef) and n.name == "Gate":
        for c in n.body:
            if isinstance(c, ast.FunctionDef) and c.name in wanted:
                funcs[c.name] = c
check("extracted methods", set(funcs) == wanted, set(funcs))

def undent(node):
    seg = ast.get_source_segment(src, node)
    if node.name == "commanded_motion":
        return seg
    # Strip the class-body indent (4 spaces) so methods become top-level defs.
    lines = seg.splitlines()
    return "\n".join(ln[4:] if ln.startswith("    ") else ln for ln in lines)

ns = {"os": os, "time": time, "json": json, "math": math,
      "EVENT": None, "DISARM_EVENT": None, "ZERO_HOLD": 2.0}
pre = "MOTION_EPS_LIN = 1e-4\nMOTION_EPS_ANG = 1e-4\n"
code = pre + "\n" + undent(funcs["commanded_motion"]) + "\n" + undent(funcs["clear_trip"]) + "\n" + undent(funcs["_trip_clear_reason"]) + "\n"
exec(compile(code, "<extract>", "exec"), ns, ns)

class FZ:
    def __init__(self):
        self.n = 0
    def reset_detectors(self):
        self.n += 1

class Fake:
    def __init__(self):
        self.last_trip = "bump detected (encoders): track stall: commanded 0.04 m/s, encoders 0.00 m/s"
        self.trip_t = time.time()
        self.trip_clear_after = self.trip_t + 2.0
        self.zero_until = self.trip_t + 2.0
        self.mode = "idle"
        self.lin = 0.0
        self.ang = 0.0
        self.fz = FZ()
        self.sens_t = 1
        self.logs = []
    def log(self, m):
        self.logs.append(m)
    def status(self):
        self.sens_t = 0.0

Fake.clear_trip = ns["clear_trip"]
Fake._trip_clear_reason = ns["_trip_clear_reason"]

with tempfile.TemporaryDirectory() as td:
    ev = os.path.join(td, "guard_event.json")
    dis = os.path.join(td, "DISARM_EVENT")
    ns["EVENT"] = ev
    ns["DISARM_EVENT"] = dis
    with open(ev, "w") as f:
        json.dump({"t": time.time(), "why": "bump detected (encoders): track stall", "src": "stream"}, f)

    g = Fake()
    r = g._trip_clear_reason(g.trip_t + 0.1)
    check("does not clear during ZERO_HOLD without disarm", r is None, r)

    r = g._trip_clear_reason(g.trip_t + 2.01)
    check("idle-timeout after ZERO_HOLD", r == "idle-timeout", r)

    g.lin = 0.08
    g.mode = "stream"
    r = g._trip_clear_reason(g.trip_t + 5.0)
    check("no idle clear while still commanding motion", r is None, r)
    g.lin = 0.0
    g.mode = "idle"

    older = g.trip_t - 10
    with open(dis, "w") as f:
        f.write("{}")
    os.utime(dis, (older, older))
    r = g._trip_clear_reason(g.trip_t + 0.1)
    check("old DISARM_EVENT does not clear a newer trip", r is None, r)

    newer = g.trip_t + 0.2
    os.utime(dis, (newer, newer))
    r = g._trip_clear_reason(g.trip_t + 0.3)
    check("disarm after trip clears immediately", r == "disarm", r)

    ok = g.clear_trip("disarm")
    check("clear_trip returns True", ok is True)
    check("last_trip cleared", g.last_trip == "")
    check("event file archived (no longer active)", not os.path.exists(ev))
    archived = [n for n in os.listdir(td) if n.startswith("guard_event.json.cleared-")]
    check("archive created", len(archived) == 1, archived)
    check("detectors reset", g.fz.n == 1, g.fz.n)
    check("log mentions cleared", any("GUARD TRIP cleared (disarm)" in x for x in g.logs), g.logs)

    g2 = Fake()
    g2.last_trip = ""
    ok2 = g2.clear_trip("idle-timeout")
    check("clear_trip is no-op when already clear", ok2 is False)
    r = g2._trip_clear_reason(time.time() + 10)
    check("no reason when no last_trip and no file", r is None, r)

if OBS.exists():
    ot = OBS.read_text()
    check("STALL_MIN_CMD raised to 0.05 (> V_MIN 0.04)", "STALL_MIN_CMD = 0.05" in ot)
    check("V_MIN still 0.04", "V_MIN" in ot and "0.04" in ot)
    check("0.27 stop distance still present in obstacle", "0.27" in ot)
else:
    check("obstacle file present", False)

if fails:
    print("FAILED", len(fails), ":", ", ".join(fails))
    sys.exit(1)
print("ALL_PASS")
