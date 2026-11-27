"""gb_lowv topic-only battery source: thresholds unchanged, stale -> alert not trigger, no serial access."""
import os, sys, types, importlib, re
D = sys.argv[1] if len(sys.argv) > 1 else "/home/pi/grokbot"
P = F = 0
def check(n, ok):
    global P, F; P += bool(ok); F += (not ok); print(("PASS " if ok else "FAIL ") + n)

def load(readings):
    for k in [k for k in os.environ if k.startswith("LOWV_")]: os.environ.pop(k)
    rec = {"serial": 0, "stop": 0, "say": [], "shutdown": [], "events": []}
    feed = list(readings)
    gr = types.ModuleType("gb_robot"); gr.BATTERY_JSON = "/tmp/lowv_test_battery.json"
    gr.battery_topic_mV = lambda timeout=1.5: feed.pop(0) if feed else None
    def ser(*a, **k): rec["serial"] += 1; return 8370
    gr.battery_serial_mV = ser; gr.battery_pct = lambda v: 50
    gr.stop = lambda: rec.__setitem__("stop", rec["stop"] + 1)
    gs = types.ModuleType("gb_store"); gs.DATA = "/tmp"; gs.atomic_write = lambda p, d: None
    class S:
        def event(self, k, d): rec["events"].append(k.replace("lowv_", "", 1))
        def checkpoint(self): pass
    gs.Store = S
    rq = types.ModuleType("requests")
    class R: ok = True
    rq.post = lambda *a, **k: R()
    sys.modules.update(gb_robot=gr, gb_store=gs, requests=rq)
    sys.path.insert(0, D); sys.modules.pop("gb_lowv", None)
    m = importlib.import_module("gb_lowv")
    m.LOG = "/tmp/lowv_test.log"; m.say = lambda t: rec["say"].append(t)
    m.subprocess = types.SimpleNamespace(run=lambda *a, **k: rec["shutdown"].append(a))
    clock = {"t": 1000.0}; n = {"i": 0}
    def sleep(x):
        clock["t"] += 5.0; n["i"] += 1
        if n["i"] >= len(readings) + 1: raise KeyboardInterrupt
    m.time = types.SimpleNamespace(time=lambda: clock["t"], sleep=sleep, strftime=__import__("time").strftime)
    try: m.main()
    except KeyboardInterrupt: pass
    return m, rec

m, r = load([8370] * 12)
check("thresholds unchanged: warn 6.8 hard 6.5 margin 0.2 delay 60 poll 5 avg 30 n 3", (m.WARN, m.HARD, m.MARGIN, m.DELAY, m.POLL, m.AVG_S, m.NEED) == (6.8, 6.5, 0.2, 60.0, 5.0, 30.0, 3))
check("normal 8.37 V: no warn/shutdown/stop", not r["shutdown"] and r["stop"] == 0 and "warn" not in r["events"])
m, r = load([6700] * 30)
check("low 6.70 V: warn event + motors stopped + spoken warning (as before)", "warn" in r["events"] and r["stop"] >= 1 and any("shutting down in one minute" in x for x in r["say"]))
check("low 6.70 V: countdown expires -> shutdown command issued", "shutdown" in r["events"] and r["shutdown"])
m, r = load([6400] * 6)
check("critical 6.40 V: hard shutdown path (as before)", any("critically low" in x for x in r["say"]) and r["shutdown"])
m, r = load([8370] * 3 + [None] * 10)
check("stale topic: 'stale' alert event logged", "stale" in r["events"])
check("stale topic: no warn / shutdown / motor stop", "warn" not in r["events"] and not r["shutdown"] and r["stop"] == 0)
m, r = load([None] * 12)
check("missing from boot: stale alert, no false trigger", "stale" in r["events"] and not r["shutdown"] and "warn" not in r["events"])
m, r = load([8370] * 2 + [None] * 4 + [8360] * 2)
check("recovery: 'fresh' event after stale", r["events"].count("stale") == 1 and "fresh" in r["events"])
check("serial reader never called in any scenario", r["serial"] == 0)
for f in ("gb_lowv.py",):
    src = "\n".join(l.split("#")[0] for l in open(os.path.join(D, f)).read().splitlines())
    check("%s: no battery_serial_mV / serial / ttyACM in code" % f, not re.search(r"battery_serial_mV\(|import serial|ttyACM", src))
print("RESULT: %d passed, %d failed" % (P, F))
