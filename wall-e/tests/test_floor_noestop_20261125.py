"""2026-11-25 noestop (John, voice 14:02 ET): on the FLOOR (mode.json != bench) Wally never latches an emergency stop.
Faults / hard tilt / remote button / spoken 'emergency stop' / clap / leftover latch files -> NORMAL stop (zero, mission
ended, disarm, cause logged), no persistent latch, 'wake up Wally' always re-arms. Bench keeps the legacy latch.
Obstacle stops 0.27/0.28/0.24 unchanged. Pure Python, no ROS, all files in temp dirs (never the live latch paths)."""
import ast, json, math, os, re, shutil, subprocess, sys, tempfile, time, types
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
TMP = tempfile.mkdtemp(prefix="noestop_")
SH = os.path.join(TMP, "shared"); DATA = os.path.join(TMP, "data"); os.makedirs(SH); os.makedirs(os.path.join(DATA, "learn"))
MODE = os.path.join(DATA, "learn", "mode.json")
os.environ.update(GB_MODE_FILE=MODE, GB_SHARED_DIR=SH, GB_ARM_DIR=SH, GB_DATA=DATA, GB_DIR=TMP,
                  GB_SHARED_ESTOP=os.path.join(SH, "ESTOP"))
sys.path.insert(0, ROOT)
RES = []
def check(name, cond, info=""):
    RES.append((name, bool(cond))); print(("PASS " if cond else "FAIL ") + name + ("  | " + str(info) if info and not cond else ""))
def set_mode(m):
    json.dump({"mode": m}, open(MODE, "w")); gb_floor._CACHE.update(t=0.0, mode=None)
def wipe():
    for d in (SH, DATA):
        for f in ("ESTOP", "MAINT", "ESTOP_REQ", "ARMED", "FLOOR_STOP"):
            p = os.path.join(d, f)
            if os.path.lexists(p): os.remove(p)

import gb_floor, gb_obstacle as OB
# ---------------------------------------------------------------- mode detection
set_mode("ground"); check("mode ground -> no_latch", gb_floor.mode() == "ground" and gb_floor.no_latch())
set_mode("bench"); check("mode bench -> legacy latching", gb_floor.mode() == "bench" and not gb_floor.no_latch())
open(MODE, "w").write("{half"); gb_floor._CACHE.update(t=0.0, mode=None)
check("unreadable mode.json -> treated as floor (no latch)", gb_floor.no_latch())
os.remove(MODE); gb_floor._CACHE.update(t=0.0, mode=None)
json.dump({"mode": "bench"}, open(gb_floor.mirror_path(), "w"))
check("container: fresh FLOOR_MODE.json mirror bench -> latching", gb_floor.mode() == "bench")
os.utime(gb_floor.mirror_path(), (time.time() - 10, time.time() - 10)); gb_floor._CACHE.update(t=0.0, mode=None)
check("container: stale mirror (>5 s) -> floor", gb_floor.no_latch())
os.remove(gb_floor.mirror_path())

# ---------------------------------------------------------------- gate methods (AST stub, no ROS)
gsrc = open(os.path.join(ROOT, "gb_safety", "gb_gate.py")).read()
check("standing rule: gb_obstacle stop distances still 0.27/0.28/0.24", (OB.STOP_FWD, OB.STOP_REV, OB.SWEEP_R) == (0.27, 0.28, 0.24))
check("standing rule: gate has no stop-distance override literals",
      not re.search(r"^(FRONT_STOP|REAR_STOP|SWEEP_R|S_FRONT_STOP|S_REAR_STOP|S_SWEEP_R)\s*=\s*[0-9.]+\s*$", gsrc, re.M))
check("deadman 0.5 s / zero hold 2 s / dual caps 0.288 and 1.0 / ang 0.8",
      'E("GATE_DEADMAN_S", "0.5")' in gsrc and 'E("GATE_ZERO_HOLD_S", "2.0")' in gsrc and 'E("GATE_MANUAL_MAX_LIN", "0.288")' in gsrc and 'E("GATE_AUTO_MAX_LIN", "1.0")' in gsrc and 'E("GATE_MAX_ANG", "0.8")' in gsrc)
gt = ast.parse(gsrc); cls = next(n for n in gt.body if isinstance(n, ast.ClassDef) and n.name == "Gate")
gns = {"time": time, "os": os, "OB": OB, "math": math, "json": json, "gb_floor": gb_floor}
for node in gt.body:
    if isinstance(node, ast.Assign) and any(getattr(t_, "id", "") in ("FAULT_LIMIT", "FAULT_WIN", "BACKOFF_MARGIN", "BACKOFF_V", "BACKOFF_S", "SOFT_SENSORS", "SOFT_LIMIT", "SOFT_WIN") for t_ in node.targets):
        exec(compile(ast.Module([node], []), "c", "exec"), {"E": lambda k, d: d, **gns}, gns)
ESTOP = os.path.join(SH, "ESTOP"); MAINT = os.path.join(SH, "MAINT"); ARMED = os.path.join(SH, "ARMED")
ABORT = os.path.join(TMP, "explore", "abort")
gns.update(ESTOP=ESTOP, MAINT=MAINT, REAR_STOP=0.28, ARMED_FILE=ARMED, EXPLORE_ABORT=ABORT, ZERO_HOLD=2.0, MAX_LIN=0.12,
           MAX_ANG=0.8, STREAM_CAP=600.0, DEADMAN=0.5)
exec(compile(ast.Module([n for n in gt.body if isinstance(n, ast.FunctionDef) and n.name == "_clamp"], []), "f", "exec"), gns)
meth = {}
NAMES = ("collision", "fault_latch", "tilt_latch", "floor_nolatch", "floor_stop", "floor_held", "_latch_sig", "_evt_sig",
         "_tick_locked", "on_stream", "on_req")
for n in cls.body:
    if isinstance(n, ast.FunctionDef) and n.name in NAMES:
        exec(compile(ast.Module([n], []), n.name, "exec"), gns); meth[n.name] = gns[n.name]
check("gate defines floor_stop / floor_held / floor_nolatch", all(k in meth for k in NAMES), sorted(meth))
import threading
class FZ:
    def __init__(s_, rev=0.6, blind_r=0.0): s_.lidar = types.SimpleNamespace(rev=rev, blind_r=blind_r); s_.q = []
    def pop_event(s_, now): return s_.q.pop(0) if s_.q else None
    def reset_detectors(s_): pass
class G:
    locals().update(meth)
    _thr = {}
    def __init__(s_):
        s_.fz = FZ(); s_.faults = []; s_.estop = False; s_.maint = False; s_.backoff = False; s_.lin = 0.06; s_.ang = 0.0
        s_.mode = "stream"; s_.trips = []; s_.latches = []; s_.logs = []; s_.finishes = []; s_.sent = []
        s_.floor_hold_t = 0.0; s_._floor_sig = None; s_._floor_chk = 0.0; s_._floor_on = True; s_._floor_evt = None
        s_.lock = threading.RLock(); s_.zero_until = 0.0; s_.last_zero = 0.0; s_.t_start = 0.0; s_.t_fresh = 0.0
        s_.deadline = 0.0; s_.src = ""; s_.rid = ""; s_.joy = {"present": False, "lin": 0.0, "ang": 0.0, "t": 0.0}
        s_.seen = []
    def trip(s_, why, src): s_.trips.append(why)
    def finish(s_, why): s_.mode = "idle"; s_.finishes.append(why); s_.zero_until = time.time() + 2.0
    def log(s_, m): s_.logs.append(m)
    def log_throttle(s_, m, every=2.0): s_.logs.append(m)
    def send(s_, l, a): s_.sent.append((l, a))
    def _write_latch(s_, f, d): s_.latches.append((os.path.basename(f), d)); open(f, "w").write(json.dumps(d))
    def _no_range_blocks_autonomy(s_): return True       # stop the tick right after the latch/hold section
    def _lease_live(s_): return False
    def _lin_cap(s_, src=None, joystick=False): return 0.288
def arm_now():
    time.sleep(0.02); json.dump({"t": time.time(), "src": "test wake up Wally"}, open(ARMED, "w"))

# ground: 5 soft stall events (the 13:57 return-turn pattern) -> normal stop, no latch
wipe(); set_mode("ground"); arm_now(); g = G()
for k in range(gns["SOFT_LIMIT"]):
    g.fz.q.append((time.time(), "imu-gyro", "turn stall (imu gyro)")); g.lin = 0.0; g.mode = "turn"; g.collision("explore")
check("GROUND 5 stall events in 120 s -> NO latch file written", not g.latches and not os.path.exists(ESTOP)
      and not os.path.exists(os.path.join(SH, "ESTOP_REQ")), g.latches)
check("GROUND 5 stall events -> gate estop flag stays False", g.estop is False)
check("GROUND -> normal stop (finish 'FLOOR STOP')", any("FLOOR STOP" in f for f in g.finishes), g.finishes[-1:])
check("GROUND -> disarmed (ARMED removed, arm.log line)", not os.path.exists(ARMED) and "gate floor stop" in open(os.path.join(SH, "arm.log")).read())
check("GROUND -> mission abort file written", os.path.exists(ABORT) and "floor stop" in open(ABORT).read())
ev = json.load(open(gb_floor.stop_event_path()))
check("GROUND -> FLOOR_STOP event for the host (cause logged)", ev.get("kind") == "floor-normal-stop" and "stall" in ev.get("why", ""), ev)
check("GROUND -> floor_stop.log has the cause", "NORMAL STOP (no latch)" in open(gb_floor.log_path()).read())
check("GROUND -> motion held until re-armed", g.floor_held() is True)
g.mode = "idle"; g.on_stream(types.SimpleNamespace(linear=types.SimpleNamespace(x=0.08), angular=types.SimpleNamespace(z=0.0)))
check("GROUND held: stream refused before re-arm", g.mode == "idle" and any("until re-armed" in m for m in g.logs))
arm_now()
check("GROUND 'wake up Wally' (ARMED rewritten) -> hold released", g.floor_held() is False)
g.on_stream(types.SimpleNamespace(linear=types.SimpleNamespace(x=0.08), angular=types.SimpleNamespace(z=0.0)))
check("GROUND after re-arm: stream accepted (no latch blocks)", g.mode == "stream" and abs(g.lin - 0.08) < 1e-9, g.mode)

# ground: 3 hard collisions (IMU jolt) -> normal stop, no latch
wipe(); arm_now(); g = G()
for k in range(3):
    g.fz.q.append((time.time(), "imu", "jolt 2.1 g")); g.lin = 0.06; g.mode = "stream"; g.collision("explore")
check("GROUND 3 collision faults in 60 s -> normal stop, no latch", not g.latches and not g.estop and any("FLOOR STOP" in f for f in g.finishes))
check("GROUND fault counter reset after the normal stop", g.faults == [])
# ground: hard tilt
wipe(); arm_now(); g = G(); g.fz.q.append((time.time(), "imu-tilt-hard", "pitch 31 deg")); g.collision("explore")
check("GROUND hard tilt -> normal stop, no latch, disarmed", not g.latches and not g.estop and not os.path.exists(ARMED)
      and any("tipping over" in f for f in g.finishes))
# ground: a latch file appears anyway (old writer / SSH touch) -> one-shot stop, never a block
wipe(); arm_now(); g = G(); g.mode = "stream"
json.dump({"src": "ssh", "kind": "safety", "why": "touch"}, open(ESTOP, "w")); open(MAINT, "w").write("{}")
g._tick_locked()
check("GROUND latch file present -> one-shot normal stop", any("latch file present" in f for f in g.finishes) and g.mode == "idle")
check("GROUND latch file present -> estop/maint flags not set", g.estop is False and g.maint is False)
n_fin = len(g.finishes); arm_now(); g._tick_locked(); g._tick_locked()
check("GROUND same file again -> no repeated stop, and after re-arm it does not block", len(g.finishes) == n_fin and g.floor_held() is False)
# ground: FLOOR_STOP event from voice/clap -> gate stops once
wipe(); arm_now(); g = G(); g._floor_evt = g._evt_sig(); g.mode = "stream"
gb_floor.request_stop("clap", "double clap", was="clap-estop"); g._tick_locked()
check("GROUND clap/voice FLOOR_STOP event -> gate stops (normal)", any("double clap" in f for f in g.finishes) and g.mode == "idle")
# bench: legacy latching kept
wipe(); set_mode("bench"); g = G(); g._floor_chk = 0.0
for k in range(3):
    g.fz.q.append((time.time(), "imu", "jolt 2.1 g")); g.lin = 0.06; g.mode = "stream"; g.collision("explore")
check("BENCH 3 collision faults -> legacy ESTOP latch (ESTOP_REQ + ESTOP, kind=safety)",
      g.estop and sorted(x[0] for x in g.latches) == ["ESTOP", "ESTOP_REQ"])
g = G(); g._floor_chk = 0.0; g.fz.q.append((time.time(), "imu-tilt-hard", "pitch 31 deg")); g.collision("explore")
check("BENCH hard tilt -> legacy latch", g.estop and any(x[0] == "ESTOP" for x in g.latches))
wipe()

# ---------------------------------------------------------------- obstacle guard still stops (fusion, ground)
N = 400; INC = 2 * math.pi / N; NAN = float("nan"); INF = float("inf")
def scan_wall(x):           # flat wall perpendicular to the heading at distance x (front if x>0, rear if x<0)
    r = []
    for i in range(N):
        a = OB._wrap(i * INC); c = math.cos(a)
        d = (x / c) if abs(c) > 1e-9 and (x / c) > 0 else INF
        d = min(d, 3.0 / abs(math.sin(a)) if abs(math.sin(a)) > 1e-9 else INF)
        r.append(d if 0.10 <= d <= 10.0 else NAN)
    return r
def fusion(r, t=100.0):
    f = OB.Fusion(); f.on_odom(t, 0.0, 0.0, 0.0, 0.0, 0.0); f.imu_t = t
    D = OB.DepthView(t); D.fwd = INF; D.valid_frac = 0.9; f.on_depth(D)
    f.on_lidar(OB.lidar_analyze(r, 0.0, INC, 0.10, 10.0, t)); return f
for d in (0.10, 0.20, 0.24):
    f = fusion(scan_wall(d)); l, a, why, kind = f.limit(0.08, 0.0, 100.0, 0.12)
    check("OBSTACLE front %.2f m -> forward refused (stop 0.27)" % d, l == 0 and kind == "block", "l=%s why=%s" % (l, why))
    f = fusion(scan_wall(-d)); l, a, why, kind = f.limit(-0.08, 0.0, 100.0, 0.12)
    check("OBSTACLE rear %.2f m -> reverse refused (stop 0.28)" % d, l == 0 and kind == "block", "l=%s why=%s" % (l, why))
f = fusion(scan_wall(1.5)); l, a, why, kind = f.limit(0.08, 0.0, 100.0, 0.12)
check("OBSTACLE clear (1.5 m) -> forward allowed", l > 0 and kind != "block", "l=%s why=%s" % (l, why))

# ---------------------------------------------------------------- spoken emergency stop (gb_estop) + re-arm (gb_arm)
import gb_estop, gb_arm
calls = []
gb_estop.zero_now = lambda why="": calls.append(why)
check("'emergency stop' is no longer a latch phrase", gb_estop.stop_action("emergency stop") is None)
check("'emergency halt' is recognised as the strong latch phrase", gb_estop.stop_action("emergency halt") == "latch")
wipe(); set_mode("ground"); arm_now(); acts = []
ok = gb_estop.estop("voice", "command: emergency stop", actions={"explore": lambda: acts.append("explore")}); time.sleep(0.2)
check("GROUND spoken 'emergency stop' -> NO latch file (host or shared)", ok and not os.path.exists(gb_estop.ESTOP) and not os.path.exists(gb_estop.SHARED_ESTOP))
check("GROUND spoken 'emergency stop' -> zero sent + mission action run", calls and acts == ["explore"], (calls, acts))
check("GROUND spoken 'emergency stop' -> disarmed", not gb_arm.is_armed() and not os.path.exists(ARMED))
check("GROUND spoken 'emergency stop' -> FLOOR_STOP event + estop.log line", os.path.exists(gb_floor.stop_event_path())
      and "FLOOR STOP (normal, no latch)" in open(gb_estop.LOG).read())
check("GROUND re-arm: 'wake up Wally' works right after", gb_arm.maybe_arm("wake up Wally", "test") and gb_arm.is_armed())
wipe(); json.dump({"src": "gate-faults", "kind": "safety", "why": "old"}, open(os.path.join(SH, "ESTOP"), "w"))
json.dump({"src": "maintenance"}, open(os.path.join(DATA, "MAINT"), "w"))
check("GROUND leftover safety latch + MAINT -> arm still succeeds", gb_arm.arm("test wake up Wally") and gb_arm.is_armed())
arch = os.listdir(os.path.join(SH, "floor_archive")) + os.listdir(os.path.join(DATA, "floor_archive"))
check("GROUND leftovers archived (moved, not deleted) with cause logged",
      any(a.startswith("ESTOP.floor-archived") for a in arch) and any(a.startswith("MAINT.floor-archived") for a in arch)
      and "ARCHIVED" in open(gb_floor.log_path()).read(), arch)
wipe(); set_mode("bench")
gb_estop.estop("voice", "command: emergency stop"); time.sleep(0.1)
check("BENCH spoken 'emergency stop' -> legacy latch written", os.path.exists(gb_estop.ESTOP))
check("BENCH latch blocks arming (unchanged)", gb_arm.arm("test") is False)
gb_estop.clear("test"); wipe()

# ---------------------------------------------------------------- clap
import gb_clap_logic as CL
set_mode("ground"); arm_now()
r = CL.latch_clap(os.path.join(DATA, "ESTOP"), os.path.join(SH, "ESTOP"), os.path.join(DATA, "MAINT"), ABORT)
check("GROUND double clap -> normal stop, no latch file, disarmed", r["transport"] == "floor-stop" and not r["wrote"]
      and not os.path.exists(os.path.join(DATA, "ESTOP")) and not os.path.exists(ARMED))
set_mode("bench"); r = CL.latch_clap(os.path.join(DATA, "ESTOP"), os.path.join(SH, "ESTOP"), os.path.join(DATA, "MAINT"), ABORT)
check("BENCH double clap -> legacy clap latch", len(r["wrote"]) == 2)
wipe()

# ---------------------------------------------------------------- watchdog / voice / launcher (source + shell)
wsrc = open(os.path.join(ROOT, "gb_safety", "gb_watchdog.py")).read()
check("watchdog: ground latch file -> 2 s zero burst only (no continuous hold); stale/dead-source zeroing untouched",
      "def _floor" in wsrc and "start_burst(\"latch file" in wsrc and "os.path.exists(ESTOP) and not self._floor()" in wsrc
      and "gate heartbeat stale" in wsrc)
vsrc = open(os.path.join(ROOT, "grokbot_voice.py")).read()
check("voice: do_estop on the floor says stopped/disarmed, not latched", "Say wake up Wally when you want to drive again." in vsrc)
check("voice: unlock on the floor always succeeds", "Nothing is locked. Say wake up Wally when you want to drive." in vsrc)
check("voice: wake rule still requires a wake phrase (maybe_arm ignores commands)",
      "wake" in gb_arm.WAKE_RE.pattern and "hey" in gb_arm.WAKE_RE.pattern
      and gb_arm.WAKE_RE.search("wake up wally")
      and gb_arm.WAKE_RE.search("hey wally")
      and gb_arm.WAKE_RE.search("wally wake up")
      and not gb_arm.WAKE_RE.search("start exploring")
      and not gb_arm.maybe_arm("start exploring", "test"))
run = open(os.path.join(ROOT, "gb_safety", "gb_safety_run.sh")).read()
a = run.index("FLOOR_ARCH_HOST="); b = run.index("\nmirror() {")
fns = run[a:b]
sh = os.path.join(TMP, "fl.sh")
open(sh, "w").write("set -u\nWHAT=test; C=__no_such_container__; GB=%s; SH=%s; HOST_ESTOP=$GB/data/ESTOP; MODE_FILE=%s\n"
                    "FLOOR_ABORT=%s\nlog() { echo \"LOG $*\"; }\ndocker() { return 1; }\n%s\n" % (TMP, SH, MODE, ABORT, fns))
def bash(cmd): return subprocess.run(["bash", "-c", "source %s; %s" % (sh, cmd)], capture_output=True, text=True, timeout=20)
set_mode("ground"); arm_now()
os.makedirs(DATA, exist_ok=True)
json.dump({"src": "gate-faults", "kind": "safety", "why": "5 repeated stall events"}, open(os.path.join(DATA, "ESTOP"), "w"))
json.dump({"src": "gate-faults", "kind": "safety", "why": "5 repeated stall events"}, open(os.path.join(SH, "ESTOP_REQ"), "w"))
out = bash("floor_mirror; echo FLOOR=$floor; floor_backstop")
check("launcher: floor_mirror -> FLOOR_MODE.json ground, floor=1", "FLOOR=1" in out.stdout and json.load(open(gb_floor.mirror_path()))["mode"] == "ground", out.stdout + out.stderr)
check("launcher ground backstop: latch files archived (not deleted), none left",
      not os.path.exists(os.path.join(DATA, "ESTOP")) and not os.path.exists(os.path.join(SH, "ESTOP_REQ"))
      and len(os.listdir(os.path.join(DATA, "floor_archive"))) >= 1, out.stderr)
check("launcher ground backstop: disarmed + cause logged", not os.path.exists(ARMED) and "5 repeated stall events" in open(os.path.join(SH, "floor_stop.log")).read())
arm_now(); gb_floor.request_stop("gate:explore", "test event", was="fault-latch")
out = bash("floor_backstop; e1=$([ -e $SH/FLOOR_STOP ] && echo kept || echo gone); sleep 0.4; floor_backstop; echo E1=$e1 E2=$([ -e $SH/FLOOR_STOP ] && echo kept || echo gone)")
check("launcher: FLOOR_STOP kept >=0.3 s for the gate, then processed + archived", "E1=kept" in out.stdout and "E2=gone" in out.stdout, out.stdout + out.stderr)
check("launcher: FLOOR_STOP -> disarm + log + mission abort (temp path)", not os.path.exists(ARMED) and "test event" in open(os.path.join(SH, "floor_stop.log")).read() and "floor stop" in open(ABORT).read())
set_mode("bench"); out = bash("floor_mirror; echo FLOOR=$floor")
check("launcher: bench -> floor=0 (legacy latch path untouched)", "FLOOR=0" in out.stdout)

shutil.rmtree(TMP, ignore_errors=True)
print("\nRESULT: %d passed, %d failed" % (sum(1 for _n, ok in RES if ok), sum(1 for _n, ok in RES if not ok)))
sys.exit(0 if all(ok for _n, ok in RES) else 1)
