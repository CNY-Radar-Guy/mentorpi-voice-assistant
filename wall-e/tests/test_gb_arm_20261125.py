"""Wake-phrase arming tests (gb_arm + hooks). No ROS, no motion."""
import os, re, sys, tempfile, types, json
R = os.environ.get("GB_REL", "/home/pi/grokbot/releases/e48d3d8"); sys.path.insert(0, R)
D = tempfile.mkdtemp(); os.environ["GB_ARM_DIR"] = D
import gb_arm
P = F = 0
def check(n, ok):
    global P, F; P += bool(ok); F += (not ok); print(("PASS " if ok else "FAIL ") + n)
check("default (no flag) -> disarmed", not gb_arm.is_armed())
check("'hey wally explore' does not arm", not gb_arm.maybe_arm("hey wally, explore the room"))
check("'wake up' alone does not arm", not gb_arm.maybe_arm("wake up"))
check("'Wake up, Wally.' arms", gb_arm.maybe_arm("Wake up, Wally.") and gb_arm.is_armed())
open(os.path.join(D, "ESTOP"), "w").write("x")
check("ESTOP present -> not armed even with flag", not gb_arm.is_armed())
check("arm refused while ESTOP latched", not gb_arm.arm("test"))
os.remove(os.path.join(D, "ESTOP"))
check("disarm removes flag", gb_arm.disarm("test") and not gb_arm.is_armed())
# gb_estop.gate_request refuses non-stop while disarmed, passes stop
import gb_estop
sent = []
gb_estop._pub = lambda topic, typ, msg, **k: sent.append(json.loads(msg["data"])) or True
r = gb_estop.gate_request({"op": "timed", "lin": 0.08, "ang": 0, "dur": 1})
check("gate_request timed while disarmed -> refused, nothing published", r is None and not sent)
gb_estop.gate_request({"op": "stop", "src": "t"})
check("gate_request stop while disarmed -> published", len(sent) == 1 and sent[0]["op"] == "stop")
gb_arm.arm("test"); r = gb_estop.gate_request({"op": "timed", "lin": 0.08, "ang": 0, "dur": 1})
check("gate_request timed while armed -> published", r is not None and len(sent) == 2)
# wakegate: disarmed refuses even in a woken utterance; armed + woken allowed
import gb_wakegate as WG
gb_arm.disarm("t"); WG.set_context(WG.motion_allowed(True, False, False), "wake up wally explore")
check("wakegate: disarmed -> mission refused with arm line", WG.block("mission", "explore") == gb_arm.REFUSE_LINE)
gb_arm.arm("t"); check("wakegate: armed + woken -> allowed", WG.block("mission", "explore") is None)
WG.set_context(False, "overheard"); check("wakegate: armed but overheard -> still rejected", WG.block("mission", "x") is not None)
WG.clear_context(); gb_arm.disarm("t")
check("wakegate: web thread (no voice context) disarmed -> refused", WG.block("intent:navigate", "couch") is not None)
# explore container publisher zeroes when disarmed
src = open(os.path.join(R, "gb_explore.py")).read()
ns = {}
exec(compile(re.search(r"class _GateRoute:.*?\n(?=\S)", src, re.S).group(0), "gr", "exec"),
     {"os": os, "ARMED_CONT": os.path.join(D, "ARMED"), "CONT_DIR": D, "_now": lambda: "now", "print": lambda *a, **k: None}, ns)
class Tw:
    def __init__(self, x=0.0, z=0.0): self.linear = types.SimpleNamespace(x=x, y=0.0); self.angular = types.SimpleNamespace(z=z)
g, dct = [], []
gr = ns["_GateRoute"](types.SimpleNamespace(publish=g.append), types.SimpleNamespace(publish=dct.append))
gr.publish(Tw(0.08, 0.0))
check("explore publisher: disarmed non-zero -> zero sent + abort written", g[-1].linear.x == 0 and open(os.path.join(D, "abort")).read() == "disarmed")
open(os.path.join(D, "ARMED"), "w").write("{}"); gr2 = ns["_GateRoute"](types.SimpleNamespace(publish=g.append), types.SimpleNamespace(publish=dct.append))
gr2.publish(Tw(0.08, 0.0)); check("explore publisher: armed non-zero passes", g[-1].linear.x == 0.08)
os.remove(os.path.join(D, "ARMED"))
check("explore request(): disarmed refusal present", "refused: disarmed" in src)
v = open(os.path.join(R, "grokbot_voice.py")).read()
check("voice: arms only via gb_arm.maybe_arm on wake match", "gb_arm.maybe_arm(text, \"voice\")" in v)
check("voice: stop phrase + stop intent disarm", v.count("gb_arm.disarm(\"stop") == 2 and "if action in (\"latch\", \"halt\"):" in v)
check("voice: service start disarms", "voice service start (default disarmed)" in v)
i_kill = v.find("KILL SWITCH FIRST"); check("voice: stop handling unchanged and ahead of motion dispatch", i_kill > 0)
g2 = open(os.path.join(R, "gb_safety", "gb_gate.py")).read()
check("gb_gate untouched: no arm hook in gate", "gb_arm" not in g2)
print("RESULT: %d passed, %d failed" % (P, F))
