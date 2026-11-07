"""Bench-hold release and the quiet-gate clap stop. No robot and no microphone."""
import json, os, sys, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import gb_clap_logic
import gb_estop
import gb_maint
import gb_wally

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

def env_ms(pieces):
    out = []
    for db, n in pieces:
        out.extend([db] * n)
    return out

def clap_at(buf, at, peak=-18, width=6):
    for i in range(width):
        buf[at + i] = peak
    return buf

# continuous speech never arms, including two spikes and the tail after it
speech = env_ms([(-62, 300), (-22, 900)])
clap_at(speech, 800)
clap_at(speech, 1100)
found, gate = gb_clap_logic.run_envelope(speech)
check("continuous speech does not arm a pair", found == [] and gate.state == "listen", (found, gate.state))

# quiet, then a real pair about a quarter second apart
quiet = env_ms([(-60, 1200)])
clap_at(quiet, 820)
clap_at(quiet, 1070)
pair, gate = gb_clap_logic.run_envelope(quiet)
check("quiet then two claps is one pair", len(pair) == 1 and 120 <= pair[0]["gap_ms"] <= 500, pair)
check("that pair did not publish motion", "motion" not in pair[0])

one = env_ms([(-60, 900)])
clap_at(one, 760)
found, gate = gb_clap_logic.run_envelope(one)
check("one clap is not a stop", found == [] and gate.state == "peak", gate.state)
check("the first clap did not demand quiet again", gate.quiet_for >= gb_clap_logic.QUIET_BEAT_S)

early = env_ms([(-60, 900)])
clap_at(early, 760)
clap_at(early, 790)
found, _gate = gb_clap_logic.run_envelope(early)
check("a same-clap echo is not a second clap", found == [])

late = env_ms([(-60, 1600)])
clap_at(late, 760)
clap_at(late, 1400)
found, _gate = gb_clap_logic.run_envelope(late)
check("a pair later than half a second is not accepted", found == [])

hum = env_ms([(-40, 6000)])
found, gate = gb_clap_logic.run_envelope(hum)
check("steady noise adapts and does not fire", found == [] and gate.floor > -50, gate.floor)

loud = env_ms([(-60, 800), (-15, 300)])
found, gate = gb_clap_logic.run_envelope(loud)
check("loud sound after quiet disarms", found == [] and gate.state == "listen", gate.state)

# bench hold
tmp = tempfile.mkdtemp(prefix="gbbench_")
maint = os.path.join(tmp, "MAINT")
shared_m = os.path.join(tmp, "shared", "MAINT")
os.makedirs(os.path.dirname(shared_m))
host = os.path.join(tmp, "ESTOP")
shared_e = os.path.join(tmp, "shared", "ESTOP")
gb_maint.write_lock([maint, shared_m], now=10)
check("deploy text is not a bench hold yet", json.load(open(maint))["kind"] == "maintenance")
gb_maint._BENCH.update(wake_at=100, source="voice")
check("wake plus John does not clear the deploy lock",
      gb_maint.release_bench([maint], True, now=101) == [] and os.path.isfile(maint))
check("arming is refused without a finished deploy", gb_maint.arm_bench(maint, success=False) is False)
fault = dict(json.load(open(maint)), kind="fault", scope="fault")
json.dump(fault, open(maint, "w"))
check("a fault lock is not armed", gb_maint.arm_bench(maint, success=True) is False)
json.dump({"t": 1, "src": "maintenance", "kind": "maintenance", "why": "something else"}, open(maint, "w"))
check("an untyped maintenance file is not armed", gb_maint.arm_bench(maint, success=True) is False)
gb_maint.write_lock([maint], now=11)
# write_lock will not overwrite; remove and rewrite the deploy file
os.remove(maint)
gb_maint.write_lock([maint, shared_m], now=12)
check("a finished deploy can arm the bench hold", gb_maint.arm_bench(maint, success=True, now=20) and json.load(open(maint))["scope"] == "bench-test")
gb_maint.arm_bench(shared_m, success=True, now=20)
gb_maint._BENCH.update(wake_at=None, source=None)
check("John without a fresh wake does not unlock", gb_maint.release_bench([maint], True, now=30, source="voice") == [])
gb_maint.note_bench_wake(40, source="model")
check("a model wake does not count", gb_maint.release_bench([maint], True, now=41, source="voice") == [])
gb_maint.note_bench_wake(50, source="voice")
check("a quoted identity does not unlock", gb_wally.fresh_john('"I am John"') is False)
check("a negated identity does not unlock", gb_wally.fresh_john("I am not John") is False)
gone = gb_maint.release_bench([maint, shared_m], gb_wally.fresh_john("wake up Wally, I am John"), now=51)
check("wake plus John clears only the bench files", set(gone) == {maint, shared_m} and not os.path.exists(maint))
json.dump({"t": 1, "src": "clap", "kind": "normal", "why": "double clap"}, open(host, "w"))
check("that release did not remove a clap stop", os.path.isfile(host))
os.environ["GB_DATA"] = tmp
os.environ["GB_SHARED_ESTOP"] = shared_e
gb_estop.ESTOP = host
gb_estop.SHARED_ESTOP = shared_e
gb_estop.zero_now = lambda *a, **k: None
gb_estop.estop("voice", "voice (barge-in): halt")
check("a barge-in stop is on the host and the shared file", os.path.isfile(host) and os.path.isfile(shared_e))
os.remove(host)
gb_estop.estop("voice", "voice (barge-in): halt again")
check("a missing host copy is written with the shared stop", os.path.isfile(host) and json.load(open(host))["kind"] == "normal")
safety = {"t": 1, "src": "gate-tilt", "kind": "safety", "why": "tilt"}
json.dump(safety, open(host, "w"))
lat = gb_clap_logic.latch_clap(host, shared_e, maint, os.path.join(tmp, "abort"))
check("a clap does not replace a hard fault or send motion", host in lat["kept_safety"] and lat["motion"] is False and json.load(open(host))["kind"] == "safety")
effect = gb_clap_logic.inhibit_effect(True, True)
check("held locks zero and cancel without a velocity command", effect == {"zero": True, "cancel_missions": True, "velocity_command": False}, effect)
check("the clap unit is not started by activation", "gb-clap was not enabled" in open(os.path.join(os.path.dirname(HERE), "..", "deploy", "wally_activate.sh")).read())

print("test_bench_clap: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
