"""GB-46 explorefix tests: Jarvis explore launch, mission-lease race, startup watchdog.

No ROS, no motion, no arming. Isolated temp dirs only.
"""
from __future__ import annotations

import importlib.util
import json
import os
import shutil
import sys
import tempfile
import time

ROOT = os.environ.get("GB_REL", "/home/pi/grokbot/releases/e48d3d8")
sys.path.insert(0, ROOT)
REMOTE_COPY = os.environ.get(
    "JARVIS_REMOTE_PY",
    "/tmp/explorefix/remote.py",
)

# Isolate gb_arm from the live robot before import.
ARM_DIR = tempfile.mkdtemp(prefix="gb_explorefix_arm_")
os.environ["GB_ARM_DIR"] = ARM_DIR
os.environ["GB_DATA"] = ARM_DIR

import gb_arm
import gb_explore

P = F = 0

def check(n, ok, detail=None):
    global P, F
    P += bool(ok)
    F += (not ok)
    line = ("PASS " if ok else "FAIL ") + n
    if detail is not None and not ok:
        line += " | " + repr(detail)
    print(line, flush=True)


# --------------------------------------------------------------------------- 1. remote.py explore launch construction
def load_remote():
    if not os.path.isfile(REMOTE_COPY):
        return None
    spec = importlib.util.spec_from_file_location("jarvis_wally_remote_explorefix", REMOTE_COPY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


remote = load_remote()
check("remote.py copy is loadable for launch tests", remote is not None, REMOTE_COPY)
if remote is not None:
    launch = remote.build_explore_launch([])
    check("explore launch path is grokbot /api/chat (not a bare CLI)",
          launch.get("launch_path") == "grokbot_api_chat" and launch.get("url") == "http://127.0.0.1:8000/api/chat")
    check("explore phrase is a real mission command",
          launch.get("phrase") == "explore the room")
    check("default explore body is live (dry_run false) with robot true",
          launch["body"]["text"] == "explore the room"
          and launch["body"]["dry_run"] is False
          and launch["body"]["robot"] is True)
    check("launch plan is not a bare gb_explore.py argv (which only prints help)",
          launch.get("cli") in (None, "") and "gb_explore.py" not in json.dumps(launch.get("argv")))
    src = open(REMOTE_COPY, encoding="utf-8").read()
    check("cmd_explore no longer Popen(['python3', script]) with no --run",
          "cmd = [\"python3\", str(script)]" not in src
          and "build_explore_launch" in src
          and "/api/chat" in src)
    check("--run help-only path is not the Jarvis launch",
          "uses_run_flags" in launch and launch["uses_run_flags"] is True)
    timed = remote.build_explore_launch(["120", "actor=jarvis"])
    check("optional seconds become 'explore the room for N seconds'",
          timed["phrase"] == "explore the room for 120 seconds" and timed["seconds"] == 120
          and "for 120 seconds" in timed["body"]["text"], timed)
    dry = remote.build_explore_launch(["dry_run=1"])
    check("dry_run flag stays dry and does not claim a live start path of bare CLI help",
          dry["dry_run"] is True and dry["body"]["dry_run"] is True and dry["body"]["robot"] is False)
    check("actor defaults to jarvis", launch.get("actor") == "jarvis")

    plan = gb_explore.parse_plan(launch["phrase"])
    check("default phrase parses to explore with cap_s/stall_s/return/save_map",
          plan and plan.get("mission") == "explore"
          and plan.get("return") is True and plan.get("save_map") is True
          and plan.get("cap_s") == 240 and plan.get("stall_s") == 30, plan)
    plan_t = gb_explore.parse_plan(timed["phrase"])
    check("timed phrase still an explore plan with requested duration",
          plan_t and plan_t.get("mission") == "explore" and plan_t.get("requested_s") == 120, plan_t)


# --------------------------------------------------------------------------- 2. mission-lease timing race
# Reproduce 9:34 ET: hold latched by an earlier stop, then a valid host-owned lease
# whose PID is invisible in the container namespace (os.kill / /proc fail).
td = tempfile.mkdtemp(prefix="gb_lease_race_")
os.environ["GB_ARM_DIR"] = td
check("lease tests isolated from live ARMED/MISSION_LEASE", gb_arm._dir() == td, gb_arm._dir())

hold_t = time.time()
time.sleep(0.02)
# Fake a host PID that this namespace cannot see (container-style).
invisible_pid = 467740
if gb_arm.pid_alive(invisible_pid):
    # extremely unlikely; pick a definitely dead pid
    invisible_pid = 1
    while gb_arm.pid_alive(invisible_pid) and invisible_pid < 10:
        invisible_pid += 1
    if gb_arm.pid_alive(invisible_pid):
        invisible_pid = 99999999

check("repro pid is not visible in this namespace (container condition)",
      not gb_arm.pid_alive(invisible_pid) and not gb_arm._pid_visible(invisible_pid),
      (invisible_pid, gb_arm.pid_alive(invisible_pid), gb_arm._pid_visible(invisible_pid)))

lease = gb_arm.set_mission_lease("voice", pid=invisible_pid, reason="explore request", mission="explore")
check("lease file was written with started_at after the prior stop hold",
      lease and float(lease["started_at"]) > hold_t and gb_arm.read_lease() is not None)
check("lease_is_live is still False for an invisible/dead pid (disarm safety unchanged)",
      gb_arm.lease_is_live(lease) is False)
check("BUGFIX: fresh_lease_since accepts a newer lease even when host pid is invisible in Docker",
      gb_arm.fresh_lease_since(hold_t, lease=lease) is True)
check("stream_allowed_after_hold lifts after a genuinely new lease (container pid invisible)",
      gb_arm.stream_allowed_after_hold(True, hold_t, joystick=False, lease=lease) is True)

# Real stop/disarm still immediately revokes and blocks stale streams
cleared = gb_arm.clear_mission_lease("stop request from halt")
check("stop/halt clears MISSION_LEASE immediately",
      cleared is True and gb_arm.read_lease() is None)
hold2 = time.time()
check("after revoke, stale/missing lease cannot stream until a new lease",
      gb_arm.fresh_lease_since(hold2) is False
      and gb_arm.stream_allowed_after_hold(True, hold2, joystick=False) is False)

# Pre-stop lease leftover (started_at <= hold) must not lift the hold
stale = {
    "owner": "voice",
    "pid": invisible_pid,
    "started_at": hold2 - 5.0,
    "reason": "old",
    "mission": "explore",
}
gb_arm._write_json(gb_arm.lease_path(), stale)
check("leftover pre-stop lease (started_at before hold) is not treated as fresh",
      gb_arm.fresh_lease_since(hold2) is False
      and gb_arm.stream_allowed_after_hold(True, hold2, joystick=False) is False)
os.remove(gb_arm.lease_path())

time.sleep(0.02)
# A genuinely new lease after stop, still with an invisible pid
new = gb_arm.set_mission_lease("voice", pid=invisible_pid, reason="new explore", mission="explore")
check("a genuinely new lease after stop is accepted with invisible host pid",
      gb_arm.fresh_lease_since(hold2, lease=new) is True)

# Visible live pid still works (host path)
live = gb_arm.set_mission_lease("voice", pid=os.getpid(), reason="host visible", mission="explore")
check("visible live pid still counts as a fresh lease",
      gb_arm.lease_is_live(live) is True and gb_arm.fresh_lease_since(hold2, lease=live) is True)

# Visible but dead pid must not lift the hold (host can observe death)
dead = 99999999
if gb_arm.pid_alive(dead):
    dead = 2147483647
check("control: dead pid is not alive", not gb_arm.pid_alive(dead))
# started_at after hold, but owner is a visible-dead? 99999999 typically not in /proc, so
# _pid_visible is False and we'd accept it -- that's the container case.
# To test the host-visible-dead path, only run the assertion when _pid_visible is True.
# We'll simulate by writing a pid that exists in /proc but is not killable... skip if not possible.
# Instead, monkey-patch _pid_visible True + pid_alive False.
real_visible = gb_arm._pid_visible
real_alive = gb_arm.pid_alive
gb_arm._pid_visible = lambda pid: True
gb_arm.pid_alive = lambda pid: False
try:
    zombie = gb_arm.set_mission_lease("voice", pid=12345, reason="dead host process", mission="explore")
    check("host-visible dead pid does not lift the lease hold",
          gb_arm.fresh_lease_since(hold2, lease=zombie) is False)
finally:
    gb_arm._pid_visible = real_visible
    gb_arm.pid_alive = real_alive

check("why_revokes_lease still treats stop/estop/disarm as immediate revoke",
      gb_arm.why_revokes_lease("stop request from halt")
      and gb_arm.why_revokes_lease("ESTOP latched")
      and gb_arm.why_revokes_lease("floor stop")
      and gb_arm.why_revokes_lease("disarm")
      and not gb_arm.why_revokes_lease("DEADMAN: no fresh stream command for 0.50s")
      and not gb_arm.why_revokes_lease("deadline reached"))
check("joystick still bypasses lease hold",
      gb_arm.stream_allowed_after_hold(True, time.time(), joystick=True) is True)
check("no hold means streams allowed",
      gb_arm.stream_allowed_after_hold(False, 0, joystick=False) is True)

# Armed manual /gb/cmd_in after an ended mission must not stay blocked by a stale hold.
# Isolate: no leftover lease, control armed flag in the temp dir.
if os.path.exists(gb_arm.lease_path()):
    os.remove(gb_arm.lease_path())
if os.path.exists(gb_arm.path()):
    os.remove(gb_arm.path())
hold_manual = time.time() - 0.2
check("stale hold + no lease + DISARMED still blocks manual /gb/cmd_in",
      gb_arm.stream_allowed_after_hold(True, hold_manual, now=hold_manual + 0.2, joystick=False) is False)
# Stay-armed through a revoke: ARMED mtime is BEFORE hold_t, so the short drain still applies.
open(gb_arm.path(), "w").write('{"actor":"jarvis","src":"test"}\n')
os.utime(gb_arm.path(), (hold_manual - 10, hold_manual - 10))
check("just after revoke, armed manual stream is still drained for leftover mission twists",
      gb_arm.stream_allowed_after_hold(True, hold_manual, now=hold_manual + 0.2, joystick=False) is False)
check("after 1s drain, armed + no mission lease allows manual /gb/cmd_in",
      gb_arm.stream_allowed_after_hold(True, hold_manual, now=hold_manual + gb_arm.LEASE_HOLD_DRAIN_S, joystick=False) is True)
# Fresh arm after the revoke (Jarvis UI arm) lifts the hold immediately, no drain wait.
open(gb_arm.path(), "w").write('{"actor":"jarvis","src":"jarvis-ui"}\n')
os.utime(gb_arm.path(), (hold_manual + 5, hold_manual + 5))
check("re-arm after revoke immediately allows armed manual /gb/cmd_in (no active mission)",
      gb_arm.stream_allowed_after_hold(True, hold_manual, now=hold_manual + 0.05, joystick=False) is True)
check("LEASE_HOLD_DRAIN_S is 1.0 s (longer than 0.5 s deadman)",
      abs(gb_arm.LEASE_HOLD_DRAIN_S - 1.0) < 1e-12)
gate_src = open(os.path.join(ROOT, "gb_safety", "gb_gate.py"), encoding="utf-8").read()
check("gate _stream_lease_ok uses stream_allowed_after_hold (clears stale hold for manual drive)",
      "stream_allowed_after_hold(" in gate_src)
# restore: leave temp dir armed state from later tests if any; research tests use their own dir

# Safety constants untouched
check("explore hard_max_lin is 1.0 m/s (GB-48 autonomous cap)", abs(gb_explore.HARD_MAX_LIN - 1.0) < 1e-12)
gate_src = open(os.path.join(ROOT, "gb_safety", "gb_gate.py"), encoding="utf-8").read()
check("gate still revokes lease on stop and holds until a fresh lease",
      "_revoke_mission_lease" in gate_src
      and "stream_allowed_after_hold(" in gate_src
      and "mission lease revoked; waiting for a fresh lease" in gate_src)
check("gate DEADMAN default stays 0.5 s", 'GATE_DEADMAN_S", "0.5"' in gate_src or "GATE_DEADMAN_S', '0.5'" in gate_src)


# --------------------------------------------------------------------------- 3. watchdog startup grace + stale hb + accurate report
check("startup grace is 15 seconds (not 5)", abs(gb_explore.HB_STARTUP_GRACE_S - 15.0) < 1e-12, gb_explore.HB_STARTUP_GRACE_S)
check("drive heartbeat timeout stays 2.0 s after grace", abs(gb_explore.HB_STALE_S - 2.0) < 1e-12)
check("finishing heartbeat tolerance stays 8.0 s", abs(gb_explore.HB_FINISH_STALE_S - 8.0) < 1e-12)
check("node-never-started guard stays 25.0 s", abs(gb_explore.HB_NEVER_STARTED_S - 25.0) < 1e-12)

t0 = 1000.0
# The 21:51 abort: elapsed 5.2 s in wait_inputs, hb 2s stale — must NOT trip with 15s grace
check("wait_inputs at +5.2s with a 2.2s-old hb is NOT stale (the 21:51 false abort)",
      gb_explore.host_heartbeat_stale(1005.2, 1003.0, "wait_inputs", t0, seen_hb=True) is False)
check("within 15s grace, even a very old hb timestamp is ignored",
      gb_explore.host_heartbeat_stale(1014.9, 990.0, "wait_inputs", t0, seen_hb=True) is False)
check("after 15s grace, 2s timeout applies in wait_inputs / explore",
      gb_explore.host_heartbeat_stale(1017.1, 1014.0, "wait_inputs", t0, seen_hb=True) is True
      and gb_explore.host_heartbeat_stale(1017.1, 1014.0, "explore", t0, seen_hb=True) is True)
check("after 15s grace, a fresh (1.5s) heartbeat is still ok during drive",
      gb_explore.host_heartbeat_stale(1017.0, 1015.5, "explore", t0, seen_hb=True) is False)
check("finishing still gets 8s tolerance after grace (4s-old hb is ok, 8.1s is stale)",
      gb_explore.host_heartbeat_stale(1020.0, 1016.0, "finishing", t0, seen_hb=True) is False
      and gb_explore.host_heartbeat_stale(1024.2, 1016.0, "finishing", t0, seen_hb=True) is True)
check("seen_hb=False is not a stale-hb trip (never-started guard is separate)",
      gb_explore.host_heartbeat_stale(1030.0, 0, "init", t0, seen_hb=False) is False)

hbdir = tempfile.mkdtemp(prefix="gb_hb_")
stale_hb = os.path.join(hbdir, "hb")
with open(stale_hb, "w") as f:
    json.dump({"t": time.time() - 30, "phase": "wait_inputs"}, f)
open(os.path.join(hbdir, "hb.tmp"), "w").write("partial")
removed = gb_explore.clear_node_heartbeat(hbdir)
check("clear_node_heartbeat removes stale hb (and tmp) before Popen",
      not os.path.exists(stale_hb) and not os.path.exists(os.path.join(hbdir, "hb.tmp"))
      and "hb" in removed)
check("clear_node_heartbeat is idempotent when files are already gone",
      gb_explore.clear_node_heartbeat(hbdir) == [])

src_ex = open(os.path.join(ROOT, "gb_explore.py"), encoding="utf-8").read()
check("_run unlinks stale hb immediately before Popen",
      "clear_node_heartbeat(" in src_ex.split("proc = subprocess.Popen")[0].split("def _run")[-1]
      or "clear_node_heartbeat(HOST_DIR)" in src_ex)
check("wait_inputs writes hb in its loop",
      "def wait_inputs" in src_ex and "hb()" in src_ex[src_ex.index("def wait_inputs"):src_ex.index("def wait_inputs")+500])
check("watchdog loop uses host_heartbeat_stale (15s grace)",
      "host_heartbeat_stale(" in src_ex)

# Accurate report: watchdog must not be described as a stop command
check("user stop command still reports as a stop command",
      gb_explore.report_abort_reason("stop command") == "a stop command"
      and gb_explore.report_abort_reason("signal 2") == "a stop command")
wd_reason = gb_explore.report_abort_reason("stop command (watchdog: node heartbeat stale)")
check("watchdog abort wrapped as stop command reports the watchdog reason",
      "watchdog" in wd_reason.lower() and "stop command" not in wd_reason.lower()
      and "node heartbeat stale" in wd_reason, wd_reason)
check("plain watchdog reason is preserved",
      "watchdog: node heartbeat stale" in gb_explore.report_abort_reason("watchdog: node heartbeat stale")
      and "stop command" not in gb_explore.report_abort_reason("watchdog: node heartbeat stale"))

# _report end-to-end
plan = {"mission": "explore", "cap_s": 240, "stall_s": 30, "return": True, "save_map": True}
speech = gb_explore._report(plan, {"outcome": "aborted", "reason": "signal 2"}, False)
check("SIGINT without watchdog still spoken as a stop command",
      "I stopped because of a stop command" in speech, speech)
speech_wd = gb_explore._report(
    plan,
    {"outcome": "aborted", "reason": "stop command (watchdog: node heartbeat stale)",
     "phases": ["wait_inputs"], "elapsed_s": 5.2, "cmds_published": 0},
    False,
)
check("watchdog abort is NOT spoken as a stop command",
      "stop command" not in speech_wd.lower() and "watchdog" in speech_wd.lower()
      and "node heartbeat stale" in speech_wd, speech_wd)
speech_wd2 = gb_explore._report(plan, {"outcome": "aborted", "reason": "watchdog: node heartbeat stale"}, False)
check("direct watchdog reason is spoken accurately",
      "watchdog: node heartbeat stale" in speech_wd2 and "stop command" not in speech_wd2.lower(),
      speech_wd2)

# cleanup temps (not the live robot)
for d in (ARM_DIR, td, hbdir):
    shutil.rmtree(d, ignore_errors=True)

print("----")
print("%d passed, %d failed" % (P, F))
sys.exit(1 if F else 0)
