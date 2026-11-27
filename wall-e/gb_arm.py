"""gb_arm.py - wake-phrase movement arming with disarm provenance and mission lease (John, 2026-11-26).

Wally stays DISARMED (no motion of any kind) until John says "wake up Wally", "hey Wally", or "Wally wake up". The armed flag is the file
ARMED in the shared gb_safety dir (host /home/pi/docker/tmp/gb_safety, container /home/ubuntu/shared/gb_safety).
- Boot / gb-armguard start / voice start: disarmed (file removed).
- Spoken "wake up Wally" / "hey Wally" / "Wally wake up" (voice main loop) -> arm. Spoken stop / e-stop -> disarm. ESTOP or MAINT present -> not armed
  (and gb-armguard deletes the flag). 2026-11-25 noestop: on the FLOOR (mode != bench) latch files never block arming;
  arm() archives any leftover one (cause logged) and arms.
- Refusal points: gb_estop.gate_request (every host motion request), gb_wakegate.block (every voice/web motion intent),
  gb_explore.request + its container publisher, and the gb-armguard backstop node (stops anything reaching the gate
  while disarmed). gb_gate itself is untouched.
- 2026-11-26 provenance: every arm/disarm is tagged with an actor (voice, jarvis, remote_script, safety, other).
  A MISSION_LEASE records the owning actor + pid while a mission runs. guarded_disarm() refuses a non-emergency
  remote_script (or unknown) cleanup while a live-PID lease is owned by someone else. voice and safety stops always
  bypass the lease. A stop / disarm / e-stop immediately clears MISSION_LEASE so a late background
  twist cannot restart motion until a fresh lease is granted. DISARM_EVENT + arm_events.jsonl store the
  last event for session logging. Plain ARMED file existence remains the armed flag so gb-armguard and
  gb-gate keep working. gb_gate also revokes the lease and holds streams until a new lease appears.
CLI: python3 gb_arm.py status|arm|disarm   (arm from the CLI is for no-motion tests only)
"""
import json, os, re, sys, time

ACTORS = ("voice", "jarvis", "remote_script", "safety", "other")
# Non-emergency actors that must not steal another actor's live mission lease.
LEASE_BLOCKED_ACTORS = ("remote_script", "other")

def _dir():
    d = os.environ.get("GB_ARM_DIR")
    if d: return d
    for d in ("/home/pi/docker/tmp/gb_safety", "/home/ubuntu/shared/gb_safety"):
        if os.path.isdir(d): return d
    return "/home/pi/docker/tmp/gb_safety"

def path(): return os.path.join(_dir(), "ARMED")
def lease_path(): return os.path.join(_dir(), "MISSION_LEASE")
def event_path(): return os.path.join(_dir(), "DISARM_EVENT")
def events_log_path(): return os.path.join(_dir(), "arm_events.jsonl")

def _floor():
    """2026-11-25 noestop: on the floor (mode != bench) no ESTOP/MAINT file ever blocks arming."""
    try:
        import gb_floor
        return gb_floor.no_latch()
    except Exception:
        return True

def _latched():
    if _floor():
        return False
    d = _dir()
    return any(os.path.exists(os.path.join(d, f)) for f in ("ESTOP", "MAINT"))

def _floor_release(src):
    """Floor: archive (never delete) any leftover latch file so it cannot block 'wake up Wally'. Cause is logged."""
    if not _floor(): return []
    out = []
    try:
        import gb_floor
        host = os.path.join(os.environ.get("GB_DATA") or "/home/pi/grokbot/data")
        for p in (os.path.join(_dir(), "ESTOP"), os.path.join(_dir(), "MAINT"),
                  os.path.join(host, "ESTOP"), os.path.join(host, "MAINT")):
            if os.path.lexists(p):
                dst = gb_floor.archive(p, "ground mode: arm request (%s)" % src)
                if dst: out.append(dst); _log(f"floor: archived leftover latch {p} -> {dst}")
    except Exception as ex:
        _log(f"floor release failed: {ex}")
    return out

WAKE_RE = re.compile(r"(?:\bwake[\s,.!-]*up[\s,.!-]*wally\b|\bhey[\s,.!-]*wally\b|\bwally[\s,.!-]*wake[\s,.!-]*up\b)", re.I)

def _log(msg):
    print(time.strftime("%H:%M:%S"), "[ARM]", msg, flush=True)
    try:
        with open(os.path.join(_dir(), "arm.log"), "a") as f: f.write(time.strftime("%Y-%m-%d %H:%M:%S ") + msg + "\n")
    except Exception: pass

def _write_json(p, obj):
    os.makedirs(os.path.dirname(p) or ".", exist_ok=True)
    tmp = p + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
        f.write("\n")
    os.replace(tmp, p)

def _read_json(p):
    try:
        return json.load(open(p))
    except Exception:
        return None

def _append_event_log(rec):
    try:
        with open(events_log_path(), "a") as f:
            f.write(json.dumps(rec) + "\n")
    except Exception:
        pass

def normalize_actor(actor, why=""):
    a = (actor or "").strip().lower().replace("-", "_").replace(" ", "_")
    aliases = {
        "voice_command": "voice", "stt": "voice", "wake": "voice",
        "api": "jarvis", "jarvis_api": "jarvis",
        "remote": "remote_script", "test_script": "remote_script", "cleanup": "remote_script",
        "script": "remote_script", "test_cleanup": "remote_script",
        "estop": "safety", "e_stop": "safety", "watchdog": "safety", "armguard": "safety",
        "floor": "safety", "emergency": "safety",
    }
    a = aliases.get(a, a)
    if a in ACTORS:
        return a
    return infer_actor(why or actor or "")

def infer_actor(src_or_why):
    """Best-effort actor from a reason/src string. Never treats a bare 'test' as remote_script
    (existing unit tests call disarm('test') / disarm('t'))."""
    s = (src_or_why or "").strip().lower()
    if not s:
        return "other"
    if "remote_script" in s or "test script" in s or "cleanup finally" in s or "cleanup:" in s:
        return "remote_script"
    if "jarvis" in s:
        return "jarvis"
    if "estop" in s or "e-stop" in s or "maint" in s or "watchdog" in s or "armguard" in s \
            or "floor normal" in s or "tilt" in s or "safety" in s or "emergency" in s:
        return "safety"
    if "stop phrase" in s or "stop intent" in s or s.startswith("voice") or ": 'wake up" in s \
            or 'voice:' in s or s.startswith("voice "):
        return "voice"
    return "other"

def is_armed():
    return os.path.exists(path()) and not _latched()

def pid_alive(pid):
    try:
        pid = int(pid)
    except (TypeError, ValueError):
        return False
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False
    except Exception:
        return False

def read_lease():
    rec = _read_json(lease_path())
    return rec if isinstance(rec, dict) else None

def lease_is_live(lease=None):
    """True only when a lease exists and its owner pid is still alive. Stale/dead PIDs never lock disarm."""
    lease = lease if lease is not None else read_lease()
    if not lease:
        return False
    return pid_alive(lease.get("pid"))

def set_mission_lease(owner, pid=None, reason=None, mission=None):
    rec = {
        "owner": normalize_actor(owner),
        "pid": int(pid) if pid not in (None, "", 0) else os.getpid(),
        "started_at": time.time(),
        "reason": reason,
        "mission": mission,
    }
    _write_json(lease_path(), rec)
    _log("MISSION LEASE set owner=%s pid=%s mission=%s" % (rec["owner"], rec["pid"], rec["mission"]))
    return rec

def clear_mission_lease(why=""):
    p = lease_path()
    was = os.path.exists(p)
    try:
        os.remove(p)
    except OSError:
        pass
    if was:
        _log("MISSION LEASE cleared (%s)" % (why or "unspecified"))
    return was

# 2026-11-26 research takeaway 3: stream command freshness (gate DEADMAN is 0.5 s).
COMMAND_DEADMAN_S = 0.5
COMMAND_DEADMAN_MIN_S = 0.3
COMMAND_DEADMAN_MAX_S = 0.5

def command_watchdog_expired(now, t_fresh, deadman=COMMAND_DEADMAN_S):
    """True when a stream twist is older than the 0.3-0.5 s command freshness watchdog."""
    try:
        return (float(now) - float(t_fresh)) > float(deadman)
    except (TypeError, ValueError):
        return True

def why_revokes_lease(why):
    """Stop / disarm / e-stop reasons revoke the mission lease; normal completions do not."""
    w = (why or "").lower()
    keys = ("stop request", "estop", "fault estop", "tilt estop", "floor stop",
            "maintenance lock", "floor hold", "remote estop", "disarm")
    return any(k in w for k in keys)

def _pid_visible(pid):
    """True when this process can observe pid (same PID namespace).

    gb-gate runs inside the MentorPi container and cannot see the host grokbot PID
    that set_mission_lease() records (os.kill and /proc/<host-pid> both fail).
    """
    try:
        pid = int(pid)
    except (TypeError, ValueError):
        return False
    if pid <= 0:
        return False
    if os.path.exists("/proc/%d" % pid):
        return True
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False
    except Exception:
        return False

def fresh_lease_since(t0, lease=None):
    """True when a MISSION_LEASE was granted after t0 (a new mission may stream again).

    A real stop / estop / disarm deletes the lease file and latches a hold timestamp,
    so a leftover pre-stop lease (started_at <= t0) can never lift the hold.

    The owner PID is the host grokbot process. gb-gate evaluates this helper inside
    the MentorPi container, where that host PID is not visible — that is why a valid
    21:34:25 ET lease was refused at 21:34:33 as "mission lease revoked; waiting for
    a fresh lease". If the PID is visible in this namespace, it must still be alive;
    if it is not visible, started_at > t0 on a present lease file is sufficient.
    """
    lease = lease if lease is not None else read_lease()
    if not lease:
        return False
    try:
        started = float(lease.get("started_at") or 0)
    except (TypeError, ValueError):
        return False
    try:
        t0 = float(t0)
    except (TypeError, ValueError):
        return False
    if started <= t0:
        return False
    pid = lease.get("pid")
    if _pid_visible(pid):
        return pid_alive(pid)
    return True

# Seconds to keep refusing leftover mission /gb/cmd_in twists after a revoke,
# before an armed manual drive (no active lease) may resume. Longer than the
# 0.5 s stream deadman so a just-stopped explore node cannot sneak a pulse through.
LEASE_HOLD_DRAIN_S = 1.0

def stream_allowed_after_hold(hold, hold_t, now=None, joystick=False, lease=None, armed=None):
    """After a stop-hold, a fresh mission lease may stream again.

    A stale hold from an earlier ended or revoked mission must not block armed
    manual drive on /gb/cmd_in when no mission is active. A just-revoked stream
    is still blocked for LEASE_HOLD_DRAIN_S, unless ARMED was rewritten after
    the hold (a fresh wake/arm). Stop/estop/disarm still revoke immediately.
    Obstacle stops, the 0.5 s deadman, and armed/estop checks are unchanged.
    """
    if joystick:
        return True
    if not hold:
        return True
    if now is None:
        now = time.time()
    if fresh_lease_since(hold_t, lease=lease):
        return True
    # A leftover pre-stop lease (not fresh) must keep the hold.
    if lease is None:
        lease = read_lease()
    if lease:
        return False
    if armed is None:
        armed = is_armed()
    if not armed:
        return False
    try:
        hold_t = float(hold_t or 0.0)
        now = float(now)
    except (TypeError, ValueError):
        return False
    try:
        armed_t = os.path.getmtime(path())
    except OSError:
        armed_t = 0.0
    if armed_t > hold_t:
        return True
    return (now - hold_t) >= LEASE_HOLD_DRAIN_S

def last_event():
    rec = _read_json(event_path())
    return rec if isinstance(rec, dict) else None

def session_abort_fields(event=None, lease=None):
    """Fields to store on a session JSONL abort record."""
    ev = event if event is not None else (last_event() or {})
    ls = lease if lease is not None else read_lease()
    if not ev and not ls:
        return {
            "disarm_actor": None,
            "disarm_reason": None,
            "mission_owner": None,
            "external_disarm_conflict": False,
        }
    owner = (ls or {}).get("owner") or ev.get("mission_owner")
    actor = ev.get("actor")
    conflict = bool(ev.get("external_disarm_conflict"))
    if ev and actor and actor != "voice" and owner == "voice" and ev.get("kind") in ("disarm", "disarm_blocked"):
        conflict = True
    return {
        "disarm_actor": actor,
        "disarm_reason": ev.get("reason"),
        "mission_owner": owner,
        "external_disarm_conflict": bool(conflict),
        "disarm_blocked": bool(ev.get("blocked")),
        "disarm_ts": ev.get("ts") or ev.get("t"),
    }

def state():
    try: j = json.load(open(path()))
    except Exception: j = None
    return {
        "armed": is_armed(),
        "flag": j,
        "latched": _latched(),
        "lease": read_lease(),
        "lease_live": lease_is_live(),
        "last_event": last_event(),
    }

def arm(src, actor=None):
    _floor_release(src)                               # 2026-11-25 noestop: on the floor nothing blocks re-arming
    if _latched():
        _log(f"arm refused ({src}): ESTOP/MAINT latched"); return False
    actor = normalize_actor(actor, src)
    payload = {"t": time.time(), "src": src, "actor": actor}
    tmp = path() + ".tmp"
    with open(tmp, "w") as f: json.dump(payload, f)
    os.replace(tmp, path()); _log(f"ARMED by {src} actor={actor}"); return True

def maybe_arm(text, src="voice"):
    """Arm on 'wake up Wally', 'hey Wally', or 'Wally wake up'."""
    if text and WAKE_RE.search(text):
        actor = "voice" if (src or "").startswith("voice") else infer_actor(src)
        return arm(f"{src}: {text[:80]!r}", actor=actor)
    return False

def _record_event(rec):
    try:
        _write_json(event_path(), rec)
    except Exception as ex:
        _log("DISARM_EVENT write failed: %s" % ex)
    _append_event_log(rec)

def disarm_result(why, actor=None, emergency=False, force=False):
    """Structured disarm (or blocked attempt). Always records provenance."""
    actor = normalize_actor(actor, why)
    was = os.path.exists(path())
    lease = read_lease()
    live = lease_is_live(lease)
    owner = (lease or {}).get("owner")
    bypass = bool(emergency or force or actor in ("voice", "safety"))
    blocked = False
    if live and not bypass and actor in LEASE_BLOCKED_ACTORS and actor != owner:
        blocked = True
    conflict = bool(live and owner == "voice" and actor not in ("voice", "safety") and not emergency)
    rec = {
        "t": time.time(),
        "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "kind": "disarm_blocked" if blocked else "disarm",
        "actor": actor,
        "reason": why,
        "was_armed": was,
        "mission_owner": owner,
        "lease": lease,
        "external_disarm_conflict": conflict,
        "blocked": blocked,
        "emergency": bool(emergency or actor == "safety"),
        "disarmed": False,
    }
    if blocked:
        _record_event(rec)
        _log("DISARM BLOCKED actor=%s why=%s owner=%s pid=%s" % (
            actor, why, owner, (lease or {}).get("pid")))
        rec["ok"] = False
        rec["disarmed"] = False
        return rec
    try:
        os.remove(path())
    except OSError:
        pass
    clear_mission_lease(why)
    if was:
        _log("DISARMED: %s actor=%s" % (why, actor))
    rec["disarmed"] = not os.path.exists(path())
    rec["ok"] = True
    rec["removed"] = was
    _record_event(rec)
    return rec

def guarded_disarm(why, actor=None, emergency=False, force=False):
    """Lease-aware disarm. Returns True if an ARMED flag was removed."""
    rec = disarm_result(why, actor=actor, emergency=emergency, force=force)
    return bool(rec.get("removed"))

def disarm(why, actor=None, emergency=False, force=False):
    """Backward compatible: returns whether an ARMED flag was removed. Records provenance."""
    return guarded_disarm(why, actor=actor, emergency=emergency, force=force)

REFUSE_LINE = "I'm staying still until you say 'wake up Wally'."

def refuse(kind, detail=""):
    _log(f"refused {kind} while disarmed: {str(detail)[:160]!r}")
    return REFUSE_LINE

if __name__ == "__main__":
    c = sys.argv[1] if len(sys.argv) > 1 else "status"
    if c == "arm": print(arm("cli test", actor="other"))
    elif c == "disarm":
        act = sys.argv[2] if len(sys.argv) > 2 else "other"
        print(json.dumps(disarm_result("cli", actor=act)))
    print(json.dumps(state()))
