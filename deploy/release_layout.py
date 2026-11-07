"""Stage a release beside the live tree and switch one process at a time.

The running voice imports gb_explore and the other modules from the directory
it was started in. Copying new files over that directory mixes versions.
A release lives under releases/<id>/ and a process uses it only after its
own systemd drop-in is installed. A crash restarts that same drop-in, so it
cannot pick up a newer tree by surprise.
"""
import os, shutil

SKIP_DIRS = {"data", "models", "voices", "__pycache__", "releases", "run"}
SKIP_FILES = {"secrets.env", "grokbot.env", "guard.env", "caps.json"}
ORDER = ("watchdog", "gate", "voice", "rgbguard", "health")
UNITS = {"gate": "gb-gate", "watchdog": "gb-watchdog", "voice": "grokbot",
         "rgbguard": "gb-rgbguard", "guard": "gb-guard", "health": "gb-health"}


def stage_release(src, dest_root, release_id):
    """Copy src into dest_root/releases/<id>. Do not modify the live modules."""
    if not release_id or "/" in release_id or release_id in (".", ".."):
        raise ValueError("bad release id")
    release = os.path.join(dest_root, "releases", release_id)
    if os.path.exists(release):
        raise FileExistsError(release)
    os.makedirs(release)

    def ignore(dirpath, names):
        rel = os.path.relpath(dirpath, src)
        if rel == ".":
            return [n for n in names if n in SKIP_DIRS or n in SKIP_FILES or n.endswith(".wav") or ".bak" in n]
        return [n for n in names if n in SKIP_DIRS or n in SKIP_FILES or n.endswith(".wav") or ".bak" in n]

    for name in os.listdir(src):
        if name in SKIP_DIRS or name in SKIP_FILES:
            continue
        src_path = os.path.join(src, name)
        dst_path = os.path.join(release, name)
        if os.path.isdir(src_path):
            shutil.copytree(src_path, dst_path, ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.wav", "*.bak*"))
        else:
            shutil.copy2(src_path, dst_path)
    with open(os.path.join(release, "VERSION"), "w") as f:
        f.write(release_id + "\n")
    return release


def live_module_untouched(dest_root, name="grokbot_voice.py"):
    """True when staging did not replace the live module."""
    live = os.path.join(dest_root, name)
    return os.path.isfile(live)


def dropin_text(component, release_root):
    """systemd drop-in. Voice is the release tree; gate and watchdog are its launcher.

    Actuation is not in ORDER. A normal activation does not switch it on.
    """
    if component == "actuation":
        script = os.path.join(release_root, "gb_safety", "gb_safety_run.sh")
        return (
            "[Service]\n"
            "Environment=GB_TREE=%s\n"
            "Environment=GB_RELEASE=%s\n"
            "Environment=GB_MAINT_FILE=/home/ubuntu/shared/gb_safety/MAINT\n"
            "Environment=GB_LEASE_FILE=/home/ubuntu/shared/gb_safety/lease.json\n"
            "Environment=GB_GATE_ENVELOPE=/home/ubuntu/shared/gb_safety/gate_envelope.json\n"
            "ExecStart=\n"
            "ExecStart=/bin/bash %s actuation\n" % (
                release_root, os.path.basename(release_root.rstrip("/")), script)
        )
    if component == "voice":
        script = os.path.join(release_root, "grokbot_voice.py")
        return (
            "[Service]\n"
            "WorkingDirectory=%s\n"
            "Environment=SUPPRESS_STARTUP_SPEECH=1\n"
            "Environment=GB_DATA=/home/pi/grokbot/data\n"
            "Environment=GB_DIR=/home/pi/grokbot\n"
            "Environment=GB_MODELS=/home/pi/grokbot/models\n"
            "Environment=GB_SHARED_ESTOP=/home/pi/docker/tmp/gb_safety/ESTOP\n"
            "Environment=PYTHONPATH=%s\n"
            "ExecStart=\n"
            "ExecStart=/home/pi/grokbot/venv/bin/python %s\n" % (release_root, release_root, script)
        )
    script = os.path.join(release_root, "gb_safety", "gb_safety_run.sh")
    return (
        "[Service]\n"
        "Environment=GB_TREE=%s\n"
        "Environment=GB_RELEASE=%s\n"
        "ExecStart=\n"
        "ExecStart=/bin/bash %s %s\n" % (release_root, os.path.basename(release_root.rstrip("/")), script, component)
    )


def begin_activation(release_id, previous_pins):
    return {
        "release": release_id,
        "previous": dict(previous_pins or {}),
        "pins": dict(previous_pins or {}),
        "switched": [],
        "step": "zero_hold",
        "failed": False,
    }


def switch_component(state, component):
    if component not in ORDER:
        raise ValueError(component)
    expect = ORDER[len(state["switched"])]
    if component != expect:
        raise ValueError("expected %s before %s" % (expect, component))
    state["pins"][component] = state["release"]
    state["switched"].append(component)
    state["step"] = component
    return state


def restart_one_at_a_time(names):
    """Recovery order. The watchdog stays up until the gate has been restored."""
    priority = ("health", "rgbguard", "voice", "gate", "watchdog")
    return [name for name in priority if name in set(names or [])]


def crash_rollback(state):
    """Restore pins for anything already switched. One restart per step.

    ESTOP and the maintenance lock are not in the delete set. Gate and
    watchdog are never restarted in the same step.
    """
    pins = dict(state.get("pins") or {})
    switched = list(state.get("switched") or [])
    if state.get("step") == "voice" and "voice" not in switched:
        switched.append("voice")
    for name in switched:
        prev = (state.get("previous") or {}).get(name)
        if prev:
            pins[name] = prev
        else:
            pins.pop(name, None)
    restart = restart_one_at_a_time(switched)
    return {
        "pins": pins,
        "restart": restart,
        "voice_left_down": False,
        "clear_estop": False,
        "clear_maint": False,
        "zero_hold": True,
        "paired_safety_restart": False,
    }


def code_dir(state_dir, release_id):
    """Private code folder. The legacy state directory is not a code target."""
    if not release_id:
        return state_dir
    parent = os.path.dirname(state_dir.rstrip("/"))
    return os.path.join(parent, "gb_code", release_id)


def watchdog_status(maint_present, release, now):
    return {"t": float(now), "maint": bool(maint_present), "zeroing": bool(maint_present), "release": release or ""}


def watchdog_confirmed(body, release, now, max_age=2.0):
    """True only for a fresh ack from this release that maintenance is held and zeros are being sent."""
    if not isinstance(body, dict) or body.get("release") != release:
        return False
    if body.get("maint") is not True or body.get("zeroing") is not True:
        return False
    try:
        age = float(now) - float(body["t"])
    except (KeyError, TypeError, ValueError):
        return False
    return 0 <= age <= float(max_age)


def versions_consistent(pins, running):
    """running maps a component to the release id its process is executing."""
    for name, release in (running or {}).items():
        pinned = (pins or {}).get(name)
        if pinned and release != pinned:
            return False
    return True


def voice_restart_env_ok(env_text):
    vals = {}
    for line in (env_text or "").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        vals[k.strip()] = v.strip().strip('"').strip("'")
    if vals.get("SUPPRESS_STARTUP_SPEECH") != "1":
        return False
    if vals.get("ESTOP_AUTOCLEAR_S", "0") != "0":
        return False
    if vals.get("ALLOW_AUTONOMY", "0") == "1":
        return False
    wake = vals.get("WAKE_WORDS", "").strip().strip('"').strip("'").lower()
    if wake != "wake up wally":
        return False
    if vals.get("WAKE_ANYWHERE", "").strip():
        return False
    fuzz = vals.get("WAKE_FUZZ", "1.0").strip()
    try:
        fuzz_n = float(fuzz) if fuzz else 1.0
    except ValueError:
        return False
    if fuzz_n != 1.0:
        return False
    return True
