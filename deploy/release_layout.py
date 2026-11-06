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
ORDER = ("gate", "watchdog", "voice")
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
    """systemd drop-in. Voice is the release tree; gate and watchdog are its launcher."""
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
        "ExecStart=\n"
        "ExecStart=/bin/bash %s %s\n" % (release_root, script, component)
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


def crash_rollback(state):
    """Restore pins for anything already switched. Voice is restarted, not left down.

    ESTOP and the maintenance lock are not in the delete set.
    """
    pins = dict(state.get("pins") or {})
    restart = []
    for name in state.get("switched") or []:
        prev = (state.get("previous") or {}).get(name)
        if prev:
            pins[name] = prev
        else:
            pins.pop(name, None)
        restart.append(name)
    if "voice" in (state.get("switched") or []) or state.get("step") == "voice":
        if "voice" not in restart:
            restart.append("voice")
    return {
        "pins": pins,
        "restart": restart,
        "voice_left_down": False,
        "clear_estop": False,
        "clear_maint": False,
        "zero_hold": True,
    }


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
    wake = vals.get("WAKE_WORDS", "wake up wally").lower()
    if "wake up wally" not in wake:
        return False
    return True
