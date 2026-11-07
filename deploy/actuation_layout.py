"""How gb-actuation is allowed to be installed. No systemd and no motion.

The host unit used to run /usr/bin/python3 on the node. That interpreter has
no rclpy. The container already starts gate and watchdog through
gb_safety_run.sh and rosrun.sh, from releases/<id>, and reads MAINT at
/home/ubuntu/shared/gb_safety/MAINT. Actuation has to use that same path.
"""
import os

CONTAINER_MAINT = "/home/ubuntu/shared/gb_safety/MAINT"
HOST_MAINT_MIRROR = "/home/pi/grokbot/data/MAINT"


def _exec_lines(text):
    lines = []
    for line in (text or "").splitlines():
        body = line.split("#", 1)[0].strip()
        if body.startswith("ExecStart"):
            lines.append(body)
    return lines


def unit_text_ok(text):
    """The unit must enter the container launcher, not host Python."""
    if not text:
        return False
    execs = _exec_lines(text)
    joined = "\n".join(execs)
    if not execs or "/usr/bin/python3" in joined:
        return False
    if "/home/pi/grokbot/gb_actuation_node.py" in joined:
        return False
    if "GB_MAINT_FILE=/home/pi/" in text.split("#")[0]:
        return False
    if "GB_CONTROLLER_REMAPPED" in text:
        return False
    return "gb_safety_run.sh actuation" in joined


def launcher_text_ok(text):
    """The shell must refuse a flat tree and pin the container hold."""
    if not text:
        return False
    needed = (
        "actuation refused without GB_TREE",
        "gb_actuation_node.py",
        "rosrun.sh",
        "GB_MAINT_FILE=/home/ubuntu/shared/gb_safety/MAINT",
        "GB_LEASE_FILE=/home/ubuntu/shared/gb_safety/lease.json",
        "GB_GATE_ENVELOPE=/home/ubuntu/shared/gb_safety/gate_envelope.json",
    )
    return all(part in text for part in needed)


def dropin_ok(text, release_root):
    if "/releases/" not in release_root:
        return False
    if not text:
        return False
    return (
        "GB_TREE=%s" % release_root in text
        and "gb_safety_run.sh actuation" in text
        and "GB_MAINT_FILE=%s" % CONTAINER_MAINT in text
        and "/usr/bin/python3" not in text
    )


def activation_mentions_supervised(text):
    """Ordinary activation must say it did not start this unit or the controller."""
    if "systemctl start gb-actuation" in text or "systemctl restart gb-actuation" in text:
        return False
    if "systemctl enable gb-actuation" in text or "systemctl enable --now gb-actuation" in text:
        return False
    return "gb-actuation was not started" in text and "controller was not restarted" in text


def files_agree(root):
    unit = open(os.path.join(root, "deploy", "gb-actuation.service")).read()
    launcher = open(os.path.join(root, "grokbot", "gb_safety", "gb_safety_run.sh")).read()
    activate = open(os.path.join(root, "deploy", "wally_activate.sh")).read()
    if not unit_text_ok(unit):
        return False, "unit still starts host python or a host MAINT path"
    if not launcher_text_ok(launcher):
        return False, "launcher does not refuse a flat tree or does not pin the container MAINT path"
    if not activation_mentions_supervised(activate):
        return False, "activation would start gb-actuation or restart the controller"
    return True, "unit uses the container launcher; activation does not start it; MAINT stays %s in the container (host mirror %s is not the path the node opens)" % (CONTAINER_MAINT, HOST_MAINT_MIRROR)
