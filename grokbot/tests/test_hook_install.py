"""Hook staging is part of the controller install. No robot and no restart.

A missing module or an unknown controller hash writes nothing. A full apply
stages the five hook modules, proves a fresh interpreter can import them
without rclpy or the controller class, and only then patches the source.
"""
import os, shutil, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
from deploy.controller_patch import (EGG_REL, HOOK_FILES, ORIGINAL_SHA256, SRC_REL,
                                     install_controller, patched_sha256, read_vendor,
                                     sha256_bytes)

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

def build(ws, source_bytes):
    if os.path.isdir(ws):
        shutil.rmtree(ws)
    src = os.path.join(ws, SRC_REL)
    os.makedirs(os.path.dirname(src))
    with open(src, "wb") as handle:
        handle.write(source_bytes)
    pkg = os.path.dirname(src)
    link = os.path.join(ws, "build", "ros_robot_controller", "ros_robot_controller")
    os.makedirs(os.path.dirname(link))
    os.symlink(pkg, link)
    egg = os.path.join(ws, EGG_REL)
    os.makedirs(os.path.dirname(egg))
    with open(egg, "w") as handle:
        handle.write(os.path.join(ws, "build", "ros_robot_controller") + "\n.\n")
    return src, link

base = os.environ["GB_SHARED_DIR"]
ws = os.path.join(base, "hook_ws")
hook = os.path.join(base, "gb_safety", "gb_hook")
src, link = build(ws, read_vendor())
link_target = os.readlink(link)
before = open(src, "rb").read()
dry = install_controller(ws, hook, ROOT, apply=False, backup_dir=os.path.join(base, "hook_bak"))
check("dry run proves the hook and does not create it",
      dry["ok"] and dry["reason"] == "ready" and dry["changed"] is False and not os.path.isdir(hook)
      and open(src, "rb").read() == before and list(dry["hook"]) == list(HOOK_FILES), dry.get("detail", dry))

bare = os.path.join(base, "bare_pkg")
os.makedirs(os.path.join(bare, "grokbot"))
for name in HOOK_FILES:
    if name == "gb_estop.py":
        continue
    shutil.copy(os.path.join(ROOT, "grokbot", name), os.path.join(bare, "grokbot", name))
missing = install_controller(ws, hook, bare, apply=True, backup_dir=os.path.join(base, "hook_bak"))
check("a missing hook module writes neither the hook nor the controller",
      missing["ok"] is False and missing["reason"] == "hook_missing" and "gb_estop.py" in missing["missing"]
      and not os.path.isdir(hook) and open(src, "rb").read() == before)

bad = os.path.join(base, "bad_ws")
bad_src, _bad_link = build(bad, b"not the vendor node\n")
bad_hook = os.path.join(base, "bad_hook")
refused = install_controller(bad, bad_hook, ROOT, apply=True, backup_dir=os.path.join(base, "bad_bak"))
check("an unknown controller hash does not stage the hook",
      refused["ok"] is False and refused["reason"] == "hash_mismatch" and not os.path.isdir(bad_hook)
      and open(bad_src, "rb").read() == b"not the vendor node\n")

done = install_controller(ws, hook, ROOT, apply=True, backup_dir=os.path.join(base, "hook_bak"))
check("full install stages the hook and then patches the one source inode",
      done["ok"] and done["reason"] == "installed" and done["restart"] is False and done["clear_maint"] is False
      and done["writes"] == [os.path.realpath(src)] and sha256_bytes(open(src, "rb").read()) == patched_sha256()
      and os.path.islink(link) and os.readlink(link) == link_target, done)
for name in HOOK_FILES:
    staged = open(os.path.join(hook, name), "rb").read()
    packaged = open(os.path.join(ROOT, "grokbot", name), "rb").read()
    check("staged " + name, staged == packaged)
again = install_controller(ws, hook, ROOT, apply=True, backup_dir=os.path.join(base, "hook_bak"))
check("a second install does not rewrite",
      again["ok"] and again["reason"] == "already_complete" and again["changed"] is False
      and sha256_bytes(open(os.path.join(base, "hook_bak", "ros_robot_controller_node.py.gb-orig"), "rb").read()) == ORIGINAL_SHA256)

env = {
    "PATH": os.environ.get("PATH", "/usr/bin"),
    "PYTHONDONTWRITEBYTECODE": "1",
    "GB_LEASE_FILE": os.path.join(hook, "no-lease.json"),
    "GB_GATE_ENVELOPE": os.path.join(hook, "no-envelope.json"),
    "GB_MAINT_FILE": os.path.join(hook, "no-MAINT"),
    "GB_SHARED_DIR": hook,
    "GB_DATA": hook,
    "GB_SHARED_ESTOP": os.path.join(hook, "no-ESTOP"),
    "GB_DIR": hook,
}
proc = subprocess.run([sys.executable, "-c",
                      "import sys; sys.path.insert(0, sys.argv[1]); "
                      "import gb_actuation, gb_intent, gb_motor_cap, gb_paths, gb_estop; "
                      "assert 'rclpy' not in sys.modules; print('import_ok')", hook],
                     env=env, capture_output=True, text=True)
check("a fresh interpreter imports the staged hook without rclpy",
      proc.returncode == 0 and "import_ok" in proc.stdout, proc.stderr[-400:])

print("test_hook_install: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
