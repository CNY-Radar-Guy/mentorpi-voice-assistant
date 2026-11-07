"""Colcon egg-link layout from the Pi. No robot and no restart.

The source file is regular. build/<pkg>/<pkg> is a symlink to that source.
The site-packages module path is absent. The console script is not the module.
"""
import os, shutil, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
from deploy.controller_patch import (EGG_REL, ORIGINAL_SHA256, SITE_REL, SRC_REL,
                                     install_layout, patched_sha256, read_vendor,
                                     rollback_file, sha256_bytes)

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
    console = os.path.join(ws, "install/ros_robot_controller/lib/ros_robot_controller/ros_robot_controller")
    os.makedirs(os.path.dirname(console))
    with open(console, "w") as handle:
        handle.write("#!/usr/bin/python3\n# EASY-INSTALL-ENTRY-SCRIPT\n")
    return src, link, egg, console

ws = os.path.join(os.environ["GB_SHARED_DIR"], "ros2_ws")
src, link, egg, console = build(ws, read_vendor())
console_bytes = open(console, "rb").read()
egg_bytes = open(egg, "rb").read()
link_target = os.readlink(link)
dry = install_layout(ws, apply=False)
check("absent site-packages does not fail the plan",
      dry["ok"] and dry["reason"] == "ready" and dry["changed"] is False and any(path.endswith(SITE_REL) for path in dry["absent"]),
      dry)
check("egg-link and source are one file",
      len(dry["files"]) == 1 and len(dry["files"][0]["aliases"]) == 2 and dry["files"][0]["realpath"] == os.path.realpath(src))
bak = os.path.join(os.environ["GB_SHARED_DIR"], "bak")
done = install_layout(ws, apply=True, backup_dir=bak)
check("one write lands on the regular source",
      done["ok"] and done["writes"] == [os.path.realpath(src)] and done["restart"] is False and done["clear_maint"] is False
      and sha256_bytes(open(src, "rb").read()) == patched_sha256())
check("the package symlink, egg-link, and console script stay as they were",
      os.path.islink(link) and os.readlink(link) == link_target and not os.path.islink(src)
      and open(egg, "rb").read() == egg_bytes and open(console, "rb").read() == console_bytes)
check("the backup is the snapshot and a second apply does not rewrite",
      sha256_bytes(open(done["backup"], "rb").read()) == ORIGINAL_SHA256
      and install_layout(ws, apply=True, backup_dir=bak)["reason"] == "already_patched"
      and install_layout(ws, apply=True, backup_dir=bak)["changed"] is False)
rolled = rollback_file(src)
check("rollback still refuses the original writer",
      rolled["restored"] is False and sha256_bytes(open(src, "rb").read()) == patched_sha256())

ws2 = os.path.join(os.environ["GB_SHARED_DIR"], "ros2_ws_bad")
src2, link2, _egg2, _console2 = build(ws2, read_vendor())
before = open(src2, "rb").read()
site = os.path.join(ws2, SITE_REL)
os.makedirs(os.path.dirname(site))
with open(site, "w") as handle:
    handle.write("not the controller\n")
refused = install_layout(ws2, apply=True, backup_dir=os.path.join(ws2, "bak"))
check("an unknown hash writes nothing, including the good source",
      refused["ok"] is False and refused["reason"] == "hash_mismatch" and refused["writes"] == []
      and open(src2, "rb").read() == before and open(site).read() == "not the controller\n"
      and os.path.islink(link2) and not os.path.isdir(os.path.join(ws2, "bak")))

env = dict(os.environ)
env["GB_CONTROLLER_PATCH_APPLY"] = "1"
env["GB_ROS_WS"] = ws
env["GB_PATCH_BACKUP"] = bak
proc = subprocess.run(["sh", os.path.join(ROOT, "deploy", "controller_patch_install.sh")],
                      cwd="/tmp", env=env, capture_output=True, text=True)
check("the installer apply on an already patched tree does not restart",
      proc.returncode == 0 and "already_patched" in proc.stdout and "was not restarted" in proc.stdout
      and "MAINT was not removed" in proc.stdout, proc.stdout[-500:] + proc.stderr[-300:])
unset = dict(os.environ)
unset.pop("GB_ROS_WS", None)
unset["GB_CONTROLLER_PATCH_APPLY"] = "0"
dry_sh = subprocess.run(["sh", os.path.join(ROOT, "deploy", "controller_patch_install.sh")],
                        cwd="/tmp", env=unset, capture_output=True, text=True)
check("no workspace means no write",
      dry_sh.returncode == 0 and "No file written" in dry_sh.stdout, dry_sh.stdout[-300:])

print("test_controller_layout: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
