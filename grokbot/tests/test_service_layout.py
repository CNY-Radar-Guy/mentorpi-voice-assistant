"""The unit starts python on the release directory, with cwd somewhere else.

sys.path is not edited. The files copied by gb_safety_run.sh have to import.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sandbox
sandbox.install()
import os, re, shutil, subprocess, sys, textwrap, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(ROOT)

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

script = open(os.path.join(ROOT, "gb_safety", "gb_safety_run.sh")).read()
copied = re.findall(r'cp -f "\$TREE/([^"]+)"', script)
check("the launcher copies gb_paths into the release directory", "gb_paths.py" in copied, copied)
check("the launcher copies the release ONNX, not only the old host model",
      "gb_safety/yolov8n.onnx" in copied)
check("the launcher copies the shared decoder", "gb_yolo.py" in copied)
check("the launcher copies the bounded look", "gb_bounded.py" in copied)

code = tempfile.mkdtemp(prefix="gbrelease_")
for rel in copied:
    src = os.path.join(ROOT, rel)
    if not os.path.isfile(src):
        continue
    dst = os.path.join(code, os.path.basename(rel))
    shutil.copy2(src, dst)
# The node itself is gb_safety/gb_$WHAT.py. Copy the tracker the unit starts.
shutil.copy2(os.path.join(ROOT, "gb_safety", "gb_rgbguard.py"), os.path.join(code, "gb_rgbguard.py"))
iso = tempfile.mkdtemp(prefix="gbiso_")
missing = os.path.join(iso, "missing-scale.json")
runner = os.path.join(code, "layout_check.py")
open(runner, "w").write(textwrap.dedent("""\
    import gb_metric
    import gb_paths
    print("import-ok", gb_paths.__file__)
    status = gb_metric.depth_scale_status()
    empty = gb_metric.sighting_record(1.0, [])
    person = gb_metric.sighting_record(1.0, [{"conf": 0.81, "box": [268, 1, 384, 270]}])
    assert status["reason"] == "no_scale_record", status
    assert status["path"].endswith("missing-scale.json"), status
    assert empty["reason"] == "no_person", empty
    assert person["person"]["conf"] == 0.81 and "depth_m" not in person["person"]
    print("layout-ok")
"""))
env = os.environ.copy()
env.pop("PYTHONPATH", None)
env["GB_DEPTH_SCALE_FILE"] = missing
env["HEALTH_FILE"] = os.path.join(iso, "health.json")
env["GB_SHARED_DIR"] = iso
proc = subprocess.run([sys.executable, runner], cwd="/tmp", env=env, capture_output=True, text=True)
check("the release directory imports with cwd /tmp and no PYTHONPATH",
      proc.returncode == 0 and "import-ok" in proc.stdout and code in proc.stdout,
      proc.stderr[-500:] + proc.stdout[-300:])
check("an empty fixture is not the live scale record",
      proc.returncode == 0 and "layout-ok" in proc.stdout, proc.stdout[-300:])

# The regression: without gb_paths the same command fails.
bare = tempfile.mkdtemp(prefix="gbbare_")
for name in os.listdir(code):
    src = os.path.join(code, name)
    if name == "gb_paths.py" or not os.path.isfile(src):
        continue
    shutil.copy2(src, os.path.join(bare, name))
proc2 = subprocess.run([sys.executable, os.path.join(bare, "layout_check.py")], cwd="/tmp", env=env, capture_output=True, text=True)
check("omitting gb_paths is the live import error", proc2.returncode != 0 and "gb_paths" in (proc2.stderr + proc2.stdout))

print("test_service_layout: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
