"""Release-tree code must write stops into the live data directory."""
import importlib, json, os, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

root = tempfile.mkdtemp(prefix="gbdata_")
shared = os.path.join(tempfile.mkdtemp(prefix="gbshare_"), "ESTOP")
os.environ["GB_DATA"] = root
os.environ["GB_SHARED_ESTOP"] = shared
os.environ["GB_MODELS"] = os.path.join(root, "models")
import gb_paths
importlib.reload(gb_paths)
import gb_estop
importlib.reload(gb_estop)
import gb_maint
importlib.reload(gb_maint)
gb_estop.zero_now = lambda *a, **k: None
gb_estop.estop("voice", "stop")
host = os.path.join(root, "ESTOP")
check("voice stop is in the canonical data dir", os.path.isfile(host) and "/releases/" not in host, host)
check("the same stop is on the container path", os.path.isfile(shared), shared)
check("both copies match", open(host, "rb").read() == open(shared, "rb").read())
body = json.load(open(host))
check("it is a normal stop", body.get("kind") == "normal" and body.get("src") == "voice", body)
check("speaking flag uses that data dir", gb_paths.speaking_flag() == os.path.join(root, "speaking.json"))
check("models stay out of the release tree", gb_paths.models_dir() == os.path.join(root, "models"))
check("maintenance uses the same data dir", gb_maint.default_path() == os.path.join(root, "MAINT"))
gb_estop.clear("test")
check("clear removes both copies", not os.path.exists(host) and not os.path.exists(shared))
text = open(os.path.join(os.path.dirname(HERE), "gb_safety", "gb_safety_run.sh")).read()
guard = open(os.path.join(os.path.dirname(HERE), "gb_guard_run.sh")).read()
check("launchers load mount_check from the release tree",
      '"$TREE/mount_check.py"' in text and '"$TREE/mount_check.py"' in guard)
check("host estop mirror path stays the live data dir", "HOST_ESTOP=$GB/data/ESTOP" in text)

print("test_paths: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
