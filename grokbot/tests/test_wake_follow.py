"""Exact wake stays exact. A cut-off capture is finished, not accepted early."""
import json, os, sys, tempfile, time
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import gb_demand
import gb_wally

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

check("wake up alone is not a wake", gb_wally.software_wake("Wake up.") is False)
check("wake up is only a prefix", gb_wally.wake_prefix("Wake up.") is True)
check("the full phrase is not a prefix", gb_wally.wake_prefix("wake up Wally") is False)
joined = gb_wally.join_wake_text("Wake up.", "Wally")
check("the stitched phrase is the exact wake", gb_wally.software_wake(joined) is True, joined)
check("a wrong continuation is still not a wake", gb_wally.software_wake(gb_wally.join_wake_text("Wake up.", "please")) is False)
check("hello is not a wake", gb_wally.software_wake("Hello Wally", hardware_event=True) is False)

def cut_read(parts):
    data = {"b": b"".join(parts), "i": 0, "hits": 0}
    def read(n):
        if data["hits"] == 0:
            data["hits"] = 1
            raise InterruptedError()
        take = data["b"][data["i"]:data["i"] + 1]
        data["i"] += len(take)
        return take
    return read

frame = gb_wally.read_pcm_frame(cut_read([b"abcd"]), 4)
check("an interrupted read still returns the whole frame", frame == b"abcd", frame)

now = time.time()
tmp = tempfile.mkdtemp(prefix="gbwake_")
person = os.path.join(tmp, "person.json")
demand = os.path.join(tmp, "demand.json")
health = os.path.join(tmp, "health.json")
wd = os.path.join(tmp, "wd.json")
os.environ["PERSON_TRACK_FILE"] = person
os.environ["GB_DEMAND_FILE"] = demand
os.environ["HEALTH_FILE"] = health
os.environ["WD_STATE_HOST"] = wd
json.dump({"t": now, "mode": "lidar", "lidar_online": True, "lidar_valid_frac": 0.98, "user_lidar": True}, open(health, "w"))
json.dump({"t": now, "maint": False, "zeroing": False}, open(wd, "w"))
json.dump({"person": {"t": now, "conf": 0.4, "depth_m": 2.2, "source": "depth", "unit": "documented_mm", "time_base": "capture"}}, open(person, "w"))
gb_wally.set_autonomy(False)
said = gb_wally.handle("follow me")
check("follow without a fresh person does not move or enable autonomy",
      gb_wally._autonomy is False and "not moved" in said and "camera stays on" in said, said)
fresh_now = time.time()
check("the camera stays in follow, not a 20 second acquire", gb_demand.read_demand(demand, fresh_now) == "follow")
check("follow is still on after the acquire window", gb_demand.read_demand(demand, fresh_now + 25) == "follow")
gb_demand.write_demand(demand, "acquire", now=fresh_now)
check("acquire still expires at 20 seconds", gb_demand.read_demand(demand, fresh_now + 25) is None)
said = gb_wally.handle("look around the room")
check("explore does not move without a home", "not moved" in said and "Nav2" in said, said)
act = open(os.path.join(os.path.dirname(ROOT), "deploy", "wally_activate.sh")).read()
check("health is enabled for the next boot", "systemctl enable gb-health" in act)
check("clap is not enabled", "systemctl enable gb-clap" not in act)

print("test_wake_follow: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
