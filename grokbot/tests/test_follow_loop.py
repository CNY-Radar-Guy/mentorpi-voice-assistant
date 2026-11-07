"""Follow loop: start, loss, timeout, and a stop. No motor publish."""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gb_follow
import gb_obstacle as OB

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

now = time.time()
health = {"t": now, "mode": "lidar", "lidar_online": True, "lidar_valid_frac": 0.98, "user_lidar": True}
person = {"fresh": True, "conf": 0.9, "source": "depth", "depth_m": 2.4, "unit": "documented_mm",
          "depth_age": 0.1, "bearing": 0.0}
loop = gb_follow.FollowLoop()
check("startup is idle", loop.step(now, health, person, True)["reason"] == "idle")
loop.start(now)
first = loop.step(now, health, None, True)
check("no person at startup does not move", first["allow"] is False and first["lin"] == 0.0 and first["reason"] == "person_lost", first)
loop.start(now)
loop.step(now, health, person, True)
gap = loop.step(now + 0.2, health, None, True)
check("a brief loss holds", gap["reason"] == "person_gap" and gap["lin"] == 0.0, gap)
lost = loop.step(now + 1.3, health, None, True)
check("a lost person stops the loop", lost["reason"] == "person_lost_later" and loop.active is False, lost)
loop.start(now)
late = loop.step(now + gb_follow.MAX_RUN_S + 1, health, person, True)
check("the follow expires", late["reason"] == "timeout" and late["lin"] == 0.0, late)
loop.start(now)
stopped = loop.step(now, health, person, True, protections={"estop": True})
check("an emergency stop ends the follow", stopped["reason"] == "estop" and stopped["lin"] == 0.0 and loop.active is False, stopped)
near = gb_follow.follow_command(1.2, "depth", 0.0, True, autonomy=True)
check("inside the contact distance the follow does not close", near["lin"] == 0.0, near)
far = gb_follow.follow_command(3.0, "depth", 0.0, True, autonomy=True)
check("a far person is only a slow command", 0 < far["lin"] <= gb_follow.MAX_LIN, far)
check("obstacle limits are unchanged", OB.STOP_FWD == 0.27 and OB.STOP_REV == 0.28 and abs(OB.SWEEP_R - 0.24) < 1e-9)

print("test_follow_loop: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
