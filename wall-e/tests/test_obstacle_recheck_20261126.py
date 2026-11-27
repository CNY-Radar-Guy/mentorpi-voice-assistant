"""2026-11-26 John: after a front obstacle stop, wait 3s, resume if clear, stay stopped if not."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gb_explore as X

P = F = 0
def check(name, ok, detail=""):
    global P, F
    P += bool(ok); F += (not ok)
    print(("PASS " if ok else "FAIL ") + name + (("  " + str(detail)) if detail and not ok else ""))

check("recheck wait default is 3.0 s (John's 3-second variant)", abs(X.OBSTACLE_RECHECK_WAIT_S - 3.0) < 1e-9, X.OBSTACLE_RECHECK_WAIT_S)
check("front obstacle trip: 'obstacle 0.26m ahead' is a front object stop",
      X.is_front_obstacle_trip("obstacle 0.26m ahead (lidar-memory, stop 0.27m)"))
check("tilt is not a front-object recheck", not X.is_front_obstacle_trip("IMU tilt 18 deg"))
check("jolt is not a front-object recheck", not X.is_front_obstacle_trip("IMU jolt"))
check("gate stops in explore params still 0.27/0.28/0.24",
      X.effective_explore_params()["gate_stops_untouched_m"] == {"front": 0.27, "rear": 0.28, "sweep": 0.24})

# Case 1: object clears within 3s -> resumes
avoid, stop = 0.49, 0.27
check("before 3s, stay in wait even if already clear",
      X.obstacle_recheck_decision(2.9, 3.0, 1.5, avoid, stop, armed=True) == "wait")
check("at 3s, clear front, armed, no estop/abort -> resume",
      X.obstacle_recheck_decision(3.0, 3.0, 1.5, avoid, stop, estop=False, abort=False, armed=True) == "resume")
check("cleared just at avoid distance resumes",
      X.obstacle_recheck_decision(3.0, 3.0, 0.49, avoid, stop, armed=True) == "resume")

# Case 2: still blocked after 3s -> stays stopped
check("at 3s, still 0.26m ahead -> stay stopped (do not force motion)",
      X.obstacle_recheck_decision(3.0, 3.0, 0.26, avoid, stop, armed=True) == "stay_stopped")
check("at 3s, front 0.30 (above gate stop but below avoid) -> stay stopped",
      X.obstacle_recheck_decision(3.0, 3.0, 0.30, avoid, stop, armed=True) == "stay_stopped")
check("clear after 3s but ESTOP set -> no resume",
      X.obstacle_recheck_decision(3.0, 3.0, 1.5, avoid, stop, estop=True, armed=True) != "resume")
check("clear after 3s but disarmed -> no resume",
      X.obstacle_recheck_decision(3.0, 3.0, 1.5, avoid, stop, armed=False) != "resume")
check("clear after 3s but abort/stop -> no resume",
      X.obstacle_recheck_decision(3.0, 3.0, 1.5, avoid, stop, abort=True, armed=True) != "resume")
check("during the 3s wait, a still-blocked front does not resume",
      X.obstacle_recheck_decision(1.0, 3.0, 0.26, avoid, stop, armed=True) == "wait")

src = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gb_explore.py"), encoding="utf-8").read()
check("wander repositions after a transient obstacle (no 3-in-20s abort)",
      "REPOSITION: transient trip" in src and "recovered via reposition" in src and "resume explore" in src)
check("legacy 3s recheck helpers remain for retrace / other modes",
      "def obstacle_recheck_decision" in src and "obstacle still present" in src)
check("gb_gate stop values remain documented as 0.27/0.28/0.24",
      '"front": 0.27, "rear": 0.28, "sweep": 0.24' in src)

print("RESULT: %d passed, %d failed" % (P, F))
sys.exit(1 if F else 0)
