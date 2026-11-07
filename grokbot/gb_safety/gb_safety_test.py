#!/usr/bin/env python3
"""gb_safety_test.py - dry unit tests for gb_gate + gb_watchdog (container). Uses ONLY /gb/test_* topics and a test
ESTOP file, so nothing reaches /controller/cmd_vel or the motors. Run test instances first (see run_dry_tests.sh)."""
import json, math, os, sys, time, threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

EST = "/home/ubuntu/shared/gb_safety/ESTOP_TEST"
R = []

class T(Node):
    def __init__(self):
        super().__init__("gb_safety_test")
        self.req = self.create_publisher(String, "/gb/test_req", 10)
        self.inp = self.create_publisher(Twist, "/gb/test_in", 10)
        self.odom = self.create_publisher(Odometry, "/gb/test_odom", 10)
        self.scan = self.create_publisher(LaserScan, "/gb/test_scan", 10)
        self.wdin = self.create_publisher(Twist, "/gb/test_wd_in", 10)
        self.out = []; self.wd = []; self.wdm = []; self.lock = threading.Lock()
        self.create_subscription(Twist, "/gb/test_out", lambda m: self.rec(self.out, m), 50)
        self.create_subscription(Twist, "/gb/wd_test_cmd", lambda m: self.rec(self.wd, m), 50)
        try:
            from ros_robot_controller_msgs.msg import MotorsState
            self.create_subscription(MotorsState, "/gb/wd_test_motor",
                                     lambda m: self.wdm.append((time.time(), max([abs(x.rps) for x in m.data] or [0]))), 50)
        except Exception: pass
        self.yaw = 0.0; self.front = 2.0; self.rear = 2.0; self.side = 2.0
        self.create_timer(0.05, self.fake_sensors)
    def rec(self, L, m):
        with self.lock: L.append((time.time(), m.linear.x, m.angular.z))
    def fake_sensors(self):
        # fake odometry: integrate the gate's test output angular velocity (closed loop for the turn test)
        with self.lock:
            w = self.out[-1][2] if self.out and time.time() - self.out[-1][0] < 0.2 else 0.0
        self.yaw = math.atan2(math.sin(self.yaw + w * 0.05), math.cos(self.yaw + w * 0.05))
        o = Odometry(); o.pose.pose.orientation.z = math.sin(self.yaw / 2); o.pose.pose.orientation.w = math.cos(self.yaw / 2)
        self.odom.publish(o)
        s = LaserScan(); s.angle_min = -math.pi; s.angle_increment = math.radians(1.0); s.range_min = 0.05; s.range_max = 12.0
        def r(i):
            a = math.degrees(-math.pi + i * s.angle_increment)
            if abs(a) < 20: return self.front
            if abs(abs(a) - 180) < 20: return self.rear
            if 80 < a < 100: return self.side
            return 2.0
        s.ranges = [float(r(i)) for i in range(360)]
        self.scan.publish(s)
    def send(self, **d):
        import uuid; d.setdefault("id", uuid.uuid4().hex[:8])
        m = String(); m.data = json.dumps(d)
        for _ in range(3): self.req.publish(m); time.sleep(0.02)

def seg(L, t0, t1=None):
    return [x for x in L if x[0] >= t0 and (t1 is None or x[0] <= t1)]

def analyse(L, t0):
    s = seg(L, t0)
    nz = [x for x in s if abs(x[1]) > 1e-6 or abs(x[2]) > 1e-6]
    if not nz: return {"first_nz": None, "last_nz": None, "dur": 0.0, "zero_after": None, "zeros_2s": 0, "sign": 0}
    a, b = nz[0][0], nz[-1][0]
    z = [x for x in s if x[0] > b and abs(x[1]) < 1e-6 and abs(x[2]) < 1e-6]
    za = z[0][0] if z else None
    zs = len([x for x in z if za and x[0] <= za + 2.05])
    return {"first_nz": a, "last_nz": b, "dur": round((za or b) - a, 3), "zero_after": za, "zeros_2s": zs,
            "sign": (1 if nz[0][1] > 0 else -1) if abs(nz[0][1]) > 1e-6 else (1 if nz[0][2] > 0 else -1),
            "max_lin": max(abs(x[1]) for x in nz), "max_ang": max(abs(x[2]) for x in nz)}

def check(name, ok, detail):
    R.append({"test": name, "result": "PASS" if ok else "FAIL", "detail": detail})
    print(("PASS " if ok else "FAIL ") + name + " :: " + json.dumps(detail, default=str), flush=True)

def main():
    rclpy.init(); n = T()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    try: os.remove(EST)
    except FileNotFoundError: pass
    time.sleep(4.0)
    def gap(): time.sleep(2.6)
    # 1 timed 4 s ("go forward for 4 seconds" / "run for four seconds then stop")
    t0 = time.time(); n.send(op="timed", lin=0.08, dur=4, src="test"); time.sleep(5.0); gap(); a = analyse(n.out, t0)
    check("timed 4s stops by gate deadline", abs(a["dur"] - 4.0) <= 0.15 and a["sign"] == 1 and a["zeros_2s"] >= 15,
          {"dur_s": a["dur"], "sign": a["sign"], "zero_msgs_in_2s": a["zeros_2s"], "latency_send_to_first_s": round(a["first_nz"] - t0, 3) if a["first_nz"] else None})
    # 2 no duration -> default 3 s
    t0 = time.time(); n.send(op="timed", lin=0.08, src="test"); time.sleep(4.0); gap(); a = analyse(n.out, t0)
    check("timed with no duration -> default 3s", abs(a["dur"] - 3.0) <= 0.15, {"dur_s": a["dur"]})
    # 3 duration 0 -> default 3 s (never indefinite)
    t0 = time.time(); n.send(op="timed", lin=0.08, dur=0, src="test"); time.sleep(4.0); gap(); a = analyse(n.out, t0)
    check("timed dur=0 -> default 3s (not indefinite)", abs(a["dur"] - 3.0) <= 0.15, {"dur_s": a["dur"]})
    # 4 duration 30 -> hard cap 10 s, speed cap
    t0 = time.time(); n.send(op="timed", lin=0.5, dur=30, src="test"); time.sleep(11.0); gap(); a = analyse(n.out, t0)
    check("timed dur=30 -> hard cap 10s, lin capped 0.12", abs(a["dur"] - 10.0) <= 0.15 and a["max_lin"] <= 0.12 + 1e-9,
          {"dur_s": a["dur"], "max_lin": a["max_lin"]})
    # 5 reverse sign
    t0 = time.time(); n.send(op="timed", lin=-0.08, dur=2, src="test"); time.sleep(3.0); gap(); a = analyse(n.out, t0)
    check("timed reverse 2s sign negative", a["sign"] == -1 and abs(a["dur"] - 2.0) <= 0.15, {"dur_s": a["dur"], "sign": a["sign"]})
    # 6 stop override mid-run
    t0 = time.time(); n.send(op="timed", lin=0.08, dur=10, src="test"); time.sleep(2.0); ts = time.time(); n.send(op="stop", src="test")
    time.sleep(1.0); gap(); a = analyse(n.out, t0)
    lat = (a["zero_after"] - ts) if a["zero_after"] else None
    check("stop request mid-run -> zero < 0.3s", lat is not None and lat < 0.3, {"stop_latency_s": round(lat, 3) if lat else None, "ran_s": a["dur"]})
    # 7 ESTOP file mid-run, latch, refuse, release
    t0 = time.time(); n.send(op="timed", lin=0.08, dur=10, src="test"); time.sleep(2.0); ts = time.time(); open(EST, "w").write("{}")
    time.sleep(1.0); a = analyse(n.out, t0); lat = (a["zero_after"] - ts) if a["zero_after"] else None
    t1 = time.time(); n.send(op="timed", lin=0.08, dur=2, src="test"); time.sleep(2.5)
    nz = [x for x in seg(n.out, t1) if abs(x[1]) > 1e-6]; zs = len(seg(n.out, t1))
    check("ESTOP file mid-run -> zero < 0.3s", lat is not None and lat < 0.3, {"estop_latency_s": round(lat, 3) if lat else None})
    check("ESTOP latched: new request refused, zero keeps publishing", not nz and zs >= 20, {"nonzero_after": len(nz), "zero_msgs_2.5s": zs})
    os.remove(EST); time.sleep(0.5)
    t1 = time.time(); n.send(op="timed", lin=0.08, dur=1, src="test"); time.sleep(1.5); gap(); a = analyse(n.out, t1)
    check("after ESTOP cleared, motion allowed again", abs(a["dur"] - 1.0) <= 0.15, {"dur_s": a["dur"]})
    # 8 stream deadman: publisher 'dies' after 1 s
    t0 = time.time()
    while time.time() - t0 < 1.0:
        m = Twist(); m.linear.x = 0.05; n.inp.publish(m); time.sleep(0.1)
    tl = time.time(); time.sleep(1.0); gap(); a = analyse(n.out, t0)
    lat = (a["zero_after"] - tl) if a["zero_after"] else None
    check("stream deadman: publisher stops -> zero within 0.5s", lat is not None and lat <= 0.55,
          {"deadman_latency_s": round(lat, 3) if lat else None, "zero_msgs_in_2s": a["zeros_2s"]})
    check("deadman zero repeated ~10 Hz for 2 s", a["zeros_2s"] >= 18, {"zero_msgs_in_2s": a["zeros_2s"]})
    # 9 turn 90 deg closed loop (fake odom)
    y0 = n.yaw; t0 = time.time(); n.send(op="turn", deg=-90, ang=0.48, src="test"); time.sleep(6.0); gap(); a = analyse(n.out, t0)
    turned = math.degrees(math.atan2(math.sin(n.yaw - y0), math.cos(n.yaw - y0)))
    check("turn -90 deg completes and stops (fake odom)", abs(turned + 90) <= 6 and a["zero_after"] is not None and a["sign"] == -1,
          {"turned_deg": round(turned, 1), "dur_s": a["dur"]})
    # 10 front block refuses forward timed
    n.front = 0.15; time.sleep(0.5); t0 = time.time(); n.send(op="timed", lin=0.08, dur=3, src="test"); time.sleep(1.0); a = analyse(n.out, t0)
    check("timed forward with obstacle 0.15m -> stops at once", a["dur"] <= 0.15, {"dur_s": a["dur"]})
    n.front = 2.0; gap()
    n.rear = 0.18; time.sleep(0.5); t0 = time.time(); n.send(op="timed", lin=-0.08, dur=3, src="test"); time.sleep(1.0); a = analyse(n.out, t0)
    check("guard: reverse with obstacle 0.18m behind -> blocked", a["dur"] <= 0.15, {"dur_s": a["dur"]})
    t0 = time.time(); n.send(op="timed", lin=0.08, dur=1, src="test"); time.sleep(1.5); a = analyse(n.out, t0)
    check("guard: forward allowed while only the rear is blocked", abs(a["dur"] - 1.0) <= 0.15, {"dur_s": a["dur"]})
    n.rear = 2.0; n.side = 0.15; time.sleep(0.5); gap(); t0 = time.time(); n.send(op="turn", deg=90, src="test"); time.sleep(1.0); a = analyse(n.out, t0)
    check("guard: turn with obstacle 0.15m at the side (sweep) -> blocked", a["dur"] <= 0.15, {"dur_s": a["dur"]})
    n.side = 2.0; gap()
    # mission stream: forward blocked at 0.25 (stream threshold) but zero-suppressed, not ended; resumes when clear
    n.front = 0.22; time.sleep(0.5); t0 = time.time()
    while time.time() - t0 < 1.0:
        m = Twist(); m.linear.x = 0.05; n.inp.publish(m); time.sleep(0.1)
    blk = [x for x in seg(n.out, t0) if abs(x[1]) > 1e-6]
    n.front = 2.0; time.sleep(0.4); t1 = time.time()
    while time.time() - t1 < 1.0:
        m = Twist(); m.linear.x = 0.05; n.inp.publish(m); time.sleep(0.1)
    ok2 = [x for x in seg(n.out, t1) if abs(x[1]) > 1e-6]
    check("guard: mission stream suppressed at 0.22m (<0.25), resumes when clear", not blk and len(ok2) > 5,
          {"nonzero_while_blocked": len(blk), "nonzero_after_clear": len(ok2)}); gap()
    # 11 watchdog: stale non-zero (latched command) -> zero within ~2 s
    t0 = time.time()
    for _ in range(10):
        m = Twist(); m.linear.x = 0.07; n.wdin.publish(m); time.sleep(0.1)
    tl = time.time(); time.sleep(3.0)
    z = [x for x in seg(n.wd, tl)]; lat = (z[0][0] - tl) if z else None
    zm = [x for x in n.wdm if x[0] >= tl + 1.5 and x[1] == 0]
    check("watchdog: non-zero left un-refreshed -> zero Twist + motor zero at ~2s", lat is not None and 1.8 <= lat <= 2.4 and len(zm) > 0,
          {"watchdog_latency_s": round(lat, 3) if lat else None, "zero_twists": len(z), "motor_zero_msgs": len(zm)})
    # 12 watchdog ESTOP
    tl = time.time(); open(EST, "w").write("{}"); time.sleep(1.5); z = seg(n.wd, tl); os.remove(EST)
    check("watchdog: ESTOP -> zero at ~10 Hz", len(z) >= 10, {"zeros_1.5s": len(z), "first_s": round(z[0][0] - tl, 3) if z else None})
    json.dump(R, open("/home/ubuntu/shared/gb_safety/dry_results.json", "w"), indent=1)
    print("SUMMARY %d/%d PASS" % (sum(r["result"] == "PASS" for r in R), len(R)), flush=True)
    os._exit(0)

if __name__ == "__main__":
    main()
