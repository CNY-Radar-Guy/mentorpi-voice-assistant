"""Bounded look through the gate. Replay only. A restart does not resume."""
import math, os, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gb_bounded
import gb_obstacle as OB
import gb_wally

OK = BAD = 0

def check(name, cond, info=""):
    global OK, BAD
    if cond:
        OK += 1
    else:
        BAD += 1
        print("FAIL", name, info)

def step(look, t, pose, front=2.0, scan_age=0.1, odom_age=0.1, source="/odom"):
    return look.step(t, front, 2.0, 2.0, pose, scan_age, odom_age, source=source)

look = gb_bounded.BoundedLook()
idle = look.step(0, 2, 2, 2, (0, 0, 0), 0.1, 0.1)
check("an unasked look does not move", idle["lin"] == 0.0 and idle["allow"] is False, idle)
refused = gb_bounded.BoundedLook().begin(0, (0, 0, 0), source="/odom_raw")
check("wheel odometry cannot start the look", refused.active is False and refused.reason == "source_refused", refused.reason)
check("declared lidar yaw is zero from the user alignment", gb_bounded.LIDAR_YAW_RAD == 0.0 and "t906" in gb_bounded.LIDAR_ALIGN_NOTE)
disagree = gb_bounded.BoundedLook().begin(0, (0, 0, 0), lidar_yaw=math.pi)
check("a contradictory lidar yaw does not start", disagree.active is False and disagree.reason == "lidar_yaw_disagrees")
look.begin(0, (1.0, 2.0, 0.0))
out = step(look, 1, (1.0, 2.0, 0.0))
check("clear space requests a slow forward", out["allow"] and 0 < out["lin"] <= gb_bounded.MAX_LIN and abs(out["ang"]) <= gb_bounded.MAX_ANG, out)
blocked = look.step(2, OB.STOP_FWD, 2.0, 2.0, (1.2, 2.0, 0.0), 0.1, 0.1)
check("the forward stop blocks closing", blocked["lin"] == 0.0, blocked)
turned = look.step(3, OB.STOP_FWD - 0.01, 1.0, 1.0, (1.2, 2.0, 0.0), 0.1, 0.1)
check("a blocked front turns instead of driving in", turned["lin"] == 0.0 and turned["allow"] and turned["ang"] != 0.0, turned)
stale = step(look, 4, (1.2, 2.0, 0.0), scan_age=1.0)
check("a stale scan stops the look", stale["reason"] == "sensors_stale" and look.active is False, stale)
future = gb_bounded.BoundedLook()
future.begin(0, (0, 0, 0))
fut = step(future, 1, (0.1, 0, 0), odom_age=-0.2)
check("a future odom stamp stops the look", fut["reason"] == "sensors_stale" and future.active is False, fut)

back = gb_bounded.BoundedLook()
back.begin(0, (0, 0, 0))
x = 0.0
for i in range(3):
    x += 0.3
    step(back, 1 + i, (x, 0, 0))
away = step(back, gb_bounded.CAP_S + 1, (x, 0, 0))
check("facing away from home turns and does not drive forward", away["reason"] == "return_turn" and away["lin"] == 0.0 and away["ang"] != 0.0, away)
yaw = 0.0
cmd = away
for i, yaw in enumerate((0.5, 1.0, 1.5, 2.0, 2.5, 3.0)):
    cmd = step(back, gb_bounded.CAP_S + 2 + i, (0.9, 0, yaw))
facing = step(back, gb_bounded.CAP_S + 9, (0.9, 0, math.pi))
check("facing home drives forward", facing["reason"] == "return_odom" and facing["lin"] > 0 and facing["allow"], facing)
x = 0.9
near = None
for i in range(6):
    x = max(0.0, x - 0.2)
    near = step(back, gb_bounded.CAP_S + 10 + i, (x, 0, math.pi))
    if not back.active:
        break
check("the odom threshold stops without claiming surveyed accuracy",
      near["reason"] == "odom_near" and near["accuracy_claimed"] is False and back.active is False, near)

neg = gb_bounded.BoundedLook()
neg.begin(0, (0, 0, -1.2))
neg.phase = "return"
down = step(neg, 1, (0.0, 0.3, -math.pi / 2))
check("a negative heading toward home drives forward", down["reason"] == "return_odom" and down["lin"] > 0, down)
side = gb_bounded.BoundedLook()
side.begin(0, (0, 0, 0))
side.phase = "return"
side.last = (-0.35, 0.0, math.pi)
spin = step(side, 1, (-0.35, 0.0, math.pi))
check("a reversed heading does not drive away from home", spin["lin"] == 0.0 and spin["reason"] == "return_turn", spin)
wrapped = gb_bounded.BoundedLook()
wrapped.begin(0, (0, 0, 0))
wrapped.phase = "return"
pose = (-math.cos(3.1), -math.sin(3.1), -3.1)
wrapped.last = pose
around = step(wrapped, 1, pose)
check("an angle wrap near pi still faces home", around["reason"] == "return_odom" and around["lin"] > 0, around)

jumped = gb_bounded.BoundedLook()
jumped.begin(0, (1.0, 0, 0))
jump = step(jumped, 1, (0.0, 0, 0))
check("an odom reset stops the look", jump["reason"] == "odom_jump" and jumped.active is False, jump)
mixed = gb_bounded.BoundedLook()
mixed.begin(0, (0, 0, 0), source="/odom")
sw = step(mixed, 1, (0.1, 0, 0), source="/odom_raw")
check("a mid-run source switch stops and does not adopt the other pose", sw["reason"] == "source_switch" and mixed.active is False, sw)

blocked_return = gb_bounded.BoundedLook()
blocked_return.begin(0, (0, 0, 0))
blocked_return.phase = "return"
blocked_return.last = (0.4, 0, math.pi)
hit = blocked_return.step(1, OB.STOP_FWD, 2.0, 2.0, (0.4, 0, math.pi), 0.1, 0.1)
check("an obstacle on the return ends the look", hit["reason"] == "obstacle" and hit["allow"] is False and blocked_return.active is False, hit)

late = gb_bounded.BoundedLook()
late.begin(0, (0, 0, 0))
late.phase = "return"
late.last = (1.0, 0, math.pi)
step(late, 1, (1.0, 0, math.pi))
over = step(late, 1 + gb_bounded.RETURN_CAP_S + 1, (0.8, 0, math.pi))
check("the return has its own deadline", over["reason"] == "return_timeout" and late.active is False, over)
budget = gb_bounded.BoundedLook()
budget.begin(0, (0, 0, 0))
budget.phase = "return"
budget.last = (1.0, 0.0, math.pi)
step(budget, 1, (1.0, 0.0, math.pi))
y = 0.0
spent = None
for i in range(1, 9):
    y += 0.2
    spent = step(budget, 1 + i, (1.0, y, math.pi))
    if not budget.active:
        break
check("the return distance budget fails closed", spent["reason"] == "return_budget" and budget.active is False, spent)
stopped = gb_bounded.BoundedLook()
stopped.begin(0, (0, 0, 0))
stop = stopped.cancel("stop")
after = step(stopped, 2, (0.2, 0, 0))
check("stop overrides a running look", stop["allow"] is False and after["allow"] is False and stopped.active is False, after)

path = os.path.join(tempfile.mkdtemp(prefix="gbbound_"), "look.json")
back.save(path)
again = gb_bounded.BoundedLook().load(path)
check("a saved look does not resume", again.active is False and again.requested is False and again.reason == "not_resumed", again.reason)
gb_wally.set_autonomy(False)
said = gb_wally.handle("look around")
check("asking does not claim the wheels moved", "not moved" in said and "Nav2" in said and "gate" in said and "not a mapped explore" in said, said)
check("look around does not arm autonomy", gb_wally._autonomy is False)
mapped = gb_wally.handle("map this room")
check("map this room does not start a drive", "not moved" in mapped and "not a mapped explore" in mapped, mapped)
home_said = gb_wally.handle("go home")
check("go home does not start an outbound look", "not moved" in home_said and "Nav2" in home_said, home_said)
gate = open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "gb_safety", "gb_gate.py")).read()
check("the gate runs the look through its own guard", 'op == "bounded"' in gate and "self.guard(" in gate)
check("the look reads only the filtered odom pose", 'self.poses.get("/odom")' in gate and "self.ox, self.oy, self.yaw" not in gate)
check("obstacle limits stay 0.27, 0.28 and 0.24", OB.STOP_FWD == 0.27 and OB.STOP_REV == 0.28 and abs(OB.SWEEP_R - 0.24) < 1e-9)

print("test_bounded: %d passed, %d failed" % (OK, BAD))
sys.exit(0 if BAD == 0 else 1)
